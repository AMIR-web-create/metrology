/*
 * Dual Smart Scale — ESP32
 * Mode 1 PREPACK : OIML R87 quantity (T1, T2, corrected mean) + R79 label fields
 * Mode 2 TRADE   : OIML R111 Class M3, error = measured - nominal, |error| <= MPE
 *
 * Buttons: MODE=32  TARE=17  SAVE=4  SUBMIT=16
 * HX711:   DOUT=26  CLK=25
 * LEDs:    green=14 red=27
 * LCD:     SDA=21   SCL=22  addr 0x20
 *
 * Boot: hold TARE = calibrate; hold MODE = factory defaults.
 * Pre-pack: V1 declared quantity ("500g"/"1L"), V3 product, V5 maker,
 * V2 lot size, V6 tare, V19 WhatsApp phone, V20 batch.
 * App controls (String): V14 mode ("prepack"/"1" or "trade"/"2"),
 * V21 tare, V22 save, V23 submit. Same actions as the push buttons.
 * TARE stores the package weight. SAVE logs gross and net. SUBMIT analyses
 * the lot and queues a BRIQ WhatsApp report.
 * Certificate: http://<ip>/   Record: http://<ip>/verify?id=<n>
 */

#include "blynk_secrets.h"

#include <Arduino.h>
#include <AsyncTCP.h>
#include <BlynkSimpleEsp32.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <HX711.h>
#include <LiquidCrystal_I2C.h>
#include <Preferences.h>
#include <WiFi.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "briq.h"
#include "config.h"

static_assert(VP_INSPECTOR == 0, "VP_INSPECTOR");
static_assert(VP_QN == 1, "VP_QN");
static_assert(VP_LOT == 2, "VP_LOT");
static_assert(VP_PRODUCT == 3, "VP_PRODUCT");
static_assert(VP_ADDRESS == 4, "VP_ADDRESS");
static_assert(VP_MAKER == 5, "VP_MAKER");
static_assert(VP_SHOP == 8, "VP_SHOP");
static_assert(VP_LOCATION == 9, "VP_LOCATION");
static_assert(VP_NOMINAL == 10, "VP_NOMINAL");
static_assert(VP_CLASS == 11, "VP_CLASS");
static_assert(VP_VERDICT == 13, "VP_VERDICT");
static_assert(VP_MODE == 14, "VP_MODE");
static_assert(VP_NET_G == 15, "VP_NET_G");
static_assert(VP_META == 18, "VP_META");
static_assert(VP_PHONE == 19, "VP_PHONE");
static_assert(VP_BATCH == 20, "VP_BATCH");
static_assert(VP_CMD_TARE == 21, "VP_CMD_TARE");
static_assert(VP_CMD_SAVE == 22, "VP_CMD_SAVE");
static_assert(VP_CMD_SUBMIT == 23, "VP_CMD_SUBMIT");
static_assert(VP_PKG_TARE == 6, "VP_PKG_TARE");
static_assert(VP_DENSITY == 7, "VP_DENSITY");

// Certificate timestamps. The desk this firmware was written for is UTC+3.
static const long TZ_OFFSET_SEC = 3 * 3600;
static const uint8_t TARE_READS = 16;
static const uint8_t CAL_READS = 20;
static const char* NOT_FOUND_PAGE =
    "<!DOCTYPE html><html><body><h1>Record not found</h1>"
    "<p>No stored trade weight or sealed lot uses that id.</p>"
    "<p><a href=\"/\">Certificate</a></p></body></html>";

enum UiState : uint8_t { UI_RUN, UI_SETTINGS, UI_CALIBRATE, UI_MESSAGE };
enum SetPage : uint8_t { SET_A, SET_B };
enum BtnEvent : uint8_t { BTN_NONE, BTN_SHORT, BTN_LONG, BTN_REPEAT };

struct Btn {
  int pin;
  bool lastRaw;
  bool down;
  uint32_t lastChange;
  uint32_t pressStart;
  uint32_t lastRepeat;
  bool longFired;
};

HX711 scale;
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);
BlynkTimer blynkTimer;
AsyncWebServer server(80);

float calFactor = DEFAULT_CAL_FACTOR;
float liveG = 0.0f;
bool stable = false;
bool overload = false;
bool scaleSeen = false;

float hist[HX711_SAMPLES];
uint8_t histN = 0;
uint8_t histI = 0;

UiState uiState = UI_RUN;
UiState msgBack = UI_RUN;
SetPage setPage = SET_A;
uint32_t msgUntil = 0;
uint32_t lastDisplay = 0;
uint32_t ledUntil = 0;
bool verdictHeld = false;
bool lastPass = false;
char lcdNote[21] = "";

bool tareRun = false;
double tareSum = 0.0;
uint8_t tareGot = 0;
uint32_t tareStart = 0;

bool calAvg = false;
double calSum = 0.0;
uint8_t calGot = 0;
uint32_t calStart = 0;
float calKnownG = 1000.0f;

BriqView view = {};
bool viewOk = false;
char verdict[140] = "Ready";
char declaredLabel[STR_LEN] = "500g";
float grossWeights[MAX_BATCH_N];
float netContents[MAX_BATCH_N];
uint16_t sampleCount = 0;
bool pendingWhatsAppReport = false;
char waBody[360];
char ipLine[21] = "WiFi connecting";
uint32_t wifiBegan = 0;
bool timeReady = false;
bool mdnsReady = false;
char lastIp[20] = "";

Btn btnMode = {PIN_BTN_MODE, true, false, 0, 0, 0, false};
Btn btnTare = {PIN_BTN_TARE, true, false, 0, 0, 0, false};
Btn btnSave = {PIN_BTN_SAVE, true, false, 0, 0, 0, false};
Btn btnSubmit = {PIN_BTN_SUBMIT, true, false, 0, 0, 0, false};

static void lcdPad(const char* s) {
  char buf[21];
  memset(buf, ' ', 20);
  buf[20] = 0;
  if (s) {
    size_t n = strlen(s);
    if (n > 20) n = 20;
    memcpy(buf, s, n);
  }
  lcd.print(buf);
}

static void showMessage(const char* a, const char* b = "", uint32_t ms = 1400,
                        UiState back = UI_RUN) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcdPad(a);
  lcd.setCursor(0, 1);
  lcdPad(b);
  uiState = UI_MESSAGE;
  msgBack = back;
  msgUntil = millis() + ms;
}

static bool refreshView() {
  BriqView tmp;
  if (!briqRead(tmp)) return viewOk;
  view = tmp;
  viewOk = true;
  return true;
}

static void blynkWriteText(int pin, const char* text) {
  if (!Blynk.connected() || !text) return;
  Blynk.virtualWrite(pin, text);
}

static void blynkWriteNumber(int pin, float value, int decimals) {
  char buf[24];
  if (decimals <= 0) snprintf(buf, sizeof(buf), "%.0f", value);
  else if (decimals == 1) snprintf(buf, sizeof(buf), "%.1f", value);
  else snprintf(buf, sizeof(buf), "%.2f", value);
  blynkWriteText(pin, buf);
}

static const char* modeLabel(BriqMode m) {
  return m == BRIQ_TRADE ? "trade" : "prepack";
}

static void blynkWriteQn() {
  if (!viewOk) return;
  const char* unit = (view.density >= 0.05f) ? "ml" : "g";
  snprintf(declaredLabel, sizeof(declaredLabel), "%.0f%s", view.qnDeclared, unit);
  blynkWriteText(VP_QN, declaredLabel);
}

static bool sameWord(const char* a, const char* b) {
  if (!a || !b) return false;
  while (*a && *b) {
    char ca = *a;
    char cb = *b;
    if (ca >= 'A' && ca <= 'Z') ca = (char)(ca - 'A' + 'a');
    if (cb >= 'A' && cb <= 'Z') cb = (char)(cb - 'A' + 'a');
    if (ca != cb) return false;
    ++a;
    ++b;
  }
  return *a == 0 && *b == 0;
}

static bool parseNumberText(const char* s, float* out) {
  if (!s || !out) return false;
  while (*s == ' ') ++s;
  if (!*s) return false;
  char* end = nullptr;
  float v = strtof(s, &end);
  if (end == s) return false;
  *out = v;
  return true;
}

static bool parseAppMode(const char* s, BriqMode* out) {
  if (!s || !out) return false;
  while (*s == ' ') ++s;
  if (!*s) return false;
  if (sameWord(s, "1") || sameWord(s, "prepack") || sameWord(s, "pre-pack") ||
      sameWord(s, "prepacked") || sameWord(s, "pre packed") ||
      sameWord(s, "inspect") || sameWord(s, "inspection") || sameWord(s, "r87")) {
    *out = BRIQ_PREPACK;
    return true;
  }
  if (sameWord(s, "2") || sameWord(s, "trade") || sameWord(s, "verify") ||
      sameWord(s, "verification") || sameWord(s, "weight") || sameWord(s, "r111")) {
    *out = BRIQ_TRADE;
    return true;
  }
  return false;
}

static bool appPressed(const char* s) {
  if (!s || !s[0]) return false;
  while (*s == ' ') ++s;
  if (!*s) return false;
  if (sameWord(s, "0") || sameWord(s, "off") || sameWord(s, "false")) return false;
  return true;
}

static bool appControlsReady() {
  return uiState != UI_CALIBRATE && uiState != UI_SETTINGS && !tareRun && !calAvg;
}

static void ledsOff() {
  digitalWrite(PIN_LED_PASS, LOW);
  digitalWrite(PIN_LED_FAIL, LOW);
}

static void ledsVerdict(bool pass) {
  digitalWrite(PIN_LED_PASS, pass ? HIGH : LOW);
  digitalWrite(PIN_LED_FAIL, pass ? LOW : HIGH);
  lastPass = pass;
  verdictHeld = true;
  ledUntil = millis() + LED_IDLE_MS;
}

static void resetHistory() {
  histN = 0;
  histI = 0;
  stable = false;
}

static float gramsFromRaw(long raw) {
  float sc = scale.get_scale();
  if (!isfinite(sc) || fabsf(sc) < 0.001f) return NAN;
  long net = raw - scale.get_offset();
  return (float)net / sc;
}

static void noteSample(float g) {
  if (!isfinite(g)) return;
  hist[histI] = g;
  histI = (histI + 1) % HX711_SAMPLES;
  if (histN < HX711_SAMPLES) histN++;

  float sum = 0.0f;
  for (uint8_t i = 0; i < histN; i++) sum += hist[i];
  liveG = sum / (float)histN;

  uint8_t spanN = histN < STABLE_WINDOW ? histN : STABLE_WINDOW;
  float mn = 0.0f;
  float mx = 0.0f;
  for (uint8_t i = 0; i < spanN; i++) {
    uint8_t idx = (uint8_t)((histI + HX711_SAMPLES - 1 - i) % HX711_SAMPLES);
    float v = hist[idx];
    if (i == 0 || v < mn) mn = v;
    if (i == 0 || v > mx) mx = v;
  }
  overload = fabsf(liveG) > SCALE_CAPACITY_G * 1.02f;
  stable = (histN >= STABLE_WINDOW) && ((mx - mn) <= STABLE_SPAN_G);
}

static void finishCal() {
  calAvg = false;
  if (calGot < 8) {
    showMessage("Cal failed", "No HX711 data", 1600, UI_CALIBRATE);
    return;
  }
  float raw = (float)(calSum / (double)calGot);
  if (fabsf(raw) < 50.0f) {
    showMessage("Cal failed", "Place known mass", 1600, UI_CALIBRATE);
    return;
  }
  calFactor = raw / calKnownG;
  scale.set_scale(calFactor);
  Preferences prefs;
  prefs.begin(NVS_NS, false);
  prefs.putFloat("cal", calFactor);
  prefs.end();
  resetHistory();
  char b[21];
  snprintf(b, sizeof(b), "k=%0.3f", calFactor);
  showMessage("Cal stored", b, 1600, UI_RUN);
  Serial.printf("Cal factor %.4f for %.1f g\n", calFactor, calKnownG);
}

static void finishTare() {
  tareRun = false;
  if (tareGot == 0) {
    showMessage("Tare failed", "HX711 not ready", 1200);
    return;
  }
  scale.set_offset((long)lround(tareSum / (double)tareGot));
  resetHistory();
  liveG = 0.0f;
  overload = false;
  verdictHeld = false;
  ledsOff();
  showMessage("TARE complete", "Platform zeroed", 900);
}

static void pollScale() {
  uint32_t now = millis();
  if (tareRun) {
    if ((uint32_t)(now - tareStart) > 8000 && tareGot < TARE_READS) {
      if (tareGot >= 8) finishTare();
      else {
        tareRun = false;
        showMessage("Tare failed", "HX711 not ready", 1200);
      }
      return;
    }
    if (!scale.is_ready()) return;
    scaleSeen = true;
    tareSum += (double)scale.read();
    if (++tareGot >= TARE_READS) finishTare();
    return;
  }
  if (calAvg) {
    if (!scale.is_ready()) {
      if ((uint32_t)(now - calStart) > 8000) finishCal();
      return;
    }
    scaleSeen = true;
    calSum += (double)scale.read() - (double)scale.get_offset();
    calGot++;
    if (calGot >= CAL_READS || (uint32_t)(now - calStart) > 8000) finishCal();
    return;
  }
  if (uiState == UI_CALIBRATE) return;
  if (!scale.is_ready()) return;
  scaleSeen = true;
  noteSample(gramsFromRaw(scale.read()));
}

static void startTare() {
  tareRun = true;
  tareSum = 0.0;
  tareGot = 0;
  tareStart = millis();
  resetHistory();
  showMessage("Taring...", "Keep pan empty", 800);
}

static void startCalAverage() {
  calAvg = true;
  calSum = 0.0;
  calGot = 0;
  calStart = millis();
  showMessage("Measuring...", "Leave mass on pan", 600, UI_CALIBRATE);
}

static const char* linkCh() {
  if (Blynk.connected()) return "B";
  if (WiFi.status() == WL_CONNECTED) return "W";
  return "-";
}

static void drawRun() {
  char l0[21], l1[21], l2[21], l3[21];
  bool showIp = ((millis() / 3000) & 1) == 0;
  float net = liveG - (viewOk ? view.packageTareG : 0.0f);

  if (!scaleSeen && millis() > 2500) {
    snprintf(l0, sizeof(l0), "HX711 not ready");
    snprintf(l1, sizeof(l1), "DOUT 26  CLK 25");
    snprintf(l2, sizeof(l2), "Check wiring");
    snprintf(l3, sizeof(l3), "%s", ipLine);
  } else if (viewOk && view.mode == BRIQ_TRADE) {
    snprintf(l0, sizeof(l0), "M3 %s %2u/50 %s",
             overload ? "OVR" : (stable ? "STB" : "..."),
             view.tradeCount, linkCh());
    snprintf(l1, sizeof(l1), "W:%8.2fg", liveG);
    snprintf(l2, sizeof(l2), "N:%6.0f MPE:%6.2f", view.nominalG, view.mpeG);
    if (showIp) snprintf(l3, sizeof(l3), "%s", ipLine);
    else if (verdictHeld && lcdNote[0]) snprintf(l3, sizeof(l3), "%s", lcdNote);
    else if (view.shop[0]) snprintf(l3, sizeof(l3), "%s", view.shop);
    else snprintf(l3, sizeof(l3), "TARE SAVE SUB MODE");
  } else {
    uint16_t fill = viewOk ? view.sampleFill : 0;
    uint16_t plan = viewOk ? view.plannedN : 0;
    float qn = viewOk ? view.qnMassG : 0.0f;
    float t = viewOk ? view.tG : 0.0f;
    snprintf(l0, sizeof(l0), "R87 %s %3u/%-3u%s",
             overload ? "OVR" : (stable ? "OK " : "..."),
             fill, plan, linkCh());
    snprintf(l1, sizeof(l1), "Net%7.1f Qn%6.0f", net, qn);
    snprintf(l2, sizeof(l2), "T%7.1f tare%6.1f", t, viewOk ? view.packageTareG : 0.0f);
    if (showIp) snprintf(l3, sizeof(l3), "%s", ipLine);
    else if (verdictHeld && lcdNote[0]) snprintf(l3, sizeof(l3), "%s", lcdNote);
    else if (viewOk && view.product[0]) snprintf(l3, sizeof(l3), "%s", view.product);
    else if (viewOk && !view.r79Ok) snprintf(l3, sizeof(l3), "R79: fill Blynk");
    else snprintf(l3, sizeof(l3), "SAVE=log SUB=report");
  }

  lcd.setCursor(0, 0); lcdPad(l0);
  lcd.setCursor(0, 1); lcdPad(l1);
  lcd.setCursor(0, 2); lcdPad(l2);
  lcd.setCursor(0, 3); lcdPad(l3);
}

static void drawSettings() {
  char l0[21], l1[21], l2[21], l3[21];
  refreshView();
  snprintf(l0, sizeof(l0), "LOCAL SET  %s",
           (viewOk && view.mode == BRIQ_TRADE) ? "M3" : "R87");
  if (viewOk && view.mode == BRIQ_TRADE) {
    snprintf(l1, sizeof(l1), "> Nom: %.1f g", view.nominalG);
    snprintf(l2, sizeof(l2), "  MPE: %.2f g", view.mpeG);
  } else if (viewOk) {
    snprintf(l1, sizeof(l1), "%c Qn: %.1f", setPage == SET_A ? '>' : ' ', view.qnDeclared);
    snprintf(l2, sizeof(l2), "%c Lot: %lu", setPage == SET_B ? '>' : ' ',
             (unsigned long)view.lotN);
  } else {
    snprintf(l1, sizeof(l1), "Busy");
    snprintf(l2, sizeof(l2), "");
  }
  snprintf(l3, sizeof(l3), "SAVE+ TARE- SUB=exit");
  lcd.setCursor(0, 0); lcdPad(l0);
  lcd.setCursor(0, 1); lcdPad(l1);
  lcd.setCursor(0, 2); lcdPad(l2);
  lcd.setCursor(0, 3); lcdPad(l3);
}

static void drawCalibrate() {
  char l[21];
  lcd.setCursor(0, 0); lcdPad(calAvg ? "MEASURING" : "CALIBRATION  20kg");
  snprintf(l, sizeof(l), "Known: %0.0f g", calKnownG);
  lcd.setCursor(0, 1); lcdPad(l);
  snprintf(l, sizeof(l), "k: %0.3f", calFactor);
  lcd.setCursor(0, 2); lcdPad(l);
  lcd.setCursor(0, 3); lcdPad(calAvg ? "Wait..." : "SAVE+/-  SUB=set");
}

static void nudgeSetting(int dir, bool coarse) {
  if (!refreshView()) {
    showMessage("Busy", "Retry", 600, UI_SETTINGS);
    return;
  }
  if (view.mode == BRIQ_PREPACK) {
    if (setPage == SET_A) {
      float step = coarse ? 10.0f : 1.0f;
      briqSetQn(view.qnDeclared + step * (float)dir);
      refreshView();
      blynkWriteQn();
    } else {
      int32_t n = (int32_t)view.lotN + dir * (coarse ? 10 : 1);
      if (n < 1) n = 1;
      if (n > 1000000) n = 1000000;
      briqSetLot((uint32_t)n);
      refreshView();
      blynkWriteNumber(VP_LOT, (float)view.lotN, 0);
    }
  } else {
    float step = coarse ? 10.0f : 1.0f;
    briqSetNominal(view.nominalG + step * (float)dir);
    refreshView();
    blynkWriteNumber(VP_NOMINAL, view.nominalG, 1);
  }
}

static bool requireStable() {
  if (overload) {
    ledsVerdict(false);
    showMessage("OVERLOAD", "Remove weight", 1200);
    return false;
  }
  if (!stable) {
    showMessage("Not stable", "Wait, then retry", 1000);
    return false;
  }
  return true;
}

static void submitTrade() {
  if (!requireStable()) return;
  TradeCheck t = briqLogTrade(liveG);
  if (!t.valid) {
    showMessage("Busy", "Retry submit", 800);
    return;
  }
  ledsVerdict(t.pass);
  snprintf(lcdNote, sizeof(lcdNote), "%s e:%+.2f", t.pass ? "PASS" : "FAIL", t.errorG);
  snprintf(verdict, sizeof(verdict), "%s id=%lu e=%+.3fg MPE=%.3fg Nom=%.3fg",
           t.pass ? "PASS" : "FAIL", (unsigned long)t.id, t.errorG, t.mpeG, t.nominalG);
  blynkWriteText(VP_VERDICT, verdict);
  char b[21];
  snprintf(b, sizeof(b), "e:%+.3f MPE %.2f", t.errorG, t.mpeG);
  showMessage(t.pass ? "R111 PASS" : "R111 FAIL", b, 1600);
  refreshView();
}

static float getTValue(float qnG) {
  if (qnG <= 50.0f) return qnG * 0.09f;
  if (qnG <= 100.0f) return 4.5f;
  if (qnG <= 200.0f) return qnG * 0.045f;
  if (qnG <= 300.0f) return 9.0f;
  if (qnG <= 500.0f) return qnG * 0.03f;
  if (qnG <= 1000.0f) return 15.0f;
  return qnG * 0.015f;
}

static float getSCF(uint16_t n) {
  if (n <= 50) return 0.379f;
  if (n <= 80) return 0.295f;
  return 0.234f;
}

static uint16_t getMaxT1Allowed(uint16_t n) {
  if (n <= 50) return 3;
  if (n <= 80) return 5;
  return 7;
}

static uint16_t sampleSizeForLot(uint32_t lot) {
  if (lot < 500) return 50;
  if (lot <= 3200) return 80;
  return 125;
}

static long gramsToMg(float grams) {
  return lroundf(grams * 1000.0f);
}

static UnitMark classifyDeficit(float declaredGrams, float net) {
  long deficit = gramsToMg(declaredGrams) - gramsToMg(net);
  long tMg = gramsToMg(getTValue(declaredGrams));
  if (deficit <= 0) return UNIT_OK;
  if (deficit <= tMg) return UNIT_T1;
  return UNIT_T2;
}

struct BatchAnalysis {
  bool batchPass;
  uint16_t sampleCount;
  uint16_t sampleSize;
  uint16_t t1Defectives;
  uint16_t t2Defectives;
  uint16_t maxT1Allowed;
  float meanNet;
  float stdDev;
  float correctedMean;
  float declaredGrams;
  float tValue;
  float scf;
};

static BatchAnalysis analyzePrePackagedBatch() {
  BatchAnalysis a = {};
  a.sampleCount = sampleCount;
  a.sampleSize = sampleSizeForLot(viewOk ? view.lotN : DEFAULT_LOT_N);
  a.declaredGrams = viewOk ? view.qnMassG : 0.0f;
  a.tValue = getTValue(a.declaredGrams);
  a.scf = getSCF(sampleCount);
  a.maxT1Allowed = getMaxT1Allowed(sampleCount);
  if (sampleCount == 0) return a;

  double sum = 0.0;
  for (uint16_t i = 0; i < sampleCount; i++) sum += netContents[i];
  double mean = sum / (double)sampleCount;
  double acc = 0.0;
  for (uint16_t i = 0; i < sampleCount; i++) {
    double d = (double)netContents[i] - mean;
    acc += d * d;
  }
  double sd = (sampleCount >= 2) ? sqrt(acc / (double)(sampleCount - 1)) : 0.0;
  a.meanNet = (float)mean;
  a.stdDev = (float)sd;
  a.correctedMean = (float)(mean - (double)a.scf * sd);

  long qnMg = gramsToMg(a.declaredGrams);
  long tMg = gramsToMg(a.tValue);
  for (uint16_t i = 0; i < sampleCount; i++) {
    long deficit = qnMg - gramsToMg(netContents[i]);
    if (deficit > tMg) a.t2Defectives++;
    else if (deficit > 0) a.t1Defectives++;
  }
  a.batchPass = (sampleCount >= a.sampleSize) &&
                (a.t1Defectives <= a.maxT1Allowed) &&
                (a.t2Defectives == 0) &&
                (gramsToMg(a.correctedMean) >= qnMg);
  return a;
}

static bool parseDeclaredQuantity(const char* text, float* qty, bool* volume) {
  if (!text || !qty || !volume) return false;
  while (*text == ' ') text++;
  char* end = nullptr;
  float value = strtof(text, &end);
  if (end == text || !isfinite(value) || value <= 0.0f) return false;
  while (*end == ' ') end++;
  *volume = false;
  if ((end[0] == 'k' || end[0] == 'K') && (end[1] == 'g' || end[1] == 'G')) {
    value *= 1000.0f;
  } else if (end[0] == 'g' || end[0] == 'G' || end[0] == 0) {
    // grams, or a bare number
  } else if ((end[0] == 'm' || end[0] == 'M') && (end[1] == 'l' || end[1] == 'L')) {
    *volume = true;
  } else if (end[0] == 'l' || end[0] == 'L') {
    value *= 1000.0f;
    *volume = true;
  } else {
    return false;
  }
  *qty = value;
  return true;
}

static void applyDeclaredQuantity(const char* text) {
  float qty = 0.0f;
  bool volume = false;
  if (!parseDeclaredQuantity(text, &qty, &volume)) {
    showMessage("Bad quantity", "Use 500g or 1L", 1200);
    return;
  }
  if (volume) {
    briqSetDensity(1.0f);
    briqSetQn(qty);
  } else {
    briqSetDensity(0.0f);
    briqSetQn(qty);
  }
  refreshView();
  if (viewOk && view.sampleFill == 0) sampleCount = 0;
  strncpy(declaredLabel, text, sizeof(declaredLabel) - 1);
  declaredLabel[sizeof(declaredLabel) - 1] = 0;
}

static void capturePackageTare() {
  if (!requireStable()) return;
  briqSetPackageTare(liveG);
  refreshView();
  blynkWriteNumber(VP_PKG_TARE, liveG, 1);
  char b[21];
  snprintf(b, sizeof(b), "Tare %.1f g", liveG);
  showMessage("Package tare", b, 1200);
}

static void logPrepackSample() {
  if (!requireStable()) return;
  if (!refreshView()) {
    showMessage("Busy", "Retry", 800);
    return;
  }
  if (sampleCount >= MAX_BATCH_N) {
    showMessage("Log full", "Hold SAVE to clear", 1200);
    return;
  }
  float gross = liveG;
  float net = gross - view.packageTareG;
  grossWeights[sampleCount] = gross;
  netContents[sampleCount] = net;
  sampleCount++;
  briqReplacePackSamples(netContents, sampleCount);
  refreshView();

  UnitMark mark = classifyDeficit(view.qnMassG, net);
  bool hardFail = (mark == UNIT_T2);
  ledsVerdict(!hardFail);
  const char* tag = (mark == UNIT_T2) ? "T2" : (mark == UNIT_T1) ? "T1" : "OK";
  snprintf(lcdNote, sizeof(lcdNote), "%s net %.1f", tag, net);
  snprintf(verdict, sizeof(verdict), "%s gross=%.2f tare=%.2f net=%.2f %u/%u",
           tag, gross, view.packageTareG, net, sampleCount,
           sampleSizeForLot(view.lotN));
  blynkWriteText(VP_VERDICT, verdict);
  char b[21];
  snprintf(b, sizeof(b), "%s %.1fg %u/%u", tag, net, sampleCount, sampleSizeForLot(view.lotN));
  showMessage("Sample logged", b, 1200);
}

static void submitPack() {
  if (!refreshView()) {
    showMessage("Busy", "Retry submit", 800);
    return;
  }
  if (sampleCount == 0) {
    showMessage("No samples", "SAVE each pack", 1200);
    return;
  }
  BatchAnalysis a = analyzePrePackagedBatch();
  ledsVerdict(a.batchPass);
  snprintf(lcdNote, sizeof(lcdNote), "%s CM %.1f", a.batchPass ? "ACCEPT" : "REJECT", a.correctedMean);
  snprintf(verdict, sizeof(verdict),
           "%s T1 %u/%u T2 %u CM %.2f Qn %.2f n=%u/%u",
           a.batchPass ? "LOT ACCEPTED" : "LOT REJECTED",
           a.t1Defectives, a.maxT1Allowed, a.t2Defectives,
           a.correctedMean, a.declaredGrams, a.sampleCount, a.sampleSize);
  blynkWriteText(VP_VERDICT, verdict);

  const char* name = view.product[0] ? view.product : "-";
  snprintf(waBody, sizeof(waBody),
           "CBE Pre-Packaged Report: %s - %s. Samples: %u/%u. Corrected Mean: %.2fg.",
           name, a.batchPass ? "LOT ACCEPTED" : "LOT REJECTED",
           a.sampleCount, a.sampleSize, a.correctedMean);
  briqArmWhatsApp(view.phone, waBody);
  pendingWhatsAppReport = true;

  char b[21];
  snprintf(b, sizeof(b), "CM %.1f Qn %.0f", a.correctedMean, a.declaredGrams);
  showMessage(a.batchPass ? "LOT ACCEPTED" : "LOT REJECTED", b, 1800);
  Serial.printf("PREPACK %s n=%u/%u T1=%u T2=%u mean=%.2f sd=%.2f CM=%.2f\n",
                a.batchPass ? "ACCEPT" : "REJECT", a.sampleCount, a.sampleSize,
                a.t1Defectives, a.t2Defectives, a.meanNet, a.stdDev, a.correctedMean);
}

static void saveAction() {
  if (!requireStable()) return;
  if (!refreshView()) {
    showMessage("Busy", "Retry", 800);
    return;
  }
  if (view.mode == BRIQ_TRADE) {
    if (liveG < 0.1f) {
      showMessage("Weight too low", "Tare, then SAVE", 1100);
      return;
    }
    briqSetNominal(liveG);
    refreshView();
    blynkWriteNumber(VP_NOMINAL, view.nominalG, 2);
    char b[21];
    snprintf(b, sizeof(b), "Nom %.2f g", view.nominalG);
    showMessage("R111 nominal", b, 1400);
  } else {
    logPrepackSample();
  }
}

static void setMode(BriqMode m) {
  briqSetMode(m);
  verdictHeld = false;
  ledsOff();
  refreshView();
  blynkWriteText(VP_MODE, modeLabel(m));
  showMessage(m == BRIQ_PREPACK ? "Mode 1 INSPECT" : "Mode 2 TRADE",
              m == BRIQ_PREPACK ? "OIML R87 / R79" : "OIML R111 M3", 1100);
}

static BtnEvent pollBtn(Btn& b) {
  bool raw = digitalRead(b.pin);
  uint32_t now = millis();
  if (raw != b.lastRaw) {
    b.lastRaw = raw;
    b.lastChange = now;
  }
  if ((uint32_t)(now - b.lastChange) < DEBOUNCE_MS) return BTN_NONE;

  bool pressed = (raw == LOW);
  if (pressed && !b.down) {
    b.down = true;
    b.pressStart = now;
    b.longFired = false;
    b.lastRepeat = now;
    return BTN_NONE;
  }
  if (pressed && b.down) {
    if (!b.longFired && (uint32_t)(now - b.pressStart) >= LONG_PRESS_MS) {
      b.longFired = true;
      b.lastRepeat = now;
      return BTN_LONG;
    }
    if (b.longFired && (uiState == UI_SETTINGS || uiState == UI_CALIBRATE)) {
      if ((uint32_t)(now - b.lastRepeat) >= REPEAT_MS) {
        b.lastRepeat = now;
        return BTN_REPEAT;
      }
    }
    return BTN_NONE;
  }
  if (!pressed && b.down) {
    b.down = false;
    if (!b.longFired) return BTN_SHORT;
  }
  return BTN_NONE;
}

static void consumeHeld(Btn& b) {
  b.lastRaw = digitalRead(b.pin);
  b.lastChange = millis();
  b.down = (b.lastRaw == LOW);
  b.longFired = b.down;
  b.pressStart = millis();
  b.lastRepeat = millis();
}

static void handleUi() {
  BtnEvent eMode = pollBtn(btnMode);
  BtnEvent eTare = pollBtn(btnTare);
  BtnEvent eSave = pollBtn(btnSave);
  BtnEvent eSub = pollBtn(btnSubmit);

  if (uiState == UI_MESSAGE) {
    if ((int32_t)(millis() - msgUntil) >= 0) uiState = msgBack;
    return;
  }
  if (tareRun || calAvg) return;

  if (uiState == UI_CALIBRATE) {
    if (eSave == BTN_SHORT || eSave == BTN_REPEAT)
      calKnownG = constrain(calKnownG + 10.0f, 10.0f, SCALE_CAPACITY_G);
    else if (eTare == BTN_SHORT || eTare == BTN_REPEAT)
      calKnownG = constrain(calKnownG - 10.0f, 10.0f, SCALE_CAPACITY_G);
    else if (eSub == BTN_SHORT) startCalAverage();
    else if (eMode == BTN_SHORT) showMessage("Cal cancelled", "", 800, UI_RUN);
    return;
  }

  if (uiState == UI_SETTINGS) {
    if (eMode == BTN_SHORT) {
      if (viewOk && view.mode == BRIQ_PREPACK) setPage = (setPage == SET_A) ? SET_B : SET_A;
      else showMessage("Settings stored", "", 700, UI_RUN);
    }
    if (eSave == BTN_SHORT || eSave == BTN_LONG || eSave == BTN_REPEAT)
      nudgeSetting(+1, eSave != BTN_SHORT);
    else if (eTare == BTN_SHORT || eTare == BTN_LONG || eTare == BTN_REPEAT)
      nudgeSetting(-1, eTare != BTN_SHORT);
    else if (eSub == BTN_SHORT || eMode == BTN_LONG)
      showMessage("Settings stored", "", 700, UI_RUN);
    return;
  }

  if (eMode == BTN_LONG) {
    uiState = UI_SETTINGS;
    setPage = SET_A;
    ledsOff();
    return;
  }
  if (eMode == BTN_SHORT) {
    refreshView();
    setMode((viewOk && view.mode == BRIQ_PREPACK) ? BRIQ_TRADE : BRIQ_PREPACK);
    return;
  }
  if (eTare == BTN_SHORT || eTare == BTN_LONG) {
    refreshView();
    if (viewOk && view.mode == BRIQ_PREPACK) capturePackageTare();
    else startTare();
    return;
  }
  if (eSave == BTN_LONG) {
    refreshView();
    if (viewOk && view.mode == BRIQ_PREPACK) {
      sampleCount = 0;
      briqClearSamples();
      verdictHeld = false;
      ledsOff();
      showMessage("Batch cleared", "", 900);
    } else {
      char b[21];
      snprintf(b, sizeof(b), "Records %u/50", viewOk ? view.tradeCount : 0);
      showMessage("Trade log", b, 1200);
    }
    return;
  }
  if (eSave == BTN_SHORT) {
    saveAction();
    return;
  }
  if (eSub == BTN_LONG) {
    refreshView();
    if (viewOk && view.mode == BRIQ_PREPACK) submitPack();
    else {
      char b[21];
      snprintf(b, sizeof(b), "Records %u/50", viewOk ? view.tradeCount : 0);
      showMessage("Trade log", b, 1200);
    }
    return;
  }
  if (eSub == BTN_SHORT) {
    refreshView();
    if (viewOk && view.mode == BRIQ_TRADE) submitTrade();
    else submitPack();
  }
}

static void publishBlynk() {
  if (!Blynk.connected()) return;
  refreshView();
  float shown = liveG;
  if (viewOk && view.mode == BRIQ_PREPACK) shown = liveG - view.packageTareG;
  blynkWriteNumber(VP_NET_G, shown, 2);
  if (viewOk) blynkWriteText(VP_META, view.mode == BRIQ_TRADE ? view.shop : view.product);
  blynkWriteText(VP_VERDICT, verdict);
}

static void copyBlynkText(const char* s, void (*setter)(const char*)) {
  String held(s ? s : "");
  setter(held.c_str());
}

BLYNK_CONNECTED() {
  Blynk.syncVirtual(VP_INSPECTOR, VP_QN, VP_LOT, VP_PRODUCT, VP_ADDRESS, VP_MAKER,
                    VP_PKG_TARE, VP_DENSITY, VP_SHOP, VP_LOCATION, VP_NOMINAL, VP_MODE,
                    VP_PHONE, VP_BATCH);
  blynkWriteText(VP_CLASS, "3");
}

BLYNK_WRITE(V0) { copyBlynkText(param.asStr(), briqSetInspector); }
BLYNK_WRITE(V1) {
  const char* s = param.asStr();
  if (s && s[0]) applyDeclaredQuantity(s);
}
BLYNK_WRITE(V2) {
  float n = 0.0f;
  if (!parseNumberText(param.asStr(), &n) || n < 1.0f) return;
  if (n > 1000000.0f) n = 1000000.0f;
  briqSetLot((uint32_t)(n + 0.5f));
}
BLYNK_WRITE(V3) { copyBlynkText(param.asStr(), briqSetProduct); }
BLYNK_WRITE(V4) { copyBlynkText(param.asStr(), briqSetAddress); }
BLYNK_WRITE(V5) { copyBlynkText(param.asStr(), briqSetManufacturer); }
BLYNK_WRITE(V19) { copyBlynkText(param.asStr(), briqSetPhone); }
BLYNK_WRITE(V20) { copyBlynkText(param.asStr(), briqSetBatch); }
BLYNK_WRITE(V6) {
  float t = 0.0f;
  if (!parseNumberText(param.asStr(), &t)) return;
  briqSetPackageTare(t);
}
BLYNK_WRITE(V7) {
  float d = 0.0f;
  if (!parseNumberText(param.asStr(), &d) || d <= 0.0f) return;
  briqSetDensity(d);
}
BLYNK_WRITE(V8) { copyBlynkText(param.asStr(), briqSetShop); }
BLYNK_WRITE(V9) { copyBlynkText(param.asStr(), briqSetLocation); }
BLYNK_WRITE(V10) {
  float g = 0.0f;
  if (!parseNumberText(param.asStr(), &g)) return;
  briqSetNominal(g);
}
BLYNK_WRITE(V11) {
  const char* s = param.asStr();
  if (!s || !s[0]) return;
  if (!sameWord(s, "3") && !sameWord(s, "m3")) blynkWriteText(VP_CLASS, "3");
}
BLYNK_WRITE(V14) {
  BriqMode m;
  if (!parseAppMode(param.asStr(), &m)) return;
  refreshView();
  if (viewOk && view.mode == m) return;
  setMode(m);
}
BLYNK_WRITE(V21) {
  if (!appPressed(param.asStr()) || !appControlsReady()) return;
  refreshView();
  if (viewOk && view.mode == BRIQ_PREPACK) capturePackageTare();
  else startTare();
}
BLYNK_WRITE(V22) {
  if (!appPressed(param.asStr()) || !appControlsReady()) return;
  saveAction();
}
BLYNK_WRITE(V23) {
  if (!appPressed(param.asStr()) || !appControlsReady()) return;
  refreshView();
  if (viewOk && view.mode == BRIQ_TRADE) submitTrade();
  else submitPack();
}

static void handleCertificate(AsyncWebServerRequest* request) {
  AsyncResponseStream* resp = request->beginResponseStream("text/html");
  resp->addHeader("Cache-Control", "no-store");
  briqWriteCertificate(*resp);
  request->send(resp);
}

static void handleVerify(AsyncWebServerRequest* request) {
  if (!request->hasParam("id", false)) {
    request->send(400, "text/html",
                  "<!DOCTYPE html><html><body><h1>Missing id</h1>"
                  "<p><a href=\"/\">Certificate</a></p></body></html>");
    return;
  }
  String raw = request->getParam("id", false)->value();
  char* end = nullptr;
  unsigned long id = strtoul(raw.c_str(), &end, 10);
  if (end == raw.c_str() || *end != '\0' || id == 0) {
    request->send(400, "text/html",
                  "<!DOCTYPE html><html><body><h1>Bad id</h1>"
                  "<p><a href=\"/\">Certificate</a></p></body></html>");
    return;
  }
  if (!briqContains((uint32_t)id)) {
    request->send(404, "text/html", NOT_FOUND_PAGE);
    return;
  }
  AsyncResponseStream* resp = request->beginResponseStream("text/html");
  resp->addHeader("Cache-Control", "no-store");
  briqWriteVerify(*resp, (uint32_t)id);
  request->send(resp);
}

static void serviceNet() {
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiBegan != 0 && (uint32_t)(millis() - wifiBegan) > 20000) {
      snprintf(ipLine, sizeof(ipLine), "WiFi not joined");
    }
    return;
  }
  Blynk.run();
  if (!timeReady) {
    configTime(TZ_OFFSET_SEC, 0, "pool.ntp.org", "time.nist.gov");
    timeReady = true;
  }
  if (!mdnsReady) {
    if (MDNS.begin("smartscale")) mdnsReady = true;
  }
  static uint32_t lastIpPoll = 0;
  if ((uint32_t)(millis() - lastIpPoll) < 2000) return;
  lastIpPoll = millis();
  String ip = WiFi.localIP().toString();
  if (ip != lastIp) {
    strncpy(lastIp, ip.c_str(), sizeof(lastIp) - 1);
    lastIp[sizeof(lastIp) - 1] = 0;
    briqSetHost(lastIp);
    snprintf(ipLine, sizeof(ipLine), "IP %s", lastIp);
    Serial.printf("Certificate http://%s/\n", lastIp);
  }
}

static void serviceDisplay() {
  uint32_t now = millis();
  if ((uint32_t)(now - lastDisplay) < DISPLAY_PERIOD_MS) return;
  lastDisplay = now;
  if (uiState == UI_MESSAGE) return;
  if (uiState == UI_RUN) {
    refreshView();
    drawRun();
  } else if (uiState == UI_SETTINGS) {
    drawSettings();
  } else if (uiState == UI_CALIBRATE) {
    drawCalibrate();
  }
}

static void loadCal() {
  Preferences prefs;
  prefs.begin(NVS_NS, true);
  calFactor = prefs.getFloat("cal", DEFAULT_CAL_FACTOR);
  prefs.end();
  if (!isfinite(calFactor) || fabsf(calFactor) < 0.001f) calFactor = DEFAULT_CAL_FACTOR;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED_PASS, OUTPUT);
  pinMode(PIN_LED_FAIL, OUTPUT);
  ledsOff();
  pinMode(PIN_BTN_MODE, INPUT_PULLUP);
  pinMode(PIN_BTN_TARE, INPUT_PULLUP);
  pinMode(PIN_BTN_SAVE, INPUT_PULLUP);
  pinMode(PIN_BTN_SUBMIT, INPUT_PULLUP);
  delay(30);

  bool factory = digitalRead(PIN_BTN_MODE) == LOW;
  bool calboot = digitalRead(PIN_BTN_TARE) == LOW;

  Wire.begin(PIN_SDA, PIN_SCL);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); lcdPad("Dual Smart Scale");
  lcd.setCursor(0, 1); lcdPad("R87/R79   R111 M3");
  lcd.setCursor(0, 2); lcdPad("20kg ESP32");
  lcd.setCursor(0, 3); lcdPad("Starting...");

  if (factory) {
    Preferences prefs;
    prefs.begin(NVS_NS, false);
    prefs.clear();
    prefs.end();
    lcd.setCursor(0, 3); lcdPad("Factory defaults");
    delay(400);
  }

  loadCal();
  bool healthy = briqBegin();
  sampleCount = briqExportNets(netContents, MAX_BATCH_N);
  refreshView();

  scale.begin(PIN_HX711_DT, PIN_HX711_SCK);
  uint32_t t0 = millis();
  while (!scale.is_ready() && (uint32_t)(millis() - t0) < 2000) delay(20);
  scale.set_scale(calFactor);
  if (scale.is_ready()) {
    scaleSeen = true;
    scale.tare(20);
    resetHistory();
  }

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setHostname("smartscale");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  wifiBegan = millis();
  Blynk.config(BLYNK_AUTH_TOKEN);
  blynkTimer.setInterval(1000L, publishBlynk);

  server.on("/", HTTP_GET, handleCertificate);
  server.on("/verify", HTTP_GET, handleVerify);
  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest* r) { r->send(204); });
  server.onNotFound([](AsyncWebServerRequest* r) {
    r->send(404, "text/plain", "Not found");
  });
  server.begin();

  if (!healthy) {
    showMessage("Metrology error", "See serial log", 2500, calboot ? UI_CALIBRATE : UI_RUN);
  } else if (!scale.is_ready()) {
    showMessage("HX711 not found", "Check DOUT/CLK", 2000, calboot ? UI_CALIBRATE : UI_RUN);
  } else if (calboot) {
    uiState = UI_CALIBRATE;
    lcd.clear();
    lcd.setCursor(0, 0); lcdPad("CALIBRATION  20kg");
    lcd.setCursor(0, 1); lcdPad("Put known mass on");
    lcd.setCursor(0, 2); lcdPad("SAVE=mass SUB=set");
  } else {
    lcd.clear();
  }

  consumeHeld(btnMode);
  consumeHeld(btnTare);
  consumeHeld(btnSave);
  consumeHeld(btnSubmit);
  Serial.println(F("Smart Scale ready (R87/R79 + R111 M3 + Blynk)"));
}

void loop() {
  blynkTimer.run();
  serviceNet();
  pollScale();
  handleUi();
  if (ledUntil && (int32_t)(millis() - ledUntil) >= 0) {
    ledsOff();
    ledUntil = 0;
  }
  serviceDisplay();
  if (pendingWhatsAppReport && uiState == UI_RUN && !tareRun && !calAvg) {
    bool sent = sendBriqWhatsAppReport();
    pendingWhatsAppReport = false;
    showMessage(sent ? "WhatsApp sent" : "WhatsApp not sent", "", 1000);
  }
}
