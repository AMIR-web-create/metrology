#include "briq.h"
#include "blynk_secrets.h"

#include <ArduinoJson.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <freertos/semphr.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// ---------------------------------------------------------------------------
// Trade weights: OIML R111 nominal series and the Class M3 MPE table (grams)
// given for this instrument.
//   50, 100, 200, 500, 1000, 2000, 5000, 10000, 15000, 20000 g
//   0.03, 0.05, 0.10, 0.25, 0.50, 1.00, 2.50, 5.00, 7.50, 10.00 g
// Values between the points are linearly interpolated. Error = measured - nominal.
// Pass when |error| <= MPE.
// ---------------------------------------------------------------------------

static const float NOM_G[] = {
  50.0f, 100.0f, 200.0f, 500.0f, 1000.0f,
  2000.0f, 5000.0f, 10000.0f, 15000.0f, 20000.0f
};
static const float MPE_G[] = {
  0.03f, 0.05f, 0.10f, 0.25f, 0.50f,
  1.00f, 2.50f, 5.00f, 7.50f, 10.00f
};
static const int NOM_N = 10;

// Pre-package rules used by this instrument:
// deficit = declaredGrams - net.
//   deficit <= 0            pass
//   0 < deficit <= T        T1
//   deficit > T             T2
// SCF k: n<=50 -> 0.379, n<=80 -> 0.295, otherwise 0.234.
// T1 allowance: n<=50 -> 3, n<=80 -> 5, otherwise 7.
// correctedMean = mean - k*s, and the lot passes only when that is >= declared.

static const uint8_t TRADE_MAX = 50;
static const uint8_t LOT_MAX = 8;
static const uint8_t LOG_STR = 24;
static const uint8_t STORE_VER = 1;

struct TradeRecord {
  uint32_t id;
  uint32_t unixTime;
  float nominalG;
  float measuredG;
  float errorG;
  float mpeG;
  uint8_t pass;
  uint8_t pad[3];
  char inspector[LOG_STR];
  char shop[LOG_STR];
  char location[LOG_STR];
};

struct LotSeal {
  uint32_t id;
  uint32_t unixTime;
  uint32_t lotN;
  uint8_t complete;
  uint8_t quantityPass;
  uint8_t t1Pass;
  uint8_t t2Pass;
  uint8_t meanPass;
  uint8_t r79Pass;
  uint16_t n;
  uint16_t nPlanned;
  uint16_t nT1;
  uint16_t nT2;
  uint16_t t1Allowed;
  float qnG;
  float tG;
  float t2G;
  float meanG;
  float stdevG;
  float correctedG;
  float kFactor;
  char inspector[LOG_STR];
  char product[LOG_STR];
  char maker[LOG_STR];
  char address[LOG_STR];
};

static_assert(sizeof(TradeRecord) * TRADE_MAX <= 8192, "trade log too large for NVS");
static_assert(sizeof(LotSeal) * LOT_MAX <= 4096, "lot log too large for NVS");
static_assert(TRADE_MAX % 2 == 0, "trade log is stored in two blobs");

static SemaphoreHandle_t dataMux = nullptr;
static SemaphoreHandle_t pageMux = nullptr;
static bool healthy = false;

static BriqMode mode = BRIQ_PREPACK;
static float declaredQn = DEFAULT_QN_G;
static float density = DEFAULT_DENSITY;
static float packageTare = DEFAULT_TARE_G;
static float nominalG = DEFAULT_NOMINAL_G;
static uint32_t lotN = DEFAULT_LOT_N;

static char inspector[STR_LEN];
static char product[STR_LEN];
static char address[STR_LEN];
static char manufacturer[STR_LEN];
static char shop[STR_LEN];
static char location[STR_LEN];
static char batchNo[STR_LEN];
static char phoneNo[STR_LEN];
static char hostIp[40];
static char waPhone[STR_LEN];
static char waText[360];

static TradeRecord trade[TRADE_MAX];
static uint8_t tradeHead = 0;
static uint8_t tradeCount = 0;
static TradeCheck lastTrade = {};

static LotSeal lots[LOT_MAX];
static uint8_t lotHead = 0;
static uint8_t lotCount = 0;

static float samples[MAX_BATCH_N];
static uint16_t sampleFill = 0;
static uint32_t samplesOwnerId = 0;
static uint32_t nextId = 1;

static BriqView snapView;
static TradeRecord snapTrade[TRADE_MAX];
static uint8_t snapTradeCount = 0;
static LotSeal snapLots[LOT_MAX];
static uint8_t snapLotCount = 0;
static float snapSamples[MAX_BATCH_N];
static uint16_t snapSampleN = 0;
static uint32_t snapOwner = 0;
static PrepackResult snapResult;
static char snapHost[40];

static bool lockData() {
  if (!dataMux) return true;
  return xSemaphoreTake(dataMux, portMAX_DELAY) == pdTRUE;
}

static bool tryData() {
  if (!dataMux) return true;
  return xSemaphoreTake(dataMux, pdMS_TO_TICKS(20)) == pdTRUE;
}

static void unlockData() {
  if (dataMux) xSemaphoreGive(dataMux);
}

static void lockPage() {
  if (pageMux) xSemaphoreTake(pageMux, portMAX_DELAY);
}

static void unlockPage() {
  if (pageMux) xSemaphoreGive(pageMux);
}

static void copyField(char* dst, size_t n, const char* src) {
  size_t j = 0;
  if (!src) {
    if (n) dst[0] = 0;
    return;
  }
  for (size_t i = 0; src[i] && j + 1 < n; i++) {
    unsigned char c = (unsigned char)src[i];
    dst[j++] = (c < 32) ? ' ' : (char)c;
  }
  dst[j] = 0;
}

static void copyShort(char* dst, const char* src) {
  copyField(dst, LOG_STR, src);
}

static uint32_t nowUnix() {
  time_t t = time(nullptr);
  if (t < 1700000000) return 0;
  return (uint32_t)t;
}

static long toMg(float grams) {
  return lroundf(grams * 1000.0f);
}

static float qnMassUnlocked() {
  if (density >= 0.05f) return declaredQn * density;
  return declaredQn;
}

static uint16_t plannedUnlocked() {
  uint16_t n = briqSampleSize(lotN);
  if (n > MAX_BATCH_N) n = MAX_BATCH_N;
  return n;
}

static bool r79Unlocked() {
  return product[0] && manufacturer[0] && address[0] && qnMassUnlocked() > 0.1f;
}

float briqMpeG(float nominal) {
  if (nominal <= NOM_G[0]) return MPE_G[0];
  if (nominal >= NOM_G[NOM_N - 1]) {
    return MPE_G[NOM_N - 1] * (nominal / NOM_G[NOM_N - 1]);
  }
  for (int i = 1; i < NOM_N; i++) {
    if (nominal <= NOM_G[i]) {
      float span = NOM_G[i] - NOM_G[i - 1];
      float t = (span <= 0.0f) ? 0.0f : (nominal - NOM_G[i - 1]) / span;
      return MPE_G[i - 1] + t * (MPE_G[i] - MPE_G[i - 1]);
    }
  }
  return MPE_G[NOM_N - 1];
}

float briqTValue(float qnG) {
  if (qnG <= 50.0f) return qnG * 0.09f;
  if (qnG <= 100.0f) return 4.5f;
  if (qnG <= 200.0f) return qnG * 0.045f;
  if (qnG <= 300.0f) return 9.0f;
  if (qnG <= 500.0f) return qnG * 0.03f;
  if (qnG <= 1000.0f) return 15.0f;
  return qnG * 0.015f;
}

uint16_t briqSampleSize(uint32_t lot) {
  if (lot < 500) return 50;
  if (lot <= 3200) return 80;
  return 125;
}

static uint16_t t1Allowance(uint16_t n) {
  if (n <= 50) return 3;
  if (n <= 80) return 5;
  return 7;
}

static float kFactor(uint16_t n) {
  if (n <= 50) return 0.379f;
  if (n <= 80) return 0.295f;
  return 0.234f;
}

static int tradePhysical(uint8_t logical) {
  uint8_t start = (tradeCount == TRADE_MAX) ? tradeHead : 0;
  return (start + logical) % TRADE_MAX;
}

static int lotPhysical(uint8_t logical) {
  uint8_t start = (lotCount == LOT_MAX) ? lotHead : 0;
  return (start + logical) % LOT_MAX;
}

static bool mpeAccept(float measured, float nominal, float* errOut, float* mpeOut) {
  float mpe = briqMpeG(nominal);
  float err = measured - nominal;
  bool pass = labs(toMg(err)) <= toMg(mpe);
  if (errOut) *errOut = err;
  if (mpeOut) *mpeOut = mpe;
  return pass;
}

static PrepackResult evaluateSamples(const float* x, uint16_t n, uint16_t planned,
                                     float qn, uint32_t lot) {
  PrepackResult r = {};
  r.n = n;
  r.nPlanned = planned;
  r.qnG = qn;
  r.tG = briqTValue(qn);
  r.t2G = r.tG;
  r.complete = (planned > 0 && n >= planned);
  (void)lot;
  if (n == 0) return r;

  double sum = 0.0;
  for (uint16_t i = 0; i < n; i++) sum += x[i];
  double mean = sum / (double)n;

  double acc = 0.0;
  for (uint16_t i = 0; i < n; i++) {
    double d = (double)x[i] - mean;
    acc += d * d;
  }
  double s = (n >= 2) ? sqrt(acc / (double)(n - 1)) : 0.0;
  float k = kFactor(n);
  double corrected = mean - (double)k * s;

  long qnMg = toMg(qn);
  long tMg = toMg(r.tG);
  uint16_t nT1 = 0;
  uint16_t nT2 = 0;
  for (uint16_t i = 0; i < n; i++) {
    long deficit = qnMg - toMg(x[i]);
    if (deficit > tMg) nT2++;
    else if (deficit > 0) nT1++;
  }

  r.nT1 = nT1;
  r.nT2 = nT2;
  r.t1Allowed = t1Allowance(n);
  r.meanG = (float)mean;
  r.stdevG = (float)s;
  r.kFactor = k;
  r.correctedMeanG = (float)corrected;
  r.t2Pass = (nT2 == 0);
  r.t1Pass = (nT1 <= r.t1Allowed);
  r.meanPass = (toMg((float)corrected) >= toMg(qn));
  r.quantityPass = r.complete && r.t1Pass && r.t2Pass && r.meanPass;
  return r;
}

static PrepackResult evaluateUnlocked() {
  PrepackResult r = evaluateSamples(samples, sampleFill, plannedUnlocked(),
                                    qnMassUnlocked(), lotN);
  r.r79Pass = r79Unlocked();
  if (r.complete && samplesOwnerId != 0) {
    for (uint8_t i = 0; i < lotCount; i++) {
      const LotSeal& s = lots[lotPhysical(i)];
      if (s.id == samplesOwnerId && s.n == sampleFill) {
        r.id = s.id;
        break;
      }
    }
  }
  return r;
}

static void fillView(BriqView& v) {
  memset(&v, 0, sizeof(v));
  v.mode = mode;
  v.qnDeclared = declaredQn;
  v.qnMassG = qnMassUnlocked();
  v.density = density;
  v.packageTareG = packageTare;
  v.nominalG = nominalG;
  v.mpeG = briqMpeG(nominalG);
  v.tG = briqTValue(v.qnMassG);
  v.lotN = lotN;
  v.plannedN = plannedUnlocked();
  v.sampleFill = sampleFill;
  v.tradeCount = tradeCount;
  v.r79Ok = r79Unlocked();
  copyField(v.inspector, STR_LEN, inspector);
  copyField(v.product, STR_LEN, product);
  copyField(v.address, STR_LEN, address);
  copyField(v.manufacturer, STR_LEN, manufacturer);
  copyField(v.shop, STR_LEN, shop);
  copyField(v.location, STR_LEN, location);
  copyField(v.batch, STR_LEN, batchNo);
  copyField(v.phone, STR_LEN, phoneNo);
}

static void saveMeta() {
  Preferences prefs;
  if (!prefs.begin(NVS_NS, false)) return;
  prefs.putUChar("bver", STORE_VER);
  prefs.putUChar("mode", (uint8_t)mode);
  prefs.putFloat("qn", declaredQn);
  prefs.putFloat("den", density);
  prefs.putFloat("ptare", packageTare);
  prefs.putFloat("nom", nominalG);
  prefs.putUInt("lot", lotN);
  prefs.putUInt("nextid", nextId);
  prefs.putString("insp", inspector);
  prefs.putString("prod", product);
  prefs.putString("addr", address);
  prefs.putString("makr", manufacturer);
  prefs.putString("shop", shop);
  prefs.putString("loc", location);
  prefs.putString("batch", batchNo);
  prefs.putString("phone", phoneNo);
  prefs.end();
}

static void saveTrade() {
  Preferences prefs;
  if (!prefs.begin(NVS_NS, false)) return;
  const size_t half = sizeof(TradeRecord) * (TRADE_MAX / 2);
  prefs.putUChar("bver", STORE_VER);
  prefs.putUChar("thd", tradeHead);
  prefs.putUChar("tcn", tradeCount);
  prefs.putUInt("nextid", nextId);
  prefs.putBytes("tl0", trade, half);
  prefs.putBytes("tl1", trade + (TRADE_MAX / 2), half);
  prefs.end();
}

static void saveSession() {
  Preferences prefs;
  if (!prefs.begin(NVS_NS, false)) return;
  prefs.putUChar("bver", STORE_VER);
  prefs.putUChar("lhd", lotHead);
  prefs.putUChar("lcn", lotCount);
  prefs.putBytes("lots", lots, sizeof(lots));
  prefs.putUShort("pfill", sampleFill);
  prefs.putUInt("pown", samplesOwnerId);
  prefs.putUInt("nextid", nextId);
  if (sampleFill > 0) prefs.putBytes("psamp", samples, sizeof(float) * sampleFill);
  else prefs.remove("psamp");
  prefs.end();
}

static void loadStore() {
  Preferences prefs;
  if (!prefs.begin(NVS_NS, true)) return;
  uint8_t ver = prefs.getUChar("bver", 0);
  mode = (prefs.getUChar("mode", BRIQ_PREPACK) == BRIQ_TRADE) ? BRIQ_TRADE : BRIQ_PREPACK;
  declaredQn = prefs.getFloat("qn", DEFAULT_QN_G);
  density = prefs.getFloat("den", DEFAULT_DENSITY);
  packageTare = prefs.getFloat("ptare", DEFAULT_TARE_G);
  nominalG = prefs.getFloat("nom", DEFAULT_NOMINAL_G);
  lotN = prefs.getUInt("lot", DEFAULT_LOT_N);
  nextId = prefs.getUInt("nextid", 1);
  if (nextId == 0) nextId = 1;
  copyField(inspector, STR_LEN, prefs.getString("insp", "").c_str());
  copyField(product, STR_LEN, prefs.getString("prod", "").c_str());
  copyField(address, STR_LEN, prefs.getString("addr", "").c_str());
  copyField(manufacturer, STR_LEN, prefs.getString("makr", "").c_str());
  copyField(shop, STR_LEN, prefs.getString("shop", "").c_str());
  copyField(location, STR_LEN, prefs.getString("loc", "").c_str());
  copyField(batchNo, STR_LEN, prefs.getString("batch", "").c_str());
  copyField(phoneNo, STR_LEN, prefs.getString("phone", "").c_str());

  if (ver == STORE_VER) {
    const size_t half = sizeof(TradeRecord) * (TRADE_MAX / 2);
    tradeHead = prefs.getUChar("thd", 0);
    tradeCount = prefs.getUChar("tcn", 0);
    if (tradeCount > TRADE_MAX) tradeCount = 0;
    if (tradeHead >= TRADE_MAX) tradeHead = 0;
    if (prefs.getBytesLength("tl0") == half && prefs.getBytesLength("tl1") == half) {
      prefs.getBytes("tl0", trade, half);
      prefs.getBytes("tl1", trade + (TRADE_MAX / 2), half);
    } else {
      tradeCount = 0;
      tradeHead = 0;
    }
    if (prefs.getBytesLength("lots") == sizeof(lots)) {
      lotHead = prefs.getUChar("lhd", 0);
      lotCount = prefs.getUChar("lcn", 0);
      if (lotCount > LOT_MAX || lotHead >= LOT_MAX) {
        lotCount = 0;
        lotHead = 0;
      } else {
        prefs.getBytes("lots", lots, sizeof(lots));
      }
    }
    sampleFill = prefs.getUShort("pfill", 0);
    samplesOwnerId = prefs.getUInt("pown", 0);
    if (sampleFill > MAX_BATCH_N) sampleFill = 0;
    if (sampleFill > 0 && prefs.getBytesLength("psamp") >= sizeof(float) * sampleFill) {
      prefs.getBytes("psamp", samples, sizeof(float) * sampleFill);
    } else {
      sampleFill = 0;
      samplesOwnerId = 0;
    }
  }
  prefs.end();

  if (!isfinite(declaredQn) || declaredQn < 0.0f) declaredQn = DEFAULT_QN_G;
  if (!isfinite(density) || density < 0.0f) density = 0.0f;
  if (!isfinite(packageTare) || packageTare < 0.0f) packageTare = 0.0f;
  if (!isfinite(nominalG) || nominalG < 0.1f) nominalG = DEFAULT_NOMINAL_G;
  if (lotN < 1) lotN = 1;
  uint16_t planned = plannedUnlocked();
  if (sampleFill > planned) {
    sampleFill = 0;
    samplesOwnerId = 0;
  }
  if (tradeCount > 0) {
    const TradeRecord& rec = trade[(tradeHead + TRADE_MAX - 1) % TRADE_MAX];
    lastTrade.valid = true;
    lastTrade.logged = true;
    lastTrade.pass = rec.pass != 0;
    lastTrade.id = rec.id;
    lastTrade.nominalG = rec.nominalG;
    lastTrade.measuredG = rec.measuredG;
    lastTrade.errorG = rec.errorG;
    lastTrade.mpeG = rec.mpeG;
  }
}

static void clearSamplesUnlocked() {
  sampleFill = 0;
  samplesOwnerId = 0;
}

static uint32_t takeId() {
  uint32_t id = nextId++;
  if (nextId == 0) nextId = 1;
  return id;
}

static void pushTrade(const TradeRecord& rec) {
  trade[tradeHead] = rec;
  tradeHead = (tradeHead + 1) % TRADE_MAX;
  if (tradeCount < TRADE_MAX) tradeCount++;
}

static void pushLot(const LotSeal& rec) {
  lots[lotHead] = rec;
  lotHead = (lotHead + 1) % LOT_MAX;
  if (lotCount < LOT_MAX) lotCount++;
}

static LotSeal makeSeal(const PrepackResult& r) {
  LotSeal s = {};
  s.id = r.id;
  s.unixTime = nowUnix();
  s.lotN = lotN;
  s.complete = r.complete ? 1 : 0;
  s.quantityPass = r.quantityPass ? 1 : 0;
  s.t1Pass = r.t1Pass ? 1 : 0;
  s.t2Pass = r.t2Pass ? 1 : 0;
  s.meanPass = r.meanPass ? 1 : 0;
  s.r79Pass = r.r79Pass ? 1 : 0;
  s.n = r.n;
  s.nPlanned = r.nPlanned;
  s.nT1 = r.nT1;
  s.nT2 = r.nT2;
  s.t1Allowed = r.t1Allowed;
  s.qnG = r.qnG;
  s.tG = r.tG;
  s.t2G = r.t2G;
  s.meanG = r.meanG;
  s.stdevG = r.stdevG;
  s.correctedG = r.correctedMeanG;
  s.kFactor = r.kFactor;
  copyShort(s.inspector, inspector);
  copyShort(s.product, product);
  copyShort(s.maker, manufacturer);
  copyShort(s.address, address);
  return s;
}

static PrepackResult sealIfComplete() {
  PrepackResult r = evaluateUnlocked();
  if (!r.complete || r.id != 0) return r;
  r.id = takeId();
  r.r79Pass = r79Unlocked();
  LotSeal s = makeSeal(r);
  pushLot(s);
  samplesOwnerId = r.id;
  saveSession();
  return r;
}

static UnitMark markNet(float net, float qn) {
  long deficit = toMg(qn) - toMg(net);
  long tMg = toMg(briqTValue(qn));
  if (deficit <= 0) return UNIT_OK;
  if (deficit <= tMg) return UNIT_T1;
  return UNIT_T2;
}

static const char* selfCheck() {
  if (fabsf(briqMpeG(50.0f) - 0.03f) > 1e-4f) return "mpe50";
  if (fabsf(briqMpeG(100.0f) - 0.05f) > 1e-4f) return "mpe100";
  if (fabsf(briqMpeG(200.0f) - 0.10f) > 1e-4f) return "mpe200";
  if (fabsf(briqMpeG(500.0f) - 0.25f) > 1e-4f) return "mpe500";
  if (fabsf(briqMpeG(1000.0f) - 0.50f) > 1e-4f) return "mpe1000";
  if (fabsf(briqMpeG(2000.0f) - 1.00f) > 1e-4f) return "mpe2000";
  if (fabsf(briqMpeG(5000.0f) - 2.50f) > 1e-4f) return "mpe5000";
  if (fabsf(briqMpeG(10000.0f) - 5.00f) > 1e-4f) return "mpe10000";
  if (fabsf(briqMpeG(15000.0f) - 7.50f) > 1e-4f) return "mpe15000";
  if (fabsf(briqMpeG(20000.0f) - 10.00f) > 1e-4f) return "mpe20000";
  if (fabsf(briqMpeG(1500.0f) - 0.75f) > 1e-3f) return "mpeInterp";

  if (fabsf(briqTValue(50.0f) - 4.5f) > 1e-3f) return "t50";
  if (fabsf(briqTValue(100.0f) - 4.5f) > 1e-3f) return "t100";
  if (fabsf(briqTValue(200.0f) - 9.0f) > 1e-3f) return "t200";
  if (fabsf(briqTValue(300.0f) - 9.0f) > 1e-3f) return "t300";
  if (fabsf(briqTValue(500.0f) - 15.0f) > 1e-3f) return "t500";
  if (fabsf(briqTValue(1000.0f) - 15.0f) > 1e-3f) return "t1000";
  if (fabsf(briqTValue(2000.0f) - 30.0f) > 1e-2f) return "t2000";

  if (briqSampleSize(10) != 50) return "n10";
  if (briqSampleSize(499) != 50) return "n499";
  if (briqSampleSize(500) != 80) return "n500";
  if (briqSampleSize(3200) != 80) return "n3200";
  if (briqSampleSize(3201) != 125) return "n3201";

  if (!mpeAccept(1000.20f, 1000.0f, nullptr, nullptr)) return "passEdge";
  if (mpeAccept(1001.00f, 1000.0f, nullptr, nullptr)) return "failEdge";
  if (!mpeAccept(50.02f, 50.0f, nullptr, nullptr)) return "pass50";
  if (mpeAccept(50.05f, 50.0f, nullptr, nullptr)) return "fail50";

  float x[50];
  for (int i = 0; i < 50; i++) x[i] = 500.0f;
  PrepackResult ok = evaluateSamples(x, 50, 50, 500.0f, 5000);
  if (!ok.complete || !ok.t1Pass || !ok.t2Pass || !ok.meanPass || !ok.quantityPass) return "lotOk";
  if (fabsf(ok.kFactor - 0.379f) > 1e-4f) return "k50";

  x[0] = 469.0f;
  PrepackResult badT2 = evaluateSamples(x, 50, 50, 500.0f, 5000);
  if (badT2.t2Pass || badT2.quantityPass) return "t2";

  for (int i = 0; i < 50; i++) x[i] = 500.0f;
  x[0] = x[1] = x[2] = 490.0f;
  PrepackResult t1ok = evaluateSamples(x, 50, 50, 500.0f, 5000);
  if (!t1ok.t1Pass || t1ok.nT1 != 3 || !t1ok.t2Pass) return "t1allow";
  x[3] = 490.0f;
  PrepackResult t1bad = evaluateSamples(x, 50, 50, 500.0f, 5000);
  if (t1bad.t1Pass || t1bad.nT1 != 4) return "t1over";

  for (int i = 0; i < 50; i++) x[i] = 499.0f;
  PrepackResult low = evaluateSamples(x, 50, 50, 500.0f, 5000);
  if (low.meanPass || low.quantityPass) return "meanLow";

  float d = 4.0f * sqrtf(49.0f / 50.0f);
  for (int i = 0; i < 25; i++) x[i] = 499.0f + d;
  for (int i = 25; i < 50; i++) x[i] = 499.0f - d;
  PrepackResult spread = evaluateSamples(x, 50, 50, 500.0f, 5000);
  if (spread.meanPass || spread.correctedMeanG >= 500.0f) return "corrected";

  float y[40];
  for (int i = 0; i < 40; i++) y[i] = 499.5f;
  PrepackResult census = evaluateSamples(y, 40, 40, 500.0f, 40);
  if (fabsf(census.kFactor - 0.379f) > 1e-4f || census.meanPass) return "census";

  PrepackResult partial = evaluateSamples(x, 10, 50, 500.0f, 5000);
  if (partial.complete || partial.quantityPass) return "partial";
  return nullptr;
}

bool briqBegin() {
  dataMux = xSemaphoreCreateMutex();
  pageMux = xSemaphoreCreateMutex();
  loadStore();
  const char* err = selfCheck();
  healthy = (err == nullptr);
  if (err) Serial.printf("briq self-check failed: %s\n", err);
  else Serial.println("briq self-check ok");
  return healthy;
}

bool briqHealthy() { return healthy; }

static void setText(char* dst, const char* src) {
  if (!lockData()) return;
  copyField(dst, STR_LEN, src);
  saveMeta();
  unlockData();
}

void briqSetMode(BriqMode m) {
  if (m != BRIQ_PREPACK && m != BRIQ_TRADE) return;
  if (!lockData()) return;
  if (mode != m) {
    mode = m;
    saveMeta();
  }
  unlockData();
}

void briqSetInspector(const char* s) { setText(inspector, s); }
void briqSetProduct(const char* s) { setText(product, s); }
void briqSetAddress(const char* s) { setText(address, s); }
void briqSetManufacturer(const char* s) { setText(manufacturer, s); }
void briqSetShop(const char* s) { setText(shop, s); }
void briqSetLocation(const char* s) { setText(location, s); }
void briqSetBatch(const char* s) { setText(batchNo, s); }
void briqSetPhone(const char* s) { setText(phoneNo, s); }

void briqSetQn(float qn) {
  if (!isfinite(qn) || qn < 0.0f) qn = 0.0f;
  if (qn > 1000000.0f) qn = 1000000.0f;
  if (!lockData()) return;
  if (fabsf(declaredQn - qn) > 0.0001f) {
    declaredQn = qn;
    clearSamplesUnlocked();
    saveMeta();
    saveSession();
  }
  unlockData();
}

void briqSetLot(uint32_t lot) {
  if (lot < 1) lot = 1;
  if (lot > 1000000UL) lot = 1000000UL;
  if (!lockData()) return;
  if (lotN != lot) {
    lotN = lot;
    saveMeta();
  }
  unlockData();
}

void briqSetPackageTare(float grams) {
  if (!isfinite(grams) || grams < 0.0f) grams = 0.0f;
  if (grams > SCALE_CAPACITY_G) grams = SCALE_CAPACITY_G;
  if (!lockData()) return;
  packageTare = grams;
  saveMeta();
  unlockData();
}

void briqSetDensity(float gPerMl) {
  if (!isfinite(gPerMl) || gPerMl < 0.0f) gPerMl = 0.0f;
  if (gPerMl > 20.0f) gPerMl = 20.0f;
  if (!lockData()) return;
  if (fabsf(density - gPerMl) > 0.0001f) {
    density = gPerMl;
    clearSamplesUnlocked();
    saveMeta();
    saveSession();
  }
  unlockData();
}

void briqSetNominal(float grams) {
  if (!isfinite(grams)) grams = DEFAULT_NOMINAL_G;
  if (grams < 0.1f) grams = 0.1f;
  if (grams > SCALE_CAPACITY_G) grams = SCALE_CAPACITY_G;
  if (!lockData()) return;
  nominalG = grams;
  saveMeta();
  unlockData();
}

void briqSetHost(const char* ip) {
  if (!lockData()) return;
  copyField(hostIp, sizeof(hostIp), ip);
  unlockData();
}

bool briqRead(BriqView& out) {
  if (!tryData()) return false;
  fillView(out);
  unlockData();
  return true;
}

bool briqFormatLimits(char* buf, size_t n) {
  if (!buf || n == 0) return false;
  if (!tryData()) return false;
  if (mode == BRIQ_TRADE) {
    snprintf(buf, n, "R111 M3 Nom=%.1fg MPE=+/-%.2fg log=%u/50",
             nominalG, briqMpeG(nominalG), tradeCount);
  } else {
    float qn = qnMassUnlocked();
    float t = briqTValue(qn);
    snprintf(buf, n, "Qn=%.1fg T=%.2f n=%u/%u",
             qn, t, sampleFill, plannedUnlocked());
  }
  unlockData();
  return true;
}

TradeCheck briqLogTrade(float measuredG) {
  TradeCheck out = {};
  if (!isfinite(measuredG)) return out;
  if (!lockData()) return out;
  float err = 0.0f;
  float mpe = 0.0f;
  bool pass = mpeAccept(measuredG, nominalG, &err, &mpe);
  TradeRecord rec = {};
  rec.id = takeId();
  rec.unixTime = nowUnix();
  rec.nominalG = nominalG;
  rec.measuredG = measuredG;
  rec.errorG = err;
  rec.mpeG = mpe;
  rec.pass = pass ? 1 : 0;
  copyShort(rec.inspector, inspector);
  copyShort(rec.shop, shop);
  copyShort(rec.location, location);
  pushTrade(rec);
  saveTrade();
  out.valid = true;
  out.pass = pass;
  out.logged = true;
  out.id = rec.id;
  out.nominalG = nominalG;
  out.measuredG = measuredG;
  out.errorG = err;
  out.mpeG = mpe;
  lastTrade = out;
  unlockData();
  Serial.printf("TRADE id=%lu nom=%.3f meas=%.3f err=%+.3f mpe=%.3f %s\n",
                (unsigned long)out.id, out.nominalG, out.measuredG, out.errorG,
                out.mpeG, out.pass ? "PASS" : "FAIL");
  return out;
}

bool briqLastTrade(TradeCheck& out) {
  if (!tryData()) return false;
  out = lastTrade;
  unlockData();
  return out.valid;
}

void briqClearSamples() {
  if (!lockData()) return;
  clearSamplesUnlocked();
  saveSession();
  unlockData();
}

void briqReplacePackSamples(const float* netG, uint16_t n) {
  if (!lockData()) return;
  if (n > MAX_BATCH_N) n = MAX_BATCH_N;
  clearSamplesUnlocked();
  if (netG && n > 0) {
    memcpy(samples, netG, sizeof(float) * n);
    sampleFill = n;
  }
  saveSession();
  unlockData();
}

uint16_t briqExportNets(float* dst, uint16_t max) {
  if (!dst || max == 0 || !lockData()) return 0;
  uint16_t n = sampleFill;
  if (n > max) n = max;
  if (n > 0) memcpy(dst, samples, sizeof(float) * n);
  unlockData();
  return n;
}

SampleAdd briqAddSample(float netG) {
  SampleAdd add = {};
  if (!isfinite(netG)) return add;
  if (!lockData()) return add;
  uint16_t planned = plannedUnlocked();
  if (sampleFill >= planned || sampleFill >= MAX_BATCH_N) {
    add.lot = evaluateUnlocked();
    unlockData();
    return add;
  }
  if (samplesOwnerId != 0) {
    clearSamplesUnlocked();
  }
  float qn = qnMassUnlocked();
  samples[sampleFill++] = netG;
  add.accepted = true;
  add.mark = markNet(netG, qn);
  add.lot = sealIfComplete();
  add.closed = add.lot.complete && add.lot.id != 0 && add.lot.n == sampleFill;
  if (!add.closed) saveSession();
  unlockData();
  Serial.printf("PACK net=%.2f %s fill=%u/%u\n", netG,
                add.mark == UNIT_T2 ? "T2" : (add.mark == UNIT_T1 ? "T1" : "OK"),
                add.lot.n, add.lot.nPlanned);
  return add;
}

PrepackResult briqCloseLot() {
  PrepackResult r = {};
  if (!lockData()) return r;
  r = evaluateUnlocked();
  unlockData();
  return r;
}

// ---------------------------------------------------------------------------
// Certificate HTML
// ---------------------------------------------------------------------------

static void pf(Print& o, const char* fmt, ...) {
  char buf[220];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  o.print(buf);
}

static void printEsc(Print& o, const char* s) {
  if (!s) return;
  for (; *s; s++) {
    switch (*s) {
      case '&': o.print(F("&amp;")); break;
      case '<': o.print(F("&lt;")); break;
      case '>': o.print(F("&gt;")); break;
      case '"': o.print(F("&quot;")); break;
      default: o.write((uint8_t)*s); break;
    }
  }
}

static void formatWhen(char* buf, size_t n, uint32_t unixTime) {
  if (unixTime < 1700000000UL) {
    snprintf(buf, n, "n/a");
    return;
  }
  time_t t = (time_t)unixTime;
  struct tm tm;
  localtime_r(&t, &tm);
  strftime(buf, n, "%Y-%m-%d %H:%M", &tm);
}

static void instrumentId(char* buf, size_t n) {
  uint64_t mac = ESP.getEfuseMac();
  snprintf(buf, n, "%04X%08X", (unsigned)(mac >> 32), (unsigned)mac);
}

static void writeCss(Print& o) {
  o.print(F("<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"utf-8\">"
            "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"));
  o.print(F("<style>"
            "body{font-family:Georgia,serif;background:#f3efe6;color:#1a1a1a;margin:0;}"
            ".page{max-width:840px;margin:16px auto;background:#fff;padding:28px 32px;border:1px solid #222;}"
            "h1{font-size:22px;text-align:center;margin:0 0 4px;letter-spacing:.04em;}"
            ".sub{text-align:center;margin:0 0 12px;color:#333;}"
            ".banner{text-align:center;font-size:26px;letter-spacing:.14em;margin:14px 0;font-weight:bold;}"
            ".pass{color:#146c2e}.fail{color:#9b1c1c}.wait{color:#8a5a00}"
            "table{border-collapse:collapse;width:100%;font-size:13px;margin:8px 0 16px;}"
            "th,td{border:1px solid #333;padding:4px 6px;text-align:right;}"
            "th.left,td.left{text-align:left;}"
            ".meta td{border:none;padding:2px 8px 2px 0;}"
            ".note{font-size:12px;line-height:1.45;}"
            ".sign{display:flex;justify-content:space-between;margin-top:28px;}"
            ".sign div{width:45%;border-top:1px solid #222;padding-top:6px;font-size:13px;}"
            "button{font:inherit;padding:6px 12px;}"
            "@media print{body{background:#fff}button,.noprint{display:none}"
            ".page{border:none;margin:0;max-width:none}}"
            "</style>"));
}

static void writeTop(Print& o, const char* title) {
  writeCss(o);
  pf(o, "<title>%s</title></head><body><div class=\"page\">", title);
  o.print(F("<p class=\"noprint\" style=\"text-align:right\"><button onclick=\"window.print()\">Print</button></p>"));
  pf(o, "<h1>%s</h1>", title);
  o.print(F("<p class=\"sub\">Dual Smart Scale &mdash; legal metrology worksheet</p>"));
}

static void writeHref(Print& o, const char* host, uint32_t id) {
  o.print(F("<a href=\""));
  if (host && host[0]) {
    o.print(F("http://"));
    printEsc(o, host);
  }
  pf(o, "/verify?id=%lu\">%lu</a>", (unsigned long)id, (unsigned long)id);
}

static void writeFooter(Print& o) {
  char id[20];
  char when[24];
  instrumentId(id, sizeof(id));
  formatWhen(when, sizeof(when), nowUnix());
  o.print(F("<div class=\"sign\"><div>Inspector signature</div><div>Date / stamp</div></div>"));
  pf(o, "<p class=\"note\">Instrument %s", id);
  if (snapHost[0]) {
    o.print(F(" &middot; "));
    printEsc(o, snapHost);
  }
  pf(o, " &middot; %s</p></div></body></html>", when);
}

static void writeField(Print& o, const char* label, const char* val) {
  o.print(F("<tr><td class=\"left\">"));
  printEsc(o, label);
  o.print(F("</td><td class=\"left\">"));
  if (val && val[0]) printEsc(o, val);
  else o.print(F("<span class=\"fail\">missing</span>"));
  o.print(F("</td></tr>"));
}

static void writeTradeTable(Print& o, const TradeRecord* rows, uint8_t count, const char* host) {
  uint16_t passN = 0;
  for (uint8_t i = 0; i < count; i++) if (rows[i].pass) passN++;
  if (count == 0) {
    o.print(F("<p class=\"banner wait\">NO RECORDS</p>"));
    o.print(F("<p class=\"note\">Submit a stable weighing in trade-weight mode. "
              "The log keeps the latest 50 records.</p>"));
    return;
  }
  bool all = (passN == count);
  pf(o, "<p class=\"banner %s\">%s</p>", all ? "pass" : "fail", all ? "PASS" : "FAIL");
  pf(o, "<p class=\"note\">%u weighed, %u passed, %u failed. "
        "A record passes when the absolute error (measured &minus; nominal) "
        "is within the Class M3 MPE.</p>",
     count, passN, (unsigned)(count - passN));
  o.print(F("<table><thead><tr>"
            "<th>No.</th><th>Record</th><th class=\"left\">Time</th>"
            "<th>Nominal g</th><th>Measured g</th><th>Error g</th>"
            "<th>MPE g</th><th>Result</th></tr></thead><tbody>"));
  char when[24];
  for (uint8_t i = 0; i < count; i++) {
    const TradeRecord& r = rows[i];
    formatWhen(when, sizeof(when), r.unixTime);
    pf(o, "<tr><td>%u</td><td>", (unsigned)(i + 1));
    writeHref(o, host, r.id);
    pf(o, "</td><td class=\"left\">%s</td><td>%.3f</td><td>%.3f</td><td>%+.3f</td><td>%.3f</td>",
       when, r.nominalG, r.measuredG, r.errorG, r.mpeG);
    pf(o, "<td class=\"%s\">%s</td></tr>", r.pass ? "pass" : "fail", r.pass ? "PASS" : "FAIL");
  }
  o.print(F("</tbody></table>"));
  o.print(F("<p class=\"note\">MPE table (g), nominal then limit: "
            "50/0.03, 100/0.05, 200/0.10, 500/0.25, 1000/0.50, 2000/1.00, "
            "5000/2.50, 10000/5.00, 15000/7.50, 20000/10.00. "
            "Intermediate nominals are interpolated.</p>"));
}

static const char* yn(bool v) { return v ? "PASS" : "FAIL"; }

static void writeQtyTable(Print& o, const PrepackResult& r) {
  const char* banner;
  const char* cls;
  if (!r.complete) {
    banner = "IN PROGRESS";
    cls = "wait";
  } else if (r.quantityPass) {
    banner = "QUANTITY PASS";
    cls = "pass";
  } else {
    banner = "QUANTITY FAIL";
    cls = "fail";
  }
  pf(o, "<p class=\"banner %s\">%s</p>", cls, banner);
  o.print(F("<table>"));
  pf(o, "<tr><td class=\"left\">Samples</td><td>%u / %u</td></tr>", r.n, r.nPlanned);
  pf(o, "<tr><td class=\"left\">Qn (mass)</td><td>%.3f g</td></tr>", r.qnG);
  pf(o, "<tr><td class=\"left\">T threshold</td><td>%.3f g</td></tr>", r.tG);
  pf(o, "<tr><td class=\"left\">T1 deficients</td><td class=\"%s\">%u (allow %u) %s</td></tr>",
     r.t1Pass ? "pass" : "fail", r.nT1, r.t1Allowed, yn(r.t1Pass));
  pf(o, "<tr><td class=\"left\">T2 non-conforming</td><td class=\"%s\">%u %s</td></tr>",
     r.t2Pass ? "pass" : "fail", r.nT2, yn(r.t2Pass));
  pf(o, "<tr><td class=\"left\">Mean</td><td>%.3f g</td></tr>", r.meanG);
  pf(o, "<tr><td class=\"left\">Std. dev. s</td><td>%.3f g</td></tr>", r.stdevG);
  pf(o, "<tr><td class=\"left\">Factor k</td><td>%.3f</td></tr>", r.kFactor);
  pf(o, "<tr><td class=\"left\">Corrected mean</td><td class=\"%s\">%.3f g %s</td></tr>",
     r.meanPass ? "pass" : "fail", r.correctedMeanG, yn(r.meanPass));
  if (r.id) {
    o.print(F("<tr><td class=\"left\">Record</td><td>"));
    writeHref(o, snapHost, r.id);
    o.print(F("</td></tr>"));
  }
  o.print(F("</table>"));
  o.print(F("<p class=\"note\">Deficit = declared &minus; net. "
            "Deficit &le; 0 passes. Up to T is a T1 error. Above T is a T2 error. "
            "T is 9% up to 50 g, 4.5 g up to 100 g, 4.5% up to 200 g, 9 g up to 300 g, "
            "3% up to 500 g, 15 g up to 1000 g, otherwise 1.5%. "
            "Corrected mean = mean &minus; k&times;s and must be at least the declared mass. "
            "k is 0.379, 0.295, or 0.234 for n &le; 50, &le; 80, and above. "
            "The lot is accepted only when the sample is complete, T2 count is zero, "
            "T1 count is within 3/5/7, and the corrected mean reaches the declared mass.</p>"));
}

static void writeSampleRows(Print& o, const float* x, uint16_t n, float qn, float t) {
  if (n == 0) {
    o.print(F("<p class=\"note\">No packages have been submitted for this lot.</p>"));
    return;
  }
  o.print(F("<table><thead><tr><th>No.</th><th>Net g</th><th>Deficit g</th><th>Flag</th></tr></thead><tbody>"));
  long qnMg = toMg(qn);
  long tMg = toMg(t);
  for (uint16_t i = 0; i < n; i++) {
    long deficit = qnMg - toMg(x[i]);
    const char* flag = "OK";
    const char* cls = "pass";
    if (deficit > tMg) { flag = "T2"; cls = "fail"; }
    else if (deficit > 0) { flag = "T1"; cls = "wait"; }
    pf(o, "<tr><td>%u</td><td>%.3f</td><td>%+.3f</td><td class=\"%s\">%s</td></tr>",
       (unsigned)(i + 1), x[i], qn - x[i], cls, flag);
  }
  o.print(F("</tbody></table>"));
}

static PrepackResult resultFromSeal(const LotSeal& s) {
  PrepackResult r = {};
  r.complete = s.complete;
  r.t1Pass = s.t1Pass;
  r.t2Pass = s.t2Pass;
  r.meanPass = s.meanPass;
  r.quantityPass = s.quantityPass;
  r.r79Pass = s.r79Pass;
  r.n = s.n;
  r.nPlanned = s.nPlanned;
  r.nT1 = s.nT1;
  r.nT2 = s.nT2;
  r.t1Allowed = s.t1Allowed;
  r.qnG = s.qnG;
  r.tG = s.tG;
  r.t2G = s.t2G;
  r.meanG = s.meanG;
  r.stdevG = s.stdevG;
  r.correctedMeanG = s.correctedG;
  r.kFactor = s.kFactor;
  r.id = s.id;
  return r;
}

static void writeR79(Print& o, const char* prod, const char* maker, const char* addr,
                     float declared, float dens, bool ok) {
  o.print(F("<h2 style=\"font-size:16px\">R79 labelling</h2><table class=\"meta\">"));
  writeField(o, "Product", prod);
  writeField(o, "Manufacturer", maker);
  writeField(o, "Address", addr);
  o.print(F("</table>"));
  pf(o, "<p class=\"note\">Declared Qn %.3f %s",
     declared, (dens >= 0.05f) ? "ml" : "g");
  if (dens >= 0.05f) pf(o, ", density %.3f g/ml", dens);
  pf(o, ". Label check: <span class=\"%s\">%s</span>.</p>",
     ok ? "pass" : "fail", ok ? "COMPLETE" : "INCOMPLETE");
}

static void captureSnap() {
  fillView(snapView);
  snapTradeCount = tradeCount;
  for (uint8_t i = 0; i < tradeCount; i++) snapTrade[i] = trade[tradePhysical(i)];
  snapLotCount = lotCount;
  for (uint8_t i = 0; i < lotCount; i++) snapLots[i] = lots[lotPhysical(i)];
  snapSampleN = sampleFill;
  if (sampleFill > 0) memcpy(snapSamples, samples, sizeof(float) * sampleFill);
  snapOwner = samplesOwnerId;
  snapResult = evaluateUnlocked();
  copyField(snapHost, sizeof(snapHost), hostIp);
}

void briqWriteCertificate(Print& out) {
  lockPage();
  if (lockData()) {
    captureSnap();
    unlockData();
  }
  if (snapView.mode == BRIQ_TRADE) {
    writeTop(out, "Certificate of trade weight verification");
    out.print(F("<p class=\"sub\">OIML R111 Class M3</p><table class=\"meta\">"));
    writeField(out, "Inspector", snapView.inspector);
    writeField(out, "Shop / trader", snapView.shop);
    writeField(out, "Location", snapView.location);
    char nom[32];
    snprintf(nom, sizeof(nom), "%.3f g  (MPE +/- %.3f g)", snapView.nominalG, snapView.mpeG);
    writeField(out, "Nominal in force", nom);
    out.print(F("</table>"));
    writeTradeTable(out, snapTrade, snapTradeCount, snapHost);
  } else {
    writeTop(out, "Certificate of pre-package inspection");
    out.print(F("<p class=\"sub\">OIML R87 quantity and R79 labelling</p><table class=\"meta\">"));
    writeField(out, "Inspector", snapView.inspector);
    char lotb[32];
    snprintf(lotb, sizeof(lotb), "%lu", (unsigned long)snapView.lotN);
    writeField(out, "Lot size", lotb);
    out.print(F("</table>"));
    writeR79(out, snapView.product, snapView.manufacturer, snapView.address,
             snapView.qnDeclared, snapView.density, snapView.r79Ok);
    writeQtyTable(out, snapResult);
    out.print(F("<h2 style=\"font-size:16px\">Sample</h2>"));
    writeSampleRows(out, snapSamples, snapSampleN, snapResult.qnG, snapResult.tG);
    if (snapLotCount > 0) {
      out.print(F("<h2 style=\"font-size:16px\">Recent sealed lots</h2><ul>"));
      char when[24];
      for (uint8_t i = 0; i < snapLotCount; i++) {
        const LotSeal& s = snapLots[i];
        formatWhen(when, sizeof(when), s.unixTime);
        o.print(F("<li>"));
        writeHref(out, snapHost, s.id);
        pf(out, " &mdash; %s &mdash; %s Qn %.1f g, n=%u</li>",
           when, s.quantityPass ? "PASS" : "FAIL", s.qnG, s.n);
      }
      out.print(F("</ul>"));
    }
  }
  writeFooter(out);
  unlockPage();
}

bool briqContains(uint32_t id) {
  if (id == 0 || !tryData()) return false;
  bool found = false;
  for (uint8_t i = 0; i < tradeCount && !found; i++) {
    if (trade[tradePhysical(i)].id == id) found = true;
  }
  for (uint8_t i = 0; i < lotCount && !found; i++) {
    if (lots[lotPhysical(i)].id == id) found = true;
  }
  unlockData();
  return found;
}

static void oWriteFieldTrade(Print& out, const TradeRecord& tr);

bool briqWriteVerify(Print& out, uint32_t id) {
  TradeRecord tr = {};
  LotSeal seal = {};
  bool isTrade = false;
  bool isLot = false;
  float localSamples[MAX_BATCH_N];
  uint16_t localN = 0;
  char host[40];

  if (!lockData()) return false;
  for (uint8_t i = 0; i < tradeCount; i++) {
    if (trade[tradePhysical(i)].id == id) {
      tr = trade[tradePhysical(i)];
      isTrade = true;
      break;
    }
  }
  if (!isTrade) {
    for (uint8_t i = 0; i < lotCount; i++) {
      if (lots[lotPhysical(i)].id == id) {
        seal = lots[lotPhysical(i)];
        isLot = true;
        if (samplesOwnerId == id && sampleFill > 0) {
          localN = sampleFill;
          memcpy(localSamples, samples, sizeof(float) * localN);
        }
        break;
      }
    }
  }
  copyField(host, sizeof(host), hostIp);
  unlockData();

  if (!isTrade && !isLot) return false;

  lockPage();
  copyField(snapHost, sizeof(snapHost), host);
  if (isTrade) {
    writeTop(out, "Verification record");
    out.print(F("<p class=\"sub\">OIML R111 Class M3 trade weight</p>"));
    bool pass = tr.pass != 0;
    pf(out, "<p class=\"banner %s\">%s</p>", pass ? "pass" : "fail", pass ? "PASS" : "FAIL");
    char when[24];
    formatWhen(when, sizeof(when), tr.unixTime);
    out.print(F("<table>"));
    pf(out, "<tr><td class=\"left\">Record</td><td>%lu</td></tr>", (unsigned long)tr.id);
    pf(out, "<tr><td class=\"left\">Time</td><td class=\"left\">%s</td></tr>", when);
    oWriteFieldTrade(out, tr);
    out.print(F("</table>"));
    pf(out, "<p class=\"note\">Error = %.3f &minus; %.3f = %+.3f g. "
            "MPE = %.3f g. The record %s because the absolute error %s the MPE. "
            "This page confirms the record is stored on this instrument.</p>",
       tr.measuredG, tr.nominalG, tr.errorG, tr.mpeG,
       pass ? "passes" : "fails",
       pass ? "is within" : "exceeds");
  } else {
    writeTop(out, "Verification record");
    out.print(F("<p class=\"sub\">OIML R87 / R79 pre-package lot</p>"));
    PrepackResult r = resultFromSeal(seal);
    writeQtyTable(out, r);
    out.print(F("<table class=\"meta\">"));
    writeField(out, "Inspector", seal.inspector);
    writeField(out, "Product", seal.product);
    writeField(out, "Manufacturer", seal.maker);
    writeField(out, "Address", seal.address);
    char lotb[32];
    snprintf(lotb, sizeof(lotb), "%lu", (unsigned long)seal.lotN);
    writeField(out, "Lot size", lotb);
    out.print(F("</table>"));
    pf(out, "<p class=\"note\">R79 label at sealing: <span class=\"%s\">%s</span>.</p>",
       seal.r79Pass ? "pass" : "fail", seal.r79Pass ? "COMPLETE" : "INCOMPLETE");
    if (localN > 0) {
      out.print(F("<h2 style=\"font-size:16px\">Sample retained on the instrument</h2>"));
      writeSampleRows(out, localSamples, localN, seal.qnG, seal.tG);
    }
  }
  out.print(F("<p class=\"noprint\"><a href=\"/\">Back to certificate</a></p>"));
  writeFooter(out);
  unlockPage();
  return true;
}

static void oWriteFieldTrade(Print& out, const TradeRecord& tr) {
  writeField(out, "Inspector", tr.inspector);
  writeField(out, "Shop / trader", tr.shop);
  writeField(out, "Location", tr.location);
  char b[48];
  snprintf(b, sizeof(b), "%.3f g", tr.nominalG);
  writeField(out, "Nominal", b);
  snprintf(b, sizeof(b), "%.3f g", tr.measuredG);
  writeField(out, "Measured", b);
  snprintf(b, sizeof(b), "%+.3f g", tr.errorG);
  writeField(out, "Error", b);
  snprintf(b, sizeof(b), "%.3f g", tr.mpeG);
  writeField(out, "MPE", b);
}

void briqArmWhatsApp(const char* phone, const char* text) {
  if (!lockData()) return;
  copyField(waPhone, STR_LEN, phone);
  copyField(waText, sizeof(waText), text);
  unlockData();
}

bool sendBriqWhatsAppReport() {
  char phone[STR_LEN];
  char text[sizeof(waText)];
  if (!lockData()) return false;
  copyField(phone, sizeof(phone), waPhone);
  copyField(text, sizeof(text), waText);
  unlockData();

  if (!phone[0] || !text[0]) {
    Serial.println(F("WhatsApp: missing phone or text"));
    return false;
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("WhatsApp: WiFi down"));
    return false;
  }
  if (!strcmp(BRIQ_API_KEY, "YOUR_BRIQ_API_KEY") || BRIQ_API_KEY[0] == 0) {
    Serial.printf("WhatsApp skipped (set BRIQ_API_KEY). To %s: %s\n", phone, text);
    return false;
  }

  JsonDocument doc;
  doc["to"] = phone;
  doc["message"] = text;
  String body;
  serializeJson(doc, body);

  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(4000);
  if (!client.connect("karibu.briq.tz", 443)) {
    Serial.println(F("WhatsApp: connect failed"));
    return false;
  }

  client.print(F("POST /v1/whatsapp/messages/text HTTP/1.1\r\n"));
  client.print(F("Host: karibu.briq.tz\r\n"));
  client.print(F("Authorization: Bearer "));
  client.print(BRIQ_API_KEY);
  client.print(F("\r\nContent-Type: application/json\r\n"));
  client.print(F("Connection: close\r\nContent-Length: "));
  client.print((unsigned)body.length());
  client.print(F("\r\n\r\n"));
  client.print(body);

  char status[48];
  size_t n = 0;
  uint32_t t0 = millis();
  while (n + 1 < sizeof(status) && (uint32_t)(millis() - t0) < 4000) {
    if (!client.available()) {
      if (!client.connected()) break;
      delay(1);
      continue;
    }
    char c = (char)client.read();
    status[n++] = c;
    if (c == '\n') break;
  }
  status[n] = 0;
  client.stop();
  Serial.printf("WhatsApp response: %s\n", status);
  return strncmp(status, "HTTP/1.1 2", 10) == 0 || strncmp(status, "HTTP/1.0 2", 10) == 0;
}
