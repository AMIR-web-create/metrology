#pragma once

// Metrology, inspection log, and printable certificate pages.
// Mutators are called from the Arduino loop. Page writers are safe to call
// from the async web task: they copy under a short lock, then print.

#include <Arduino.h>
#include <Print.h>
#include "config.h"

enum BriqMode : uint8_t { BRIQ_PREPACK = 1, BRIQ_TRADE = 2 };

enum UnitMark : uint8_t { UNIT_OK = 0, UNIT_T1 = 1, UNIT_T2 = 2 };

struct TradeCheck {
  bool valid;
  bool pass;
  bool logged;
  uint32_t id;
  float nominalG;
  float measuredG;
  float errorG;
  float mpeG;
};

struct PrepackResult {
  bool complete;
  bool t1Pass;
  bool t2Pass;
  bool meanPass;
  bool quantityPass;
  bool r79Pass;
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
  float correctedMeanG;
  float kFactor;
  uint32_t id;
};

struct SampleAdd {
  bool accepted;
  bool closed;
  UnitMark mark;
  PrepackResult lot;
};

struct BriqView {
  BriqMode mode;
  float qnDeclared;
  float qnMassG;
  float density;
  float packageTareG;
  float nominalG;
  float mpeG;
  float tG;
  uint32_t lotN;
  uint16_t plannedN;
  uint16_t sampleFill;
  uint8_t tradeCount;
  bool r79Ok;
  char inspector[STR_LEN];
  char product[STR_LEN];
  char address[STR_LEN];
  char manufacturer[STR_LEN];
  char shop[STR_LEN];
  char location[STR_LEN];
  char batch[STR_LEN];
  char phone[STR_LEN];
};

// Loads NVS and runs the metrology self-check. False means the self-check failed.
bool briqBegin();
bool briqHealthy();

// OIML R111 Class M3 table used by this instrument (grams).
float briqMpeG(float nominalG);
// OIML R87 tolerable deficiency T (grams of product).
float briqTValue(float qnG);
// Sample size: 50 if lot < 500, 80 if lot <= 3200, 125 otherwise, capped by the lot.
uint16_t briqSampleSize(uint32_t lotN);

void briqSetMode(BriqMode mode);
void briqSetInspector(const char* s);
void briqSetProduct(const char* s);
void briqSetAddress(const char* s);
void briqSetManufacturer(const char* s);
void briqSetShop(const char* s);
void briqSetLocation(const char* s);
void briqSetBatch(const char* s);
void briqSetPhone(const char* s);
void briqSetQn(float qn);
void briqSetLot(uint32_t lotN);
void briqSetPackageTare(float grams);
void briqSetDensity(float gPerMl);
void briqSetNominal(float grams);
void briqSetHost(const char* ip);

// False when a certificate render currently owns the data lock.
bool briqRead(BriqView& out);
bool briqFormatLimits(char* buf, size_t n);

TradeCheck briqLogTrade(float measuredG);
bool briqLastTrade(TradeCheck& out);

void briqClearSamples();
void briqReplacePackSamples(const float* netG, uint16_t n);
uint16_t briqExportNets(float* dst, uint16_t max);
SampleAdd briqAddSample(float netG);
PrepackResult briqCloseLot();

// Copies the text, then POST it on the next sendBriqWhatsAppReport() call.
void briqArmWhatsApp(const char* phone, const char* text);
// One HTTPS POST to BRIQ. Called from loop(), not from the button handler.
bool sendBriqWhatsAppReport();

bool briqContains(uint32_t id);
void briqWriteCertificate(Print& out);
// False when id is not in the trade log or the recent lot log.
bool briqWriteVerify(Print& out, uint32_t id);
