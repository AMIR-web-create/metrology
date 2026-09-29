#pragma once

// ---------------------------------------------------------------------------
// Dual Smart Scale — hardware map
// Mode 1: pre-packed inspection (OIML R87 quantity + R79 labelling data)
// Mode 2: weight verification (OIML R111 MPE)
// ---------------------------------------------------------------------------

#include <Arduino.h>

static const uint8_t LCD_ADDR = 0x20;
static const uint8_t LCD_COLS = 20;
static const uint8_t LCD_ROWS = 4;
static const int PIN_SDA = 21;
static const int PIN_SCL = 22;

static const int PIN_HX711_DT  = 26;  // DOUT
static const int PIN_HX711_SCK = 25;  // CLK

static const float DEFAULT_CAL_FACTOR = 21.50f;
static const float SCALE_CAPACITY_G   = 20000.0f;

static const int PIN_LED_PASS = 14;  // green
static const int PIN_LED_FAIL = 27;  // red

static const int PIN_BTN_MODE   = 32;
static const int PIN_BTN_TARE   = 17;
static const int PIN_BTN_SAVE   = 4;
static const int PIN_BTN_SUBMIT = 16;

static const uint32_t DEBOUNCE_MS    = 40;
static const uint32_t LONG_PRESS_MS  = 900;
static const uint32_t REPEAT_MS      = 180;

static const uint8_t  HX711_SAMPLES     = 8;
static const uint8_t  STABLE_WINDOW     = 6;
static const float    STABLE_SPAN_G     = 1.2f;
static const uint32_t DISPLAY_PERIOD_MS = 200;
static const uint32_t LED_IDLE_MS       = 2500;

static const float    DEFAULT_QN_G       = 500.0f;
static const float    DEFAULT_NOMINAL_G  = 1000.0f;
static const uint16_t DEFAULT_LOT_N      = 10;
static const uint16_t MAX_BATCH_N        = 200;  // gross/net log capacity
static const float    DEFAULT_TARE_G     = 0.0f;
static const float    DEFAULT_DENSITY    = 0.0f;  // 0 = Qn is already grams

static const uint8_t STR_LEN = 48;

// Blynk virtual pins. Every datastream is a String so the app can type
// letters or numbers. Create one String datastream per pin.
static const int VP_INSPECTOR   = 0;   // inspector name
static const int VP_QN          = 1;   // declared quantity ("500g", "1L")
static const int VP_LOT         = 2;   // lot size
static const int VP_PRODUCT     = 3;   // product name
static const int VP_ADDRESS     = 4;   // address
static const int VP_MAKER       = 5;   // manufacturer
static const int VP_PKG_TARE    = 6;   // package tare, grams
static const int VP_DENSITY     = 7;   // density g/ml, blank or 0 keeps the parsed value
static const int VP_SHOP        = 8;   // shop / trader
static const int VP_LOCATION    = 9;   // location
static const int VP_NOMINAL     = 10;  // trade nominal, grams
static const int VP_CLASS       = 11;  // kept at "3" (Class M3)
static const int VP_VERDICT     = 13;  // device → app
static const int VP_MODE        = 14;  // "1"/"prepack" or "2"/"trade"
static const int VP_NET_G       = 15;  // device → app, grams as text
static const int VP_META        = 18;  // product or shop snapshot
static const int VP_PHONE       = 19;  // WhatsApp recipient
static const int VP_BATCH       = 20;  // batch number
static const int VP_CMD_TARE    = 21;  // send "1" to tare
static const int VP_CMD_SAVE    = 22;  // send "1" to save
static const int VP_CMD_SUBMIT  = 23;  // send "1" to submit

static const char* NVS_NS = "scale";
