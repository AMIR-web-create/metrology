/*
 * Dual Smart Scale
 *
 * Open this folder in the Arduino IDE (the folder name matches this file).
 * Board: ESP32 Dev Module.
 *
 * Install from Library Manager:
 *   Blynk, HX711 by bogde, LiquidCrystal I2C, ArduinoJson,
 *   ESPAsyncWebServer, AsyncTCP
 *
 * The program is in scale_app.cpp. Fill WiFi, Blynk, and BRIQ values
 * in blynk_secrets.h before upload.
 *
 * Every Blynk datastream is a String:
 *   V0 inspector   V1 declared quantity (500g / 1L)   V2 lot size
 *   V3 product     V4 address   V5 manufacturer   V6 package tare
 *   V7 density     V8 shop      V9 location       V10 nominal grams
 *   V11 class (kept as 3)       V13 verdict       V14 mode
 *   V15 net grams  V18 product or shop             V19 WhatsApp phone
 *   V20 batch
 *   V21 tare       V22 save     V23 submit
 *
 * V14 accepts prepack, 1, inspect, or trade, 2, verify, weight.
 * V21, V22, and V23 run when the app sends 1. The same actions are on
 * the MODE, TARE, SAVE, and SUBMIT buttons.
 */
