# Dual Smart Weight Verification & Pre-Packaged Inspection

ESP32 firmware for a 20 kg load-cell scale that does two jobs:

1. **VERIFY** — check a piece against a saved target weight ± tolerance.
2. **INSPECT** — check pre-packaged goods against a nominal quantity **Qn** using OIML R87-style **TU1 / TU2** limits, then judge a batch (average and deficient packs).

Hardware: ESP32, HX711 + 20 kg load cell, 20×4 I2C LCD at **0x20**, buck converter, green/red LEDs, four buttons.

---

## Wiring

Power the ESP32 **VIN** (5 V) and the LCD VCC from the **buck converter 5 V** output. Power the HX711 from **ESP32 3.3 V** so DT/SCK stay 3.3 V-safe. Common **GND** everywhere.

```
Adapter / battery ──► Buck IN
                      Buck 5V ──► ESP32 VIN
                                └► LCD VCC
                      Buck GND ──► ESP32 GND, LCD GND, HX711 GND, button GND, LED GND

Load cell (typical E+ E- A+ A-):
  E+  → HX711 E+
  E-  → HX711 E-
  A+  → HX711 A+
  A-  → HX711 A-
```

| Device | Pin | ESP32 GPIO |
|--------|-----|------------|
| HX711 DT (DOUT) | DOUT | **26** |
| HX711 CLK | CLK | **25** |
| HX711 VCC | VCC | **3.3 V** |
| LCD SDA | SDA | **21** |
| LCD SCL | SCL (SCK) | **22** |
| LCD VCC | VCC | **5 V** (buck) |
| Green LED (+) | via 220–330 Ω | **14** |
| Red LED (+) | via 220–330 Ω | **27** |
| RESET button | to GND | **13** |
| TARE button | to GND | **17** |
| SAVE button | to GND | **4** |
| SUBMIT button | to GND | **16** |

Buttons use the ESP32 internal pull-ups (press = GND). LED cathodes go to GND.

If the LCD stays blank, run an I2C scanner. Address **0x20** is PCF8574 with A0/A1/A2 to GND. Change `LCD_ADDR` in `include/config.h` if your backpack is 0x27 / 0x3F.

---

## Build / flash

**PlatformIO** (this folder):

```
pio run -t upload
pio device monitor
```

**Arduino IDE:** open `arduino/SmartScale/SmartScale.ino`, board **ESP32 Dev Module**, install libraries **HX711** (Bogde) and **LiquidCrystal_I2C**.

---

## First-time calibration (required)

1. Keep the pan **empty**.
2. Hold **TARE** while you reset / power on.
3. Wait for tare, then put a **known mass** on the pan (default 1000 g).
4. **SAVE** / **TARE** change the known-mass value in 10 g steps.
5. **SUBMIT** stores the calibration factor in flash.

Hold **RESET** at boot to wipe saved settings (factory defaults).

---

## Daily use

| Button | Short press | Long press (~0.9 s) |
|--------|-------------|---------------------|
| **RESET** | VERIFY ↔ INSPECT | Settings for the current mode |
| **TARE** | Zero the scale | Zero again |
| **SAVE** | VERIFY: current stable weight becomes **target**. INSPECT: current weight becomes **Qn** (clears batch) | INSPECT: clear batch |
| **SUBMIT** | Lock a **stable** reading, PASS/FAIL on LEDs. INSPECT also adds the pack to the batch | INSPECT: close batch (average + TU1/TU2) |

**Green LED** = PASS. **Red LED** = FAIL. SUBMIT is ignored until the reading is **STABLE** (span ≤ 1.2 g).

### VERIFY

PASS if `|weight − target| ≤ tolerance`.  
Set target/tolerance in settings, or SAVE a golden sample as the target.

### INSPECT (pre-packaged)

- **TU1** and **TU2 = 2×TU1** from OIML R87 Table 1 (by Qn).
- One pack **FAIL** if it is below `Qn − TU2`.
- Below `Qn − TU1` is a **T1 warn** (still recorded).
- **Batch PASS** if: no pack below TU2, batch average ≥ Qn, and T1-deficient count is within the allowed fraction.

---

## Settings (long RESET)

- **RESET** = next field  
- **SAVE** = increase, **TARE** = decrease (hold for faster steps)  
- **SUBMIT** = save and exit  

VERIFY fields: target (g), ± tolerance (g).  
INSPECT fields: Qn (g), batch size N (1–32).

---

## Pin / default edits

Edit `include/config.h` (and `arduino/SmartScale/config.h` if you use the IDE). After changing the load cell or HX711, recalibrate; do not guess `DEFAULT_CAL_FACTOR`.
