/*
  ============================================================
   JOY-CON KANAN (RIGHT) - ESP32-C3 Supermini
   BLE Gamepad Controller (mirip Nintendo Joy-Con)
  ============================================================
  Library dibutuhkan (install lewat Library Manager):
    - "ESP32 BLE Gamepad" by lemmingDev
      (akan otomatis butuh NimBLE-Arduino, ikuti prompt install)

  WIRING:
    Analog Stick Module:
      VRx -> GPIO0  (ADC1_CH0)
      VRy -> GPIO1  (ADC1_CH1)
      SW  -> GPIO2  (klik stick, tekan ke bawah)
      VCC -> 3V3
      GND -> GND

    Tombol digital (contoh: A, B, X, Y, Plus, Home):
      BTN_A     -> GPIO4
      BTN_B     -> GPIO5
      BTN_X     -> GPIO6
      BTN_Y     -> GPIO7
      BTN_PLUS  -> GPIO8
      BTN_HOME  -> GPIO10
    Semua tombol: kaki lain ke GND, mode INPUT_PULLUP (aktif LOW)

  CATATAN ANTI-JITTER: sama seperti Joy-Con Kiri
    1. Oversampling ADC
    2. EMA (exponential moving average) filter
    3. Deadzone radial di titik tengah
    4. Auto-calibrate titik tengah saat boot (jangan sentuh stick saat nyala)
  ============================================================
*/

#include <BleGamepad.h>

// ---------- KONFIGURASI PIN ----------
const int PIN_VRX   = 0;   // GPIO0
const int PIN_VRY   = 1;   // GPIO1
const int PIN_SW    = 2;   // GPIO2 (klik stick)

const int PIN_BTN_B     = 4;   // GPIO4 -> Tombol B
const int PIN_BTN_X     = 5;   // GPIO5 -> Tombol X
const int PIN_BTN_RIGHT = 6;   // GPIO6 -> D-Pad Kanan
const int PIN_BTN_LEFT  = 7;   // GPIO7 -> D-Pad Kiri
const int PIN_BTN_RB    = 8;   // GPIO8 -> RB (R1)
const int PIN_BTN_RT    = 9;   // GPIO9 -> RT (R2)

// ---------- KONFIGURASI FILTER ANALOG ----------
const int   OVERSAMPLE_COUNT   = 8;
const float EMA_ALPHA          = 0.15f;
const int   DEADZONE_RADIUS    = 6;
const int   ADC_MAX            = 4095;

// ---------- STATE ----------
float emaX = 0, emaY = 0;
int centerX = 2048, centerY = 2048;

BleGamepad bleGamepad("JoyCon Kanan v2", "DIY", 100);

int readAveraged(int pin) {
  long sum = 0;
  for (int i = 0; i < OVERSAMPLE_COUNT; i++) {
    sum += analogRead(pin);
    delayMicroseconds(150);
  }
  return sum / OVERSAMPLE_COUNT;
}

void calibrateCenter() {
  long sumX = 0, sumY = 0;
  const int samples = 50;
  for (int i = 0; i < samples; i++) {
    sumX += readAveraged(PIN_VRX);
    sumY += readAveraged(PIN_VRY);
    delay(5);
  }
  centerX = sumX / samples;
  centerY = sumY / samples;
}

int8_t processAxis(int raw, int center, float &emaState) {
  emaState = (EMA_ALPHA * raw) + ((1.0f - EMA_ALPHA) * emaState);

  float diff = emaState - center;
  float scaled;
  if (diff >= 0) {
    scaled = (diff / (ADC_MAX - center)) * 127.0f;
  } else {
    scaled = (diff / center) * 127.0f;
  }

  int val = (int)scaled;
  val = constrain(val, -127, 127);

  if (abs(val) < DEADZONE_RADIUS) val = 0;

  return (int8_t)val;
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_SW,        INPUT_PULLUP);
  pinMode(PIN_BTN_B,     INPUT_PULLUP);
  pinMode(PIN_BTN_X,     INPUT_PULLUP);
  pinMode(PIN_BTN_RIGHT, INPUT_PULLUP);
  pinMode(PIN_BTN_LEFT,  INPUT_PULLUP);
  pinMode(PIN_BTN_RB,    INPUT_PULLUP);
  pinMode(PIN_BTN_RT,    INPUT_PULLUP);

  analogReadResolution(12);

  Serial.println("Kalibrasi stick... jangan sentuh joystick!");
  delay(800);
  calibrateCenter();
  emaX = centerX;
  emaY = centerY;
  Serial.printf("Kalibrasi selesai. CenterX=%d CenterY=%d\n", centerX, centerY);

  BleGamepadConfiguration cfg;
  cfg.setAutoReport(false);
  cfg.setControllerType(CONTROLLER_TYPE_GAMEPAD);
  cfg.setButtonCount(7);
  cfg.setHatSwitchCount(0);
  cfg.setWhichAxes(true, true, false, false, false, false, false, false);
  bleGamepad.begin(&cfg);
}

void loop() {
  if (bleGamepad.isConnected()) {
    int rawX = readAveraged(PIN_VRX);
    int rawY = readAveraged(PIN_VRY);

    int8_t gx = processAxis(rawX, centerX, emaX);
    int8_t gy = processAxis(rawY, centerY, emaY);

    bleGamepad.setAxes(gx, gy, 0, 0, 0, 0, 0, 0);

    if (digitalRead(PIN_SW)        == LOW) bleGamepad.press(BUTTON_1); else bleGamepad.release(BUTTON_1); // Click Stick (R3)
    if (digitalRead(PIN_BTN_B)     == LOW) bleGamepad.press(BUTTON_2); else bleGamepad.release(BUTTON_2); // Tombol B
    if (digitalRead(PIN_BTN_X)     == LOW) bleGamepad.press(BUTTON_3); else bleGamepad.release(BUTTON_3); // Tombol X
    if (digitalRead(PIN_BTN_RIGHT) == LOW) bleGamepad.press(BUTTON_4); else bleGamepad.release(BUTTON_4); // D-Pad Kanan
    if (digitalRead(PIN_BTN_LEFT)  == LOW) bleGamepad.press(BUTTON_5); else bleGamepad.release(BUTTON_5); // D-Pad Kiri
    if (digitalRead(PIN_BTN_RB)    == LOW) bleGamepad.press(BUTTON_6); else bleGamepad.release(BUTTON_6); // RB (R1)
    if (digitalRead(PIN_BTN_RT)    == LOW) bleGamepad.press(BUTTON_7); else bleGamepad.release(BUTTON_7); // RT (R2)

    bleGamepad.sendReport();
  }

  delay(8);
}
