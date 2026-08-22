#include <BleGamepad.h>

// ---------- KONFIGURASI PIN ----------
const int PIN_VRX   = 0;   // GPIO0
const int PIN_VRY   = 1;   // GPIO1
const int PIN_SW    = 2;   // GPIO2 (klik stick)

const int PIN_BTN_A     = 4;   // GPIO4 -> Tombol A
const int PIN_BTN_Y     = 5;   // GPIO5 -> Tombol Y
const int PIN_BTN_UP    = 6;   // GPIO6 -> D-Pad Atas
const int PIN_BTN_DOWN  = 7;   // GPIO7 -> D-Pad Bawah
const int PIN_BTN_LB    = 8;   // GPIO8 -> LB (L1)
const int PIN_BTN_LT    = 9;   // GPIO9 -> LT (L2)

// ---------- KONFIGURASI FILTER ANALOG ----------
const int   OVERSAMPLE_COUNT   = 8;      // jumlah sample per pembacaan
const float EMA_ALPHA          = 0.15f;  // makin kecil = makin halus, tapi makin lag
const int   DEADZONE_RADIUS    = 6;      // rentang deadzone (-127..127)
const int   ADC_MAX            = 4095;   // ESP32-C3 ADC 12-bit

// ---------- STATE ----------
float emaX = 0, emaY = 0;
int centerX = 2048, centerY = 2048;   
bool firstRead = true;

BleGamepad bleGamepad("JoyCon Kiri v2", "DIY", 100);

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

  pinMode(PIN_SW,       INPUT_PULLUP);
  pinMode(PIN_BTN_A,    INPUT_PULLUP);
  pinMode(PIN_BTN_Y,    INPUT_PULLUP);
  pinMode(PIN_BTN_UP,   INPUT_PULLUP);
  pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
  pinMode(PIN_BTN_LB,   INPUT_PULLUP);
  pinMode(PIN_BTN_LT,   INPUT_PULLUP);

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

    // Tombol Joy-Con Kiri (aktif LOW)
    if (digitalRead(PIN_SW)       == LOW) bleGamepad.press(BUTTON_1); else bleGamepad.release(BUTTON_1); // Click Stick (L3)
    if (digitalRead(PIN_BTN_A)    == LOW) bleGamepad.press(BUTTON_2); else bleGamepad.release(BUTTON_2); // Tombol A
    if (digitalRead(PIN_BTN_Y)    == LOW) bleGamepad.press(BUTTON_3); else bleGamepad.release(BUTTON_3); // Tombol Y
    if (digitalRead(PIN_BTN_UP)   == LOW) bleGamepad.press(BUTTON_4); else bleGamepad.release(BUTTON_4); // D-Pad Atas
    if (digitalRead(PIN_BTN_DOWN) == LOW) bleGamepad.press(BUTTON_5); else bleGamepad.release(BUTTON_5); // D-Pad Bawah
    if (digitalRead(PIN_BTN_LB)   == LOW) bleGamepad.press(BUTTON_6); else bleGamepad.release(BUTTON_6); // LB (L1)
    if (digitalRead(PIN_BTN_LT)   == LOW) bleGamepad.press(BUTTON_7); else bleGamepad.release(BUTTON_7); // LT (L2)

    bleGamepad.sendReport();
  }

  delay(8); // Responsif 125Hz, sama persis dengan Joy-Con Kanan
}