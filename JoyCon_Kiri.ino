/*
  ============================================================
   JOY-CON KIRI (LEFT) - ESP32-C3 Supermini
   Fitur: BLE Gamepad + ESP-NOW + TTP223 (Double Tap) + HC-SR04 Kiri + Macro Mode Permanen
  ============================================================
*/

#include <BleGamepad.h>
#include <esp_now.h>
#include <WiFi.h>

// ---------- KONFIGURASI PIN GAMEPAD ----------
const int PIN_VRX    = 0;   // GPIO0
const int PIN_VRY    = 1;   // GPIO1
const int PIN_SW     = 2;   // GPIO2 (klik stick)

const int PIN_TTP223 = 3;   // GPIO3 -> Sensor Sentuh TTP223 (Substitusi PIN_BTN_ACTIVATE)

const int PIN_BTN_A    = 4;   // GPIO4 -> Tombol A
const int PIN_BTN_Y    = 5;   // GPIO5 -> Tombol Y
const int PIN_BTN_UP   = 6;   // GPIO6 -> D-Pad Atas
const int PIN_BTN_DOWN = 7;   // GPIO7 -> D-Pad Bawah
const int PIN_BTN_LB   = 8;   // GPIO8 -> LB (L1)
const int PIN_BTN_LT   = 9;   // GPIO9 -> LT (L2)

// ---------- PIN SENSOR ULTRASONIK (HC-SR04 KIRI) ----------
const int PIN_TRIG = 10;  // GPIO10
const int PIN_ECHO = 20;  // GPIO20
const int TRIGGER_DISTANCE_CM = 15; // Batas jarak tangan (15cm)

// ---------- ALAMAT MAC ESP32 KANAN ----------
uint8_t broadcastAddress[] = {0xE8, 0x3D, 0xC1, 0x9D, 0x7B, 0x48}; // MAC Joy-Con Kanan

// ---------- STRUKTUR DATA ESP-NOW ----------
struct SystemState {
  int currentState; // 0: Reset, 1: Standby (0001), 2: Left Trigger (0002), 3: Right Trigger (0003), 4: Complete (0004)
};
SystemState stateData;

// ---------- KONFIGURASI FILTER ANALOG ----------
const int   OVERSAMPLE_COUNT   = 8;
const float EMA_ALPHA          = 0.15f;
const int   DEADZONE_RADIUS    = 6;
const int   ADC_MAX            = 4095;

// ---------- STATE SYSTEM ----------
float emaX = 0, emaY = 0;
int centerX = 2048, centerY = 2048;   
bool isDriverActivated = false;
bool leftSensorTriggered = false;
bool isMacroActive = false; // Flag Mode Makro aktif setelah Complete

// ---------- VARIABLE DETEKSI DOUBLE TAP TTP223 ----------
bool lastTouchState = LOW;
unsigned long lastTapTime = 0;
const unsigned long DOUBLE_TAP_TIMEOUT = 400; // Maksimal jeda 400ms untuk double tap
unsigned long activateStartTime = 0;
const unsigned long HCSR_WINDOW_MS = 7000;   // Window 7 detik untuk HC-SR04

BleGamepad bleGamepad("JoyCon Kiri v2", "DIY", 100);

// Callback Pengiriman Data
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  // Callback status pengiriman
}

// Callback Penerimaan Data (Menerima sinyal State 4 / State 0 dari Joy-Con Kanan)
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len) {
  SystemState recvState;
  memcpy(&recvState, incomingData, sizeof(recvState));
  
  if (recvState.currentState == 4) {
    isMacroActive = true;
    isDriverActivated = false; // Proses aktivasi awal selesai, berpindah ke Mode Makro Permanen
    Serial.println("[KIRI] Sinyal Complete (State 4) Diterima -> Mode Makro PERMANEN AKTIF!");
  } else if (recvState.currentState == 0) {
    isMacroActive = false;
    isDriverActivated = false;
    leftSensorTriggered = false;
    Serial.println("[KIRI] Reset State Diterima -> Normal Mode");
  }
}

// ---------- FUNGSI ULTRASONIK & ANALOG ----------
long measureDistance() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  long duration = pulseIn(PIN_ECHO, HIGH, 6000); 
  if (duration == 0) return -1;
  return (duration * 0.0343 / 2);
}

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
  for (int i = 0; i < 50; i++) {
    sumX += readAveraged(PIN_VRX);
    sumY += readAveraged(PIN_VRY);
    delay(5);
  }
  centerX = sumX / 50;
  centerY = sumY / 50;
}

int8_t processAxis(int raw, int center, float &emaState) {
  emaState = (EMA_ALPHA * raw) + ((1.0f - EMA_ALPHA) * emaState);
  float diff = emaState - center;
  float scaled = (diff >= 0) ? (diff / (ADC_MAX - center)) * 127.0f : (diff / center) * 127.0f;
  int val = constrain((int)scaled, -127, 127);
  if (abs(val) < DEADZONE_RADIUS) val = 0;
  return (int8_t)val;
}

void sendStateToRight(int state) {
  stateData.currentState = state;
  esp_now_send(broadcastAddress, (uint8_t *) &stateData, sizeof(stateData));
  delay(5);
  esp_now_send(broadcastAddress, (uint8_t *) &stateData, sizeof(stateData));
}

void resetSystem() {
  isDriverActivated = false;
  leftSensorTriggered = false;
  isMacroActive = false;
  sendStateToRight(0); // Sinyal Reset ke Kanan (Stop Audio)
  Serial.println("[RESET] System Reset -> Normal Mode Kembali");
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);

  pinMode(PIN_SW,       INPUT_PULLUP);
  pinMode(PIN_TTP223, INPUT_PULLDOWN); // TTP223 mengeluarkan sinyal HIGH saat disentuh
  pinMode(PIN_BTN_A,    INPUT_PULLUP);
  pinMode(PIN_BTN_Y,    INPUT_PULLUP);
  pinMode(PIN_BTN_UP,   INPUT_PULLUP);
  pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
  pinMode(PIN_BTN_LB,   INPUT_PULLUP);
  pinMode(PIN_BTN_LT,   INPUT_PULLUP);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() == ESP_OK) {
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
    
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
  }

  analogReadResolution(12);
  delay(800);
  calibrateCenter();
  emaX = centerX;
  emaY = centerY;

  BleGamepadConfiguration cfg;
  cfg.setAutoReport(false);
  cfg.setControllerType(CONTROLLER_TYPE_GAMEPAD);
  cfg.setButtonCount(8);
  cfg.setHatSwitchCount(0);
  cfg.setWhichAxes(true, true, false, false, false, false, false, false); 
  bleGamepad.begin(&cfg);
}

// ---------- LOOP ----------
void loop() {
  if (bleGamepad.isConnected()) {
    // 1. Baca Joystick Analog
    int rawX = readAveraged(PIN_VRX);
    int rawY = readAveraged(PIN_VRY);
    bleGamepad.setAxes(processAxis(rawX, centerX, emaX), processAxis(rawY, centerY, emaY), 0, 0, 0, 0, 0, 0);

    // 2. Baca Tombol Joy-Con (Mode Normal vs Mode Makro)
    if (!isMacroActive) {
      // --- MODE NORMAL (Gamepad Standar) ---
      if (digitalRead(PIN_SW)       == LOW) bleGamepad.press(BUTTON_1); else bleGamepad.release(BUTTON_1);
      if (digitalRead(PIN_BTN_A)    == LOW) bleGamepad.press(BUTTON_2); else bleGamepad.release(BUTTON_2);
      if (digitalRead(PIN_BTN_Y)    == LOW) bleGamepad.press(BUTTON_3); else bleGamepad.release(BUTTON_3);
      if (digitalRead(PIN_BTN_UP)   == LOW) bleGamepad.press(BUTTON_4); else bleGamepad.release(BUTTON_4);
      if (digitalRead(PIN_BTN_DOWN) == LOW) bleGamepad.press(BUTTON_5); else bleGamepad.release(BUTTON_5);
      if (digitalRead(PIN_BTN_LB)   == LOW) bleGamepad.press(BUTTON_6); else bleGamepad.release(BUTTON_6);
      if (digitalRead(PIN_BTN_LT)   == LOW) bleGamepad.press(BUTTON_7); else bleGamepad.release(BUTTON_7);
    } else {
      // --- MODE MAKRO PERMANEN (Setelah Henshin Complete) ---
      if (digitalRead(PIN_BTN_A) == LOW) {
        bleGamepad.press(BUTTON_2);
        bleGamepad.press(BUTTON_6);
      } else {
        bleGamepad.release(BUTTON_2);
        bleGamepad.release(BUTTON_6);
      }
      
      if (digitalRead(PIN_SW)       == LOW) bleGamepad.press(BUTTON_1); else bleGamepad.release(BUTTON_1);
      if (digitalRead(PIN_BTN_Y)    == LOW) bleGamepad.press(BUTTON_3); else bleGamepad.release(BUTTON_3);
      if (digitalRead(PIN_BTN_UP)   == LOW) bleGamepad.press(BUTTON_4); else bleGamepad.release(BUTTON_4);
      if (digitalRead(PIN_BTN_DOWN) == LOW) bleGamepad.press(BUTTON_5); else bleGamepad.release(BUTTON_5);
      if (digitalRead(PIN_BTN_LB)   == LOW) bleGamepad.press(BUTTON_6); else bleGamepad.release(BUTTON_6);
      if (digitalRead(PIN_BTN_LT)   == LOW) bleGamepad.press(BUTTON_7); else bleGamepad.release(BUTTON_7);
    }

    // 3. LOGIKA DOUBLE TAP TTP223
    bool currentTouchState = (digitalRead(PIN_TTP223) == HIGH);
    
    // Deteksi RISING EDGE (disentuh)
    if (currentTouchState == HIGH && lastTouchState == LOW) {
      unsigned long now = millis();
      
      if (now - lastTapTime <= DOUBLE_TAP_TIMEOUT) {
        // DOUBLE TAP TERDETEKSI!
        if (isMacroActive || isDriverActivated) {
          // Jika sedang aktif/Henshin, double tap akan MENGEMBALIKAN KE MODE NORMAL
          resetSystem();
        } else {
          // Jika belum aktif, double tap akan MEMULAI HENSHIN
          isDriverActivated = true;
          leftSensorTriggered = false;
          activateStartTime = now; // Catat waktu mulai window 7 detik
          bleGamepad.press(BUTTON_8);
          sendStateToRight(1);     // Perintah Kanan: Putar 0001.mp3
          Serial.println("[DOUBLE TAP] Driver Active -> Sinyal 0001.mp3 terkirim ke Kanan");
        }
        lastTapTime = 0; // Reset timer tap
      } else {
        lastTapTime = now; // Single tap pertama
      }
      delay(50); // Debounce sederhana
    }
    lastTouchState = currentTouchState;

    // 4. LOGIKA TIMEOUT 7 DETIK HC-SR04 KIRI
    if (isDriverActivated && !leftSensorTriggered) {
      // Cek apakah window 7 detik sudah habis
      if (millis() - activateStartTime > HCSR_WINDOW_MS) {
        Serial.println("[TIMEOUT 7s] Tidak ada pemicu HC-SR04 -> Reset System");
        bleGamepad.release(BUTTON_8);
        resetSystem();
      } else {
        // Proses pengecekan ultrasonik selama dalam window 7 detik
        static unsigned long lastUltrasoundTime = 0;
        if (millis() - lastUltrasoundTime > 60) {
          lastUltrasoundTime = millis();
          long dist = measureDistance();
          
          if (dist > 0 && dist <= TRIGGER_DISTANCE_CM) {
            leftSensorTriggered = true;
            sendStateToRight(2); // Perintah Kanan: Putar 0002.mp3
            Serial.printf("[STATE 2] HC-SR04 Kiri Trigger (%ld cm) -> Sinyal 0002.mp3 terkirim ke Kanan\n", dist);
          }
        }
      }
    }

    // Lepas pemicu BUTTON_8 jika window aktivasi telah selesai
    if (!isDriverActivated && !isMacroActive) {
      bleGamepad.release(BUTTON_8);
    }

    bleGamepad.sendReport();
  }

  delay(8);
}
