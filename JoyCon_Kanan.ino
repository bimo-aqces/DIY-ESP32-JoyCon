/*
  ============================================================
   JOY-CON KANAN (RIGHT) - ESP32-C3 Supermini
   Fitur: BLE Gamepad + ESP-NOW + DFPlayer Mini + HC-SR04 + Macro Permanen
  ============================================================
   Struktur SD Card (Wajib di Root SD Card):
     SD:/0001.mp3  -> Audio Standby / Driver Aktif
     SD:/0002.mp3  -> Audio Sensor Kiri Terpemicu
     SD:/0003.mp3  -> Audio Sensor Kanan Terpemicu
     SD:/0004.mp3  -> Audio Henshin Complete (Masuk Mode Makro)
  ============================================================
*/

#include <BleGamepad.h>
#include <esp_now.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <DFRobotDFPlayerMini.h>

// ---------- KONFIGURASI PIN GAMEPAD ----------
const int PIN_VRX       = 0;   // GPIO0
const int PIN_VRY       = 1;   // GPIO1
const int PIN_SW        = 2;   // GPIO2 (klik stick)

const int PIN_BTN_B     = 4;   // GPIO4 -> Tombol B
const int PIN_BTN_X     = 5;   // GPIO5 -> Tombol X
const int PIN_BTN_RIGHT = 6;   // GPIO6 -> D-Pad Kanan
const int PIN_BTN_LEFT  = 7;   // GPIO7 -> D-Pad Kiri
const int PIN_BTN_RB    = 8;   // GPIO8 -> RB (R1)
const int PIN_BTN_RT    = 9;   // GPIO9 -> RT (R2)

// ---------- PIN SENSOR ULTRASONIK (HC-SR04 KANAN) ----------
const int PIN_TRIG_RIGHT = 10; // GPIO10
const int PIN_ECHO_RIGHT = 3;  // GPIO3
const int TRIGGER_DISTANCE_CM = 15; // Jarak pemicu (15cm)

// ---------- PIN DFPLAYER MINI ----------
const int PIN_DFP_RX = 20; // RX ESP32 (Pin 20) <- Terhubung ke TX DFPlayer
const int PIN_DFP_TX = 21; // TX ESP32 (Pin 21) -> Terhubung ke RX DFPlayer (lewat Resistor 1k)

// ---------- ALAMAT MAC ESP32 KIRI ----------
uint8_t leftMacAddress[] = {0xE8, 0x3D, 0xC1, 0x9D, 0x54, 0xD8};

// ---------- STRUKTUR DATA ESP-NOW ----------
struct SystemState {
  int currentState; // 0: Reset, 1: Standby, 2: Left Trigger, 3: Right Trigger, 4: Complete
};
SystemState stateData;

// ---------- OBJECT & PERIPHERAL ----------
HardwareSerial dfpSerial(1);
DFRobotDFPlayerMini myDFPlayer;
BleGamepad bleGamepad("JoyCon Kanan v2", "DIY", 100);

// ---------- KONFIGURASI FILTER ANALOG ----------
const int   OVERSAMPLE_COUNT   = 8;
const float EMA_ALPHA          = 0.15f;
const int   DEADZONE_RADIUS    = 6;
const int   ADC_MAX            = 4095;

// ---------- STATE SYSTEM ----------
float emaX = 0, emaY = 0;
int centerX = 2048, centerY = 2048;

int currentState = 0;
bool isMacroActive = false;

// Variable untuk Delay Non-Blocking State 3 ke 4 & Timeout 7 detik
unsigned long state3StartTime = 0;
bool waitingForComplete = false;
unsigned long state2StartTime = 0; // Timer window 7 detik sensor Kanan
const unsigned long RIGHT_HCSR_WINDOW_MS = 7000;

// ---------- ESP-NOW CALLBACKS ----------
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  // Callback status pengiriman
}

void sendStateToLeft(int state) {
  stateData.currentState = state;
  // Kirim 2x berturut-turut dengan jeda 5ms untuk menembus bentrokan traffic BLE
  esp_now_send(leftMacAddress, (uint8_t *) &stateData, sizeof(stateData));
  delay(5);
  esp_now_send(leftMacAddress, (uint8_t *) &stateData, sizeof(stateData));
}

void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len) {
  SystemState recvState;
  memcpy(&recvState, incomingData, sizeof(recvState));
  
  currentState = recvState.currentState;

  if (currentState == 1) {
    // Sinyal Driver Aktif dari Kiri -> Putar 0001.mp3
    myDFPlayer.play(1); 
    isMacroActive = false;
    waitingForComplete = false;
    Serial.println("[RIGHT] State 1 -> Play 0001.mp3");
  } 
  else if (currentState == 2) {
    // Sinyal Sensor Kiri Terpemicu -> Putar 0002.mp3 & Mulai Timer 7s Kanan
    myDFPlayer.play(2);
    state2StartTime = millis(); // Catat waktu sensor Kanan mulai aktif
    waitingForComplete = false;
    Serial.println("[RIGHT] State 2 -> Play 0002.mp3 (Sensor Kanan Ready 7s)");
  } 
  else if (currentState == 0) {
    // Reset State -> Stop Audio & Kembali ke Normal Mode
    myDFPlayer.stop();
    isMacroActive = false;
    waitingForComplete = false;
    Serial.println("[RIGHT] Reset State -> Stop Audio & Normal Mode");
  }
}

// ---------- FUNGSI ANALOG & ULTRASONIK ----------
long measureDistanceRight() {
  digitalWrite(PIN_TRIG_RIGHT, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG_RIGHT, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG_RIGHT, LOW);

  long duration = pulseIn(PIN_ECHO_RIGHT, HIGH, 6000); 
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

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);

  // Inisialisasi Hardware Serial untuk DFPlayer
  dfpSerial.begin(9600, SERIAL_8N1, PIN_DFP_RX, PIN_DFP_TX);
  if (myDFPlayer.begin(dfpSerial)) {
    myDFPlayer.volume(15); // Volume (0 - 30)
    Serial.println("DFPlayer Mini Terhubung.");
  } else {
    Serial.println("DFPlayer Mini Gagal Ditemukan!");
  }

  // Pin Tombol
  pinMode(PIN_SW,        INPUT_PULLUP);
  pinMode(PIN_BTN_B,     INPUT_PULLUP);
  pinMode(PIN_BTN_X,     INPUT_PULLUP);
  pinMode(PIN_BTN_RIGHT, INPUT_PULLUP);
  pinMode(PIN_BTN_LEFT,  INPUT_PULLUP);
  pinMode(PIN_BTN_RB,    INPUT_PULLUP);
  pinMode(PIN_BTN_RT,    INPUT_PULLUP);

  // Pin Sensor Ultrasonik
  pinMode(PIN_TRIG_RIGHT, OUTPUT);
  pinMode(PIN_ECHO_RIGHT, INPUT);
  digitalWrite(PIN_TRIG_RIGHT, LOW);

  // ESP-NOW Setup
  WiFi.mode(WIFI_STA);
  if (esp_now_init() == ESP_OK) {
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, leftMacAddress, 6);
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
  cfg.setButtonCount(7);
  cfg.setHatSwitchCount(0);
  cfg.setWhichAxes(true, true, false, false, false, false, false, false);
  bleGamepad.begin(&cfg);
}

// ---------- LOOP ----------
void loop() {
  // 1. Cek Sensor Ultrasonik Kanan (Hanya aktif jika State 2 tercapai)
  if (currentState == 2) {
    // Pengecekan Timeout 7 detik sensor Kanan
    if (millis() - state2StartTime > RIGHT_HCSR_WINDOW_MS) {
      Serial.println("[RIGHT TIMEOUT 7s] Tidak ada trigger Kanan -> Kirim Reset ke Kiri");
      currentState = 0;
      myDFPlayer.stop();
      sendStateToLeft(0); // Sinyal reset balik ke Kiri
    } else {
      static unsigned long lastCheckTime = 0;
      if (millis() - lastCheckTime > 60) {
        lastCheckTime = millis();
        long dist = measureDistanceRight();

        if (dist > 0 && dist <= TRIGGER_DISTANCE_CM) {
          currentState = 3;
          state3StartTime = millis();
          waitingForComplete = true;
          
          Serial.printf("[RIGHT] Sensor Kanan Trigger (%ld cm) -> Play 0003.mp3\n", dist);
          myDFPlayer.play(3); // Putar 0003.mp3
        }
      }
    }
  }

  // 1b. Transisi Otomatis State 3 -> State 4 (Henshin Complete)
  if (waitingForComplete && (millis() - state3StartTime >= 1200)) {
    waitingForComplete = false;
    currentState = 4;
    isMacroActive = true; // Kunci Mode Makro secara Permanen
    
    myDFPlayer.play(4); // Putar 0004.mp3 (Henshin Complete)
    Serial.println("[RIGHT] Henshin Complete! -> Play 0004.mp3 & Aktifkan Mode Makro PERMANEN");

    // Kirim sinyal State 4 ke Joy-Con Kiri
    sendStateToLeft(4);
  }

  // 2. Baca Input BLE Gamepad
  if (bleGamepad.isConnected()) {
    int rawX = readAveraged(PIN_VRX);
    int rawY = readAveraged(PIN_VRY);

    int8_t gx = processAxis(rawX, centerX, emaX);
    int8_t gy = processAxis(rawY, centerY, emaY);

    bleGamepad.setAxes(gx, gy, 0, 0, 0, 0, 0, 0);

    if (!isMacroActive) {
      // --- MODE NORMAL ---
      if (digitalRead(PIN_SW)        == LOW) bleGamepad.press(BUTTON_1); else bleGamepad.release(BUTTON_1);
      if (digitalRead(PIN_BTN_B)     == LOW) bleGamepad.press(BUTTON_2); else bleGamepad.release(BUTTON_2);
      if (digitalRead(PIN_BTN_X)     == LOW) bleGamepad.press(BUTTON_3); else bleGamepad.release(BUTTON_3);
      if (digitalRead(PIN_BTN_RIGHT) == LOW) bleGamepad.press(BUTTON_4); else bleGamepad.release(BUTTON_4);
      if (digitalRead(PIN_BTN_LEFT)  == LOW) bleGamepad.press(BUTTON_5); else bleGamepad.release(BUTTON_5);
      if (digitalRead(PIN_BTN_RB)    == LOW) bleGamepad.press(BUTTON_6); else bleGamepad.release(BUTTON_6);
      if (digitalRead(PIN_BTN_RT)    == LOW) bleGamepad.press(BUTTON_7); else bleGamepad.release(BUTTON_7);
    } else {
      // --- MODE MAKRO PERMANEN (Setelah Henshin Complete) ---
      if (digitalRead(PIN_BTN_B) == LOW) {
        bleGamepad.press(BUTTON_2);
        bleGamepad.press(BUTTON_7);
      } else {
        bleGamepad.release(BUTTON_2);
        bleGamepad.release(BUTTON_7);
      }

      if (digitalRead(PIN_SW)        == LOW) bleGamepad.press(BUTTON_1); else bleGamepad.release(BUTTON_1);
      if (digitalRead(PIN_BTN_X)     == LOW) bleGamepad.press(BUTTON_3); else bleGamepad.release(BUTTON_3);
      if (digitalRead(PIN_BTN_RIGHT) == LOW) bleGamepad.press(BUTTON_4); else bleGamepad.release(BUTTON_4);
      if (digitalRead(PIN_BTN_LEFT)  == LOW) bleGamepad.press(BUTTON_5); else bleGamepad.release(BUTTON_5);
      if (digitalRead(PIN_BTN_RB)    == LOW) bleGamepad.press(BUTTON_6); else bleGamepad.release(BUTTON_6);
      if (digitalRead(PIN_BTN_RT)    == LOW) bleGamepad.press(BUTTON_7); else bleGamepad.release(BUTTON_7);
    }

    bleGamepad.sendReport();
  }

  delay(8);
}
