/*
  Antweight BLE remote - ESP32-C3 + joystick + button
  GATT client talking to robot-bp32 / web-app protocol.

  Board: ESP32C3 Dev Module  |  Arduino-ESP32 core 3.x  |  no extra libs

  Wiring:
    Joystick X  -> GPIO4 (ADC1)  // forward / back on this module
    Joystick Y  -> GPIO3 (ADC1)  // left / right turn
    Button      -> GPIO2 to GND (weapon toggle / pairing)
    Joystick VCC-> 3V3, GND -> GND
    LED         -> LED_BUILTIN (GPIO8 on most C3 devkits, active-low)

  BUTTON:
    short press  -> toggle weapon on/off
    hold 3 s     -> PAIR: LED blinks fast, scan for robot, strongest RSSI

  Protocol (same as web app):
    "ML%d MR%d MW%d SL%d SR%d"  write-without-response
  Service 99b96fd7-dd0e-49cd-b255-f7b692c3de5e
  Char    4fce1dff-9151-498f-aa72-581f3f9241f3

  Last robot address is stored in NVS; power-up reconnects automatically.
  Note: GPIO2 is a strapping pin — pair is runtime long-press only.

  BLE connect/scan block for seconds — they run on a worker task so loop()
  keeps polling the button and printing serial info.
*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>
#include <BLERemoteCharacteristic.h>
#include <Preferences.h>

#define PIN_JOY_X 4
#define PIN_JOY_Y 3
#define PIN_BTN 2
// ESP32-C3 DevKit "LED_BUILTIN" is a WS2812 on GPIO8 (via RGB_BUILTIN), not a
// plain GPIO — must use rgbLedWrite(), not digitalWrite().

#define ROBOT_SERVICE_UUID "99b96fd7-dd0e-49cd-b255-f7b692c3de5e"
#define ROBOT_CMD_UUID "4fce1dff-9151-498f-aa72-581f3f9241f3"

#define SEND_INTERVAL_MS 20  // ~50 Hz
#define INFO_INTERVAL_MS 500
#define ADC_MAX 4095
#define DEADZONE 200
#define WEAPON_SPEED 255
#define PAIR_HOLD_MS 3000
#define SCAN_SECONDS 4
#define RECONNECT_MS 4000
#define CONNECT_TIMEOUT_MS 2000
#define PAIR_BLINK_MS 50

// Prefer snappy link (1.25 ms units): 7.5–15 ms, latency 0, timeout 4 s
#define CONN_INTERVAL_MIN 6
#define CONN_INTERVAL_MAX 12
#define CONN_LATENCY 0
#define CONN_TIMEOUT 400

// Stick orientation tweaks
// #define INVERT_DRIVE 1
// #define INVERT_TURN 1
// #define ROBOT_NAME_FILTER "Anty"

#define LED_BRIGHT 40  // WS2812 full-scale is harsh on a desk

enum BleJob : uint8_t { JOB_NONE = 0, JOB_RECONNECT, JOB_PAIR };

static Preferences prefs;
static BLEClient* client = nullptr;
static BLERemoteCharacteristic* cmdChar = nullptr;
static BLEAddress* savedAddr = nullptr;
static uint8_t savedAddrType = 0;
static char savedAddrStr[18] = "-";

static int centerX = ADC_MAX / 2, centerY = ADC_MAX / 2;
static volatile bool weaponOn = false;

static bool prevBtn = HIGH;
static uint32_t pressStart = 0;
static bool holdHandled = false;

static volatile bool linkDrop = false;
static volatile bool pairingBlink = false;
static volatile BleJob bleJob = JOB_NONE;
static volatile bool bleBusy = false;
static volatile bool wantPair = false;  // sticky until worker accepts JOB_PAIR
static uint32_t lastReconnectTry = 0;

static bool isConnected();  // fwd

// USB serial monitors want CRLF; printf("...\n") alone stair-steps the cursor.
#define LOG(msg) Serial.print(F(msg "\r\n"))
#define LOGF(fmt, ...) Serial.printf(fmt "\r\n", ##__VA_ARGS__)

// ---- LED (WS2812 via Arduino RGB API) ---------------------------------------
static void ledRgb(uint8_t r, uint8_t g, uint8_t b) {
#ifdef RGB_BUILTIN
  rgbLedWrite(RGB_BUILTIN, r, g, b);
#else
  rgbLedWrite(8, r, g, b);
#endif
}

static void ledOff() { ledRgb(0, 0, 0); }

static void ledInit() { ledOff(); }

// Linked=green, reconnect busy=dim amber, else off. Pairing uses its own blink task.
static void ledShowStatus() {
  if (pairingBlink) return;
  if (isConnected()) {
    ledRgb(0, LED_BRIGHT / 2, 0);
  } else if (bleBusy) {
    ledRgb(LED_BRIGHT / 2, LED_BRIGHT / 4, 0);  // amber while connect/scan
  } else {
    ledOff();
  }
}

static void pairingBlinkTask(void* /*arg*/) {
  while (pairingBlink) {
    ledRgb(LED_BRIGHT, LED_BRIGHT, LED_BRIGHT);  // white flash
    vTaskDelay(pdMS_TO_TICKS(PAIR_BLINK_MS));
    ledOff();
    vTaskDelay(pdMS_TO_TICKS(PAIR_BLINK_MS));
  }
  vTaskDelete(nullptr);
}

static void pairingBlinkStart() {
  if (pairingBlink) return;
  pairingBlink = true;
  xTaskCreate(pairingBlinkTask, "pairled", 2048, nullptr, 1, nullptr);
}

static void pairingBlinkStop() {
  pairingBlink = false;
  vTaskDelay(pdMS_TO_TICKS(PAIR_BLINK_MS * 3));
  ledOff();
}

// ---- helpers ----------------------------------------------------------------
static int axisToSpeed(int raw, int center) {
  int d = raw - center;
  if (d > -DEADZONE && d < DEADZONE) return 0;
  if (d > 0) d -= DEADZONE;
  else d += DEADZONE;
  int span = (ADC_MAX / 2) - DEADZONE;
  if (span < 1) span = 1;
  return constrain((int)((long)d * 255 / span), -255, 255);
}

static bool isConnected() {
  return client && client->isConnected() && cmdChar;
}

static void clearLink() {
  cmdChar = nullptr;
  if (client) {
    if (client->isConnected()) client->disconnect();
  }
}

static void rememberAddr(BLEAddress addr, uint8_t type) {
  String s = addr.toString();
  prefs.putString("addr", s);
  prefs.putUChar("atype", type);
  if (savedAddr) delete savedAddr;
  savedAddr = new BLEAddress(addr);
  savedAddrType = type;
  strncpy(savedAddrStr, s.c_str(), sizeof(savedAddrStr) - 1);
  savedAddrStr[sizeof(savedAddrStr) - 1] = 0;
}

class ClientCbs : public BLEClientCallbacks {
  void onConnect(BLEClient* c) override {
    LOG("BLE connected");
    c->updateConnParams(CONN_INTERVAL_MIN, CONN_INTERVAL_MAX, CONN_LATENCY, CONN_TIMEOUT);
  }
  void onDisconnect(BLEClient* c) override {
    (void)c;
    LOG("BLE disconnected");
    cmdChar = nullptr;
    linkDrop = true;
  }
};

static ClientCbs clientCbs;

static bool attachCmdChar() {
  BLERemoteService* svc = client->getService(ROBOT_SERVICE_UUID);
  if (!svc) {
    LOG("service not found");
    return false;
  }
  cmdChar = svc->getCharacteristic(ROBOT_CMD_UUID);
  if (!cmdChar) {
    LOG("cmd char not found");
    return false;
  }
  if (!cmdChar->canWrite() && !cmdChar->canWriteNoResponse()) {
    LOG("cmd char not writable");
    cmdChar = nullptr;
    return false;
  }
  LOG("cmd char OK");
  return true;
}

static bool connectAddr(BLEAddress addr, uint8_t type) {
  clearLink();
  if (!client) {
    client = BLEDevice::createClient();
    client->setClientCallbacks(&clientCbs);
  }
  LOGF("Connecting %s type=%u ...", addr.toString().c_str(), (unsigned)type);
  if (!client->connect(addr, type, CONNECT_TIMEOUT_MS)) {
    LOG("connect failed");
    return false;
  }
  client->setMTU(185);
  if (!attachCmdChar()) {
    clearLink();
    return false;
  }
  rememberAddr(addr, type);
  LOG("Paired/connected");
  return true;
}

// Scan for robot service UUID; pick strongest RSSI (workshop multi-robot friendly).
static bool pairScanAndConnect() {
  LOG("Scanning for robot...");
  pairingBlinkStart();

  BLEScan* scan = BLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setInterval(80);
  scan->setWindow(60);
  BLEScanResults* results = scan->start(SCAN_SECONDS, false);

  BLEAdvertisedDevice* best = nullptr;
  int bestRssi = -127;
  int n = results->getCount();
  for (int i = 0; i < n; i++) {
    BLEAdvertisedDevice d = results->getDevice(i);
    if (!d.isAdvertisingService(BLEUUID(ROBOT_SERVICE_UUID))) continue;
#ifdef ROBOT_NAME_FILTER
    if (!d.haveName() || d.getName() != ROBOT_NAME_FILTER) continue;
#endif
    int rssi = d.getRSSI();
    LOGF("  cand %s  rssi=%d  name='%s'", d.getAddress().toString().c_str(), rssi,
         d.haveName() ? d.getName().c_str() : "");
    if (rssi > bestRssi) {
      bestRssi = rssi;
      static BLEAdvertisedDevice bestDev;
      bestDev = d;
      best = &bestDev;
    }
  }
  scan->clearResults();

  bool ok = false;
  if (!best) {
    LOG("No robot found");
  } else {
    LOGF("Picking %s rssi=%d", best->getAddress().toString().c_str(), bestRssi);
    ok = connectAddr(best->getAddress(), best->getAddressType());
  }

  pairingBlinkStop();
  return ok;
}

static bool tryReconnectSaved() {
  if (!savedAddr) return false;
  return connectAddr(*savedAddr, savedAddrType);
}

// Blocking BLE lives here — loop() only schedules jobs via bleJob.
static void bleWorker(void* /*arg*/) {
  for (;;) {
    BleJob job = bleJob;
    if (job == JOB_NONE) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }
    bleJob = JOB_NONE;
    bleBusy = true;
    if (job == JOB_RECONNECT) {
      // Pair wins: drop a queued/started reconnect if user asked to pair.
      if (wantPair) {
        LOG("reconnect skipped — pair pending");
      } else {
        LOG("Reconnect try...");
        tryReconnectSaved();
      }
    } else if (job == JOB_PAIR) {
      weaponOn = false;
      clearLink();
      pairScanAndConnect();
    }
    bleBusy = false;
    ledShowStatus();
  }
}

static bool scheduleBleJob(BleJob job) {
  if (bleBusy || bleJob != JOB_NONE) return false;
  bleJob = job;
  return true;
}

static void handleButton(uint32_t now) {
  bool b = digitalRead(PIN_BTN);
  if (b == LOW && prevBtn == HIGH) {
    pressStart = now;
    holdHandled = false;
    LOG("btn down");
  }
  if (b == LOW && !holdHandled) {
    uint32_t held = now - pressStart;
    if (held >= PAIR_HOLD_MS) {
      holdHandled = true;
      wantPair = true;  // sticky — loop schedules when worker is free
      LOGF("btn hold %ums — pair/scan requested", (unsigned)held);
    }
  }
  if (b == HIGH && prevBtn == LOW) {
    uint32_t held = now - pressStart;
    LOGF("btn up held=%ums", (unsigned)held);
    if (!holdHandled && held >= 30) {
      weaponOn = !weaponOn;
      LOGF("Weapon %s", weaponOn ? "ON" : "OFF");
    }
  }
  prevBtn = b;
}

// Map sticks → motor mix. rawX/rawY optional out for info print.
static void readMix(int* left, int* right, int* weapon, int* rawX, int* rawY) {
  int rx = analogRead(PIN_JOY_X);
  int ry = analogRead(PIN_JOY_Y);
  if (rawX) *rawX = rx;
  if (rawY) *rawY = ry;

  // GPIO4 (X pot) = forward/back, GPIO3 (Y pot) = turn.
  int drive = axisToSpeed(rx, centerX);
  int turn = axisToSpeed(ry, centerY);
#ifdef INVERT_DRIVE
  drive = -drive;
#endif
#ifdef INVERT_TURN
  turn = -turn;
#endif
  *left = constrain(drive - turn, -255, 255);
  *right = constrain(drive + turn, -255, 255);
  *weapon = weaponOn ? WEAPON_SPEED : 0;
}

static void sendControl() {
  if (!isConnected()) return;

  int left, right, weapon;
  readMix(&left, &right, &weapon, nullptr, nullptr);

  char buf[48];
  int n = snprintf(buf, sizeof(buf), "ML%d MR%d MW%d SL%d SR%d", left, right, weapon, 0, 0);
  if (n <= 0) return;

  cmdChar->writeValue((uint8_t*)buf, (size_t)n, /*response=*/false);
}

static void printInfo() {
  int left, right, weapon, rawX, rawY;
  readMix(&left, &right, &weapon, &rawX, &rawY);
  int btn = digitalRead(PIN_BTN) == LOW ? 1 : 0;
  LOGF("link=%d busy=%d pair=%d L=%d R=%d W=%d  rawX=%d rawY=%d  ctr=%d,%d  btn=%d  robot=%s type=%u",
       isConnected() ? 1 : 0, bleBusy ? 1 : 0, wantPair ? 1 : 0, left, right, weapon, rawX, rawY,
       centerX, centerY, btn, savedAddrStr, (unsigned)savedAddrType);
}

// ---- setup / loop -----------------------------------------------------------
void setup() {
  Serial.begin(115200);
  // USB CDC re-enumerates after reset; wait so boot lines aren't lost on the host.
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {
    delay(10);
  }
  delay(200);
  LOG("");
  LOG("=== AntyRemote boot ===");

  pinMode(PIN_BTN, INPUT_PULLUP);
  ledInit();
  analogReadResolution(12);

  long sx = 0, sy = 0;
  for (int i = 0; i < 64; i++) {
    sx += analogRead(PIN_JOY_X);
    sy += analogRead(PIN_JOY_Y);
    delay(2);
  }
  centerX = sx / 64;
  centerY = sy / 64;
  LOGF("Joy center X=%d Y=%d", centerX, centerY);

  BLEDevice::init("AntyRemote");
  prefs.begin("bind", false);
  String a = prefs.getString("addr", "");
  savedAddrType = prefs.getUChar("atype", 0);
  if (a.length()) {
    savedAddr = new BLEAddress(a.c_str());
    strncpy(savedAddrStr, a.c_str(), sizeof(savedAddrStr) - 1);
    savedAddrStr[sizeof(savedAddrStr) - 1] = 0;
    LOGF("Saved robot %s type=%u", savedAddrStr, (unsigned)savedAddrType);
  } else {
    LOG("No saved robot in NVS");
  }

  xTaskCreate(bleWorker, "ble", 8192, nullptr, 1, nullptr);

  LOG("Remote ready (BLE). Short=weapon, hold 3s=pair");
  LOGF("Pins joyX=%d joyY=%d btn=%d  send=%dms info=%dms", PIN_JOY_X, PIN_JOY_Y, PIN_BTN,
       SEND_INTERVAL_MS, INFO_INTERVAL_MS);

  if (savedAddr) {
    scheduleBleJob(JOB_RECONNECT);
  } else {
    LOG("Not paired — hold button 3s next to robot");
  }
}

void loop() {
  uint32_t now = millis();
  handleButton(now);

  if (linkDrop) {
    linkDrop = false;
    cmdChar = nullptr;
  }

  // Pair is sticky and outranks auto-reconnect.
  if (wantPair && scheduleBleJob(JOB_PAIR)) {
    wantPair = false;
    LOG("pair scheduled");
  }

  static uint32_t lastInfo = 0;
  if (now - lastInfo >= INFO_INTERVAL_MS) {
    lastInfo = now;
    printInfo();
  }

  if (!isConnected()) {
    bool btnDown = digitalRead(PIN_BTN) == LOW;
    if (savedAddr && !btnDown && !wantPair && !bleBusy &&
        (now - lastReconnectTry) >= RECONNECT_MS) {
      lastReconnectTry = now;
      scheduleBleJob(JOB_RECONNECT);
    }
  } else {
    static uint32_t lastSend = 0;
    if (now - lastSend >= SEND_INTERVAL_MS) {
      lastSend = now;
      sendControl();
    }
  }

  static uint32_t lastLed = 0;
  if (now - lastLed >= 100) {
    lastLed = now;
    ledShowStatus();
  }

  delay(5);
}
