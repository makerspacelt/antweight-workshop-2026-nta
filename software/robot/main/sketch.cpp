// Bluepad32 gamepad host + Web-BLE / C3-remote GATT on one firmware.

#include "sdkconfig.h"

#include <Arduino.h>
#include <Bluepad32.h>
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>
#include <esp_console.h>

#include "robot_gatt.h"

// ---------------- pins -------------------------------------------------------
// #define PINS_NTA_2025 1
// #define PINS_KMS_2024 1
// #define PINS_KMS_2023 1
// #define PINS_NTA_2023 1
// Default: PINS_NTA_2026 (new board — weapon motor + single servo + LEDs)

#if defined(PINS_NTA_2025)
  #define PIN_MOT_R_IN1 13
  #define PIN_MOT_R_IN2 27
  #define PIN_MOT_L_IN1 26
  #define PIN_MOT_L_IN2 25
  #define PIN_SERVO_1 32
  #define PIN_SERVO_2 23
#elif defined(PINS_KMS_2024)
  #define PIN_MOT_R_IN1 27
  #define PIN_MOT_R_IN2 13
  #define PIN_MOT_L_IN1 26
  #define PIN_MOT_L_IN2 25
  #define PIN_SERVO_1 32
  #define PIN_SERVO_2 23
#elif defined(PINS_KMS_2023)
  #define PIN_MOT_R_IN1 26
  #define PIN_MOT_R_IN2 25
  #define PIN_MOT_L_IN1 13
  #define PIN_MOT_L_IN2 27
  #define PIN_SERVO_1 32
#elif defined(PINS_NTA_2023)
  #define PIN_MOT_R_IN1 19
  #define PIN_MOT_R_IN2 18
  #define PIN_MOT_L_IN1 17
  #define PIN_MOT_L_IN2 16
#else
  // PINS_NTA_2026 (default)
  #define PIN_MOT_R_IN1 26
  #define PIN_MOT_R_IN2 27
  #define PIN_MOT_L_IN1 16
  #define PIN_MOT_L_IN2 4
  #define PIN_MOT_W_IN1 17
  #define PIN_MOT_W_IN2 18
  #define PIN_SERVO_1 19
  #define PIN_VBAT 34
  #define PIN_LEDS 14
  #define NUM_LEDS 4
#endif

#ifndef ROBOT_NAME
#define ROBOT_NAME "Anty"
#endif

#define MOTOR_PWM_FREQ 20000
#define MOTOR_PWM_BIT_RES 8
#define CMD_TIMEOUT_MS 500
#define WEAPON_SLEW 12
#define LED_BRIGHTNESS 60
#define LED_UPDATE_MS 20

// Divider on NTA_2026: 100k+38k → scale pack voltage from ADC pin.
#ifdef PIN_VBAT
#define VBAT_DIVIDER (138.0f / 38.0f)
// Calibrated vs meter: firmware 8.06 V, meter 7.810 V → 7.810/8.06
#define VBAT_CAL 0.969f
#define VBAT_ABSENT_V 4.00f
#define VBAT_LOW_V 7.00f
#define VBAT_RECOVER_V 7.20f
#define VBAT_PRINT_MS 5000
#endif

// #define INVERT_LEFT 1
#define INVERT_RIGHT 1
// #define INVERT_WEAPON 1

// Exclusive control: first source to send a command owns the bot until failsafe.
enum ControlSrc : uint8_t { SRC_NONE = 0, SRC_GATT = 1, SRC_PAD = 2 };

static ControllerPtr myControllers[BP32_MAX_GAMEPADS];

static int cmd_left, cmd_right, cmd_weapon, cmd_servo1, cmd_servo2;
static uint32_t last_cmd_ms;
static int weapon_out;
static int servo1_last = -1, servo2_last = -1;
static ControlSrc active_src = SRC_NONE;

#ifdef PIN_SERVO_1
static Servo servo1;
#endif
#ifdef PIN_SERVO_2
static Servo servo2;
#endif

#ifdef PIN_LEDS
static Adafruit_NeoPixel leds(NUM_LEDS, PIN_LEDS, NEO_GRB + NEO_KHZ800);
#endif

#ifdef PIN_VBAT
static float vbat = 0.0f;
static bool battery_low = false;

static float read_vbat() {
  uint32_t mv = 0;
  for (int i = 0; i < 8; i++) mv += analogReadMilliVolts(PIN_VBAT);
  float v = (mv / 8.0f) / 1000.0f * VBAT_DIVIDER * VBAT_CAL;
  if (vbat <= 0.0f) vbat = v;
  else vbat = vbat * 0.8f + v * 0.2f;

  if (vbat < VBAT_ABSENT_V) battery_low = false;
  else if (vbat < VBAT_LOW_V) battery_low = true;
  else if (vbat > VBAT_RECOVER_V) battery_low = false;
  return vbat;
}

static int cmd_vbat(int argc, char** argv) {
  (void)argc;
  (void)argv;
  float v = read_vbat();
  if (v < VBAT_ABSENT_V) {
    Console.printf("vbat=%.2f V (absent / USB only?) low=%d\n", v, (int)battery_low);
  } else {
    Console.printf("vbat=%.2f V low=%d\n", v, (int)battery_low);
  }
  return 0;
}

static void register_vbat_cmd() {
  const esp_console_cmd_t cmd = {
      .command = "vbat",
      .help = "Read pack voltage (PIN_VBAT)",
      .hint = NULL,
      .func = &cmd_vbat,
      .argtable = NULL,
  };
  esp_console_cmd_register(&cmd);
}
#endif

static bool any_pad_connected() {
  for (auto c : myControllers) {
    if (c && c->isConnected()) return true;
  }
  return false;
}

static bool any_controller_present() {
  return robot_gatt_client_connected() || any_pad_connected();
}

// ---- motors -----------------------------------------------------------------
static void driveMotor(int pinA, int pinB, int spd) {
  spd = constrain(spd, -255, 255);
  if (spd >= 0) {
    ledcWrite(pinA, 255);
    ledcWrite(pinB, 255 - spd);
  } else {
    ledcWrite(pinB, 255);
    ledcWrite(pinA, 255 + spd);
  }
}

static void setLeft(int s) {
#ifdef INVERT_LEFT
  s = -s;
#endif
  driveMotor(PIN_MOT_L_IN1, PIN_MOT_L_IN2, s);
}
static void setRight(int s) {
#ifdef INVERT_RIGHT
  s = -s;
#endif
  driveMotor(PIN_MOT_R_IN1, PIN_MOT_R_IN2, s);
}
#ifdef PIN_MOT_W_IN1
static void setWeapon(int s) {
#ifdef INVERT_WEAPON
  s = -s;
#endif
  driveMotor(PIN_MOT_W_IN1, PIN_MOT_W_IN2, s);
}
#endif

// Returns false if another source owns control.
static bool try_apply_command(ControlSrc src, int l, int r, int w, int s1, int s2, uint32_t t) {
  if (active_src != SRC_NONE && active_src != src) return false;
  active_src = src;
  cmd_left = constrain(l, -255, 255);
  cmd_right = constrain(r, -255, 255);
  cmd_weapon = constrain(w, -255, 255);
  cmd_servo1 = constrain(s1, 0, 180);
  cmd_servo2 = constrain(s2, 0, 180);
  last_cmd_ms = t;
  return true;
}

// ---- LEDs -------------------------------------------------------------------
#ifdef PIN_LEDS
static void update_leds(uint32_t now, bool failsafe, int drive_l, int drive_r, int drive_w) {
  uint32_t color;
  const bool driving =
      !failsafe && (abs(drive_l) > 15 || abs(drive_r) > 15 || abs(drive_w) > 15);

#ifdef PIN_VBAT
  // Low pack: red blink, but only while motors are idle (2S + 2A draw sag).
  if (battery_low && !driving) {
    color = (now % 400 < 200) ? leds.Color(255, 30, 0) : leds.Color(50, 0, 0);
    for (int i = 0; i < NUM_LEDS; i++) leds.setPixelColor(i, color);
    leds.show();
    return;
  }
#endif

  if (!failsafe) {
    // Recent command from the active source
    color = leds.Color(0, 255, 0);
  } else if (any_controller_present()) {
    // Linked but idle — slow green breathe (still "have a remote", no packets)
    uint32_t p = now % 2400;
    uint8_t g = (p < 1200) ? (uint8_t)(p * 180 / 1200) : (uint8_t)((2400 - p) * 180 / 1200);
    if (g < 15) g = 15;
    color = leds.Color(0, g, 0);
  } else {
    // Nobody connected — blue breathe (waiting to pair/connect)
    uint32_t p = now % 1600;
    uint8_t b = (p < 800) ? (uint8_t)(p * 255 / 800) : (uint8_t)((1600 - p) * 255 / 800);
    if (b < 20) b = 20;
    color = leds.Color(0, 0, b);
  }

  for (int i = 0; i < NUM_LEDS; i++) leds.setPixelColor(i, color);
  leds.show();
}
#endif

// ---- Bluepad32 --------------------------------------------------------------
static void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Console.printf("Pad connected slot=%d model=%s\n", i, ctl->getModelName());
      myControllers[i] = ctl;
      return;
    }
  }
  Console.println("Pad connected but no slot");
}

static void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Console.printf("Pad disconnected slot=%d\n", i);
      myControllers[i] = nullptr;
      if (active_src == SRC_PAD) active_src = SRC_NONE;
      return;
    }
  }
}

// Arcade mix: full forward authority, softened steering.
// Turn: cubic expo (soft center) then * TURN_GAIN/512 (~40% at full stick).
#define TURN_GAIN 102

static void processGamepad(ControllerPtr ctl) {
  int x = ctl->axisX();
  int y = ctl->axisY();
  if (abs(x) < 30) x = 0;
  if (abs(y) < 30) y = 0;

  // axis ±512 → ±255
  int drive = constrain((int)((long)-y * 255 / 512), -255, 255);

  // Cubic expo on turn, then attenuate: t^3/512^2 keeps sign, |t|<1 soft.
  long t = constrain(x, -512, 512);
  long turn_exp = t * t * t / (512 * 512);  // ±512
  int turn = constrain((int)(turn_exp * TURN_GAIN / 512), -255, 255);

  int l = constrain(drive - turn, -255, 255);
  int r = constrain(drive + turn, -255, 255);

  int w = 0;
  if (ctl->throttle() > 50) {
    w = constrain((int)((long)ctl->throttle() * 255 / 1023), 0, 255);
  }
  if (ctl->b()) w = 255;

  int s1 = 90, s2 = 90;
  if (ctl->a()) s1 = 0;
  if (ctl->y()) s1 = 180;

  try_apply_command(SRC_PAD, l, r, w, s1, s2, millis());
}

static void processControllers() {
  for (auto c : myControllers) {
    if (c && c->isConnected() && c->hasData() && c->isGamepad()) {
      processGamepad(c);
    }
  }
}

// ---- setup / loop -----------------------------------------------------------
void setup() {
  Console.printf("Anty — BP32 + robot GATT\n");
  Console.printf("FW %s\n", BP32.firmwareVersion());

#ifdef PIN_LEDS
  leds.begin();
  leds.setBrightness(LED_BRIGHTNESS);
  leds.clear();
  leds.show();
#endif

#ifdef PIN_SERVO_1
  servo1.setPeriodHertz(50);
  servo1.attach(PIN_SERVO_1, 400, 2400);
#endif
#ifdef PIN_SERVO_2
  servo2.setPeriodHertz(50);
  servo2.attach(PIN_SERVO_2, 400, 2400);
#endif

  ledcAttach(PIN_MOT_R_IN1, MOTOR_PWM_FREQ, MOTOR_PWM_BIT_RES);
  ledcAttach(PIN_MOT_R_IN2, MOTOR_PWM_FREQ, MOTOR_PWM_BIT_RES);
  ledcAttach(PIN_MOT_L_IN1, MOTOR_PWM_FREQ, MOTOR_PWM_BIT_RES);
  ledcAttach(PIN_MOT_L_IN2, MOTOR_PWM_FREQ, MOTOR_PWM_BIT_RES);
#ifdef PIN_MOT_W_IN1
  ledcAttach(PIN_MOT_W_IN1, MOTOR_PWM_FREQ, MOTOR_PWM_BIT_RES);
  ledcAttach(PIN_MOT_W_IN2, MOTOR_PWM_FREQ, MOTOR_PWM_BIT_RES);
  setWeapon(0);
#endif
  setLeft(0);
  setRight(0);

#ifdef PIN_VBAT
  analogSetPinAttenuation(PIN_VBAT, ADC_11db);
  pinMode(PIN_VBAT, INPUT);
#endif

  BP32.setup(&onConnectedController, &onDisconnectedController, true);
  BP32.enableBLEService(false);
  BP32.enableVirtualDevice(false);

#ifdef PIN_VBAT
  register_vbat_cmd();
  read_vbat();
#endif

  robot_gatt_start(ROBOT_NAME);

  const uint8_t* addr = BP32.localBdAddress();
  Console.printf("BD %02X:%02X:%02X:%02X:%02X:%02X name=%s\n", addr[0], addr[1], addr[2],
                 addr[3], addr[4], addr[5], ROBOT_NAME);
  Console.println("Ready: exclusive control (GATT | pad), LEDs on");
#ifdef PIN_VBAT
  Console.println("Shell: type 'vbat'");
#endif
}

void loop() {
  const uint32_t now = millis();

  if (BP32.update()) {
    processControllers();
  }

  // GATT path (phone / C3 remote) — ignored while a pad owns control.
  robot_cmd_t gc;
  robot_gatt_get_cmd(&gc);
  if (gc.valid) {
    try_apply_command(SRC_GATT, gc.left, gc.right, gc.weapon, gc.servo_l, gc.servo_r, gc.last_ms);
  }

  // Drop owner after silence so the other method can take over cleanly.
  if (active_src != SRC_NONE && (now - last_cmd_ms) > CMD_TIMEOUT_MS) {
    active_src = SRC_NONE;
  }

  const bool failsafe = (active_src == SRC_NONE) || ((now - last_cmd_ms) > CMD_TIMEOUT_MS);
  int l = failsafe ? 0 : cmd_left;
  int r = failsafe ? 0 : cmd_right;
  int w = failsafe ? 0 : cmd_weapon;

  if (w > weapon_out) weapon_out = min(w, weapon_out + WEAPON_SLEW);
  else if (w < weapon_out) weapon_out = max(w, weapon_out - WEAPON_SLEW);

  setLeft(l);
  setRight(r);
#ifdef PIN_MOT_W_IN1
  setWeapon(weapon_out);
#else
  (void)weapon_out;
#endif

#ifdef PIN_SERVO_1
  if (!failsafe && cmd_servo1 != servo1_last) {
    servo1.write(cmd_servo1);
    servo1_last = cmd_servo1;
  }
#endif
#ifdef PIN_SERVO_2
  if (!failsafe && cmd_servo2 != servo2_last) {
    servo2.write(cmd_servo2);
    servo2_last = cmd_servo2;
  }
#endif

#ifdef PIN_VBAT
  static uint32_t last_vbat_ms = 0;
  if (now - last_vbat_ms >= 200) {
    last_vbat_ms = now;
    read_vbat();
  }
  static uint32_t last_vbat_print = 0;
  if (now - last_vbat_print >= VBAT_PRINT_MS) {
    last_vbat_print = now;
    if (vbat >= VBAT_ABSENT_V) Console.printf("vbat %.2f V%s\n", vbat, battery_low ? " LOW" : "");
  }
#endif

#ifdef PIN_LEDS
  static uint32_t last_led = 0;
  if (now - last_led >= LED_UPDATE_MS) {
    last_led = now;
    update_leds(now, failsafe, l, r, weapon_out);
  }
#endif

  delay(1);
}
