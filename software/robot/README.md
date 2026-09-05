# Anty — Bluepad32 + Web BLE spike

One firmware, classic ESP32:

| Input | How |
|---|---|
| Web app / phone | BLE GATT write (same UUIDs as `robot2024`) |
| C3 joystick remote | same GATT client |
| DualSense / DS4 / Switch / … | Bluepad32 (BTstack HID host) |

Protocol:

```
ML<left> MR<right> MW<weapon> SL<servoL> SR<servoR>
```

- Service `99b96fd7-dd0e-49cd-b255-f7b692c3de5e`
- Char    `4fce1dff-9151-498f-aa72-581f3f9241f3`

## Build (PlatformIO + ESP-IDF)

```sh
make deps                   # once: arduino-esp32 + ESP32Servo
make build ROBOT_NAME=Anty  # env=esp32dev
make upload serial
```

`make build ENV=esp32-c3-devkitc-02` — compiles, but **no BT Classic** (no DualSense/DS4/Switch).

Default pins: `PINS_NTA_2026`. Other boards:

```sh
pio run -e esp32dev --project-option='build_flags=-DPINS_ANTY_ESPNOW'
```

## Layout

```
main/sketch.cpp     motors + BP32 gamepad map + failsafe
main/robot_gatt.c   BTstack ATT server + advertising (Anty + service UUID)
components/bluepad32* btstack  from BP32 template
```

BP32’s own config GATT stays **disabled**; we own advertising.
