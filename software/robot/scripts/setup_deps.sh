#!/usr/bin/env bash
# Fetch components not stored in git (arduino-esp32 is large).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ARDUINO_REV=ac961f671abd5ae1da0a15fd4bee71ed807c2cf3

if [[ ! -f "$ROOT/components/arduino/CMakeLists.txt" ]]; then
  echo "Cloning arduino-esp32 @ $ARDUINO_REV ..."
  rm -rf "$ROOT/components/arduino"
  git clone --depth 1 https://github.com/espressif/arduino-esp32.git "$ROOT/components/arduino"
  git -C "$ROOT/components/arduino" fetch --depth 1 origin "$ARDUINO_REV"
  git -C "$ROOT/components/arduino" checkout "$ARDUINO_REV"
  rm -rf "$ROOT/components/arduino/.git"
fi

if [[ ! -f "$ROOT/components/ESP32Servo/src/ESP32Servo.h" ]]; then
  echo "Cloning ESP32Servo ..."
  rm -rf "$ROOT/components/ESP32Servo"
  git clone --depth 1 https://github.com/madhephaestus/ESP32Servo.git "$ROOT/components/ESP32Servo"
  rm -rf "$ROOT/components/ESP32Servo/.git"
  cat > "$ROOT/components/ESP32Servo/CMakeLists.txt" <<'CMAKE'
idf_component_register(SRC_DIRS "src" INCLUDE_DIRS "src" REQUIRES arduino)
CMAKE
fi

if [[ ! -f "$ROOT/components/Adafruit_NeoPixel/Adafruit_NeoPixel.h" ]]; then
  echo "Cloning Adafruit_NeoPixel ..."
  rm -rf "$ROOT/components/Adafruit_NeoPixel"
  git clone --depth 1 https://github.com/adafruit/Adafruit_NeoPixel.git "$ROOT/components/Adafruit_NeoPixel"
  rm -rf "$ROOT/components/Adafruit_NeoPixel/.git"
  cat > "$ROOT/components/Adafruit_NeoPixel/CMakeLists.txt" <<'CMAKE'
idf_component_register(SRCS "Adafruit_NeoPixel.cpp" "esp.c" INCLUDE_DIRS "." REQUIRES arduino)
CMAKE
fi

# PIO/esptool on this host needs click<8.2; ensure penv is sane.
if [[ -x "$HOME/.platformio/penv/bin/python" ]]; then
  command -v uv >/dev/null && uv pip install --python "$HOME/.platformio/penv/bin/python" 'click==8.1.7' pip wheel >/dev/null || true
fi

echo "Deps OK."
