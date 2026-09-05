set(srcs
        "main.c"
        "sketch.cpp"
        "robot_gatt.c")

set(requires "bluepad32" "bluepad32_arduino" "arduino" "btstack" "ESP32Servo" "Adafruit_NeoPixel")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "."
                    REQUIRES "${requires}")
