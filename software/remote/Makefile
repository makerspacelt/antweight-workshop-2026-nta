PORT ?= /dev/ttyACM0
# CDCOnBoot=cdc: Serial ↔ native USB (ttyACM). Default routes Serial to UART0 pins only.
BOARD ?= esp32:esp32:esp32c3:CDCOnBoot=cdc

build:
	arduino-cli compile -b $(BOARD)

upload: build
	arduino-cli upload -b $(BOARD) -p $(PORT)

serial:
	picocom -b 115200 $(PORT)

bu: upload serial

deps:
	arduino-cli core install esp32:esp32

