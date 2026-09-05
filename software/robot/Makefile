ENV ?= esp32dev
NAME ?= Anty
PIO ?= $(HOME)/.platformio/penv/bin/python -m platformio

build: deps
	sed -i 's/^#define ROBOT_NAME.*/#define ROBOT_NAME "${NAME}"/g' main/sketch.cpp
	sed -i 's/^#define ROBOT_NAME.*/#define ROBOT_NAME "${NAME}"/g' main/robot_gatt.h
	$(PIO) run -e $(ENV)

upload: deps
	$(PIO) run -e $(ENV) -t upload

serial:
	$(PIO) device monitor -b 115200

bu: build upload

clean:
	$(PIO) run -e $(ENV) -t clean

deps:
	./scripts/setup_deps.sh

