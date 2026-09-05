// Robot BLE GATT service (Web Bluetooth + C3 remote) on top of BTstack/Bluepad32.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Same UUIDs as robot2024 / web app.
#define ROBOT_GATT_SERVICE_UUID "99b96fd7-dd0e-49cd-b255-f7b692c3de5e"
#define ROBOT_GATT_CMD_UUID     "4fce1dff-9151-498f-aa72-581f3f9241f3"

#ifndef ROBOT_NAME
#define ROBOT_NAME "Anty"
#endif

typedef struct {
  int16_t left, right, weapon, servo_l, servo_r;
  uint32_t last_ms;
  bool valid;
} robot_cmd_t;

// Schedule GATT + advertising init on the BTstack thread (call once from setup()).
void robot_gatt_start(const char* name);

// Snapshot of last command written by a BLE client (phone / remote).
void robot_gatt_get_cmd(robot_cmd_t* out);

// True if a GATT client is currently connected.
bool robot_gatt_client_connected(void);

#ifdef __cplusplus
}
#endif
