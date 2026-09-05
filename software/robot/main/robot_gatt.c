// Custom ATT server: one writable characteristic, text protocol:
//   "ML%d MR%d MW%d SL%d SR%d"
// Coexists with Bluepad32 HID host (BP32 BLE config service must stay disabled).
//
// IMPORTANT: gap_advertisements_set_data() does NOT copy — buffers must be static.

#include "robot_gatt.h"

#include <stdio.h>
#include <string.h>

#include "btstack.h"
#include "ble/att_db.h"
#include "ble/att_db_util.h"
#include "ble/att_server.h"
#include "bluetooth_gatt.h"

#include "bt/uni_bt.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// LE General Discoverable | BR/EDR Not Supported (same as BP32 service adv).
// The BR/EDR-not-supported AD flag only affects LE scanners; classic pads still work.
#define ADV_FLAGS 0x06

// att_db_util_* reverse_128() internally — pass UUID **string/big-endian** order.
// Advertising packets need **little-endian** wire order (see k*UuidLe below).
// 99b96fd7-dd0e-49cd-b255-f7b692c3de5e
static const uint8_t kServiceUuidBe[16] = {
    0x99, 0xb9, 0x6f, 0xd7, 0xdd, 0x0e, 0x49, 0xcd,
    0xb2, 0x55, 0xf7, 0xb6, 0x92, 0xc3, 0xde, 0x5e,
};
// 4fce1dff-9151-498f-aa72-581f3f9241f3
static const uint8_t kCmdUuidBe[16] = {
    0x4f, 0xce, 0x1d, 0xff, 0x91, 0x51, 0x49, 0x8f,
    0xaa, 0x72, 0x58, 0x1f, 0x3f, 0x92, 0x41, 0xf3,
};
// LE wire order for AD type 0x07 complete 128-bit service list
static const uint8_t kServiceUuidLe[16] = {
    0x5e, 0xde, 0xc3, 0x92, 0xb6, 0xf7, 0x55, 0xb2,
    0xcd, 0x49, 0x0e, 0xdd, 0xd7, 0x6f, 0xb9, 0x99,
};

static uint16_t cmd_value_handle;
static char cmd_buf[48] = "ML0 MR0 MW0 SL0 SR0";
static char device_name[16] = ROBOT_NAME;

// Must outlive gap_advertisements_set_data / gap_scan_response_set_data.
static uint8_t adv_data[31];
static uint8_t adv_data_len;
static uint8_t scan_rsp[31];
static uint8_t scan_rsp_len;

static robot_cmd_t g_cmd;
static SemaphoreHandle_t g_cmd_mu;
static hci_con_handle_t g_client = HCI_CON_HANDLE_INVALID;
static hci_con_handle_t g_periph = HCI_CON_HANDLE_INVALID;  // slave link (phone/remote)
static bool g_client_is_writer;  // true only after a write to our char

static btstack_context_callback_registration_t g_start_reg;
static btstack_packet_callback_registration_t g_hci_event_cb;
static bool g_started;

// Prefer low-latency link for WebBLE joystick stream (units: 1.25 ms / 10 ms).
// 6..12 → 7.5..15 ms interval, 0 slave latency, 4 s supervision.
#define GATT_CONN_INTERVAL_MIN 6
#define GATT_CONN_INTERVAL_MAX 12
#define GATT_CONN_LATENCY 0
#define GATT_SUPERVISION_TIMEOUT 400

static uint32_t now_ms(void) {
  return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static void parse_cmd(const char* s, uint16_t len) {
  int ml = g_cmd.left, mr = g_cmd.right, mw = g_cmd.weapon;
  int sl = g_cmd.servo_l, sr = g_cmd.servo_r;

  char tmp[48];
  if (len >= sizeof(tmp)) len = sizeof(tmp) - 1;
  memcpy(tmp, s, len);
  tmp[len] = 0;

  if (sscanf(tmp, "ML%d MR%d MW%d SL%d SR%d", &ml, &mr, &mw, &sl, &sr) >= 2 ||
      sscanf(tmp, "ML%d MR%d SL%d SR%d", &ml, &mr, &sl, &sr) >= 2) {
    if (xSemaphoreTake(g_cmd_mu, pdMS_TO_TICKS(10)) == pdTRUE) {
      g_cmd.left = (int16_t)ml;
      g_cmd.right = (int16_t)mr;
      g_cmd.weapon = (int16_t)mw;
      g_cmd.servo_l = (int16_t)sl;
      g_cmd.servo_r = (int16_t)sr;
      g_cmd.last_ms = now_ms();
      g_cmd.valid = true;
      xSemaphoreGive(g_cmd_mu);
    }
  }
}

static uint16_t att_read_cb(hci_con_handle_t conn, uint16_t handle, uint16_t offset,
                            uint8_t* buffer, uint16_t buffer_size) {
  (void)conn;
  if (handle == cmd_value_handle) {
    return att_read_callback_handle_blob((const uint8_t*)cmd_buf, (uint16_t)strlen(cmd_buf),
                                         offset, buffer, buffer_size);
  }
  return 0;
}

static int att_write_cb(hci_con_handle_t conn, uint16_t handle, uint16_t transaction_mode,
                        uint16_t offset, uint8_t* buffer, uint16_t buffer_size) {
  (void)transaction_mode;
  if (handle != cmd_value_handle || offset != 0 || buffer_size == 0) {
    return ATT_ERROR_REQUEST_NOT_SUPPORTED;
  }
  // First write proves this connection is a real robot client (not a BP32 HID link).
  g_client = conn;
  g_client_is_writer = true;

  if (buffer_size >= sizeof(cmd_buf)) buffer_size = sizeof(cmd_buf) - 1;
  memcpy(cmd_buf, buffer, buffer_size);
  cmd_buf[buffer_size] = 0;
  parse_cmd(cmd_buf, buffer_size);
  return ATT_ERROR_SUCCESS;
}

static void failsafe_clear(void) {
  if (xSemaphoreTake(g_cmd_mu, pdMS_TO_TICKS(10)) == pdTRUE) {
    g_cmd.left = g_cmd.right = g_cmd.weapon = 0;
    g_cmd.valid = false;
    xSemaphoreGive(g_cmd_mu);
  }
}

static void restart_advertising(void) {
  // BTstack usually restarts connectable adv itself; be explicit after our client leaves.
  gap_advertisements_enable(1);
}

static void request_fast_conn_params(hci_con_handle_t h) {
  int rc = gap_request_connection_parameter_update(h, GATT_CONN_INTERVAL_MIN, GATT_CONN_INTERVAL_MAX,
                                                   GATT_CONN_LATENCY, GATT_SUPERVISION_TIMEOUT);
  printf("robot_gatt: request conn params %d..%d (1.25ms) latency=%d -> rc=%d\n",
         GATT_CONN_INTERVAL_MIN, GATT_CONN_INTERVAL_MAX, GATT_CONN_LATENCY, rc);
}

static void hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size) {
  (void)channel;
  (void)size;
  if (packet_type != HCI_EVENT_PACKET) return;
  uint8_t event = hci_event_packet_get_type(packet);
  if (event == HCI_EVENT_LE_META) {
    switch (hci_event_le_meta_get_subevent_code(packet)) {
      case HCI_SUBEVENT_LE_CONNECTION_COMPLETE: {
        hci_con_handle_t h = hci_subevent_le_connection_complete_get_connection_handle(packet);
        uint16_t interval = hci_subevent_le_connection_complete_get_conn_interval(packet);
        uint16_t latency = hci_subevent_le_connection_complete_get_conn_latency(packet);
        printf("robot_gatt: LE conn complete handle=%u interval=%u (%.2fms) latency=%u\n",
               (unsigned)h, (unsigned)interval, interval * 1.25f, (unsigned)latency);
        break;
      }
      case HCI_SUBEVENT_LE_CONNECTION_UPDATE_COMPLETE: {
        uint8_t status = hci_subevent_le_connection_update_complete_get_status(packet);
        hci_con_handle_t h = hci_subevent_le_connection_update_complete_get_connection_handle(packet);
        uint16_t interval = hci_subevent_le_connection_update_complete_get_conn_interval(packet);
        uint16_t latency = hci_subevent_le_connection_update_complete_get_conn_latency(packet);
        printf("robot_gatt: LE conn update handle=%u status=%u interval=%u (%.2fms) latency=%u\n",
               (unsigned)h, (unsigned)status, (unsigned)interval, interval * 1.25f, (unsigned)latency);
        break;
      }
      default:
        break;
    }
  }
}

static void att_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size) {
  (void)channel;
  (void)size;
  if (packet_type != HCI_EVENT_PACKET) return;
  switch (hci_event_packet_get_type(packet)) {
    case ATT_EVENT_CONNECTED: {
      hci_con_handle_t h = att_event_connected_get_handle(packet);
      hci_role_t role = gap_get_role(h);
      printf("robot_gatt: LE link up handle=%u role=%s\n", (unsigned)h,
             role == HCI_ROLE_SLAVE ? "slave/periph" : role == HCI_ROLE_MASTER ? "master" : "?");
      // Incoming GATT client (phone/remote): pause HID scanning so ESP32 radio
      // stays stable for the peripheral link, and ask master for a fast interval.
      if (role == HCI_ROLE_SLAVE) {
        g_periph = h;
        uni_bt_stop_scanning_unsafe();
        printf("robot_gatt: paused BP32 scan for GATT client\n");
        request_fast_conn_params(h);
      }
      break;
    }
    case ATT_EVENT_DISCONNECTED: {
      hci_con_handle_t h = att_event_disconnected_get_handle(packet);
      printf("robot_gatt: LE link down handle=%u\n", (unsigned)h);
      if (h == g_periph) g_periph = HCI_CON_HANDLE_INVALID;
      if (g_client_is_writer && h == g_client) {
        g_client = HCI_CON_HANDLE_INVALID;
        g_client_is_writer = false;
        failsafe_clear();
        printf("robot_gatt: robot client gone — failsafe\n");
      }
      // Resume gamepad discovery after any peripheral session ends.
      if (g_periph == HCI_CON_HANDLE_INVALID) {
        uni_bt_start_scanning_and_autoconnect_unsafe();
        restart_advertising();
      }
      break;
    }
    default:
      break;
  }
}

static void build_adv_and_scan_rsp(const char* name) {
  // Adv: flags + 128-bit service UUID (required by web-app filters).
  // Name goes in scan response so it always fits and shows in scanners.
  uint8_t i = 0;
  adv_data[i++] = 2;
  adv_data[i++] = BLUETOOTH_DATA_TYPE_FLAGS;
  adv_data[i++] = ADV_FLAGS;

  adv_data[i++] = 17;
  adv_data[i++] = BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS;
  memcpy(&adv_data[i], kServiceUuidLe, 16);
  i = (uint8_t)(i + 16);
  adv_data_len = i;

  size_t nlen = strlen(name);
  if (nlen > 29) nlen = 29;
  scan_rsp[0] = (uint8_t)(1 + nlen);
  scan_rsp[1] = BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME;
  memcpy(&scan_rsp[2], name, nlen);
  scan_rsp_len = (uint8_t)(2 + nlen);
}

static void gatt_init_on_btstack(void* context) {
  (void)context;
  if (g_started) return;

  printf("robot_gatt: init ATT DB name=%s\n", device_name);

  att_db_util_init();

  att_db_util_add_service_uuid16(ORG_BLUETOOTH_SERVICE_GENERIC_ACCESS);
  att_db_util_add_characteristic_uuid16(
      ORG_BLUETOOTH_CHARACTERISTIC_GAP_DEVICE_NAME, ATT_PROPERTY_READ,
      ATT_SECURITY_NONE, ATT_SECURITY_NONE, (uint8_t*)device_name, (uint16_t)strlen(device_name));

  att_db_util_add_service_uuid16(ORG_BLUETOOTH_SERVICE_GENERIC_ATTRIBUTE);

  att_db_util_add_service_uuid128(kServiceUuidBe);
  // DYNAMIC so read/write go through att_*_cb (not only static cmd_buf).
  cmd_value_handle = att_db_util_add_characteristic_uuid128(
      kCmdUuidBe,
      ATT_PROPERTY_READ | ATT_PROPERTY_WRITE | ATT_PROPERTY_WRITE_WITHOUT_RESPONSE |
          ATT_PROPERTY_DYNAMIC,
      ATT_SECURITY_NONE, ATT_SECURITY_NONE, NULL, 0);

  printf("robot_gatt: cmd handle=0x%04x db_size=%u\n", cmd_value_handle,
         (unsigned)att_db_util_get_size());

  att_server_init(att_db_util_get_address(), att_read_cb, att_write_cb);
  att_server_register_packet_handler(att_packet_handler);

  g_hci_event_cb.callback = &hci_packet_handler;
  hci_add_event_handler(&g_hci_event_cb);

  build_adv_and_scan_rsp(device_name);

  // Helpful for stacks that read GAP local name separately.
  gap_set_local_name(device_name);
  gap_set_max_number_peripheral_connections(2);

  // Accept low-latency params if the phone proposes them (and when we request).
  le_connection_parameter_range_t range = {
      .le_conn_interval_min = 6,     // 7.5 ms
      .le_conn_interval_max = 40,    // 50 ms
      .le_conn_latency_min = 0,
      .le_conn_latency_max = 4,
      .le_supervision_timeout_min = 100,   // 1 s
      .le_supervision_timeout_max = 1000,  // 10 s
  };
  gap_set_connection_parameter_range(&range);

  uint16_t adv_int_min = 0x30;  // ~30 ms
  uint16_t adv_int_max = 0x30;
  uint8_t adv_type = 0;  // connectable undirected
  bd_addr_t null_addr = {0};
  gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);
  gap_advertisements_set_data(adv_data_len, adv_data);
  gap_scan_response_set_data(scan_rsp_len, scan_rsp);
  gap_advertisements_enable(1);

  g_started = true;
  printf("robot_gatt: advertising adv=%u scan_rsp=%u service UUID in adv\n",
         (unsigned)adv_data_len, (unsigned)scan_rsp_len);
}

void robot_gatt_start(const char* name) {
  if (!g_cmd_mu) g_cmd_mu = xSemaphoreCreateMutex();
  if (name && name[0]) {
    strncpy(device_name, name, sizeof(device_name) - 1);
    device_name[sizeof(device_name) - 1] = 0;
  }
  g_start_reg.callback = &gatt_init_on_btstack;
  g_start_reg.context = NULL;
  btstack_run_loop_execute_on_main_thread(&g_start_reg);
}

void robot_gatt_get_cmd(robot_cmd_t* out) {
  if (!out) return;
  if (!g_cmd_mu) {
    memset(out, 0, sizeof(*out));
    return;
  }
  if (xSemaphoreTake(g_cmd_mu, pdMS_TO_TICKS(5)) == pdTRUE) {
    *out = g_cmd;
    xSemaphoreGive(g_cmd_mu);
  }
}

bool robot_gatt_client_connected(void) {
  // True while a central (phone / C3 remote) holds a peripheral link to us.
  return g_periph != HCI_CON_HANDLE_INVALID;
}
