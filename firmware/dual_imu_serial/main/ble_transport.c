/* ESP-IDF NimBLE transport for synchronized AURORA IMU and EMG samples.
 *
 * The service and IMU characteristic UUIDs intentionally match LIMB-HT25 so
 * the existing Bleak tooling remains compatible.  A new time-sync
 * characteristic implements a four-timestamp exchange: the central sends t0,
 * the peripheral records t1/t2, and the central records t3 on notification.
 */

#include "ble_transport.h"

#include <string.h>

#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
#include "esp_nimble_hci.h"
#include "host/util/util.h"
#endif

#define DEVICE_NAME "LIMBServer"
#define SYNC_PROTOCOL_VERSION 1U
#define IMU_PACKET_VERSION 1U
#define EMG_PACKET_VERSION 1U

/* 23011525-1212-efde-1523-785feabcd122 */
static const ble_uuid128_t k_service_uuid = BLE_UUID128_INIT(
    0x22, 0xd1, 0xbc, 0xea, 0x5f, 0x78, 0x23, 0x15, 0xde, 0xef, 0x12, 0x12,
    0x25, 0x15, 0x01, 0x23);
/* 24011525-1212-efde-1523-785feabcd122 */
static const ble_uuid128_t k_emg_uuid = BLE_UUID128_INIT(
    0x22, 0xd1, 0xbc, 0xea, 0x5f, 0x78, 0x23, 0x15, 0xde, 0xef, 0x12, 0x12,
    0x25, 0x15, 0x01, 0x24);
/* 25011525-1212-efde-1523-785feabcd122 */
static const ble_uuid128_t k_imu_uuid = BLE_UUID128_INIT(
    0x22, 0xd1, 0xbc, 0xea, 0x5f, 0x78, 0x23, 0x15, 0xde, 0xef, 0x12, 0x12,
    0x25, 0x15, 0x01, 0x25);
/* 27011525-1212-efde-1523-785feabcd122 */
static const ble_uuid128_t k_time_sync_uuid = BLE_UUID128_INIT(
    0x22, 0xd1, 0xbc, 0xea, 0x5f, 0x78, 0x23, 0x15, 0xde, 0xef, 0x12, 0x12,
    0x25, 0x15, 0x01, 0x27);

typedef struct __attribute__((packed)) {
  /* High byte: protocol version. Low bits: shoulder/wrist connected mask. */
  uint16_t format_flags;
  uint32_t sequence;
  uint64_t device_time_us;
  /* shoulder accel/gyro followed by wrist accel/gyro, scaled by 1000. */
  int16_t values[12];
} imu_packet_t;

typedef struct __attribute__((packed)) {
  /* High byte: protocol version. Low bit: ADC sample valid. */
  uint16_t format_flags;
  uint32_t sequence;
  uint64_t device_time_us;
  uint16_t adc_raw;
} emg_packet_t;

typedef struct __attribute__((packed)) {
  uint16_t version;
  uint16_t flags;
  uint32_t exchange_id;
  uint64_t host_tx_ns;
} time_sync_request_t;

typedef struct __attribute__((packed)) {
  uint16_t version;
  uint16_t flags;
  uint32_t exchange_id;
  uint64_t host_tx_ns;
  uint64_t device_rx_us;
  uint64_t device_tx_us;
} time_sync_response_t;

_Static_assert(sizeof(imu_packet_t) == 38, "IMU packet wire size changed");
_Static_assert(sizeof(emg_packet_t) == 16, "EMG packet wire size changed");
_Static_assert(sizeof(time_sync_request_t) == 16,
               "time-sync request wire size changed");
_Static_assert(sizeof(time_sync_response_t) == 32,
               "time-sync response wire size changed");

static const char *const k_tag = "AURORA_BLE";
static uint8_t s_own_address_type;
static uint16_t s_connection_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_emg_value_handle;
static uint16_t s_imu_value_handle;
static uint16_t s_time_sync_value_handle;
static bool s_emg_notify_enabled;
static bool s_imu_notify_enabled;
static bool s_time_sync_notify_enabled;
static emg_packet_t s_latest_emg;
static imu_packet_t s_latest_imu;
static time_sync_response_t s_latest_sync;
static portMUX_TYPE s_state_mux = portMUX_INITIALIZER_UNLOCKED;

static int sensor_access(uint16_t connection_handle, uint16_t attribute_handle,
                         struct ble_gatt_access_ctxt *context, void *argument);

static const struct ble_gatt_chr_def k_characteristics[] = {
    {
        .uuid = &k_emg_uuid.u,
        .access_cb = sensor_access,
        .val_handle = &s_emg_value_handle,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
    },
    {
        .uuid = &k_imu_uuid.u,
        .access_cb = sensor_access,
        .val_handle = &s_imu_value_handle,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
    },
    {
        .uuid = &k_time_sync_uuid.u,
        .access_cb = sensor_access,
        .val_handle = &s_time_sync_value_handle,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                 BLE_GATT_CHR_F_NOTIFY,
    },
    {0},
};

static const struct ble_gatt_svc_def k_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &k_service_uuid.u,
        .characteristics = k_characteristics,
    },
    {0},
};

static int append_value(struct os_mbuf *output, const void *value, size_t size) {
  return os_mbuf_append(output, value, size) == 0 ? 0
                                                  : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int notify_custom(uint16_t connection_handle, uint16_t value_handle,
                         const void *value, size_t size) {
  struct os_mbuf *packet = ble_hs_mbuf_from_flat(value, size);
  if (packet == NULL) {
    return BLE_HS_ENOMEM;
  }
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  return ble_gatts_notify_custom(connection_handle, value_handle, packet);
#else
  /* NimBLE 1.x names the server-side notification helper gattc because the
   * resulting ATT operation is consumed by the connected GATT client. */
  return ble_gattc_notify_custom(connection_handle, value_handle, packet);
#endif
}

static int sensor_access(uint16_t connection_handle, uint16_t attribute_handle,
                         struct ble_gatt_access_ctxt *context, void *argument) {
  (void)argument;
  if (attribute_handle == s_emg_value_handle) {
    if (context->op != BLE_GATT_ACCESS_OP_READ_CHR) {
      return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }
    emg_packet_t packet;
    taskENTER_CRITICAL(&s_state_mux);
    packet = s_latest_emg;
    taskEXIT_CRITICAL(&s_state_mux);
    return append_value(context->om, &packet, sizeof(packet));
  }
  if (attribute_handle == s_imu_value_handle) {
    if (context->op != BLE_GATT_ACCESS_OP_READ_CHR) {
      return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }
    imu_packet_t packet;
    taskENTER_CRITICAL(&s_state_mux);
    packet = s_latest_imu;
    taskEXIT_CRITICAL(&s_state_mux);
    return append_value(context->om, &packet, sizeof(packet));
  }

  if (attribute_handle != s_time_sync_value_handle) {
    return BLE_ATT_ERR_INVALID_HANDLE;
  }
  if (context->op == BLE_GATT_ACCESS_OP_READ_CHR) {
    time_sync_response_t response;
    taskENTER_CRITICAL(&s_state_mux);
    response = s_latest_sync;
    taskEXIT_CRITICAL(&s_state_mux);
    return append_value(context->om, &response, sizeof(response));
  }
  if (context->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
    return BLE_ATT_ERR_UNLIKELY;
  }

  const uint64_t device_rx_us = (uint64_t)esp_timer_get_time();
  time_sync_request_t request = {0};
  uint16_t copied = 0;
  const int error = ble_hs_mbuf_to_flat(context->om, &request, sizeof(request),
                                        &copied);
  if (error != 0 || copied != sizeof(request)) {
    return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
  }
  if (request.version != SYNC_PROTOCOL_VERSION) {
    return BLE_ATT_ERR_UNLIKELY;
  }

  time_sync_response_t response = {
      .version = SYNC_PROTOCOL_VERSION,
      .flags = 0,
      .exchange_id = request.exchange_id,
      .host_tx_ns = request.host_tx_ns,
      .device_rx_us = device_rx_us,
      .device_tx_us = (uint64_t)esp_timer_get_time(),
  };
  taskENTER_CRITICAL(&s_state_mux);
  s_latest_sync = response;
  const bool subscribed = s_time_sync_notify_enabled;
  taskEXIT_CRITICAL(&s_state_mux);
  if (subscribed && connection_handle != BLE_HS_CONN_HANDLE_NONE) {
    const int notify_error =
        notify_custom(connection_handle, s_time_sync_value_handle, &response,
                      sizeof(response));
    if (notify_error != 0) {
      ESP_LOGW(k_tag, "time-sync notification failed: %d", notify_error);
    }
  }
  return 0;
}

static void start_advertising(void);

static int gap_event(struct ble_gap_event *event, void *argument) {
  (void)argument;
  switch (event->type) {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    case BLE_GAP_EVENT_LINK_ESTAB:
      if (event->link_estab.status == 0) {
        taskENTER_CRITICAL(&s_state_mux);
        s_connection_handle = event->link_estab.conn_handle;
        taskEXIT_CRITICAL(&s_state_mux);
        ESP_LOGI(k_tag, "central connected");
      } else {
        start_advertising();
      }
      return 0;
#else
    case BLE_GAP_EVENT_CONNECT:
      if (event->connect.status == 0) {
        taskENTER_CRITICAL(&s_state_mux);
        s_connection_handle = event->connect.conn_handle;
        taskEXIT_CRITICAL(&s_state_mux);
        ESP_LOGI(k_tag, "central connected");
      } else {
        start_advertising();
      }
      return 0;
#endif
    case BLE_GAP_EVENT_DISCONNECT:
      taskENTER_CRITICAL(&s_state_mux);
      s_connection_handle = BLE_HS_CONN_HANDLE_NONE;
      s_emg_notify_enabled = false;
      s_imu_notify_enabled = false;
      s_time_sync_notify_enabled = false;
      taskEXIT_CRITICAL(&s_state_mux);
      ESP_LOGI(k_tag, "central disconnected; advertising again");
      start_advertising();
      return 0;
    case BLE_GAP_EVENT_SUBSCRIBE:
      taskENTER_CRITICAL(&s_state_mux);
      if (event->subscribe.attr_handle == s_emg_value_handle) {
        s_emg_notify_enabled = event->subscribe.cur_notify != 0;
      } else if (event->subscribe.attr_handle == s_imu_value_handle) {
        s_imu_notify_enabled = event->subscribe.cur_notify != 0;
      } else if (event->subscribe.attr_handle == s_time_sync_value_handle) {
        s_time_sync_notify_enabled = event->subscribe.cur_notify != 0;
      }
      taskEXIT_CRITICAL(&s_state_mux);
      return 0;
    case BLE_GAP_EVENT_ADV_COMPLETE:
      start_advertising();
      return 0;
    case BLE_GAP_EVENT_MTU:
      ESP_LOGI(k_tag, "ATT MTU updated to %d", event->mtu.value);
      return 0;
    default:
      return 0;
  }
}

static void start_advertising(void) {
  struct ble_hs_adv_fields fields = {0};
  fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
  fields.uuids128 = (ble_uuid128_t *)&k_service_uuid;
  fields.num_uuids128 = 1;
  fields.uuids128_is_complete = 1;
  int error = ble_gap_adv_set_fields(&fields);
  if (error != 0) {
    ESP_LOGE(k_tag, "advertising fields failed: %d", error);
    return;
  }

  struct ble_hs_adv_fields response = {0};
  response.name = (uint8_t *)DEVICE_NAME;
  response.name_len = strlen(DEVICE_NAME);
  response.name_is_complete = 1;
  error = ble_gap_adv_rsp_set_fields(&response);
  if (error != 0) {
    ESP_LOGE(k_tag, "scan-response fields failed: %d", error);
    return;
  }

  struct ble_gap_adv_params parameters = {0};
  parameters.conn_mode = BLE_GAP_CONN_MODE_UND;
  parameters.disc_mode = BLE_GAP_DISC_MODE_GEN;
  /* Advertising intervals are expressed in 0.625 ms units in every NimBLE
   * version supported by this target. */
  parameters.itvl_min = 100000U / BLE_HCI_ADV_ITVL;
  parameters.itvl_max = 120000U / BLE_HCI_ADV_ITVL;
  error = ble_gap_adv_start(s_own_address_type, NULL, BLE_HS_FOREVER,
                            &parameters, gap_event, NULL);
  if (error != 0) {
    ESP_LOGE(k_tag, "advertising start failed: %d", error);
  }
}

static void host_sync(void) {
  int error = ble_hs_util_ensure_addr(0);
  if (error == 0) {
    error = ble_hs_id_infer_auto(0, &s_own_address_type);
  }
  if (error != 0) {
    ESP_LOGE(k_tag, "BLE address setup failed: %d", error);
    return;
  }
  start_advertising();
}

static void host_task(void *argument) {
  (void)argument;
  nimble_port_run();
  nimble_port_freertos_deinit();
}

esp_err_t ble_transport_init(void) {
  esp_err_t error = nvs_flash_init();
  if (error == ESP_ERR_NVS_NO_FREE_PAGES ||
      error == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    error = nvs_flash_init();
  }
  if (error != ESP_OK) {
    return error;
  }

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  error = nimble_port_init();
  if (error != ESP_OK) {
    return error;
  }
#else
  error = esp_nimble_hci_and_controller_init();
  if (error != ESP_OK) {
    return error;
  }
  nimble_port_init();
#endif
  ble_hs_cfg.sync_cb = host_sync;
  ble_svc_gap_init();
  ble_svc_gatt_init();
  if (ble_svc_gap_device_name_set(DEVICE_NAME) != 0 ||
      ble_gatts_count_cfg(k_services) != 0 ||
      ble_gatts_add_svcs(k_services) != 0) {
    return ESP_FAIL;
  }
  nimble_port_freertos_init(host_task);
  ESP_LOGI(k_tag, "ESP-IDF NimBLE service initialized as %s", DEVICE_NAME);
  return ESP_OK;
}

static int16_t scaled_value(float value) {
  float scaled = value * 1000.0F;
  if (scaled > 32767.0F) {
    scaled = 32767.0F;
  } else if (scaled < -32768.0F) {
    scaled = -32768.0F;
  }
  return (int16_t)(scaled + (scaled >= 0.0F ? 0.5F : -0.5F));
}

static void write_sensor(int16_t *destination,
                         const aurora_ble_imu_sample_t *sample) {
  if (sample == NULL || !sample->connected) {
    memset(destination, 0, 6 * sizeof(*destination));
    return;
  }
  for (size_t axis = 0; axis < 3; ++axis) {
    destination[axis] = scaled_value(sample->accel_g[axis]);
    destination[axis + 3] = scaled_value(sample->gyro_dps[axis]);
  }
}

void ble_transport_publish(uint32_t sequence, uint64_t device_time_us,
                           const aurora_ble_imu_sample_t *shoulder,
                           const aurora_ble_imu_sample_t *wrist,
                           bool emg_connected, uint16_t emg_raw) {
  const uint16_t connected_mask =
      (shoulder != NULL && shoulder->connected ? 0x01U : 0U) |
      (wrist != NULL && wrist->connected ? 0x02U : 0U);
  imu_packet_t packet = {
      .format_flags = (uint16_t)((IMU_PACKET_VERSION << 8) | connected_mask),
      .sequence = sequence,
      .device_time_us = device_time_us,
  };
  emg_packet_t emg_packet = {
      .format_flags = (uint16_t)((EMG_PACKET_VERSION << 8) |
                                 (emg_connected ? 0x01U : 0U)),
      .sequence = sequence,
      .device_time_us = device_time_us,
      .adc_raw = emg_connected ? emg_raw : 0U,
  };
  write_sensor(&packet.values[0], shoulder);
  write_sensor(&packet.values[6], wrist);

  taskENTER_CRITICAL(&s_state_mux);
  s_latest_emg = emg_packet;
  s_latest_imu = packet;
  const uint16_t connection_handle = s_connection_handle;
  const bool emg_subscribed = s_emg_notify_enabled;
  const bool imu_subscribed = s_imu_notify_enabled;
  taskEXIT_CRITICAL(&s_state_mux);
  if (connection_handle == BLE_HS_CONN_HANDLE_NONE) {
    return;
  }
  if (emg_subscribed) {
    const int error = notify_custom(connection_handle, s_emg_value_handle,
                                    &emg_packet, sizeof(emg_packet));
    if (error != 0 && error != BLE_HS_ENOTCONN) {
      ESP_LOGW(k_tag, "EMG notification failed: %d", error);
    }
  }
  if (imu_subscribed) {
    const int error = notify_custom(connection_handle, s_imu_value_handle,
                                    &packet, sizeof(packet));
    if (error != 0 && error != BLE_HS_ENOTCONN) {
      ESP_LOGW(k_tag, "IMU notification failed: %d", error);
    }
  }
}
