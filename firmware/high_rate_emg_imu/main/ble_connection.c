/* Majd */
#include "ble_connection.h"

#include "esp_log.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_esp_gap.h"

static const char *TAG = "AURORA_BLE";

void ble_configure_connection(uint16_t connection_handle)
{
    // Keep HT25's link settings so sensor packets can clear the transmit queue.
    int error = ble_hs_hci_util_set_data_len(connection_handle, 251, 2120);
    if (error != 0) {
        ESP_LOGW(TAG, "Data length request failed: %d", error);
    }
    error = ble_gap_set_prefered_le_phy(connection_handle, BLE_GAP_LE_PHY_2M_MASK,
                                      BLE_GAP_LE_PHY_2M_MASK, BLE_GAP_LE_PHY_CODED_ANY);
    if (error != 0) {
        ESP_LOGW(TAG, "2M PHY request failed: %d", error);
    }

    struct ble_gap_conn_desc connection = {0};
    error = ble_gap_conn_find(connection_handle, &connection);
    if (error != 0) {
        ESP_LOGW(TAG, "Connection lookup failed: %d", error);
        return;
    }
    ESP_LOGI(TAG, "Initial connection interval: %u units (1.25 ms each)", connection.conn_itvl);
    // HT25 requests 10 ms for its 100 Hz notification cadence.
    struct ble_gap_upd_params parameters = {
        .itvl_min = 8,
        .itvl_max = 8,
        .latency = 0,
        .supervision_timeout = connection.supervision_timeout,
    };
    error = ble_gap_update_params(connection_handle, &parameters);
    if (error != 0) {
        ESP_LOGW(TAG, "Connection interval request failed: %d", error);
    }
}
