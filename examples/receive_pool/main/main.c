#include "mcp2515_multi.h"
#include <stdio.h>
#include "examples_utils.h"
#include "config_receive.h"
#include "esp_log.h"

static const char *TAG = "receive_poll_multi";

#ifdef __cplusplus
extern "C"
#endif
void app_main(void)
{
    // Initialize MCP2515 multi library directly with bundle config, see config_hw_mcp2515_multi_receive.h
    (void)canif_multi_init_default(&CAN_HW_CFG);

    const uint32_t receive_interval_ms = 1;
    can_message_t msg;

    can_bus_handle_t bus = canif_bus_default();
    size_t n = canif_bus_device_count(bus);

    ESP_LOGI(TAG, "Receiver poll-driven, MCP2515 multi, %zu instances", n);
    while (1) {
        // poll all instances
        for (size_t i=0; i<n; ++i) {
            can_dev_handle_t dev = canif_device_at(bus, i);
            if (canif_receive_from(dev, &msg)) {
                process_received_message_multi(&msg, false);
            }
        }
        sleep_ms_min_ticks(receive_interval_ms);
    }
}
