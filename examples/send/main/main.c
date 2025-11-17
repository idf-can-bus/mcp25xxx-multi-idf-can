#include "mcp25xxx_multi.h"
#include "esp_log.h"
#include "examples_utils.h"
#include "config_send.h"

/**
 * @file main.c
 * @brief CAN multi-sender example using polling transmission
 * 
 * This example demonstrates sending CAN messages from multiple MCP25xxx devices.
 * Each device sends test messages with heartbeat counter and timestamp at regular intervals.
 * 
 * Hardware configuration: See examples/config_send.h
 * - 2 MCP25xxx TX devices on SPI3
 * - No interrupt pins (polling mode)
 */

static const char *TAG = "send_multi";

#ifdef __cplusplus
extern "C"
#endif
void app_main(void)
{
    // Identify example and backend
    ESP_LOGI(TAG, "=== example: send-multi, backend: %s ===", can_backend_get_name());

    // Initialize MCP25xxx multi library with hardware configuration from config_send.h
    (void)canif_multi_init_default(&CAN_HW_CFG);

    const uint32_t send_interval_ms = 10;

    // Query number of devices on default bus
    can_bus_handle_t bus = canif_bus_default();
    size_t n = canif_bus_device_count(bus);

    ESP_LOGI(TAG, "=== instances: %u ===", (unsigned)n);

    // Size arrays dynamically by number of instances
    uint8_t heartbeat[n];
    uint8_t sender_ids[n];
    for (size_t i = 0; i < n; ++i) {
        heartbeat[i] = 0;
        sender_ids[i] = (uint8_t)(i + 1); // IDs 1..N
        ESP_LOGI(TAG, "SENDER_ID: %u", sender_ids[i]);
    }


    twai_message_t msg;
    uint64_t index = 0;
    const uint64_t stats_every = 2000;
    bool print_during_send = false;

    while (1) {
        for (size_t i=0; i<n; ++i) {
            fullfill_test_messages(sender_ids[i], heartbeat[i], &msg);
            if ((index % stats_every == 0) && (index != 0)) {
                set_test_flag(&msg, TEST_FLAG_STATS_REQUEST);
            }
            can_dev_handle_t dev = canif_device_at(bus, i);
            bool ok = canif_send_to(dev, &msg);
            if (!ok) {
                ESP_LOGE(TAG, "TX%u: send failed", (unsigned)i);
                print_can_message(&msg);
            } else {
                debug_send_message(&msg, print_during_send);
                heartbeat[i] = next_heartbeat(heartbeat[i]);
            }
        }
        index++;
        sleep_ms_min_ticks(send_interval_ms);
    }
}
