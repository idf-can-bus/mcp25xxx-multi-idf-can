#include "mcp25xxx_multi.h"
#include <stdio.h>
#include "examples_utils.h"
#include "config_receive.h"
#include "esp_log.h"

/**
 * @file main.c
 * @brief CAN multi-receiver example using polling reception
 * 
 * This example demonstrates receiving CAN messages from multiple MCP25xxx devices
 * using a polling approach. The main loop continuously polls all devices for
 * incoming messages and processes them with per-sender statistics tracking.
 * 
 * Hardware configuration: See examples/config_receive.h
 * - 3 MCP25xxx RX devices on SPI2
 * - Interrupt pins defined but not used (polling mode)
 */

static const char *TAG = "receive_poll_multi";

#ifdef __cplusplus
extern "C"
#endif
void app_main(void)
{
    // Identify example and backend
    ESP_LOGI(TAG, "=== example: receive_poll-multi, backend: %s, INSTANCES:%u ===",
        can_backend_get_name(), (unsigned)n);
        
    // Initialize MCP25xxx multi library with hardware configuration from config_receive.h
    (void)canif_multi_init_default(&CAN_HW_CFG);

    const uint32_t receive_interval_ms = 1;
    twai_message_t msg;

    can_bus_handle_t bus = canif_bus_default();
    size_t n = canif_bus_device_count(bus);

    
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


