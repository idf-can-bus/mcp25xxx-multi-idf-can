#include "mcp2515_multi.h"
#include <stdio.h>
#include "esp_log.h"
#include "examples_utils.h"
#include "config_receive.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

/*
 * Example: receive_interrupt_multi
 *
 * Note: Hardware configuration (number of instances, CS/INT pins) je stanovena
 * v examples/init_hardware.c. Tento příklad si počet instancí načítá dynamicky
 * přes can_configured_instance_count().
 */

static const char *TAG = "receive_interrupt_multi";

#define RX_QUEUE_LENGTH 128
#define PRODUCER_STACK  4096
#define CONSUMER_STACK  4096
#define PRODUCER_PRIO   12
#define CONSUMER_PRIO   10

static QueueHandle_t rx_queue;

typedef struct {
    size_t index; // MCP2515 instance index
} producer_arg_t;

static void can_rx_producer_task(void *arg)
{
    producer_arg_t *parg = (producer_arg_t *)arg;
    const size_t idx = parg->index;
    can_bus_handle_t bus = canif_bus_default();
    can_dev_handle_t dev = canif_device_at(bus, idx);
    can_message_t message;
    for (;;) {
        // Drain all available frames from this instance
        bool any = false;
        while (canif_receive_from(dev, &message)) {
            (void)xQueueSend(rx_queue, &message, 0);
            any = true;
        }
        if (!any) {
            sleep_ms_min_ticks(1);
        }
    }
}

static void can_rx_consumer_task(void *arg)
{
    (void)arg;
    can_message_t message;
    const bool print_during_receive = false;
    for (;;) {
        if (xQueueReceive(rx_queue, &message, portMAX_DELAY) == pdTRUE) {
            process_received_message_multi(&message, print_during_receive);
        }
    }
}

#ifdef __cplusplus
extern "C"
#endif
void app_main(void)
{
    // Initialize MCP2515 multi library directly with bundle config, see config_hw_mcp2515_multi_receive.h
    (void)canif_multi_init_default(&CAN_HW_CFG);

    rx_queue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(can_message_t));
    if (!rx_queue) {
        ESP_LOGE(TAG, "Failed to create RX queue");
        return;
    }

    ESP_LOGI(TAG, "Receiver interrupt-driven");

    can_bus_handle_t bus = canif_bus_default();
    size_t n = canif_bus_device_count(bus);
    // Create one producer per instance
    for (size_t i=0; i<n; ++i) {
        producer_arg_t *parg = (producer_arg_t*)pvPortMalloc(sizeof(producer_arg_t));
        if (!parg) continue;
        parg->index = i;
        char name[24]; snprintf(name, sizeof(name), "rx_prod%u", (unsigned)i);
        xTaskCreate(can_rx_producer_task, name, PRODUCER_STACK, parg, PRODUCER_PRIO, NULL);
    }
    xTaskCreate(can_rx_consumer_task, "can_rx_cons",  CONSUMER_STACK, NULL,      CONSUMER_PRIO, NULL);
}



