/**
 * @file config_send.h
 * @brief Hardware configuration for send example
 * 
 * This file defines the hardware setup for the send example:
 * - 2 MCP2515 devices on SPI3 host
 * - TX-only configuration (no INT pins)
 * - 16 MHz crystal, 1 Mbps CAN bitrate
 * 
 * **IMPORTANT:** Adapt GPIO pins, SPI host, crystal frequency, and CAN bitrate
 * to match your actual hardware before building the example.
 * 
 * @note GPIO pin assignments are for ESP32-S3. Other ESP32 variants may
 *       require different pin selections.
 */

#pragma once

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "mcp2515_multi.h"

// Multi-device configuration for send_multi example:
// Two MCP2515 devices on SPI3 host, no INT lines (TX only)

const mcp2515_bundle_config_t CAN_HW_CFG = {
    .bus = {
        .bus_id = (can_bus_id_t)1,
        .wiring = {
            .miso_io_num   = GPIO_NUM_15,
            .mosi_io_num   = GPIO_NUM_16,
            .sclk_io_num   = GPIO_NUM_14,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
        },
        .params = {
            .host            = SPI3_HOST,
            .max_transfer_sz = 0,
            .flags           = SPICOMMON_BUSFLAG_MASTER,
            .dma_chan        = SPI_DMA_CH_AUTO,
            .intr_flags      = 0,
            .isr_cpu_id      = 0,
        },
        .manage_bus_lifetime = true,
    },
    .devices = (const mcp2515_device_config_t[]){
        {
            .dev_id = (can_dev_id_t)1,
            .wiring = {
                .cs_gpio  = GPIO_NUM_11,   // TX A
                .int_gpio = (gpio_num_t)-1,
                .stby_gpio= (gpio_num_t)-1,
                .rst_gpio = (gpio_num_t)-1,
            },
            .spi_params = {
                .mode            = 0,
                .clock_speed_hz  = 10000000,
                .queue_size      = 64,
                .flags           = 0,
                .command_bits    = 0,
                .address_bits    = 0,
                .dummy_bits      = 0,
            },
            .hw   = { .crystal_frequency = MCP_16MHZ },
            .can  = { .can_speed = CAN_1000KBPS, .use_loopback = false },
        },
        {
            .dev_id = (can_dev_id_t)2,
            .wiring = {
                .cs_gpio  = GPIO_NUM_17,   // TX B
                .int_gpio = (gpio_num_t)-1,
                .stby_gpio= (gpio_num_t)-1,
                .rst_gpio = (gpio_num_t)-1,
            },
            .spi_params = {
                .mode            = 0,
                .clock_speed_hz  = 10000000,
                .queue_size      = 64,
                .flags           = 0,
                .command_bits    = 0,
                .address_bits    = 0,
                .dummy_bits      = 0,
            },
            .hw   = { .crystal_frequency = MCP_16MHZ },
            .can  = { .can_speed = CAN_1000KBPS, .use_loopback = false },
        },
    },
    .device_count = 2,
};


