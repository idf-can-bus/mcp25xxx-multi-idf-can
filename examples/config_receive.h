/**
 * @file config_receive.h
 * @brief Hardware configuration for receive examples
 * 
 * This file defines the hardware setup for receive_poll and receive_interrupt examples:
 * - 3 MCP2515 devices on SPI2 host
 * - Each device has its own INT (interrupt) pin for event-driven reception
 * - 16 MHz crystal, 1 Mbps CAN bitrate
 * 
 * **IMPORTANT:** Adapt GPIO pins, SPI host, crystal frequency, and CAN bitrate
 * to match your actual hardware before building the examples.
 * 
 * @note GPIO pin assignments are for ESP32-S3. Other ESP32 variants may
 *       require different pin selections.
 * @note For receive_poll example, INT pins are defined but not actively used.
 *       For receive_interrupt example, INT pins must be properly connected.
 */

#pragma once

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "mcp2515_multi.h"

// Multi-device configuration for receive_*_multi examples:
// Three MCP2515 devices on SPI2 host, each with its own INT line.

const mcp2515_bundle_config_t CAN_HW_CFG = {
    .bus = {
        .bus_id = (can_bus_id_t)1,
        .wiring = {
            .miso_io_num   = GPIO_NUM_37,
            .mosi_io_num   = GPIO_NUM_38,
            .sclk_io_num   = GPIO_NUM_36,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
        },
        .params = {
            .host            = SPI2_HOST,
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
                .cs_gpio  = GPIO_NUM_33,  // RX A
                .int_gpio = GPIO_NUM_34,
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
                .cs_gpio  = GPIO_NUM_35,  // RX B
                .int_gpio = GPIO_NUM_39,
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
            .dev_id = (can_dev_id_t)3,
            .wiring = {
                .cs_gpio  = GPIO_NUM_40,  // RX C
                .int_gpio = GPIO_NUM_12,
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
    .device_count = 3,
};


