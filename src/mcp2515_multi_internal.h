/**
 * @file mcp2515_multi_internal.h
 * @brief Internal backend API for MCP2515 device control
 * 
 * This header defines the low-level backend interface for direct MCP2515 control.
 * It is NOT intended for application use - applications should use mcp2515_multi.h instead.
 * 
 * The backend provides:
 * - Direct SPI communication with MCP2515 registers
 * - Low-level reset, configuration, and mode control
 * - Raw frame transmission and reception
 * - Event handling infrastructure
 * 
 * @warning This API is subject to change without notice. Do not use directly in applications.
 * @note The public API (mcp2515_multi.h) wraps this backend with a stable interface.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "mcp2515_multi.h"

#ifdef __cplusplus
extern "C" {
#endif

// --------------------------------------------------------------------------------------
// Public types

/** @brief Backend error codes */
typedef enum {
    ERROR_OK        = 0,  /**< Operation successful */
    ERROR_FAIL      = 1,  /**< Generic failure */
    ERROR_ALLTXBUSY = 2,  /**< All TX buffers busy */
    ERROR_FAILINIT  = 3,  /**< Initialization failed */
    ERROR_FAILTX    = 4,  /**< Transmission failed */
    ERROR_NOMSG     = 5   /**< No message available (RX buffer empty) */
} ERROR_t;

/** @brief Opaque handle to MCP2515 context */
typedef struct MCP2515_Context* MCP2515_Handle;

/**
 * @brief Backend configuration for MCP2515 initialization
 */
typedef struct {
    CAN_SPEED_t can_speed;  /**< CAN bitrate */
    CAN_CLOCK_t can_clock;  /**< Crystal frequency */
} mcp2515_multi_config_t;

/** @brief Event mask bit: RX message ready */
#define MCP2515_EVENT_RX_READY   (1u << 0)

/** @brief Event mask bit: Error condition */
#define MCP2515_EVENT_ERROR      (1u << 1)

/**
 * @brief Event callback function type for backend
 * @param h MCP2515 handle
 * @param eventMask Bitmask of events
 * @param userData User data pointer
 */
typedef void (*MCP2515_EventCallback)(MCP2515_Handle h, uint32_t eventMask, void* userData);

/**
 * @brief Internal CAN frame structure
 * 
 * Minimal representation used by the backend. The can_id field may include
 * EFF (Extended Frame Format) and RTR bits encoded internally.
 */
typedef struct {
    uint32_t can_id;   /**< CAN identifier (may include EFF/RTR bits) */
    uint8_t  can_dlc;  /**< Data Length Code (0-8) */
    uint8_t  data[8];  /**< Payload data */
} CAN_FRAME;

// --------------------------------------------------------------------------------------
// Creation / destruction

/**
 * @brief Creates MCP2515 context using existing SPI device handle
 * @param spi Pre-configured SPI device handle
 * @param int_gpio Interrupt GPIO pin (GPIO_NUM_NC if unused)
 * @param cfg Backend configuration
 * @param out_handle Output parameter for created handle
 * @return ERROR_OK on success, error code otherwise
 */
ERROR_t MCP2515_CreateOnDevice(spi_device_handle_t spi,
                               gpio_num_t int_gpio,
                               const mcp2515_multi_config_t* cfg,
                               MCP2515_Handle* out_handle);

/**
 * @brief Creates MCP2515 context and initializes SPI bus if needed
 * @param host SPI host peripheral
 * @param bus_cfg SPI bus configuration
 * @param dev_cfg SPI device configuration
 * @param int_gpio Interrupt GPIO pin
 * @param cfg Backend configuration
 * @param out_handle Output parameter for created handle
 * @return ERROR_OK on success, error code otherwise
 */
ERROR_t MCP2515_CreateOnBus(spi_host_device_t host,
                            const spi_bus_config_t* bus_cfg,
                            const spi_device_interface_config_t* dev_cfg,
                            gpio_num_t int_gpio,
                            const mcp2515_multi_config_t* cfg,
                            MCP2515_Handle* out_handle);

/**
 * @brief Destroys MCP2515 context and frees resources
 * @param h MCP2515 handle
 */
void    MCP2515_Destroy(MCP2515_Handle h);

// SPI helper (optional)

/** @brief Initializes SPI bus if not already initialized (idempotent) */
esp_err_t mcp2515_spi_init_bus_if_needed(spi_host_device_t host, const spi_bus_config_t* bus_cfg);

/** @brief Adds SPI device to initialized bus */
esp_err_t mcp2515_spi_add_device(spi_host_device_t host, const spi_device_interface_config_t* dev_cfg, spi_device_handle_t* out_spi);

/** @brief Removes SPI device from bus */
esp_err_t mcp2515_spi_remove_device(spi_device_handle_t spi);

// --------------------------------------------------------------------------------------
// Basic control

/** @brief Resets MCP2515 to default state */
ERROR_t MCP2515_Reset(MCP2515_Handle h);

/** @brief Configures CAN bitrate */
ERROR_t MCP2515_SetBitrate(MCP2515_Handle h, CAN_SPEED_t speed, CAN_CLOCK_t clock);

/** @brief Switches to normal operating mode */
ERROR_t MCP2515_SetNormalMode(MCP2515_Handle h);

/** @brief Switches to loopback test mode */
ERROR_t MCP2515_SetLoopbackMode(MCP2515_Handle h);

// Filters & masks

/** @brief Configures acceptance filter */
ERROR_t MCP2515_SetFilter(MCP2515_Handle h, uint8_t filter_idx, bool extended, uint32_t id);

/** @brief Configures acceptance mask */
ERROR_t MCP2515_SetMask(MCP2515_Handle h, uint8_t mask_idx, bool extended, uint32_t mask);

// Tx/Rx

/** @brief Sends CAN frame (after checking controller status) */
ERROR_t MCP2515_SendMessageAfterCtrlCheck(MCP2515_Handle h, const CAN_FRAME* frame);

/** @brief Reads CAN frame (after checking status) */
ERROR_t MCP2515_ReadMessageAfterStatCheck(MCP2515_Handle h, CAN_FRAME* frame);

// Events

/** @brief Registers event callback */
void     MCP2515_SetEventCallback(MCP2515_Handle h, MCP2515_EventCallback cb, void* userData);

/** @brief Waits for events with timeout */
uint32_t MCP2515_WaitForEvent(MCP2515_Handle h, uint32_t timeout_ticks);

// Errors

/** @brief Reads error flags register (EFLG) */
uint8_t MCP2515_GetErrorFlags(MCP2515_Handle h);

/** @brief Clears RX overrun flags */
void    MCP2515_ClearRXnOVR(MCP2515_Handle h);

/** @brief Clears error interrupt flag */
void    MCP2515_ClearERRIF(MCP2515_Handle h);

#ifdef __cplusplus
}
#endif
