/**
 * @file mcp25xxx_multi_internal.h
 * @brief Internal backend API for MCP25xxx device control
 * 
 * This header defines the low-level backend interface for direct MCP25xxx control.
 * It is NOT intended for application use - applications should use mcp25xxx_multi.h instead.
 * 
 * The backend provides:
 * - Direct SPI communication with MCP25xxx registers
 * - Low-level reset, configuration, and mode control
 * - Raw frame transmission and reception
 * - Event handling infrastructure
 * 
 * @warning This API is subject to change without notice. Do not use directly in applications.
 * @note The public API (mcp25xxx_multi.h) wraps this backend with a stable interface.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "mcp25xxx_multi.h"

#ifdef __cplusplus
extern "C" {
#endif

// Forward declaration for SW buffer (opaque pointer, defined in .c file)
struct spsc_ring_buffer_s;
typedef struct spsc_ring_buffer_s spsc_ring_buffer_t;

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

/** @brief Opaque handle to MCP25xxx context */
typedef struct MCP25XXX_Context* MCP25XXX_Handle;

/**
 * @brief Backend configuration for MCP25xxx initialization
 */
typedef struct {
    mcp25xxx_speed_t can_speed;  /**< CAN bitrate */
    mcp25xxx_clock_t can_clock;  /**< Crystal frequency */
} mcp2515_multi_config_t;

/** @brief Event mask bit: RX message ready */
#define MCP25XXX_EVENT_RX_READY   (1u << 0)

/** @brief Event mask bit: Error condition */
#define MCP25XXX_EVENT_ERROR      (1u << 1)

/**
 * @brief Event callback function type for backend
 * @param h MCP25xxx handle
 * @param eventMask Bitmask of events
 * @param userData User data pointer
 */
typedef void (*MCP25XXX_EventCallback)(MCP25XXX_Handle h, uint32_t eventMask, void* userData);

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
 * @brief Creates MCP25xxx context using existing SPI device handle
 * @param spi Pre-configured SPI device handle
 * @param int_gpio Interrupt GPIO pin (GPIO_NUM_NC if unused)
 * @param cfg Backend configuration
 * @param out_handle Output parameter for created handle
 * @return ERROR_OK on success, error code otherwise
 */
ERROR_t MCP25XXX_CreateOnDevice(spi_device_handle_t spi,
                               gpio_num_t int_gpio,
                               const mcp2515_multi_config_t* cfg,
                               MCP25XXX_Handle* out_handle);

/**
 * @brief Creates MCP25xxx context and initializes SPI bus if needed
 * @param host SPI host peripheral
 * @param bus_cfg SPI bus configuration
 * @param dev_cfg SPI device configuration
 * @param int_gpio Interrupt GPIO pin
 * @param cfg Backend configuration
 * @param out_handle Output parameter for created handle
 * @return ERROR_OK on success, error code otherwise
 */
ERROR_t MCP25XXX_CreateOnBus(spi_host_device_t host,
                            const spi_bus_config_t* bus_cfg,
                            const spi_device_interface_config_t* dev_cfg,
                            gpio_num_t int_gpio,
                            const mcp2515_multi_config_t* cfg,
                            MCP25XXX_Handle* out_handle);

/**
 * @brief Destroys MCP25xxx context and frees resources
 * @param h MCP25xxx handle
 */
void    MCP25XXX_Destroy(MCP25XXX_Handle h);

// SPI helper (optional)

/** @brief Initializes SPI bus if not already initialized (idempotent) */
esp_err_t mcp2515_spi_init_bus_if_needed(spi_host_device_t host, const spi_bus_config_t* bus_cfg);

/** @brief Adds SPI device to initialized bus */
esp_err_t mcp2515_spi_add_device(spi_host_device_t host, const spi_device_interface_config_t* dev_cfg, spi_device_handle_t* out_spi);

/** @brief Removes SPI device from bus */
esp_err_t mcp2515_spi_remove_device(spi_device_handle_t spi);

// --------------------------------------------------------------------------------------
// Basic control

/** @brief Resets MCP25xxx to default state */
ERROR_t MCP25XXX_Reset(MCP25XXX_Handle h);

/** @brief Configures CAN bitrate */
ERROR_t MCP25XXX_SetBitrate(MCP25XXX_Handle h, mcp25xxx_speed_t speed, mcp25xxx_clock_t clock);

/** @brief Switches to normal operating mode */
ERROR_t MCP25XXX_SetNormalMode(MCP25XXX_Handle h);

/** @brief Switches to loopback test mode */
ERROR_t MCP25XXX_SetLoopbackMode(MCP25XXX_Handle h);

// Filters & masks

/** @brief Configures acceptance filter */
ERROR_t MCP25XXX_SetFilter(MCP25XXX_Handle h, uint8_t filter_idx, bool extended, uint32_t id);

/** @brief Configures acceptance mask */
ERROR_t MCP25XXX_SetMask(MCP25XXX_Handle h, uint8_t mask_idx, bool extended, uint32_t mask);

// Tx/Rx

/** @brief Sends CAN frame (after checking controller status) */
ERROR_t MCP25XXX_SendMessageAfterCtrlCheck(MCP25XXX_Handle h, const CAN_FRAME* frame);

/** @brief Reads CAN frame (after checking status) */
ERROR_t MCP25XXX_ReadMessageAfterStatCheck(MCP25XXX_Handle h, CAN_FRAME* frame);

/**
 * @brief Read all available messages from both RX buffers.
 * 
 * MCP25625/MCP2515 has two RX buffers (RXB0, RXB1). When multiple messages
 * arrive rapidly, both buffers may contain data. This function reads all
 * available messages in priority order (RXB0 first, then RXB1) to prevent
 * hardware buffer overflow.
 * 
 * @param h MCP25xxx handle
 * @param frames Output array for received frames (must have space for at least 2 frames)
 * @param max_count Maximum number of frames to read (typically 2)
 * @param out_count Pointer to store the actual number of frames read (0-2)
 * @return ERROR_OK if at least one message was read, ERROR_NOMSG if no messages available
 */
ERROR_t MCP25XXX_ReadAllAvailable(MCP25XXX_Handle h, CAN_FRAME* frames, 
                                   uint8_t max_count, uint8_t* out_count);

// Events

/** @brief Registers event callback */
void     MCP25XXX_SetEventCallback(MCP25XXX_Handle h, MCP25XXX_EventCallback cb, void* userData);

/** @brief Waits for events with timeout */
uint32_t MCP25XXX_WaitForEvent(MCP25XXX_Handle h, uint32_t timeout_ticks);

// Errors

/** @brief Reads error flags register (EFLG) */
uint8_t MCP25XXX_GetErrorFlags(MCP25XXX_Handle h);

/** @brief Clears RX overrun flags */
void    MCP25XXX_ClearRXnOVR(MCP25XXX_Handle h);

/** @brief Clears error interrupt flag */
void    MCP25XXX_ClearERRIF(MCP25XXX_Handle h);

/**
 * @brief Read frame from SW RX FIFO.
 *
 * Reads from software buffer (populated by ISR). This is the preferred
 * method for interrupt-driven devices as it avoids direct HW access.
 *
 * @param h MCP25xxx handle
 * @param frame Output frame structure
 * @return ERROR_OK if frame read, ERROR_NOMSG if FIFO empty or not configured
 */
ERROR_t MCP25XXX_ReadFromFifo(MCP25XXX_Handle h, CAN_FRAME* frame);

/**
 * @brief Get SW RX buffer statistics.
 *
 * @param h MCP25xxx handle
 * @param out_size Current FIFO size (or NULL)
 * @param out_dropped Total dropped frames (or NULL)
 * @param out_seq Last sequence number (or NULL)
 * @return true if buffer exists, false otherwise
 */
bool    MCP25XXX_GetRxFifoStats(MCP25XXX_Handle h, uint32_t* out_size, 
                                 uint32_t* out_dropped, uint32_t* out_seq);

/**
 * @brief Get global ISR debug counters.
 *
 * Useful for diagnosing interrupt and FIFO issues.
 *
 * @param out_isr_calls Total ISR invocations (or NULL)
 * @param out_frames_read Total frames read in ISR (or NULL)
 * @param out_fifo_pushes Total FIFO pushes in ISR (or NULL)
 */
void    MCP25XXX_GetIsrDebugCounters(uint32_t* out_isr_calls, uint32_t* out_frames_read,
                                      uint32_t* out_fifo_pushes);

#ifdef __cplusplus
}
#endif
