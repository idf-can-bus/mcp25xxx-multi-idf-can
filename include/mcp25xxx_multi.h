/**
 * @file mcp25xxx_multi.h
 * @brief High-level multi-device MCP25xxx CAN controller library for ESP-IDF
 * 
 * This library provides a comprehensive interface for managing multiple MCP25xxx
 * CAN controllers connected via SPI. It supports multiple devices on one or more
 * SPI buses with independent or parallel operation.
 * 
 * Key features:
 * - Multiple MCP25xxx devices on shared or separate SPI buses
 * - Numeric bus/device IDs for efficient addressing
 * - Registry-based device management
 * - Flexible initialization and lifecycle control
 * - Event-driven or polling reception modes
 * - Hardware filter/mask configuration
 * 
 * @author Ivo Marvan, 2025
 * @copyright MIT License
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "sdkconfig.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/twai.h"


#ifdef __cplusplus
extern "C" {
#endif

// ======================================================================================
// Basic identifiers (must be defined before configuration types)
// ======================================================================================

/** @brief User-assigned SPI bus identifier (0-255) */
typedef uint8_t can_bus_id_t;

/** @brief User-assigned CAN device identifier (0-255) */
typedef uint8_t can_dev_id_t;

// ======================================================================================
// Configuration types
// ======================================================================================

// ---------- SPI BUS (wiring + params) ----------

/**
 * @brief SPI bus wiring configuration
 * 
 * Defines GPIO pins used for SPI bus communication with MCP25xxx devices.
 */
typedef struct {
    gpio_num_t miso_io_num;    /**< Master In Slave Out (MISO) GPIO pin */
    gpio_num_t mosi_io_num;    /**< Master Out Slave In (MOSI) GPIO pin */
    gpio_num_t sclk_io_num;    /**< Serial Clock (SCLK) GPIO pin */
    int        quadwp_io_num;  /**< Quad Write Protect pin (-1 if unused) */
    int        quadhd_io_num;  /**< Quad Hold pin (-1 if unused) */
} mcp_spi_bus_wiring_t;

/**
 * @brief SPI bus parameters configuration
 * 
 * ESP-IDF specific parameters for SPI bus initialization.
 */
typedef struct {
    spi_host_device_t host;           /**< SPI host peripheral (SPI2_HOST, SPI3_HOST, etc.) */
    int               max_transfer_sz; /**< Maximum transfer size in bytes (0 = default 4096) */
    uint32_t          flags;          /**< Bus flags (SPICOMMON_BUSFLAG_MASTER, etc.) */
    int               dma_chan;       /**< DMA channel (SPI_DMA_CH_AUTO recommended) */
    int               intr_flags;     /**< Interrupt allocation flags (ESP_INTR_FLAG_*) */
    int               isr_cpu_id;     /**< CPU core for ISR (ESP_INTR_CPU_AFFINITY_*) */
} mcp_spi_bus_params_t;

/**
 * @brief Complete SPI bus configuration
 * 
 * Combines wiring, parameters, and lifecycle management for one SPI bus.
 * Multiple MCP25xxx devices can share the same SPI bus.
 */
typedef struct {
    can_bus_id_t         bus_id;             /**< User-assigned bus identifier (0-255) */
    mcp_spi_bus_wiring_t wiring;             /**< GPIO pin assignments */
    mcp_spi_bus_params_t params;             /**< ESP-IDF SPI parameters */
    bool                 manage_bus_lifetime; /**< If true, library initializes/deinitializes the bus */
} mcp_spi_bus_config_t;

/**
 * @brief Converts high-level SPI bus configuration to ESP-IDF structures
 * 
 * Helper function to translate library-specific configuration into
 * ESP-IDF native types for SPI bus initialization.
 * 
 * @param[in]  src          Pointer to high-level bus configuration
 * @param[out] out_host     SPI host identifier (SPIx_HOST)
 * @param[out] out_bus_cfg  ESP-IDF bus configuration structure
 * @param[out] out_dma_chan DMA channel selection
 * @return true if conversion successful, false if any pointer is NULL
 */
static inline bool mcp_spi_bus_to_idf(const mcp_spi_bus_config_t *src,
                                      spi_host_device_t *out_host,
                                      spi_bus_config_t *out_bus_cfg,
                                      int *out_dma_chan)
{
    if (!src || !out_host || !out_bus_cfg || !out_dma_chan) return false;
    *out_host = src->params.host;
    *out_dma_chan = src->params.dma_chan;
    out_bus_cfg->miso_io_num = src->wiring.miso_io_num;
    out_bus_cfg->mosi_io_num = src->wiring.mosi_io_num;
    out_bus_cfg->sclk_io_num = src->wiring.sclk_io_num;
    out_bus_cfg->quadwp_io_num = src->wiring.quadwp_io_num;
    out_bus_cfg->quadhd_io_num = src->wiring.quadhd_io_num;
    out_bus_cfg->max_transfer_sz = src->params.max_transfer_sz;
    out_bus_cfg->flags = src->params.flags;
    out_bus_cfg->intr_flags = src->params.intr_flags; // ensure valid interrupt flags
    return true;
}

// ---------- SPI DEVICE (wiring + params) ----------

/**
 * @brief SPI device wiring configuration
 * 
 * Defines device-specific GPIO pins for a single MCP25xxx controller.
 * Each device on the bus requires a unique CS (Chip Select) pin.
 */
typedef struct {
    gpio_num_t cs_gpio;    /**< Chip Select pin (required, must be unique per device) */
    gpio_num_t int_gpio;   /**< Interrupt pin (GPIO_NUM_NC if unused, recommended for efficient RX) */
    gpio_num_t stby_gpio;  /**< Standby control pin (GPIO_NUM_NC if unused) */
    gpio_num_t rst_gpio;   /**< Hardware reset pin (GPIO_NUM_NC if unused) */
} mcp_spi_dev_wiring_t;

/**
 * @brief SPI device parameters
 * 
 * SPI communication parameters specific to one MCP25xxx device.
 */
typedef struct {
    uint8_t   mode;             /**< SPI mode (0-3), MCP25xxx uses mode 0 */
    uint32_t  clock_speed_hz;   /**< SPI clock frequency in Hz (typically 10 MHz max for MCP25xxx) */
    uint32_t  queue_size;       /**< Transaction queue depth (e.g., 64) */
    uint32_t  flags;            /**< Device-specific flags (SPI_DEVICE_*) */
    uint32_t  command_bits;     /**< Number of command bits (usually 0 for MCP25xxx) */
    uint32_t  address_bits;     /**< Number of address bits (usually 0 for MCP25xxx) */
    uint32_t  dummy_bits;       /**< Number of dummy bits (usually 0 for MCP25xxx) */
} mcp_spi_dev_params_t;

static inline void mcp_spi_dev_to_idf(const mcp_spi_dev_wiring_t *w,
                                      const mcp_spi_dev_params_t *p,
                                      spi_device_interface_config_t *out)
{
    /*
     * Fills ESP-IDF spi_device_interface_config_t from high-level device wiring/params.
     * Parameters:
     *  - w:   device wiring (CS pin must be valid)
     *  - p:   SPI device parameters (mode, clock, queue size, etc.)
     *  - out: target IDF config structure to populate
     */
    out->mode = p->mode;
    out->clock_speed_hz = p->clock_speed_hz;
    out->spics_io_num = w->cs_gpio;
    out->queue_size = p->queue_size;
    out->flags = p->flags;
    out->command_bits = p->command_bits;
    out->address_bits = p->address_bits;
    out->dummy_bits = p->dummy_bits;
}

// ---------- MCP25xxx device (HW + CAN params) ----------

/**
 * @brief MCP25xxx crystal oscillator frequency
 * 
 * The MCP25xxx family can use different crystal frequencies. This must match
 * the actual crystal mounted on your MCP25xxx module.
 */
typedef enum {
    MCP25XXX_20MHZ,  /**< 20 MHz crystal oscillator */
    MCP25XXX_16MHZ,  /**< 16 MHz crystal oscillator (most common) */
    MCP25XXX_8MHZ    /**< 8 MHz crystal oscillator */
} mcp25xxx_clock_t;

/**
 * @brief CAN bus bitrate (baud rate)
 * 
 * Supported CAN communication speeds. The actual bitrate configuration
 * depends on both the speed and the crystal frequency.
 */
typedef enum {
    MCP25XXX_5KBPS,     /**< 5 kbit/s */
    MCP25XXX_10KBPS,    /**< 10 kbit/s */
    MCP25XXX_20KBPS,    /**< 20 kbit/s */
    MCP25XXX_31K25BPS,  /**< 31.25 kbit/s */
    MCP25XXX_33KBPS,    /**< 33 kbit/s */
    MCP25XXX_40KBPS,    /**< 40 kbit/s */
    MCP25XXX_50KBPS,    /**< 50 kbit/s */
    MCP25XXX_80KBPS,    /**< 80 kbit/s */
    MCP25XXX_83K3BPS,   /**< 83.3 kbit/s */
    MCP25XXX_95KBPS,    /**< 95 kbit/s */
    MCP25XXX_100KBPS,   /**< 100 kbit/s */
    MCP25XXX_125KBPS,   /**< 125 kbit/s */
    MCP25XXX_200KBPS,   /**< 200 kbit/s */
    MCP25XXX_250KBPS,   /**< 250 kbit/s (common industrial) */
    MCP25XXX_500KBPS,   /**< 500 kbit/s (common automotive) */
    MCP25XXX_1000KBPS   /**< 1 Mbit/s (maximum) */
} mcp25xxx_speed_t;

/**
 * @brief MCP25xxx hardware configuration
 * 
 * Hardware-specific settings for the MCP25xxx controller.
 */
typedef struct {
    mcp25xxx_clock_t crystal_frequency;   /**< Crystal oscillator frequency (must match hardware) */
} mcp2515_hw_t;

/**
 * @brief CAN protocol parameters
 * 
 * Configuration for CAN protocol behavior.
 */
typedef struct {
    mcp25xxx_speed_t can_speed;      /**< CAN bus bitrate */
    bool             use_loopback;   /**< Enable loopback mode for testing (messages sent are immediately received) */
} mcp2515_can_params_t;

/**
 * @brief Complete MCP25xxx device configuration
 * 
 * Combines all configuration aspects for a single MCP25xxx controller:
 * device ID, SPI connection, hardware settings, and CAN parameters.
 */
typedef struct {
    uint8_t                dev_id;     /**< User-assigned device identifier (0-255) */
    mcp_spi_dev_wiring_t   wiring;     /**< Device wiring (CS, INT, optional STBY/RST pins) */
    mcp_spi_dev_params_t   spi_params; /**< SPI communication parameters */
    mcp2515_hw_t           hw;         /**< MCP25xxx hardware configuration */
    mcp2515_can_params_t   can;        /**< CAN protocol parameters */
} mcp2515_device_config_t;

/**
 * @brief Bundle configuration: one SPI bus with multiple MCP25xxx devices
 * 
 * A bundle represents a complete setup with one SPI bus and one or more
 * MCP25xxx devices sharing that bus. This is the primary configuration
 * structure used for initialization.
 * 
 * @note All devices in a bundle share the same MISO/MOSI/SCLK pins but
 *       must have unique CS (Chip Select) pins.
 */
typedef struct {
    mcp_spi_bus_config_t       bus;          /**< SPI bus configuration (shared by all devices) */
    const mcp2515_device_config_t *devices;  /**< Array of device configurations */
    size_t                     device_count; /**< Number of devices in the array */
} mcp2515_bundle_config_t;


// ======================================================================================
// Identification types
// ======================================================================================

/**
 * @brief Opaque handle to a registered SPI bus
 * 
 * Obtained through registry functions (canif_bus_at, canif_bus_get_by_id, etc.).
 * Internals are hidden from the application.
 */
typedef struct can_bus_handle_s* can_bus_handle_t;

/**
 * @brief Opaque handle to a registered CAN device
 * 
 * Obtained through registry functions (canif_device_at, canif_dev_get_by_id, etc.).
 * Internals are hidden from the application.
 */
typedef struct can_dev_handle_s* can_dev_handle_t;

/**
 * @brief Composite target identifier combining bus_id and dev_id
 * 
 * A 16-bit value encoding both bus and device identifiers in a single compact form.
 * Upper 8 bits = bus_id, lower 8 bits = dev_id.
 */
typedef uint16_t can_target_t;

/**
 * @brief Packs bus_id and dev_id into a composite target
 * 
 * @param bus_id Bus identifier (0-255)
 * @param dev_id Device identifier (0-255)
 * @return Composite target value
 */
static inline can_target_t can_target_from_ids(can_bus_id_t bus_id, can_dev_id_t dev_id) {
    return (can_target_t)(((uint16_t)bus_id << 8) | (uint16_t)dev_id);
}

/**
 * @brief Extracts bus_id from a composite target
 * 
 * @param t Composite target value
 * @return Bus identifier (upper 8 bits)
 */
static inline can_bus_id_t can_target_bus_id(can_target_t t) { return (can_bus_id_t)(t >> 8); }

/**
 * @brief Extracts dev_id from a composite target
 * 
 * @param t Composite target value
 * @return Device identifier (lower 8 bits)
 */
static inline can_dev_id_t can_target_dev_id(can_target_t t) { return (can_dev_id_t)(t & 0xFF); }

// ======================================================================================
// Registry and lookup API
// ======================================================================================
/**
 * @defgroup registry Registry and Lookup API
 * @brief Functions for managing and querying registered buses and devices
 * 
 * The registry maintains information about all configured SPI buses and MCP25xxx
 * devices. Use these functions to enumerate, lookup, and validate handles.
 * @{
 */

/** @brief Returns the number of registered SPI buses */
size_t            canif_bus_count(void);

// Returns handle of bus at index in range [0, canif_bus_count()).
can_bus_handle_t  canif_bus_at(size_t index);

// Returns number of devices registered on a bus handle.
size_t            canif_bus_device_count(can_bus_handle_t bus);

// Returns handle of device at index on given bus.
can_dev_handle_t  canif_device_at(can_bus_handle_t bus, size_t index);

// Returns bus handle by user-assigned ID (or NULL if not found).
can_bus_handle_t  canif_bus_get_by_id(can_bus_id_t bus_id);

// Returns device handle by bus/device IDs (or NULL if not found).
can_dev_handle_t  canif_dev_get_by_id(can_bus_id_t bus_id, can_dev_id_t dev_id);

// Validates that a bus/device handle is non-NULL and currently registered.
bool              canif_is_valid_bus(can_bus_handle_t bus);
bool              canif_is_valid_device(can_dev_handle_t dev);

// Returns default bus/device handles (application-defined; typically first configured).
can_bus_handle_t  canif_bus_default(void);
can_dev_handle_t  canif_device_default(void);

// Clears the internal registry of registered bundles. Use before re-registering.
void              canif_clear_registry(void);

/**
 * @brief Registers a bundle (SPI bus + devices) into the registry
 * @param bundle Pointer to bundle configuration
 * @return true if registered successfully, false if registry is full
 */
bool              canif_register_bundle(const mcp2515_bundle_config_t* bundle);

/** @} */ // end of registry group

// ======================================================================================
// Messaging operations
// ======================================================================================
/**
 * @defgroup messaging Messaging Operations
 * @brief CAN message transmission and reception functions
 * 
 * These functions provide different ways to send and receive CAN messages:
 * - Direct handle-based access (fastest)
 * - ID-based lookup (flexible)
 * - Composite target (compact)
 * @{
 */

/**
 * @brief Sends a CAN message to the specified device
 * @param dev Device handle (must be valid and open)
 * @param msg Pointer to message structure
 * @return true on successful transmission initiation, false otherwise
 */
bool              canif_send_to(can_dev_handle_t dev, const twai_message_t* msg);

// Receives a frame from the specified device handle if available (non-blocking).
// Returns true if a frame was read into 'msg'.
bool              canif_receive_from(can_dev_handle_t dev, twai_message_t* msg);

// Sends a frame using numeric IDs; resolves to the target device at runtime.
bool              canif_send_id(can_bus_id_t bus_id, can_dev_id_t dev_id, const twai_message_t* msg);

// Receives a frame using numeric IDs; non-blocking.
bool              canif_receive_id(can_bus_id_t bus_id, can_dev_id_t dev_id, twai_message_t* msg);

// Sends a frame using a composite target (bus_id | dev_id).
bool              canif_send_target(can_target_t target, const twai_message_t* msg);

// Receives a frame using a composite target (non-blocking).
bool              canif_receive_target(can_target_t target, twai_message_t* msg);

/**
 * @brief High-level initialization helper: registers and opens a bundle
 * @param cfg Bundle configuration
 * @return true on success, false otherwise
 */
bool              canif_multi_init_default(const mcp2515_bundle_config_t* cfg);

/** @brief Deinitializes the default bundle */
bool              canif_multi_deinit_default(void);

/** @brief Sends a message using the default device */
bool              canif_multi_send_default(const twai_message_t* msg);

/** @brief Receives a message from the default device */
bool              canif_receive_default(twai_message_t* msg);

/** @} */ // end of messaging group

// ======================================================================================
// Initialization & lifecycle
// ======================================================================================
/**
 * @defgroup lifecycle Device Lifecycle Management
 * @brief Functions for opening, closing, and managing device lifecycle
 * 
 * Devices must be opened before use and should be closed when no longer needed.
 * Opening performs: SPI device binding, MCP25xxx reset, bitrate configuration,
 * and mode setting.
 * @{
 */

/**
 * @brief Opens a device for operation
 * 
 * Performs complete initialization: SPI binding, MCP25xxx reset, bitrate
 * configuration, and mode setting according to device configuration.
 * 
 * @param dev Device handle
 * @return true on success, false otherwise
 */
bool              canif_open_device(can_dev_handle_t dev);

// Closes a device: releases SPI device and related resources. Returns true on success.
bool              canif_close_device(can_dev_handle_t dev);

// Convenience wrappers using numeric IDs
bool              canif_open_id(can_bus_id_t bus_id, can_dev_id_t dev_id);
bool              canif_close_id(can_bus_id_t bus_id, can_dev_id_t dev_id);

// Convenience wrappers using composite target
bool              canif_open_target(can_target_t target);
bool              canif_close_target(can_target_t target);

/** @brief Opens all devices on a specific bus */
bool              canif_open_all_on_bus(can_bus_handle_t bus);

/** @brief Closes all devices on a specific bus */
bool              canif_close_all_on_bus(can_bus_handle_t bus);

/** @brief Opens all registered devices */
bool              canif_open_all(void);

/** @brief Closes all registered devices */
bool              canif_close_all(void);

/** @} */ // end of lifecycle group

// ======================================================================================
// Mode & bitrate control
// ======================================================================================
/**
 * @defgroup mode_control Mode and Bitrate Control
 * @brief Functions for changing CAN mode and bitrate at runtime
 * 
 * These functions allow dynamic reconfiguration of operating mode and communication speed.
 * @{
 */

/**
 * @brief Sets CAN bitrate for a device
 * @param dev Device handle
 * @param speed Desired CAN speed
 * @param clock Crystal frequency
 * @return true on success, false otherwise
 */
bool              canif_set_bitrate_to(can_dev_handle_t dev, mcp25xxx_speed_t speed, mcp25xxx_clock_t clock);

// Switches device mode.
bool              canif_set_mode_normal(can_dev_handle_t dev);
bool              canif_set_mode_loopback(can_dev_handle_t dev);

// ID-based convenience variants
bool              canif_set_bitrate_id(can_bus_id_t bus_id, can_dev_id_t dev_id, mcp25xxx_speed_t s, mcp25xxx_clock_t c);
bool              canif_set_mode_normal_id(can_bus_id_t bus_id, can_dev_id_t dev_id);
bool              canif_set_mode_loopback_id(can_bus_id_t bus_id, can_dev_id_t dev_id);

/** @brief Sets bitrate using composite target */
bool              canif_set_bitrate_target(can_target_t t, mcp25xxx_speed_t s, mcp25xxx_clock_t c);

/** @brief Sets normal mode using composite target */
bool              canif_set_mode_normal_target(can_target_t t);

/** @brief Sets loopback mode using composite target */
bool              canif_set_mode_loopback_target(can_target_t t);

/** @} */ // end of mode_control group

// ======================================================================================
// Events
// ======================================================================================
/**
 * @defgroup events Event Handling
 * @brief Event callback registration and waiting functions
 * 
 * Events include RX message arrival, errors, and other asynchronous notifications.
 * @{
 */

/**
 * @brief Event callback function type
 * @param dev Device that triggered the event
 * @param eventMask Bitmask of events (MCP25XXX_EVENT_* flags)
 * @param userData User data pointer passed during registration
 */
typedef void (*canif_event_cb)(can_dev_handle_t dev, uint32_t eventMask, void* userData);

// Registers or updates an event callback for the device.
void              canif_set_event_callback(can_dev_handle_t dev, canif_event_cb cb, void* userData);

/**
 * @brief Waits for device events with timeout
 * @param dev Device handle
 * @param timeout_ticks Timeout in FreeRTOS ticks (portMAX_DELAY for infinite)
 * @return Bitmask of received events (0 on timeout)
 */
uint32_t          canif_wait_for_event(can_dev_handle_t dev, uint32_t timeout_ticks);

/** @} */ // end of events group

// ======================================================================================
// Errors & diagnostics
// ======================================================================================
/**
 * @defgroup diagnostics Error and Diagnostics
 * @brief Functions for reading and clearing error conditions
 * @{
 */

/**
 * @brief Reads MCP25xxx error flags register (EFLG)
 * @param dev Device handle
 * @return Error flags byte (0 = no errors)
 */
uint8_t           canif_get_error_flags(can_dev_handle_t dev);

// Clears RX overrun related flags.
void              canif_clear_rx_overrun(can_dev_handle_t dev);

/** @brief Clears generic error interrupt flag */
void              canif_clear_error_int(can_dev_handle_t dev);

/** @} */ // end of diagnostics group

// ======================================================================================
// Filters & masks
// ======================================================================================
/**
 * @defgroup filters Acceptance Filters and Masks
 * @brief Hardware message filtering configuration
 * 
 * MCP25xxx provides 6 acceptance filters (0-5) and 2 masks (0-1).
 * Filters determine which CAN IDs are received; masks specify which bits are checked.
 * @{
 */

/**
 * @brief Configures an acceptance filter
 * @param dev Device handle
 * @param filter_idx Filter index (0-5)
 * @param extended true for 29-bit extended ID, false for 11-bit standard
 * @param id CAN identifier to match
 * @return true on success, false otherwise
 */
bool              canif_set_filter(can_dev_handle_t dev, uint8_t filter_idx, bool extended, uint32_t id);

/**
 * @brief Configures an acceptance mask
 * @param dev Device handle
 * @param mask_idx Mask index (0-1)
 * @param extended true for 29-bit extended ID, false for 11-bit standard
 * @param mask Bitmask (1 = must match, 0 = don't care)
 * @return true on success, false otherwise
 */
bool              canif_set_mask(can_dev_handle_t dev, uint8_t mask_idx, bool extended, uint32_t mask);

/** @} */ // end of filters group

// ======================================================================================
// Introspection & utilities
// ======================================================================================
/**
 * @defgroup introspection Introspection and Utilities
 * @brief Functions for querying device configuration and properties
 * @{
 */

/**
 * @brief Returns read-only device configuration
 * @param dev Device handle
 * @return Pointer to device configuration, or NULL if invalid
 */
const mcp2515_device_config_t*
                   canif_device_config(can_dev_handle_t dev);

/** @brief Extracts bus_id from a bus handle */
can_bus_id_t       canif_bus_id_of(can_bus_handle_t bus);

/** @brief Extracts dev_id from a device handle */
can_dev_id_t       canif_dev_id_of(can_dev_handle_t dev);

/**
 * @brief Get human-readable backend name for MCP25xxx multi adapter
 *
 * Standalone usage of this component gets its default implementation
 * from mcp25xxx_multi.c. In the multi-backend integration project,
 * a dispatcher layer may override this symbol to report a different
 * backend name while keeping the default available as a weak symbol.
 *
 * @return const char* Static C string with backend name
 */
const char *can_backend_get_name(void);

/** @} */ // end of introspection group

#ifdef __cplusplus
}
#endif




