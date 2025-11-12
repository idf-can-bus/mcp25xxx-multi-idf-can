/**
 * @file can_message.h
 * @brief CAN message structure definition for the MCP2515 multi-device library.
 * 
 * This header defines the unified CAN message structure used across all
 * MCP2515 devices in the library. It supports both standard (11-bit) and
 * extended (29-bit) CAN identifiers, as well as Remote Transmission Request (RTR) frames.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Maximum CAN message payload length in bytes (standard CAN 2.0) */
#define CANIF_MAX_DATA_LEN 8

/**
 * @brief CAN message structure
 * 
 * This structure represents a complete CAN frame with all necessary fields
 * for transmission and reception. It supports both standard (11-bit) and
 * extended (29-bit) identifiers.
 * 
 * @note For standard CAN frames, only the lower 11 bits of 'id' are significant.
 * @note For extended CAN frames, all 29 bits of 'id' are used.
 * @note The 'dlc' field must be in the range 0-8. Values larger than 8 are invalid.
 */
typedef struct {
    uint32_t id;                      /**< CAN identifier (11-bit standard or 29-bit extended) */
    bool extended_id;                 /**< Identifier format: true = 29-bit extended, false = 11-bit standard */
    bool rtr;                         /**< Remote Transmission Request flag */
    uint8_t dlc;                      /**< Data Length Code: number of valid data bytes (0-8) */
    uint8_t data[CANIF_MAX_DATA_LEN]; /**< Payload data buffer (up to 8 bytes) */
} can_message_t;

#ifdef __cplusplus
}
#endif

