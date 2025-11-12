/**
 * @file examples_utils.h
 * @brief Utility functions for CAN message testing and logging
 * 
 * This module provides helper functions for the example applications:
 * - Test message generation with heartbeat and timestamp
 * - Message logging and statistics
 * - Multi-sender tracking with per-sender statistics
 * - Portable sleep functions
 * 
 * @note These utilities are designed for examples and testing, not for production use.
 */

#pragma once

#include <stdint.h>
#include <string.h>
#include "can_message.h"

#ifdef __cplusplus
extern "C" {
#endif


// ------------------------------------------------------------------------------------------------
// CAN message payload structures and accessors
// ------------------------------------------------------------------------------------------------
/**
 * @defgroup message_types Test Message Types
 * @brief Predefined message structures for testing
 * @{
 */

/** @brief CAN ID for test messages with heartbeat/timestamp */
#define TEST_MSG_ID ((uint8_t)1)

/** @brief 40-bit timestamp stored as 5 bytes (big-endian) */
typedef uint8_t unit40_big_endian_t[5];

/**
 * @brief Test message flags
 * 
 * Flags embedded in test messages for controlling behavior and requesting actions.
 */
typedef enum {
    TEST_FLAG_STATS_REQUEST = 1u << 0,  /**< Request sender to print statistics */
    TEST_FLAG_RESERVED_1    = 1u << 1,  /**< Reserved for future use */
    TEST_FLAG_RESERVED_2    = 1u << 2,  /**< Reserved for future use */
    TEST_FLAG_RESERVED_3    = 1u << 3,  /**< Reserved for future use */
} test_flag_bits_t;

typedef struct {
    uint8_t value;
} test_flags_t;

static inline void test_flags_set(test_flags_t *f, test_flag_bits_t bit) { f->value |= (uint8_t)bit; }
static inline bool test_flags_is_set(const test_flags_t *f, test_flag_bits_t bit) { return (f->value & (uint8_t)bit) != 0; }

/**
 * @brief Test CAN message payload structure (8 bytes)
 * 
 * Layout:
 * - Byte 0: sender_id (identifies message source)
 * - Byte 1: heartbeat (sequence number, wraps at 255)
 * - Byte 2: flags (control flags, see test_flag_bits_t)
 * - Bytes 3-7: timestamp40 (40-bit microsecond timestamp, big-endian)
 * 
 * This structure is used for testing message delivery, latency, and ordering.
 */
typedef struct __attribute__((packed)) {
    uint8_t sender_id;                /**< Message sender identifier */
    uint8_t heartbeat;                /**< Sequence number (increments with each message) */
    test_flags_t flags;               /**< Control flags */
    unit40_big_endian_t timestamp40;  /**< 40-bit timestamp in microseconds */
} test_can_message_t;

/** @brief CAN ID for message with two uint32 values */
#define CAN_MSG_WITH_TWO_UINT32_ID ((uint8_t)2)

/** @brief Message payload: two 32-bit unsigned integers */
typedef struct  __attribute__((packed)) {
    uint32_t value_uint32_1;  /**< First 32-bit value (bytes 0-3) */
    uint32_t value_uint32_2;  /**< Second 32-bit value (bytes 4-7) */
} can_message_with_two_uint32_t;

/** @brief CAN ID for message with one uint64 value */
#define CAN_MSG_WITH_ONE_UINT64_ID ((uint8_t)3)

/** @brief Message payload: single 64-bit unsigned integer */
typedef struct  __attribute__((packed)) {
    uint64_t value_uint64;    /**< 64-bit value (bytes 0-7) */
} can_message_with_one_uint64_t; 

/** @brief CAN ID for generic 8-byte array message */
#define EIGHT_BYTES_ARRAY_MESSAGE_ID ((uint8_t)4)

/** @brief Message payload: 8-byte array */
typedef struct  __attribute__((packed)) {
    uint8_t data[8];          /**< 8 bytes of data */
} eight_bytes_array_message_t;

/**
 * @brief Union for different payload interpretations
 * 
 * Allows viewing the same 8-byte CAN payload as different data types.
 * This demonstrates type-safe message parsing based on CAN ID.
 */
typedef union {
    test_can_message_t test_message;                    /**< Test message with heartbeat */
    can_message_with_two_uint32_t two_uint32_message;   /**< Two uint32 values */
    can_message_with_one_uint64_t one_uint64_message;   /**< One uint64 value */
    eight_bytes_array_message_t eight_bytes_array_message; /**< Raw byte array */
} can_message_payload_t;

/** @} */ // end of message_types group

// ------------------------------------------------------------------------------------------------
// Timestamp utilities
// ------------------------------------------------------------------------------------------------
/**
 * @defgroup timestamp Timestamp Utilities
 * @brief 40-bit timestamp encoding/decoding
 * @{
 */

/**
 * @brief Stores 40 least significant bits of 64-bit value into 5-byte array (big-endian)
 * @param source 64-bit source value (e.g., microsecond timestamp)
 * @param target_ptr Pointer to 5-byte destination array
 */
void store_timestamp40(uint64_t source, unit40_big_endian_t *target_ptr);

/**
 * @brief Restores 5-byte big-endian timestamp back to 64-bit value
 * @param src_ptr Pointer to 5-byte source array
 * @return Reconstructed 64-bit value
 */
uint64_t restore_timestamp40(const unit40_big_endian_t *src_ptr);

/** @} */ // end of timestamp group

// ------------------------------------------------------------------------------------------------
// Message generation and manipulation
// ------------------------------------------------------------------------------------------------
/**
 * @defgroup message_gen Message Generation
 * @brief Test message creation and modification
 * @{
 */

/**
 * @brief Generates a test CAN message with heartbeat and timestamp
 * @param sender_id Sender identifier (0-255)
 * @param heartbeat Sequence number (auto-increments)
 * @param message Pointer to message structure to fill
 */
void fullfill_test_messages(uint8_t sender_id, uint8_t heartbeat, can_message_t *message);

/**
 * @brief Sets a flag in an already-prepared test message
 * @param message Pointer to message with test_can_message_t payload
 * @param flag Flag bit to set (test_flag_bits_t)
 */
static inline void set_test_flag(can_message_t *message, uint8_t flag)
{
    test_can_message_t *p = (test_can_message_t*)message->data;
    test_flags_set(&p->flags, (test_flag_bits_t)flag);
}

/** @} */ // end of message_gen group

// ------------------------------------------------------------------------------------------------
// Message processing and logging
// ------------------------------------------------------------------------------------------------
/**
 * @defgroup processing Message Processing
 * @brief Message logging, statistics, and debugging
 * @{
 */

/**
 * @brief Prints CAN message details to console
 * @param message Pointer to message to print
 */
void print_can_message(const can_message_t *message);

/**
 * @brief Increments heartbeat value (wraps at 255)
 * @param heartbeat Current heartbeat value
 * @return Next heartbeat value (current + 1, mod 256)
 */
uint8_t next_heartbeat(const uint8_t heartbeat);

/**
 * @brief Processes received message with single-sender statistics
 * @param message Pointer to received message
 * @param print_during_receive If true, prints full message details
 */
void process_received_message(can_message_t *message, const bool print_during_receive);

/**
 * @brief Processes received message with per-sender statistics tracking
 * 
 * Maintains separate statistics for up to 256 different senders, tracking
 * sequence order, lost messages, and duplicates per sender.
 * 
 * @param message Pointer to received message
 * @param print_during_receive If true, prints full message details
 */
void process_received_message_multi(can_message_t *message, const bool print_during_receive);

/**
 * @brief Logs sent message (for debugging)
 * @param message Pointer to sent message
 * @param print_during_send If true, prints full message details
 */
void debug_send_message(can_message_t *message, const bool print_during_send);

/**
 * @brief Generic message logging function
 * @param send true for TX messages, false for RX messages
 * @param message Pointer to message
 * @param print_details If true, prints full details; otherwise compact visualization
 */
void log_message(const bool send, can_message_t *message, const bool print_details);

/** @} */ // end of processing group

// ------------------------------------------------------------------------------------------------
// System utilities
// ------------------------------------------------------------------------------------------------
/**
 * @defgroup system System Utilities
 * @brief Sleep and timing functions
 * @{
 */

/**
 * @brief Sleeps for specified milliseconds, ensuring at least one RTOS tick
 * 
 * This function prevents zero-tick delays, which would return immediately.
 * Useful for short delays in FreeRTOS tasks.
 * 
 * @param ms Delay in milliseconds
 */
void sleep_ms_min_ticks(uint32_t ms);

/** @} */ // end of system group

#ifdef __cplusplus
}
#endif