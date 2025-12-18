/**
 * @file spsc_ring_buffer_seq_c.h
 * @brief Lock-free single-producer/single-consumer ring buffer (Pure C version)
 *
 * Pure C implementation of SPSC ring buffer with sequence numbers, designed for
 * ISR-safe operation on ESP32/Xtensa architecture.
 *
 * Key features:
 * - Lock-free SPSC pattern using C11 atomics
 * - Fixed-size payload (compile-time configuration)
 * - Drop-oldest policy on overflow
 * - Sequence numbers for diagnostics
 * - Bounded retry on torn reads
 *
 * ESP32/Xtensa Workarounds:
 * - Uses memcpy() instead of struct assignment (toolchain copy corruption issues)
 * - Computed value size via offsetof() (sizeof evaluation issues)
 * - Seqlock pattern with commit counter for tear-free overwrites
 * - Bounded retry limit to prevent infinite loops in consumer
 *
 * Threading Model:
 * - Producer: ISR or high-priority task (calls push functions)
 * - Consumer: Lower-priority task (calls try_pop)
 * - SPSC only - do not use with multiple producers or consumers
 *
 * @warning ESP32 Dual-Core: Pin producer and consumer to same core or use single-core chip.
 * @warning Capacity MUST be a power of 2 (2, 4, 8, 16, 32, 64, 128, 256, ...)
 *
 * @author Ivo Marvan (based on C++ template version)
 * @date 2025
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdatomic.h>

#ifdef __cplusplus
extern "C" {
#endif

// Configuration: Define payload size before including this header
// Example: #define SPSC_RING_PAYLOAD_SIZE 64
#ifndef SPSC_RING_PAYLOAD_SIZE
#define SPSC_RING_PAYLOAD_SIZE 64  // Default: enough for CAN frame (13 bytes)
#endif

// Configuration: Ring buffer capacity (must be power of 2)
#ifndef SPSC_RING_CAPACITY
#define SPSC_RING_CAPACITY 128  // Default capacity
#endif

// Configuration: Maximum retries for torn read detection
#ifndef SPSC_RING_MAX_RETRIES
#define SPSC_RING_MAX_RETRIES 1000
#endif

// Compile-time check: Capacity must be power of 2
_Static_assert((SPSC_RING_CAPACITY & (SPSC_RING_CAPACITY - 1)) == 0,
               "SPSC_RING_CAPACITY must be a power of 2");
_Static_assert(SPSC_RING_CAPACITY >= 2, "SPSC_RING_CAPACITY must be >= 2");

/**
 * @brief Sample stored in ring buffer slot.
 *
 * Contains timestamp, sequence number, and user payload.
 * Aligned to 8 bytes for efficient atomic operations.
 */
typedef struct __attribute__((aligned(8))) {
    uint32_t time100us;                      ///< Timestamp in 100us units
    uint32_t seq;                            ///< Monotonic sequence number
    uint8_t  value[SPSC_RING_PAYLOAD_SIZE];  ///< User payload
} spsc_sample_t;

/**
 * @brief Ring buffer slot with sequence lock.
 *
 * The commit counter implements seqlock pattern:
 * - Even value: slot is stable (not being written)
 * - Odd value: writer is updating the slot
 */
typedef struct __attribute__((aligned(8))) {
    _Atomic uint32_t commit;  ///< Seqlock commit counter (odd = writing, even = stable)
    spsc_sample_t data;       ///< Sample data
} spsc_slot_t;

/**
 * @brief SPSC ring buffer instance.
 *
 * Opaque structure - do not access fields directly.
 * Use provided API functions.
 */
struct spsc_ring_buffer_s {
    spsc_slot_t slots[SPSC_RING_CAPACITY];  ///< Ring buffer slots
    
    // Producer owns tail, consumer owns head
    _Atomic uint32_t head;  ///< Read index (consumer-owned)
    _Atomic uint32_t tail;  ///< Write index (producer-owned)
    
    _Atomic uint32_t seq_counter;  ///< Monotonic sequence number
    
    // Consumer-only counters (no concurrent modification from producer)
    _Atomic uint32_t dropped_overflow;        ///< Dropped since last reset
    _Atomic uint32_t dropped_overflow_total;  ///< Total dropped (monotonic)
    _Atomic uint32_t torn_read_failures;      ///< Torn read failures (monotonic)
};

typedef struct spsc_ring_buffer_s spsc_ring_buffer_t;

// Bit mask for efficient index calculation
#define SPSC_RING_INDEX_MASK (SPSC_RING_CAPACITY - 1)

// Computed value size (workaround for ESP32/Xtensa sizeof issues)
#define SPSC_RING_VALUE_SIZE \
    (sizeof(spsc_sample_t) - offsetof(spsc_sample_t, value))

/**
 * @brief Initialize ring buffer to empty state.
 *
 * Must be called before using the buffer.
 * Safe to call multiple times.
 *
 * @param ring Pointer to ring buffer instance
 */
void spsc_ring_init(spsc_ring_buffer_t* ring);

/**
 * @brief Push a new sample into the ring buffer (ISR-safe).
 *
 * Non-blocking, constant-time operation. If buffer is full, overwrites
 * the oldest element (drop-oldest policy).
 *
 * @param ring Pointer to ring buffer
 * @param time100us Timestamp in 100us units
 * @param value Pointer to payload data (SPSC_RING_PAYLOAD_SIZE bytes, copied via memcpy)
 */
void spsc_ring_push_isr(spsc_ring_buffer_t* ring, uint32_t time100us, const void* value);

/**
 * @brief Try to pop one sample from the ring buffer.
 *
 * Non-blocking. Returns false if buffer is empty or torn read retry limit exceeded.
 *
 * @param ring Pointer to ring buffer
 * @param out_sample Pointer to receive the popped sample
 * @return true if sample was successfully popped, false if empty or torn read failure
 *
 * @note On torn read failure, torn_read_failures counter is incremented.
 */
bool spsc_ring_try_pop(spsc_ring_buffer_t* ring, spsc_sample_t* out_sample);

/**
 * @brief Check if ring buffer is empty (best-effort).
 *
 * @param ring Pointer to ring buffer
 * @return true if buffer appears empty, false otherwise
 *
 * @note In SPSC pattern this is safe, but producer may push immediately after.
 */
bool spsc_ring_empty(const spsc_ring_buffer_t* ring);

/**
 * @brief Get current buffer size (best-effort snapshot).
 *
 * @param ring Pointer to ring buffer
 * @return Number of elements currently in buffer
 */
size_t spsc_ring_size(const spsc_ring_buffer_t* ring);

/**
 * @brief Get and reset overflow drop counter.
 *
 * @param ring Pointer to ring buffer
 * @return Number of dropped elements since last reset
 */
uint32_t spsc_ring_take_dropped(spsc_ring_buffer_t* ring);

/**
 * @brief Get total overflow drops (monotonic).
 *
 * @param ring Pointer to ring buffer
 * @return Total number of dropped elements since initialization
 */
uint32_t spsc_ring_dropped_total(const spsc_ring_buffer_t* ring);

/**
 * @brief Get total torn read failures (monotonic).
 *
 * @param ring Pointer to ring buffer
 * @return Total torn read failures (indicates stuck producer or timing issues)
 */
uint32_t spsc_ring_torn_failures(const spsc_ring_buffer_t* ring);

/**
 * @brief Get last assigned sequence number.
 *
 * @param ring Pointer to ring buffer
 * @return Last sequence number assigned on push
 */
uint32_t spsc_ring_last_seq(const spsc_ring_buffer_t* ring);

/**
 * @brief Clear buffer to empty state (unsafe - call only when producer stopped).
 *
 * Resets head and tail to 0. Does not reset drop counters or sequence counter.
 *
 * @param ring Pointer to ring buffer
 */
void spsc_ring_clear_unsafe(spsc_ring_buffer_t* ring);

#ifdef __cplusplus
}
#endif

