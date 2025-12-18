/**
 * @file spsc_ring_buffer_seq_c.c
 * @brief Implementation of lock-free SPSC ring buffer (Pure C)
 *
 * @author Ivo Marvan
 * @date 2025
 */

#include "spsc_ring_buffer_seq_c.h"
#include <string.h>

// Ensure struct size is reasonable
_Static_assert(sizeof(spsc_ring_buffer_t) < 100000, "Ring buffer too large");

/**
 * @brief Compute slot index from counter using bitmask.
 *
 * Requires SPSC_RING_CAPACITY to be a power of 2.
 * This is faster than modulo operation.
 */
static inline size_t slot_index(uint32_t counter) {
    return (size_t)(counter & SPSC_RING_INDEX_MASK);
}

void spsc_ring_init(spsc_ring_buffer_t* ring) {
    if (!ring) return;
    
    // Initialize all slots
    for (size_t i = 0; i < SPSC_RING_CAPACITY; i++) {
        atomic_store_explicit(&ring->slots[i].commit, 0, memory_order_relaxed);
        memset(&ring->slots[i].data, 0, sizeof(spsc_sample_t));
    }
    
    // Initialize indices
    atomic_store_explicit(&ring->head, 0, memory_order_relaxed);
    atomic_store_explicit(&ring->tail, 0, memory_order_relaxed);
    
    // Initialize counters
    atomic_store_explicit(&ring->seq_counter, 0, memory_order_relaxed);
    atomic_store_explicit(&ring->dropped_overflow, 0, memory_order_relaxed);
    atomic_store_explicit(&ring->dropped_overflow_total, 0, memory_order_relaxed);
    atomic_store_explicit(&ring->torn_read_failures, 0, memory_order_relaxed);
}

void spsc_ring_push_isr(spsc_ring_buffer_t* ring, uint32_t time100us, const void* value) {
    if (!ring || !value) return;
    
    const uint32_t t = atomic_load_explicit(&ring->tail, memory_order_relaxed);
    const size_t idx = slot_index(t);
    spsc_slot_t* slot = &ring->slots[idx];
    
    // Seqlock write: increment commit to odd (writer in progress)
    (void)atomic_fetch_add_explicit(&slot->commit, 1, memory_order_acq_rel);
    
    // Write timestamp and sequence
    slot->data.time100us = time100us;
    slot->data.seq = atomic_fetch_add_explicit(&ring->seq_counter, 1, memory_order_relaxed);
    
    // CRITICAL ESP32 WORKAROUND: Use memcpy instead of struct assignment.
    // ESP32/Xtensa GCC has known issues with aggregate copy via assignment
    // that can corrupt data. memcpy with computed size is reliable.
    // Use SPSC_RING_VALUE_SIZE (computed via offsetof) to work around sizeof() issues.
    memcpy(slot->data.value, value, SPSC_RING_VALUE_SIZE);
    
    // Seqlock write: increment commit to even (stable)
    (void)atomic_fetch_add_explicit(&slot->commit, 1, memory_order_release);
    
    // Publish new tail (producer-owned, never blocks, never touches head)
    atomic_store_explicit(&ring->tail, t + 1, memory_order_release);
}

bool spsc_ring_try_pop(spsc_ring_buffer_t* ring, spsc_sample_t* out_sample) {
    if (!ring || !out_sample) return false;
    
    uint32_t h = atomic_load_explicit(&ring->head, memory_order_relaxed);
    const uint32_t t = atomic_load_explicit(&ring->tail, memory_order_acquire);
    
    // Check if buffer is empty
    if (h == t) {
        return false;
    }
    
    // Consumer-owned drop-oldest policy:
    // Effective capacity is (SPSC_RING_CAPACITY - 1) elements.
    // If producer ran far ahead, catch up head to keep only the most recent items.
    const uint32_t available = t - h;
    const uint32_t max_keep = SPSC_RING_CAPACITY - 1;
    uint32_t h_read = h;
    
    if (available > max_keep) {
        const uint32_t new_h = t - max_keep;
        const uint32_t dropped = new_h - h;
        
        // Update drop counters (consumer-only, no concurrent access)
        uint32_t curr_drop = atomic_load_explicit(&ring->dropped_overflow, memory_order_relaxed);
        atomic_store_explicit(&ring->dropped_overflow, curr_drop + dropped, memory_order_relaxed);
        
        uint32_t total_drop = atomic_load_explicit(&ring->dropped_overflow_total, memory_order_relaxed);
        atomic_store_explicit(&ring->dropped_overflow_total, total_drop + dropped, memory_order_relaxed);
        
        h_read = new_h;
    }
    
    // Tear-free read with seqlock and bounded retries
    const size_t idx = slot_index(h_read);
    spsc_slot_t* slot = &ring->slots[idx];
    
    size_t retries = 0;
    while (true) {
        const uint32_t c1 = atomic_load_explicit(&slot->commit, memory_order_acquire);
        
        // Check if writer is in progress (commit is odd)
        if (c1 & 1u) {
            // Writer in progress - retry with limit check
            if (SPSC_RING_MAX_RETRIES > 0) {
                if (++retries > SPSC_RING_MAX_RETRIES) {
                    // Producer appears stuck - increment failure counter and give up
                    uint32_t failures = atomic_load_explicit(&ring->torn_read_failures, memory_order_relaxed);
                    atomic_store_explicit(&ring->torn_read_failures, failures + 1, memory_order_relaxed);
                    return false;
                }
            }
            continue;  // Retry
        }
        
        // CRITICAL ESP32 WORKAROUND: Use memcpy for entire sample.
        // Direct struct assignment can produce corrupted copies on ESP32/Xtensa.
        memcpy(out_sample, &slot->data, sizeof(spsc_sample_t));
        
        // Memory barrier before re-checking commit
        atomic_thread_fence(memory_order_acquire);
        const uint32_t c2 = atomic_load_explicit(&slot->commit, memory_order_acquire);
        
        // Check if read was consistent (commit didn't change during copy)
        if (c1 == c2) {
            // Consistent read - success
            break;
        }
        
        // Commit changed during read (torn read) - retry with limit check
        if (SPSC_RING_MAX_RETRIES > 0) {
            if (++retries > SPSC_RING_MAX_RETRIES) {
                uint32_t failures = atomic_load_explicit(&ring->torn_read_failures, memory_order_relaxed);
                atomic_store_explicit(&ring->torn_read_failures, failures + 1, memory_order_relaxed);
                return false;
            }
        }
    }
    
    // Update head to mark element as consumed
    atomic_store_explicit(&ring->head, h_read + 1, memory_order_release);
    return true;
}

bool spsc_ring_empty(const spsc_ring_buffer_t* ring) {
    if (!ring) return true;
    
    const uint32_t h = atomic_load_explicit(&ring->head, memory_order_relaxed);
    const uint32_t t = atomic_load_explicit(&ring->tail, memory_order_acquire);
    return h == t;
}

size_t spsc_ring_size(const spsc_ring_buffer_t* ring) {
    if (!ring) return 0;
    
    const uint32_t h = atomic_load_explicit(&ring->head, memory_order_relaxed);
    const uint32_t t = atomic_load_explicit(&ring->tail, memory_order_acquire);
    return (size_t)(t - h);
}

uint32_t spsc_ring_take_dropped(spsc_ring_buffer_t* ring) {
    if (!ring) return 0;
    
    return atomic_exchange_explicit(&ring->dropped_overflow, 0, memory_order_relaxed);
}

uint32_t spsc_ring_dropped_total(const spsc_ring_buffer_t* ring) {
    if (!ring) return 0;
    
    return atomic_load_explicit(&ring->dropped_overflow_total, memory_order_relaxed);
}

uint32_t spsc_ring_torn_failures(const spsc_ring_buffer_t* ring) {
    if (!ring) return 0;
    
    return atomic_load_explicit(&ring->torn_read_failures, memory_order_relaxed);
}

uint32_t spsc_ring_last_seq(const spsc_ring_buffer_t* ring) {
    if (!ring) return 0;
    
    return atomic_load_explicit(&ring->seq_counter, memory_order_relaxed);
}

void spsc_ring_clear_unsafe(spsc_ring_buffer_t* ring) {
    if (!ring) return;
    
    atomic_store_explicit(&ring->head, 0, memory_order_relaxed);
    atomic_store_explicit(&ring->tail, 0, memory_order_relaxed);
    // Note: Does not reset drop counters or seq counter
}

