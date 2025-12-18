/**
 * @file test_spsc_ring_buffer_seq_c.c
 * @brief Unit tests for SPSC ring buffer (Pure C version)
 *
 * Tests cover:
 * - Basic push/pop operations
 * - Overflow behavior (drop-oldest)
 * - Sequence number consistency
 * - Torn read detection
 * - Empty/size queries
 * - ESP32-specific workarounds
 *
 * @author Ivo Marvan
 * @date 2025
 */

#include <string.h>
#include "unity.h"
#include "spsc_ring_buffer_seq_c.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "TEST_SPSC_C";

// Test payload structure (must fit in SPSC_RING_PAYLOAD_SIZE)
typedef struct {
    uint32_t id;
    uint32_t data;
    uint8_t  bytes[8];
} test_payload_t;

_Static_assert(sizeof(test_payload_t) <= SPSC_RING_PAYLOAD_SIZE, 
               "test_payload_t too large for configured SPSC_RING_PAYLOAD_SIZE");

static spsc_ring_buffer_t ring;

void setUp(void) {
    spsc_ring_init(&ring);
}

void tearDown(void) {
    // Cleanup if needed
}

/**
 * @brief Test basic initialization.
 */
void test_init(void) {
    TEST_ASSERT_TRUE(spsc_ring_empty(&ring));
    TEST_ASSERT_EQUAL(0, spsc_ring_size(&ring));
    TEST_ASSERT_EQUAL(0, spsc_ring_last_seq(&ring));
    TEST_ASSERT_EQUAL(0, spsc_ring_dropped_total(&ring));
    TEST_ASSERT_EQUAL(0, spsc_ring_torn_failures(&ring));
}

/**
 * @brief Test single push and pop.
 */
void test_push_pop_single(void) {
    test_payload_t payload_in = {
        .id = 42,
        .data = 0xDEADBEEF,
        .bytes = {1, 2, 3, 4, 5, 6, 7, 8}
    };
    
    spsc_ring_push_isr(&ring, 1234, &payload_in);
    
    TEST_ASSERT_FALSE(spsc_ring_empty(&ring));
    TEST_ASSERT_EQUAL(1, spsc_ring_size(&ring));
    
    spsc_sample_t sample_out;
    TEST_ASSERT_TRUE(spsc_ring_try_pop(&ring, &sample_out));
    
    TEST_ASSERT_EQUAL(1234, sample_out.time100us);
    TEST_ASSERT_EQUAL(0, sample_out.seq);
    
    test_payload_t* payload_out = (test_payload_t*)sample_out.value;
    TEST_ASSERT_EQUAL(42, payload_out->id);
    TEST_ASSERT_EQUAL(0xDEADBEEF, payload_out->data);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(payload_in.bytes, payload_out->bytes, 8);
    
    TEST_ASSERT_TRUE(spsc_ring_empty(&ring));
}

/**
 * @brief Test multiple push/pop operations.
 */
void test_push_pop_multiple(void) {
    const int count = 10;
    
    for (int i = 0; i < count; i++) {
        test_payload_t payload = { .id = (uint32_t)i, .data = (uint32_t)(i * 100) };
        spsc_ring_push_isr(&ring, (uint32_t)(i * 10), &payload);
    }
    
    TEST_ASSERT_EQUAL(count, spsc_ring_size(&ring));
    
    for (int i = 0; i < count; i++) {
        spsc_sample_t sample;
        TEST_ASSERT_TRUE(spsc_ring_try_pop(&ring, &sample));
        
        TEST_ASSERT_EQUAL(i * 10, sample.time100us);
        TEST_ASSERT_EQUAL(i, sample.seq);
        
        test_payload_t* payload = (test_payload_t*)sample.value;
        TEST_ASSERT_EQUAL(i, payload->id);
        TEST_ASSERT_EQUAL(i * 100, payload->data);
    }
    
    TEST_ASSERT_TRUE(spsc_ring_empty(&ring));
}

/**
 * @brief Test overflow behavior (drop-oldest policy).
 *
 * Drop-oldest logic is executed in try_pop() when consumer detects
 * that producer has overrun the effective capacity.
 */
void test_overflow_drop_oldest(void) {
    // Fill buffer to effective capacity
    const size_t max_fill = SPSC_RING_CAPACITY - 1;
    
    for (size_t i = 0; i < max_fill; i++) {
        test_payload_t payload = { .id = (uint32_t)i };
        spsc_ring_push_isr(&ring, (uint32_t)i, &payload);
    }
    
    TEST_ASSERT_EQUAL(max_fill, spsc_ring_size(&ring));
    TEST_ASSERT_EQUAL(0, spsc_ring_dropped_total(&ring));
    
    // Push one more - producer doesn't block, just overwrites
    test_payload_t overflow_payload = { .id = 999 };
    spsc_ring_push_isr(&ring, 999, &overflow_payload);
    
    // NOTE: size() returns raw (tail - head) BEFORE consumer processes drops.
    // After pushing beyond capacity, size will be CAPACITY (128), not CAPACITY-1 (127).
    TEST_ASSERT_EQUAL(SPSC_RING_CAPACITY, spsc_ring_size(&ring));
    
    // First try_pop will detect overflow and skip oldest element(s)
    spsc_sample_t sample;
    TEST_ASSERT_TRUE(spsc_ring_try_pop(&ring, &sample));
    
    // Should get SECOND element (id=1), because first (id=0) was dropped
    test_payload_t* payload = (test_payload_t*)sample.value;
    TEST_ASSERT_EQUAL(1, payload->id);
    
    // Dropped counter should show 1 element was skipped
    TEST_ASSERT_EQUAL(1, spsc_ring_dropped_total(&ring));
    
    // After drop detection, size should be back to effective capacity - 1
    TEST_ASSERT_EQUAL(max_fill - 1, spsc_ring_size(&ring));
}

/**
 * @brief Test sequence number continuity.
 */
void test_sequence_numbers(void) {
    const int count = 20;
    
    for (int i = 0; i < count; i++) {
        test_payload_t payload = { .id = (uint32_t)i };
        spsc_ring_push_isr(&ring, 0, &payload);
    }
    
    for (int i = 0; i < count; i++) {
        spsc_sample_t sample;
        TEST_ASSERT_TRUE(spsc_ring_try_pop(&ring, &sample));
        TEST_ASSERT_EQUAL(i, sample.seq);
    }
    
    TEST_ASSERT_EQUAL(count, spsc_ring_last_seq(&ring));
}

/**
 * @brief Test pop from empty buffer.
 */
void test_pop_empty(void) {
    spsc_sample_t sample;
    TEST_ASSERT_FALSE(spsc_ring_try_pop(&ring, &sample));
    TEST_ASSERT_TRUE(spsc_ring_empty(&ring));
}

/**
 * @brief Test clear operation.
 */
void test_clear(void) {
    // Add some elements
    for (int i = 0; i < 5; i++) {
        test_payload_t payload = { .id = (uint32_t)i };
        spsc_ring_push_isr(&ring, 0, &payload);
    }
    
    TEST_ASSERT_EQUAL(5, spsc_ring_size(&ring));
    
    spsc_ring_clear_unsafe(&ring);
    
    TEST_ASSERT_TRUE(spsc_ring_empty(&ring));
    TEST_ASSERT_EQUAL(0, spsc_ring_size(&ring));
    
    // Sequence counter should NOT be reset
    TEST_ASSERT_EQUAL(5, spsc_ring_last_seq(&ring));
}

/**
 * @brief Test drop counter reset.
 */
void test_drop_counter_reset(void) {
    // Fill buffer and cause overflow
    for (size_t i = 0; i < SPSC_RING_CAPACITY + 10; i++) {
        test_payload_t payload = { .id = (uint32_t)i };
        spsc_ring_push_isr(&ring, 0, &payload);
    }
    
    // Pop one element to trigger drop detection
    spsc_sample_t sample;
    TEST_ASSERT_TRUE(spsc_ring_try_pop(&ring, &sample));
    
    uint32_t dropped = spsc_ring_take_dropped(&ring);
    TEST_ASSERT_TRUE(dropped > 0);
    
    // After reset, counter should be 0
    TEST_ASSERT_EQUAL(0, spsc_ring_take_dropped(&ring));
    
    // But total should remain
    TEST_ASSERT_TRUE(spsc_ring_dropped_total(&ring) > 0);
}

/**
 * @brief Test wraparound of indices.
 */
void test_wraparound(void) {
    // Simulate many operations to cause wraparound
    for (uint32_t cycle = 0; cycle < 3; cycle++) {
        for (uint32_t i = 0; i < SPSC_RING_CAPACITY / 2; i++) {
            test_payload_t payload = { .id = cycle * 1000 + i };
            spsc_ring_push_isr(&ring, cycle * 1000 + i, &payload);
        }
        
        for (uint32_t i = 0; i < SPSC_RING_CAPACITY / 2; i++) {
            spsc_sample_t sample;
            TEST_ASSERT_TRUE(spsc_ring_try_pop(&ring, &sample));
            
            test_payload_t* payload = (test_payload_t*)sample.value;
            TEST_ASSERT_EQUAL(cycle * 1000 + i, payload->id);
        }
    }
    
    TEST_ASSERT_TRUE(spsc_ring_empty(&ring));
}

/**
 * @brief Test with CAN-like payload.
 */
void test_can_frame_payload(void) {
    // Simulate CAN frame structure
    typedef struct {
        uint32_t can_id;
        uint8_t  can_dlc;
        uint8_t  data[8];
    } can_frame_t;
    
    _Static_assert(sizeof(can_frame_t) <= SPSC_RING_PAYLOAD_SIZE, 
                   "CAN frame too large");
    
    can_frame_t frame_in = {
        .can_id = 0x123,
        .can_dlc = 8,
        .data = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22}
    };
    
    spsc_ring_push_isr(&ring, 5000, &frame_in);
    
    spsc_sample_t sample;
    TEST_ASSERT_TRUE(spsc_ring_try_pop(&ring, &sample));
    
    can_frame_t* frame_out = (can_frame_t*)sample.value;
    TEST_ASSERT_EQUAL(0x123, frame_out->can_id);
    TEST_ASSERT_EQUAL(8, frame_out->can_dlc);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(frame_in.data, frame_out->data, 8);
}

/**
 * @brief Test memcpy workaround for ESP32.
 *
 * Verifies that payload is copied correctly using memcpy,
 * not struct assignment (which can corrupt on ESP32/Xtensa).
 */
void test_esp32_memcpy_workaround(void) {
    // Create payload with specific bit pattern
    test_payload_t payload_in = {
        .id = 0x12345678,
        .data = 0xABCDEF01,
        .bytes = {0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA, 0x99, 0x88}
    };
    
    spsc_ring_push_isr(&ring, 100, &payload_in);
    
    spsc_sample_t sample;
    TEST_ASSERT_TRUE(spsc_ring_try_pop(&ring, &sample));
    
    // Verify byte-by-byte copy (not corrupted by assignment operator)
    test_payload_t* payload_out = (test_payload_t*)sample.value;
    
    TEST_ASSERT_EQUAL_HEX32(0x12345678, payload_out->id);
    TEST_ASSERT_EQUAL_HEX32(0xABCDEF01, payload_out->data);
    
    // Critical: verify entire struct was copied correctly
    TEST_ASSERT_EQUAL_MEMORY(&payload_in, payload_out, sizeof(test_payload_t));
}

/**
 * @brief Test capacity boundary behavior.
 *
 * Verifies that ring buffer correctly handles capacity limits.
 * Drop detection happens in try_pop(), not in push().
 */
void test_capacity_boundary(void) {
    // Effective capacity is (CAPACITY - 1) due to ring buffer design
    const size_t effective_capacity = SPSC_RING_CAPACITY - 1;
    
    // Fill to effective capacity
    for (size_t i = 0; i < effective_capacity; i++) {
        test_payload_t payload = { .id = (uint32_t)i };
        spsc_ring_push_isr(&ring, (uint32_t)i, &payload);
    }
    
    TEST_ASSERT_EQUAL(effective_capacity, spsc_ring_size(&ring));
    
    // Push one more element beyond effective capacity
    test_payload_t overflow = { .id = 9999 };
    spsc_ring_push_isr(&ring, 9999, &overflow);
    
    // size() returns raw (tail - head), which can exceed effective capacity
    // before consumer detects overflow in try_pop()
    TEST_ASSERT_EQUAL(SPSC_RING_CAPACITY, spsc_ring_size(&ring));
    
    // Now pop - drop detection happens here
    spsc_sample_t sample;
    TEST_ASSERT_TRUE(spsc_ring_try_pop(&ring, &sample));
    
    // After drop detection, size should be effective_capacity - 1
    TEST_ASSERT_EQUAL(effective_capacity - 1, spsc_ring_size(&ring));
    
    // One element should have been dropped
    TEST_ASSERT_EQUAL(1, spsc_ring_dropped_total(&ring));
}

/**
 * @brief Test NULL pointer safety.
 */
void test_null_safety(void) {
    spsc_sample_t sample;
    test_payload_t payload = {0};
    
    // NULL ring pointer
    spsc_ring_push_isr(NULL, 0, &payload);
    TEST_ASSERT_FALSE(spsc_ring_try_pop(NULL, &sample));
    TEST_ASSERT_TRUE(spsc_ring_empty(NULL));
    
    // NULL value pointer
    spsc_ring_push_isr(&ring, 0, NULL);
    TEST_ASSERT_TRUE(spsc_ring_empty(&ring));
    
    // NULL output sample
    spsc_ring_push_isr(&ring, 0, &payload);
    TEST_ASSERT_FALSE(spsc_ring_try_pop(&ring, NULL));
}

/**
 * @brief Test statistics tracking.
 */
void test_statistics(void) {
    // Fill buffer beyond capacity to trigger drops
    const size_t push_count = SPSC_RING_CAPACITY + 20;
    
    for (size_t i = 0; i < push_count; i++) {
        test_payload_t payload = { .id = (uint32_t)i };
        spsc_ring_push_isr(&ring, (uint32_t)i, &payload);
    }
    
    TEST_ASSERT_EQUAL(push_count, spsc_ring_last_seq(&ring));
    
    // Pop all available (should be effective capacity)
    size_t popped = 0;
    spsc_sample_t sample;
    while (spsc_ring_try_pop(&ring, &sample)) {
        popped++;
    }
    
    TEST_ASSERT_EQUAL(SPSC_RING_CAPACITY - 1, popped);
    
    // Check that drops were detected
    uint32_t total_dropped = spsc_ring_dropped_total(&ring);
    TEST_ASSERT_TRUE(total_dropped > 0);
}

/**
 * @brief Stress test: rapid push/pop cycles.
 */
void test_stress_rapid_cycles(void) {
    const int iterations = 1000;
    
    for (int iter = 0; iter < iterations; iter++) {
        test_payload_t payload = { .id = (uint32_t)iter, .data = (uint32_t)(iter * 7) };
        spsc_ring_push_isr(&ring, (uint32_t)iter, &payload);
        
        spsc_sample_t sample;
        TEST_ASSERT_TRUE(spsc_ring_try_pop(&ring, &sample));
        
        test_payload_t* out = (test_payload_t*)sample.value;
        TEST_ASSERT_EQUAL(iter, out->id);
        TEST_ASSERT_EQUAL(iter * 7, out->data);
    }
    
    TEST_ASSERT_TRUE(spsc_ring_empty(&ring));
    TEST_ASSERT_EQUAL(0, spsc_ring_dropped_total(&ring));
}

/**
 * @brief Test batch operations.
 */
void test_batch_operations(void) {
    const int batch_size = 50;
    
    // Push batch
    for (int i = 0; i < batch_size; i++) {
        test_payload_t payload = { .id = (uint32_t)i };
        spsc_ring_push_isr(&ring, (uint32_t)i, &payload);
    }
    
    TEST_ASSERT_EQUAL(batch_size, spsc_ring_size(&ring));
    
    // Pop batch
    for (int i = 0; i < batch_size; i++) {
        spsc_sample_t sample;
        TEST_ASSERT_TRUE(spsc_ring_try_pop(&ring, &sample));
        
        test_payload_t* payload = (test_payload_t*)sample.value;
        TEST_ASSERT_EQUAL(i, payload->id);
    }
    
    TEST_ASSERT_TRUE(spsc_ring_empty(&ring));
}

/**
 * @brief Test interleaved push/pop.
 */
void test_interleaved_operations(void) {
    for (int i = 0; i < 100; i++) {
        // Push 3
        for (int j = 0; j < 3; j++) {
            test_payload_t payload = { .id = (uint32_t)(i * 3 + j) };
            spsc_ring_push_isr(&ring, (uint32_t)(i * 3 + j), &payload);
        }
        
        // Pop 2
        for (int j = 0; j < 2; j++) {
            spsc_sample_t sample;
            TEST_ASSERT_TRUE(spsc_ring_try_pop(&ring, &sample));
        }
    }
    
    // Should have 100 elements remaining (pushed 300, popped 200)
    TEST_ASSERT_EQUAL(100, spsc_ring_size(&ring));
}

// Test runner
void app_main(void) {
    ESP_LOGI(TAG, "Starting SPSC Ring Buffer (Pure C) tests");
    
#ifdef CONFIG_ESP_TASK_WDT_ENABLE
    // Disable task watchdog for long-running tests and interactive menu
    esp_task_wdt_deinit();
    ESP_LOGI(TAG, "Task watchdog disabled for testing");
#endif
    
    UNITY_BEGIN();
    
    RUN_TEST(test_init);
    RUN_TEST(test_push_pop_single);
    RUN_TEST(test_push_pop_multiple);
    RUN_TEST(test_pop_empty);
    RUN_TEST(test_sequence_numbers);
    RUN_TEST(test_overflow_drop_oldest);
    RUN_TEST(test_capacity_boundary);
    RUN_TEST(test_clear);
    RUN_TEST(test_drop_counter_reset);
    RUN_TEST(test_null_safety);
    RUN_TEST(test_can_frame_payload);
    RUN_TEST(test_esp32_memcpy_workaround);
    RUN_TEST(test_batch_operations);
    RUN_TEST(test_stress_rapid_cycles);
    RUN_TEST(test_interleaved_operations);
    RUN_TEST(test_wraparound);
    RUN_TEST(test_statistics);
    
    UNITY_END();
    
    // Print summary
    ESP_LOGI(TAG, "Tests: %u, Passed: %u, Failed: %u, Ignored: %u",
             (unsigned)Unity.NumberOfTests,
             (unsigned)(Unity.NumberOfTests - Unity.TestFailures - Unity.TestIgnores),
             (unsigned)Unity.TestFailures,
             (unsigned)Unity.TestIgnores);
    
    // Interactive menu for re-running specific tests
    ESP_LOGI(TAG, "Starting interactive test menu. Press ENTER to see options...");
    unity_run_menu();
}

