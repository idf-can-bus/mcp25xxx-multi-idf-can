# SPSC Ring Buffer (Pure C)

Lock-free single-producer/single-consumer ring buffer implementation in pure C for ESP32/Xtensa.

## Features

- **Lock-free SPSC pattern** using C11 atomics
- **ISR-safe push** operations (constant time, no blocking)
- **Drop-oldest policy** on overflow
- **Sequence numbers** for diagnostics
- **Bounded retry** on torn reads
- **ESP32/Xtensa workarounds** for toolchain issues

## ESP32 Workarounds

This implementation includes critical workarounds for ESP32/Xtensa GCC toolchain issues:

1. **memcpy instead of assignment**: Struct assignment can produce corrupted copies
2. **offsetof for size calculation**: `sizeof()` evaluation can be incorrect in some contexts
3. **Seqlock pattern**: Prevents torn reads during concurrent access
4. **Bounded retries**: Prevents infinite loops if producer gets stuck

## Configuration

Define these macros before including the header or in your CMakeLists.txt:

```c
#define SPSC_RING_PAYLOAD_SIZE 64      // Payload size in bytes
#define SPSC_RING_CAPACITY 128         // Must be power of 2 (2, 4, 8, 16, ..., 256)
#define SPSC_RING_MAX_RETRIES 1000     // Torn read retry limit (0 = infinite)
```

## Usage Example

```c
#include "spsc_ring_buffer_seq_c.h"

// Define CAN frame structure
typedef struct {
    uint32_t can_id;
    uint8_t can_dlc;
    uint8_t data[8];
} can_frame_t;

_Static_assert(sizeof(can_frame_t) <= SPSC_RING_PAYLOAD_SIZE, "Payload too large");

// Create buffer instance
static spsc_ring_buffer_t rx_buffer;

void init(void) {
    spsc_ring_init(&rx_buffer);
}

// Producer (ISR context)
void IRAM_ATTR can_rx_isr(void) {
    can_frame_t frame = read_from_hardware();
    spsc_ring_push_isr(&rx_buffer, get_time_100us(), &frame);
}

// Consumer (task context)
void consumer_task(void* arg) {
    spsc_sample_t sample;
    while (1) {
        if (spsc_ring_try_pop(&rx_buffer, &sample)) {
            can_frame_t* frame = (can_frame_t*)sample.value;
            process_frame(frame);
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}
```

## Testing

Run unit tests:
```bash
cd src/cpp/components/spsc_ring_buffer_seq_pure_c/test
idf.py build flash monitor
```

Tests verify:
- Basic push/pop operations
- Overflow behavior
- Sequence number consistency
- ESP32 memcpy workarounds
- Wraparound handling
- CAN frame payloads

## Threading Model

- **Producer**: ISR or high-priority task
- **Consumer**: Lower-priority task
- **SPSC only**: Do not use with multiple producers/consumers

## ESP32 Dual-Core Warning

On dual-core ESP32, pin both producer and consumer to the same core to avoid cache coherency issues:

```c
// Pin ISR to core 0
esp_intr_alloc(..., ESP_INTR_FLAG_IRAM, ..., core_id=0);

// Pin consumer task to core 0
xTaskCreatePinnedToCore(consumer_task, ..., core_id=0, ...);
```

Or use single-core chips (ESP32-S2/S3/C3).

