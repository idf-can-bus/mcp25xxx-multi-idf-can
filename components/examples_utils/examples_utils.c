#include <stddef.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "examples_utils.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "EXAMPLES_UTILS"

// store 40 least significant bits of 64bit value into 5B array (big-endian)
void store_timestamp40(uint64_t source, unit40_big_endian_t *target_ptr)
{
    for (int i = 0; i < 5; ++i)
        (*target_ptr)[4 - i] = (source >> (i * 8)) & 0xFF;
}

// restore 5B big-endian timestamp back to 64bit value
uint64_t restore_timestamp40(const unit40_big_endian_t *src_ptr)
{
    uint64_t result = 0;
    for (int i = 0; i < 5; ++i)
        result = (result << 8) | (*src_ptr)[i];
    return result;
}

void fullfill_test_messages(uint8_t sender_id, uint8_t heartbeat, can_message_t *message) 
{
    if (message == NULL) {
        ESP_LOGE(TAG, "Invalid frame pointer");
        return;
    }

    // view to message->data (8 bytes) as a test_can_message_t
    test_can_message_t * payload = (test_can_message_t *)message->data;

    message->id = TEST_MSG_ID;
    message->extended_id = false;
    message->rtr = false;
    message->dlc = 8;
    payload->sender_id = sender_id;
    payload->heartbeat = heartbeat;
    payload->flags.value = 0;
    store_timestamp40(esp_timer_get_time(), & (payload->timestamp40));
} // get_test_messages

void print_can_message(const can_message_t *message) {
    if (message == NULL) {
        ESP_LOGE(TAG, "Invalid frame pointer");
        return;
    }
    
    printf("CAN message ID: %lu\n", message->id);
    switch (message->id) {
        case TEST_MSG_ID:
            // view payload as test_can_message_t
            test_can_message_t *payload1 = (test_can_message_t *)message->data;

            printf("Test message\n");
            printf("Sender ID: %u\n", payload1->sender_id);
            printf("Heartbeat: %u\n", payload1->heartbeat);
            printf("Timestamp: %llu [us]\n", restore_timestamp40(& (payload1->timestamp40)));
            break;

        case CAN_MSG_WITH_TWO_UINT32_ID:
            // view payload as can_message_with_two_uint32_t
            can_message_with_two_uint32_t *payload2 = (can_message_with_two_uint32_t *)message->data;
            printf("Two uint32 message\n");
            printf("Value uint32_1: %lu\n", payload2->value_uint32_1);
            printf("Value uint32_2: %lu\n", payload2->value_uint32_2);
            break;

        case CAN_MSG_WITH_ONE_UINT64_ID:
            // view payload as can_message_with_one_uint64_t
            can_message_with_one_uint64_t *payload3 = (can_message_with_one_uint64_t *)message->data;

            printf("One uint64 message\n");
            printf("Value uint64: %llu\n", payload3->value_uint64);
            break;

        case EIGHT_BYTES_ARRAY_MESSAGE_ID:
            // view payload as eight_bytes_array_message_t
            eight_bytes_array_message_t *payload4 = (eight_bytes_array_message_t *)message->data;

            printf("Eight bytes array message\n");
            printf("Data:");
            for (int i = 0; i < 8; i++) {
                printf(" %02X", payload4->data[i]);
            }
            printf("\n");
            break;

        default:
            printf("Unknown message ID: %lu\n", message->id);
            break;
    }
        // debug print message->data
    printf("message->data (dec):|");
    for (int i = 0; i < message->dlc; i++) {
        printf("%03d|", message->data[i]);
    }
    printf("\n");
}

bool check_heartbeat(uint8_t received_heartbeat, uint8_t expected_heartbeat)
{
    bool success = received_heartbeat == expected_heartbeat;
    if (!success) {
        ESP_LOGE(TAG, "Heartbeat mismatch: expected_heartbeat %u, payload->heartbeat %u", expected_heartbeat, received_heartbeat);
    } 
    
    return success;
}

// Here we assume that heartbeat is uint8_t in range 0-255
uint8_t next_heartbeat(const uint8_t heartbeat) {
    return heartbeat + 1;  // uint8_t automatically wraps 255->0
}

static uint64_t count_of_messages_for_log = 0;
#define PRINT_DOT_EVERY_N_MESSAGES 10
#define MAX_INDEX_ON_ONE_LINE 50
#define PRINT_NL_EVERY_N_MESSAGES (PRINT_DOT_EVERY_N_MESSAGES*MAX_INDEX_ON_ONE_LINE)


void log_message(const bool send, can_message_t *message, const bool print_details) {
    if (print_details) {
        print_can_message(message);
    } else {
        if (count_of_messages_for_log % PRINT_DOT_EVERY_N_MESSAGES == 0) {
            printf(".");
        }
        if (count_of_messages_for_log % PRINT_NL_EVERY_N_MESSAGES == 0) {
            if (send) {
                printf("\n->");                
            } else {
                printf("\n<-");                                
            }
            printf(" (%lld) ", count_of_messages_for_log);
        }        
        fflush(stdout);
        count_of_messages_for_log++;
    }
}

// Sequence statistics 
static uint64_t seq_rx_count = 0;                 // total payload->heartbeat TEST_MSG_ID frames
static uint64_t seq_ok_in_order = 0;              // frames arriving exactly in expected_heartbeat order
static uint64_t seq_lost = 0;                     // estimated number of lost frames (forward jump delta)
static uint64_t seq_out_of_order_or_dup = 0;      // frames that appear behind expected_heartbeat (out-of-order or duplicate)
static uint64_t seq_start_time_us = 0;            // Start time for sequence statistics
static uint8_t expected_heartbeat = 0;            // expected_heartbeat heartbeat

// ----------------------- single-sender processing -----------------------

void process_received_message(can_message_t *message, const bool print_during_receive) {
    if (message == NULL) {
        ESP_LOGE(TAG, "Invalid message pointer");
        return;
    }
    if (print_during_receive) {
        print_can_message(message);
    } else {
        log_message(false, message, print_during_receive);        
    }
    if (message->id == TEST_MSG_ID) {
        test_can_message_t *payload = (test_can_message_t *)message->data;

        // Sequence check 
        // Calculate delta as signed difference with proper overflow handling
        // Positive delta: payload->heartbeat > expected_heartbeat (lost frames)
        // Negative delta: payload->heartbeat < expected_heartbeat (out-of-order/duplicate frames)
        int16_t delta = (int16_t)payload->heartbeat - (int16_t)expected_heartbeat;
        if (delta < -127) delta += 256;  // Handle uint8_t overflow
        if (delta > 127) delta -= 256;   // Handle uint8_t overflow

        // @TODO: remove this debug print
        /*
        if (! check_heartbeat(payload->heartbeat, expected_heartbeat)) {
            ESP_LOGE(TAG, "%d != %d, delta: %d", payload->heartbeat, expected_heartbeat, delta);
        }
        */
       
        seq_rx_count++;
        if (seq_start_time_us == 0) {
            seq_start_time_us = esp_timer_get_time();  // Start timing on first message
            seq_rx_count = 0;  // Reset counter to 1 for first message
        }
        
        if (delta == 0) {
            seq_ok_in_order++;
        } else if (delta > 0) {
            seq_lost += delta;  // Positive delta = lost frames
        } else {
            seq_out_of_order_or_dup++;  // Negative delta = out-of-order/duplicate
        }

        // Advance expected_heartbeat to next after the payload->heartbeat
        expected_heartbeat = next_heartbeat(payload->heartbeat);

        // Stats request via flags (single-sender path uses first-seen sender)
        if (test_flags_is_set(&((test_can_message_t*)message->data)->flags, TEST_FLAG_STATS_REQUEST)) {
            uint64_t current_time = esp_timer_get_time();
            uint64_t elapsed_time_us = current_time - seq_start_time_us;
            float elapsed_time_ms = (float)elapsed_time_us / 1000.0f;
            float rx_rate_hz = (elapsed_time_ms > 0) ? ((float)seq_rx_count / (elapsed_time_ms / 1000.0f)) : 0.0f;
            printf("\n");
            ESP_LOGI(TAG,
                     "Sequence stats: frames=%llu in_order=%llu lost=%llu dup=%llu elapsed=%.1f ms f=%.1f Hz",
                     seq_rx_count, seq_ok_in_order, seq_lost, seq_out_of_order_or_dup, elapsed_time_ms, rx_rate_hz);
            seq_rx_count = 0;
            seq_ok_in_order = 0;
            seq_lost = 0;
            seq_out_of_order_or_dup = 0;
            seq_start_time_us = current_time;
        }
    } 
}

// ----------------------- multi-sender processing -----------------------
typedef struct {
    uint64_t seq_rx_count;
    uint64_t seq_ok_in_order;
    uint64_t seq_lost;
    uint64_t seq_out_of_order_or_dup;
    uint64_t seq_start_time_us;
    uint8_t  expected_heartbeat;
    char     dot_char; // visualization char assigned to this sender
    uint64_t print_count; // per-sender print throttling
} sender_stats_t;

#define MAX_SENDERS_TRACKED 10
static sender_stats_t sender_stats[256]; // index by sender_id (0..255)
static bool sender_seen[256] = {false};
static uint8_t sender_order[256]; // order of first appearance
static uint8_t sender_order_count = 0;

static char next_dot_char_for_sender(uint8_t sender_id)
{
    // Assign visualization char on first sight; reuse on next calls.
    // Mapping: order 0..8 -> '1'..'9', 9+ -> '9'
    if (!sender_seen[sender_id]) {
        uint8_t order = sender_order_count;
        sender_order[sender_id] = order;
        sender_order_count++;
        if (order < 9) return (char)('1' + order);
        return '9';
    } else {
        uint8_t order = sender_order[sender_id];
        if (order < 9) return (char)('1' + order);
        return '9';
    }
}

void process_received_message_multi(can_message_t *message, const bool print_during_receive)
{
    if (message == NULL) {
        ESP_LOGE(TAG, "Invalid message pointer");
        return;
    }
    if (message->id != TEST_MSG_ID) {
        // Fallback to simple processing for non-test messages
        process_received_message(message, print_during_receive);
        return;
    }

    test_can_message_t *payload = (test_can_message_t *)message->data;
    uint8_t sid = payload->sender_id;


    // Initialize sender state on first sight
    if (!sender_seen[sid]) {
        sender_stats[sid].seq_rx_count = 0;
        sender_stats[sid].seq_ok_in_order = 0;
        sender_stats[sid].seq_lost = 0;
        sender_stats[sid].seq_out_of_order_or_dup = 0;
        sender_stats[sid].seq_start_time_us = 0;
        sender_stats[sid].expected_heartbeat = 0;
        sender_stats[sid].dot_char = next_dot_char_for_sender(sid);
        sender_stats[sid].print_count = 0;
        sender_seen[sid] = true;
    }

    sender_stats_t *st = &sender_stats[sid];

    if (print_during_receive) {
        print_can_message(message);
    } else {
        // compact log: per-sender throttled printing to visualize concurrent senders
        st->print_count++;
        if (st->print_count % PRINT_DOT_EVERY_N_MESSAGES == 0) {
            printf("%c", st->dot_char);
        }
        if (count_of_messages_for_log % PRINT_NL_EVERY_N_MESSAGES == 0) {
            printf("\n<-%c", st->dot_char);
            printf(" (%lld) ", count_of_messages_for_log);
        }
        fflush(stdout);
        count_of_messages_for_log++;
    }

    // Sequence stats per sender
    int16_t delta = (int16_t)payload->heartbeat - (int16_t)st->expected_heartbeat;
    if (delta < -127) delta += 256;
    if (delta > 127) delta -= 256;

    st->seq_rx_count++;
    if (st->seq_start_time_us == 0) {
        st->seq_start_time_us = esp_timer_get_time();
        st->seq_rx_count = 0; // reset to start from 0 in the window
    }
    if (delta == 0)      st->seq_ok_in_order++;
    else if (delta > 0)  st->seq_lost += delta;
    else                 st->seq_out_of_order_or_dup++;

    st->expected_heartbeat = next_heartbeat(payload->heartbeat);

    // If sender requests statistics, print and reset per-sender window
    if (test_flags_is_set(&payload->flags, TEST_FLAG_STATS_REQUEST)) {
        uint64_t now = esp_timer_get_time();
        uint64_t elapsed_us = now - st->seq_start_time_us;
        float elapsed_ms = (float)elapsed_us / 1000.0f;
        float rx_rate_hz = (elapsed_ms > 0) ? ((float)st->seq_rx_count / (elapsed_ms / 1000.0f)) : 0.0f;
        ESP_LOGI(TAG,
                 "sender=%u char=%c frames=%llu in_order=%llu lost=%llu dup=%llu elapsed=%.1fms rate=%.1fHz",
                 sid, st->dot_char,
                 st->seq_rx_count, st->seq_ok_in_order, st->seq_lost, st->seq_out_of_order_or_dup,
                 elapsed_ms, rx_rate_hz);

        // Reset per-sender window
        st->seq_rx_count = 0;
        st->seq_ok_in_order = 0;
        st->seq_lost = 0;
        st->seq_out_of_order_or_dup = 0;
        st->seq_start_time_us = now;
    }
}

void debug_send_message(can_message_t *message, const bool print_during_send) {
    if (message == NULL) {
        ESP_LOGE(TAG, "Invalid frame pointer");
        return;
    }
    log_message(true, message, print_during_send);
}

uint8_t default_sender_id_from_mac(void)
{
    // Read station MAC (factory-programmed)
    uint8_t mac[6] = {0};
    (void)esp_read_mac(mac, ESP_MAC_WIFI_STA);

    // Compose a tiny hash from last 3 bytes for better distribution
    uint32_t v = ((uint32_t)mac[3] << 16) | ((uint32_t)mac[4] << 8) | mac[5];
    // Jenkins one-at-a-time mix (lightweight)
    uint32_t h = 0;
    h += (v & 0xFF); h += (h << 10); h ^= (h >> 6);
    h += ((v >> 8) & 0xFF); h += (h << 10); h ^= (h >> 6);
    h += ((v >> 16) & 0xFF); h += (h << 10); h ^= (h >> 6);
    h += (h << 3); h ^= (h >> 11); h += (h << 15);
    uint8_t sender_id = (uint8_t)(h % 255); // 0..254, 255 reserved

    // Log for visual collision check
    ESP_LOGI(TAG, "Device MAC: %02X:%02X:%02X:%02X:%02X:%02X -> sender_id=%u",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], sender_id);

    return sender_id;
}
void sleep_ms_min_ticks(uint32_t ms)
{
    TickType_t ticks = pdMS_TO_TICKS(ms);
    if (ticks == 0) {
        ticks = 1;
    }
    vTaskDelay(ticks);
}