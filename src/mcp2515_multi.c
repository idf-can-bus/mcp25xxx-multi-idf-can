#include <string.h>
#include "mcp2515_multi.h"
#include "mcp2515_multi_internal.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Internal registry for configured SPI busses and their devices.
// The registry is intentionally simple; the application (or a higher layer)
// will populate it in a later step via dedicated registration helpers.

// Forward-declare the opaque handle structs declared in the header
struct can_bus_handle_s {
    const mcp2515_bundle_config_t* bundle;
    size_t                         bus_index;
};

struct can_dev_handle_s {
    const mcp2515_bundle_config_t* bundle;
    size_t                         device_index;
    size_t                         bus_index;
};

#ifndef CANIF_MAX_BUSES
#define CANIF_MAX_BUSES 4
#endif

#ifndef CANIF_MAX_DEVICES_PER_BUS
#define CANIF_MAX_DEVICES_PER_BUS 8
#endif

static const mcp2515_bundle_config_t* s_bundles[CANIF_MAX_BUSES];
static size_t s_bus_count = 0;

// Storage for bus/device handles (stable addresses)
static struct can_bus_handle_s s_bus_handles[CANIF_MAX_BUSES];
static struct can_dev_handle_s s_dev_handles[CANIF_MAX_BUSES][CANIF_MAX_DEVICES_PER_BUS];

// Per-device runtime context (created on open, cleared on close)
typedef struct {
    MCP2515_Handle         h;          // backend handle
    spi_device_handle_t    spi;        // SPI device for removal
    gpio_num_t             int_gpio;   // INT pin (for info)
    uint8_t                opened;     // 1 if opened
} canif_dev_runtime_t;

static canif_dev_runtime_t s_dev_rt[CANIF_MAX_BUSES][CANIF_MAX_DEVICES_PER_BUS];

static const char* TAG = "mcp_multi_init";

// Ensures internal arrays reflect current bundles (idempotent for static config)
static void ensure_handles_built(void)
{
    for (size_t bi = 0; bi < s_bus_count; ++bi) {
        s_bus_handles[bi].bundle = s_bundles[bi];
        s_bus_handles[bi].bus_index = bi;
        size_t dev_count = s_bundles[bi] ? s_bundles[bi]->device_count : 0;
        if (dev_count > CANIF_MAX_DEVICES_PER_BUS) dev_count = CANIF_MAX_DEVICES_PER_BUS;
        for (size_t di = 0; di < dev_count; ++di) {
            s_dev_handles[bi][di].bundle = s_bundles[bi];
            s_dev_handles[bi][di].device_index = di;
            s_dev_handles[bi][di].bus_index = bi;
        }
    }
}

// Optional helpers to allow external code to populate the registry early.
// Not exposed in the public header yet; provided to enable incremental adoption.
void canif_clear_registry(void)
{
    s_bus_count = 0;
    memset(s_bundles, 0, sizeof(s_bundles));
}

bool canif_register_bundle(const mcp2515_bundle_config_t* bundle)
{
    if (!bundle || s_bus_count >= CANIF_MAX_BUSES) return false;
    s_bundles[s_bus_count++] = bundle;
    ensure_handles_built();
    return true;
}

size_t canif_bus_count(void)
{
    return s_bus_count;
}

can_bus_handle_t canif_bus_at(size_t index)
{
    if (index >= s_bus_count) return NULL;
    return &s_bus_handles[index];
}

size_t canif_bus_device_count(can_bus_handle_t bus)
{
    if (!bus || !bus->bundle) return 0;
    return bus->bundle->device_count;
}

can_dev_handle_t canif_device_at(can_bus_handle_t bus, size_t index)
{
    if (!bus || !bus->bundle) return NULL;
    size_t bi = bus->bus_index;
    if (bi >= s_bus_count) return NULL;
    if (index >= bus->bundle->device_count) return NULL;
    return &s_dev_handles[bi][index];
}

can_bus_handle_t canif_bus_default(void)
{
    return canif_bus_at(0);
}

can_dev_handle_t canif_device_default(void)
{
    can_bus_handle_t b = canif_bus_default();
    return canif_device_at(b, 0);
}

can_bus_handle_t canif_bus_get_by_id(can_bus_id_t bus_id)
{
    for (size_t i = 0; i < s_bus_count; ++i) {
        const mcp2515_bundle_config_t* b = s_bundles[i];
        if (b && b->bus.bus_id == bus_id) {
            return &s_bus_handles[i];
        }
    }
    return NULL;
}

can_dev_handle_t canif_dev_get_by_id(can_bus_id_t bus_id, can_dev_id_t dev_id)
{
    can_bus_handle_t bus = canif_bus_get_by_id(bus_id);
    if (!bus || !bus->bundle || !bus->bundle->devices) return NULL;
    size_t bi = bus->bus_index;
    for (size_t di = 0; di < bus->bundle->device_count; ++di) {
        if (bus->bundle->devices[di].dev_id == dev_id) {
            return &s_dev_handles[bi][di];
        }
    }
    return NULL;
}

bool canif_is_valid_bus(can_bus_handle_t bus)
{
    if (!bus || !bus->bundle) return false;
    for (size_t i = 0; i < s_bus_count; ++i) {
        if (s_bundles[i] == bus->bundle) return true;
    }
    return false;
}

bool canif_is_valid_device(can_dev_handle_t dev)
{
    if (!dev || !dev->bundle) return false;
    for (size_t i = 0; i < s_bus_count; ++i) {
        if (s_bundles[i] == dev->bundle) {
            return dev->device_index < dev->bundle->device_count;
        }
    }
    return false;
}

// Helper: translate handles to indices (returns false if invalid)
static bool resolve_indices(can_dev_handle_t dev, size_t* out_bi, size_t* out_di)
{
    if (!canif_is_valid_device(dev)) return false;
    for (size_t bi = 0; bi < s_bus_count; ++bi) {
        if (s_bundles[bi] == dev->bundle) {
            if (out_bi) *out_bi = bi;
            if (out_di) *out_di = dev->device_index;
            return true;
        }
    }
    return false;
}

// Probe all devices on default bus: ensure normal mode is set and gather basic diagnostics.
static void canif_probe_and_heal_all(void)
{
    can_bus_handle_t bus = canif_bus_default();
    if (!bus) return;
    size_t n = canif_bus_device_count(bus);
    for (size_t i = 0; i < n; ++i) {
        can_dev_handle_t dev = canif_device_at(bus, i);
        if (!dev) continue;
        bool mode_ok = canif_set_mode_normal(dev);
        uint8_t eflg = canif_get_error_flags(dev);
        ESP_LOGI(TAG, "probe dev %u: mode_ok=%d, EFLG=0x%02X", (unsigned)i, (int)mode_ok, (unsigned)eflg);
        if (!mode_ok) {
            (void)canif_close_device(dev);
            vTaskDelay(pdMS_TO_TICKS(10));
            (void)canif_open_device(dev);
            (void)canif_set_mode_normal(dev);
        }
    }
}

// ======================================================================================
// Initialization & lifecycle
// ======================================================================================

bool canif_open_device(can_dev_handle_t dev)
{
    size_t bi, di;
    if (!resolve_indices(dev, &bi, &di)) return false;
    if (bi >= CANIF_MAX_BUSES || di >= CANIF_MAX_DEVICES_PER_BUS) return false;
    const mcp2515_bundle_config_t* bundle = dev->bundle;
    const mcp2515_device_config_t* dcfg = &bundle->devices[di];
    canif_dev_runtime_t* rt = &s_dev_rt[bi][di];
    if (rt->opened) return true; // already open

    // Convert bus config (ensure zero-initialized so unused fields are clean)
    spi_bus_config_t bus_cfg = {};
    spi_host_device_t host;
    int dma_chan;
    if (!mcp_spi_bus_to_idf(&bundle->bus, &host, &bus_cfg, &dma_chan)) return false;
    esp_err_t init_rc = mcp2515_spi_init_bus_if_needed(host, &bus_cfg);
    if (init_rc != ESP_OK && init_rc != ESP_ERR_INVALID_STATE) return false;

    // Convert device config
    spi_device_interface_config_t dev_cfg = {};
    mcp_spi_dev_to_idf(&dcfg->wiring, &dcfg->spi_params, &dev_cfg);

    // Add device to bus
    spi_device_handle_t spi = NULL;
    if (mcp2515_spi_add_device(host, &dev_cfg, &spi) != ESP_OK) return false;

    // Create controller instance on device
    mcp2515_multi_config_t mc = { .can_speed = dcfg->can.can_speed, .can_clock = dcfg->hw.crystal_frequency };
    MCP2515_Handle h = NULL;
    if (MCP2515_CreateOnDevice(spi, dcfg->wiring.int_gpio, &mc, &h) != ERROR_OK) {
        (void)mcp2515_spi_remove_device(spi);
        return false;
    }

    // Set bitrate explicitly and mode according to config
    if (MCP2515_SetBitrate(h, dcfg->can.can_speed, dcfg->hw.crystal_frequency) != ERROR_OK) {
        MCP2515_Destroy(h);
        (void)mcp2515_spi_remove_device(spi);
        return false;
    }
    if (dcfg->can.use_loopback) {
        if (MCP2515_SetLoopbackMode(h) != ERROR_OK) {
            MCP2515_Destroy(h);
            (void)mcp2515_spi_remove_device(spi);
            return false;
        }
    } else {
        if (MCP2515_SetNormalMode(h) != ERROR_OK) {
            MCP2515_Destroy(h);
            (void)mcp2515_spi_remove_device(spi);
            return false;
        }
    }

    rt->h = h;
    rt->spi = spi;
    rt->int_gpio = dcfg->wiring.int_gpio;
    rt->opened = 1u;
    return true;
}

bool canif_close_device(can_dev_handle_t dev)
{
    size_t bi, di;
    if (!resolve_indices(dev, &bi, &di)) return false;
    if (bi >= CANIF_MAX_BUSES || di >= CANIF_MAX_DEVICES_PER_BUS) return false;
    canif_dev_runtime_t* rt = &s_dev_rt[bi][di];
    if (!rt->opened) return true;

    // Destroy controller and remove SPI device
    if (rt->h) {
        MCP2515_Destroy(rt->h);
        rt->h = NULL;
    }
    if (rt->spi) {
        (void)mcp2515_spi_remove_device(rt->spi);
        rt->spi = NULL;
    }
    rt->opened = 0u;
    return true;
}

bool canif_open_id(can_bus_id_t bus_id, can_dev_id_t dev_id)
{
    can_dev_handle_t dev = canif_dev_get_by_id(bus_id, dev_id);
    if (!dev) return false;
    return canif_open_device(dev);
}

bool canif_close_id(can_bus_id_t bus_id, can_dev_id_t dev_id)
{
    can_dev_handle_t dev = canif_dev_get_by_id(bus_id, dev_id);
    if (!dev) return false;
    return canif_close_device(dev);
}

bool canif_open_target(can_target_t target)
{
    return canif_open_id(can_target_bus_id(target), can_target_dev_id(target));
}

bool canif_close_target(can_target_t target)
{
    return canif_close_id(can_target_bus_id(target), can_target_dev_id(target));
}

// ======================================================================================
// Mode & bitrate control
// ======================================================================================

bool canif_set_bitrate_to(can_dev_handle_t dev, CAN_SPEED_t speed, CAN_CLOCK_t clock)
{
    size_t bi, di; if (!resolve_indices(dev, &bi, &di)) return false;
    canif_dev_runtime_t* rt = &s_dev_rt[bi][di];
    if (!rt->opened || !rt->h) return false;
    return MCP2515_SetBitrate(rt->h, speed, clock) == ERROR_OK;
}

bool canif_set_mode_normal(can_dev_handle_t dev)
{
    size_t bi, di; if (!resolve_indices(dev, &bi, &di)) return false;
    canif_dev_runtime_t* rt = &s_dev_rt[bi][di];
    if (!rt->opened || !rt->h) return false;
    return MCP2515_SetNormalMode(rt->h) == ERROR_OK;
}

bool canif_set_mode_loopback(can_dev_handle_t dev)
{
    size_t bi, di; if (!resolve_indices(dev, &bi, &di)) return false;
    canif_dev_runtime_t* rt = &s_dev_rt[bi][di];
    if (!rt->opened || !rt->h) return false;
    return MCP2515_SetLoopbackMode(rt->h) == ERROR_OK;
}

bool canif_set_bitrate_id(can_bus_id_t bus_id, can_dev_id_t dev_id, CAN_SPEED_t s, CAN_CLOCK_t c)
{
    can_dev_handle_t dev = canif_dev_get_by_id(bus_id, dev_id);
    if (!dev) return false;
    return canif_set_bitrate_to(dev, s, c);
}

bool canif_set_mode_normal_id(can_bus_id_t bus_id, can_dev_id_t dev_id)
{
    can_dev_handle_t dev = canif_dev_get_by_id(bus_id, dev_id);
    if (!dev) return false;
    return canif_set_mode_normal(dev);
}

bool canif_set_mode_loopback_id(can_bus_id_t bus_id, can_dev_id_t dev_id)
{
    can_dev_handle_t dev = canif_dev_get_by_id(bus_id, dev_id);
    if (!dev) return false;
    return canif_set_mode_loopback(dev);
}

bool canif_set_bitrate_target(can_target_t t, CAN_SPEED_t s, CAN_CLOCK_t c)
{
    return canif_set_bitrate_id(can_target_bus_id(t), can_target_dev_id(t), s, c);
}

bool canif_set_mode_normal_target(can_target_t t)
{
    return canif_set_mode_normal_id(can_target_bus_id(t), can_target_dev_id(t));
}

bool canif_set_mode_loopback_target(can_target_t t)
{
    return canif_set_mode_loopback_id(can_target_bus_id(t), can_target_dev_id(t));
}

// ======================================================================================
// Events
// ======================================================================================

typedef struct {
    canif_event_cb user_cb;
    void*          user_data;
    can_dev_handle_t dev_handle;
} canif_event_binding_t;

static canif_event_binding_t s_event_bind[CANIF_MAX_BUSES][CANIF_MAX_DEVICES_PER_BUS];

static void event_trampoline(MCP2515_Handle h, uint32_t mask, void* user)
{
    (void)h;
    canif_event_binding_t* bind = (canif_event_binding_t*)user;
    if (!bind || !bind->user_cb) return;
    bind->user_cb(bind->dev_handle, mask, bind->user_data);
}

void canif_set_event_callback(can_dev_handle_t dev, canif_event_cb cb, void* userData)
{
    size_t bi, di; if (!resolve_indices(dev, &bi, &di)) return;
    canif_dev_runtime_t* rt = &s_dev_rt[bi][di];
    if (!rt->opened || !rt->h) return;
    s_event_bind[bi][di].user_cb = cb;
    s_event_bind[bi][di].user_data = userData;
    s_event_bind[bi][di].dev_handle = &s_dev_handles[bi][di];
    MCP2515_SetEventCallback(rt->h, event_trampoline, &s_event_bind[bi][di]);
}

uint32_t canif_wait_for_event(can_dev_handle_t dev, uint32_t timeout_ticks)
{
    size_t bi, di; if (!resolve_indices(dev, &bi, &di)) return 0;
    canif_dev_runtime_t* rt = &s_dev_rt[bi][di];
    if (!rt->opened || !rt->h) return 0;
    return MCP2515_WaitForEvent(rt->h, timeout_ticks);
}

// ======================================================================================
// Open/close helpers
// ======================================================================================

bool canif_open_all_on_bus(can_bus_handle_t bus)
{
    if (!canif_is_valid_bus(bus)) return false;
    size_t n = canif_bus_device_count(bus);
    bool any = false;
    for (size_t i = 0; i < n; ++i) {
        can_dev_handle_t dev = canif_device_at(bus, i);
        if (dev && canif_open_device(dev)) any = true;
    }
    return any;
}

bool canif_close_all_on_bus(can_bus_handle_t bus)
{
    if (!canif_is_valid_bus(bus)) return false;
    size_t n = canif_bus_device_count(bus);
    bool ok = true;
    for (size_t i = 0; i < n; ++i) {
        can_dev_handle_t dev = canif_device_at(bus, i);
        if (!dev || !canif_close_device(dev)) ok = false;
    }
    return ok;
}

bool canif_open_all(void)
{
    bool any = false;
    for (size_t bi = 0; bi < canif_bus_count(); ++bi) {
        can_bus_handle_t bus = canif_bus_at(bi);
        if (canif_open_all_on_bus(bus)) any = true;
    }
    return any;
}

bool canif_close_all(void)
{
    bool ok = true;
    for (size_t bi = 0; bi < canif_bus_count(); ++bi) {
        can_bus_handle_t bus = canif_bus_at(bi);
        if (!canif_close_all_on_bus(bus)) ok = false;
    }
    return ok;
}

// ======================================================================================
// Errors & diagnostics
// ======================================================================================

uint8_t canif_get_error_flags(can_dev_handle_t dev)
{
    size_t bi, di; if (!resolve_indices(dev, &bi, &di)) return 0;
    canif_dev_runtime_t* rt = &s_dev_rt[bi][di];
    if (!rt->opened || !rt->h) return 0;
    return MCP2515_GetErrorFlags(rt->h);
}

void canif_clear_rx_overrun(can_dev_handle_t dev)
{
    size_t bi, di; if (!resolve_indices(dev, &bi, &di)) return;
    canif_dev_runtime_t* rt = &s_dev_rt[bi][di];
    if (!rt->opened || !rt->h) return;
    MCP2515_ClearRXnOVR(rt->h);
}

void canif_clear_error_int(can_dev_handle_t dev)
{
    size_t bi, di; if (!resolve_indices(dev, &bi, &di)) return;
    canif_dev_runtime_t* rt = &s_dev_rt[bi][di];
    if (!rt->opened || !rt->h) return;
    MCP2515_ClearERRIF(rt->h);
}

// ======================================================================================
// Filters & masks
// ======================================================================================

bool canif_set_filter(can_dev_handle_t dev, uint8_t filter_idx, bool extended, uint32_t id)
{
    size_t bi, di; if (!resolve_indices(dev, &bi, &di)) return false;
    canif_dev_runtime_t* rt = &s_dev_rt[bi][di];
    if (!rt->opened || !rt->h) return false;
    return MCP2515_SetFilter(rt->h, filter_idx, extended, id) == ERROR_OK;
}

bool canif_set_mask(can_dev_handle_t dev, uint8_t mask_idx, bool extended, uint32_t mask)
{
    size_t bi, di; if (!resolve_indices(dev, &bi, &di)) return false;
    canif_dev_runtime_t* rt = &s_dev_rt[bi][di];
    if (!rt->opened || !rt->h) return false;
    return MCP2515_SetMask(rt->h, mask_idx, extended, mask) == ERROR_OK;
}

// ======================================================================================
// Introspection & utilities
// ======================================================================================

const mcp2515_device_config_t* canif_device_config(can_dev_handle_t dev)
{
    if (!canif_is_valid_device(dev)) return NULL;
    return &dev->bundle->devices[dev->device_index];
}

can_bus_id_t canif_bus_id_of(can_bus_handle_t bus)
{
    if (!canif_is_valid_bus(bus)) return (can_bus_id_t)0;
    return bus->bundle->bus.bus_id;
}

can_dev_id_t canif_dev_id_of(can_dev_handle_t dev)
{
    const mcp2515_device_config_t* cfg = canif_device_config(dev);
    if (!cfg) return (can_dev_id_t)0;
    return cfg->dev_id;
}

// ======================================================================================
// Messaging
// ======================================================================================

bool canif_send_to(can_dev_handle_t dev, const can_message_t* msg)
{
    if (!msg || msg->dlc > 8) return false;
    size_t bi, di; if (!resolve_indices(dev, &bi, &di)) return false;
    canif_dev_runtime_t* rt = &s_dev_rt[bi][di];
    if (!rt->opened || !rt->h) return false;
    CAN_FRAME f;
    f.can_id = msg->id; // Note: extended/rtr flags may be encoded by backend if needed
    f.can_dlc = msg->dlc;
    for (uint8_t i=0;i<msg->dlc;i++) f.data[i] = msg->data[i];
    return MCP2515_SendMessageAfterCtrlCheck(rt->h, &f) == ERROR_OK;
}

bool canif_receive_from(can_dev_handle_t dev, can_message_t* msg)
{
    if (!msg) return false;
    size_t bi, di; if (!resolve_indices(dev, &bi, &di)) return false;
    canif_dev_runtime_t* rt = &s_dev_rt[bi][di];
    if (!rt->opened || !rt->h) return false;
    CAN_FRAME f;
    ERROR_t rc = MCP2515_ReadMessageAfterStatCheck(rt->h, &f);
    if (rc != ERROR_OK) return false;
    if (f.can_dlc > 8) return false;
    msg->id = f.can_id;
    msg->dlc = f.can_dlc;
    msg->extended_id = false; // not encoded by backend here
    msg->rtr = false;
    for (uint8_t i=0;i<f.can_dlc;i++) msg->data[i] = f.data[i];
    return true;
}

bool canif_send_id(can_bus_id_t bus_id, can_dev_id_t dev_id, const can_message_t* msg)
{
    can_dev_handle_t dev = canif_dev_get_by_id(bus_id, dev_id);
    if (!dev) return false;
    return canif_send_to(dev, msg);
}

bool canif_receive_id(can_bus_id_t bus_id, can_dev_id_t dev_id, can_message_t* msg)
{
    can_dev_handle_t dev = canif_dev_get_by_id(bus_id, dev_id);
    if (!dev) return false;
    return canif_receive_from(dev, msg);
}

bool canif_send_target(can_target_t target, const can_message_t* msg)
{
    return canif_send_id(can_target_bus_id(target), can_target_dev_id(target), msg);
}

bool canif_receive_target(can_target_t target, can_message_t* msg)
{
    return canif_receive_id(can_target_bus_id(target), can_target_dev_id(target), msg);
}

// ======================================================================================
// High-level helpers for default device: init/deinit/send
// ======================================================================================

bool canif_multi_init_default(const mcp2515_bundle_config_t* cfg)
{
    if (!cfg) return false;
    canif_clear_registry();
    if (!canif_register_bundle(cfg)) return false;
    // Open all devices from the single registered bundle so they ACK and receive
    can_bus_handle_t bus = canif_bus_default();
    if (!bus) return false;
    size_t n = canif_bus_device_count(bus);
    bool any_opened = false;
    for (size_t i = 0; i < n; ++i) {
        can_dev_handle_t dev = canif_device_at(bus, i);
        if (dev && canif_open_device(dev)) {
            any_opened = true;
        }
    }
    // Post-initialization probe to ensure devices are responsive across boots
    canif_probe_and_heal_all();
    return any_opened;
}

bool canif_multi_deinit_default(void)
{
    can_bus_handle_t bus = canif_bus_default();
    if (!bus) return false;
    size_t n = canif_bus_device_count(bus);
    bool all_closed = true;
    for (size_t i = 0; i < n; ++i) {
        can_dev_handle_t dev = canif_device_at(bus, i);
        if (!dev || !canif_close_device(dev)) {
            all_closed = false;
        }
    }
    return all_closed;
}

bool canif_multi_send_default(const can_message_t* msg)
{
    return canif_send_to(canif_device_default(), msg);
}

bool canif_receive_default(can_message_t* msg)
{
    return canif_receive_from(canif_device_default(), msg);
}


