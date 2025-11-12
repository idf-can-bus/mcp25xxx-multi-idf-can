#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "mcp2515_multi_internal.h"

// MCP2515 instructions / registers (subset)
#define INSTRUCTION_RESET       0xC0
#define INSTRUCTION_READ        0x03
#define INSTRUCTION_WRITE       0x02
#define INSTRUCTION_BITMOD      0x05
#define INSTRUCTION_READ_STATUS 0xA0

#define MCP_CANSTAT  0x0E
#define MCP_CANCTRL  0x0F
#define MCP_CNF1     0x2A
#define MCP_CNF2     0x29
#define MCP_CNF3     0x28
#define MCP_CANINTE  0x2B
#define MCP_CANINTF  0x2C
#define MCP_EFLG     0x2D

#define CANCTRL_REQOP       0xE0
#define CANCTRL_REQOP_NORMAL     0x00
#define CANCTRL_REQOP_LOOPBACK   0x40
#define CANCTRL_REQOP_CONFIG     0x80

#define CANSTAT_OPMOD       0xE0

#define STAT_RX0IF (1<<0)
#define STAT_RX1IF (1<<1)

#define TXB_EXIDE_MASK 0x08
#define DLC_MASK       0x0F
#define RTR_MASK       0x40

#define RXB0SIDH 0x61
#define RXB0CTRL 0x60
#define RXB0SIDL 0x62
#define RXB0EID8 0x63
#define RXB0EID0 0x64
#define RXB0DLC  0x65
#define RXB0DATA 0x66

#define RXB1SIDH 0x71
#define RXB1CTRL 0x70
#define RXB1SIDL 0x72
#define RXB1EID8 0x73
#define RXB1EID0 0x74
#define RXB1DLC  0x75
#define RXB1DATA 0x76

#define TXB0CTRL 0x30
#define TXB0SIDH 0x31
#define TXB0SIDL 0x32
#define TXB0EID8 0x33
#define TXB0EID0 0x34
#define TXB0DLC  0x35
#define TXB0DATA 0x36

#define TXB_ABTF  0x40
#define TXB_MLOA  0x20
#define TXB_TXERR 0x10
#define TXB_TXREQ 0x08

#define RXM_MASK  0x60
#define RXB0_BUKT 0x04

// Filter and mask base addresses
#define RXF0SIDH 0x00
#define RXF1SIDH 0x04
#define RXF2SIDH 0x08
#define RXF3SIDH 0x10
#define RXF4SIDH 0x14
#define RXF5SIDH 0x18

#define RXM0SIDH 0x20
#define RXM1SIDH 0x24

typedef struct MCP2515_Context {
    spi_device_handle_t spi;
    gpio_num_t          int_gpio;
    SemaphoreHandle_t   event_sem;   // signaled by ISR
    MCP2515_EventCallback cb;
    void*               cb_user;
    CAN_SPEED_t         can_speed;
    CAN_CLOCK_t         can_clock;
} MCP2515_Context;

// static const char* TAG = "mcp2515_multi";

static void IRAM_ATTR mcp2515_isr_handler(void* arg)
{
    MCP2515_Context* ctx = (MCP2515_Context*)arg;
    BaseType_t woken = pdFALSE;
    if (ctx && ctx->event_sem) {
        xSemaphoreGiveFromISR(ctx->event_sem, &woken);
    }
    if (woken) portYIELD_FROM_ISR();
}

esp_err_t mcp2515_spi_init_bus_if_needed(spi_host_device_t host, const spi_bus_config_t* bus_cfg)
{
    esp_err_t err = spi_bus_initialize(host, bus_cfg, SPI_DMA_CH_AUTO);
    if (err == ESP_ERR_INVALID_STATE) return ESP_OK;
    return err;
}

esp_err_t mcp2515_spi_add_device(spi_host_device_t host, const spi_device_interface_config_t* dev_cfg, spi_device_handle_t* out_spi)
{
    return spi_bus_add_device(host, dev_cfg, out_spi);
}

esp_err_t mcp2515_spi_remove_device(spi_device_handle_t spi)
{
    return spi_bus_remove_device(spi);
}

// ---------------------- Low-level helpers ----------------------

static ERROR_t mcp2515_ll_reset(MCP2515_Handle h)
{
    spi_transaction_t trans = {};
    trans.length = 8;
    trans.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    trans.tx_data[0] = INSTRUCTION_RESET;
    if (spi_device_transmit(h->spi, &trans) != ESP_OK) return ERROR_FAIL;
    vTaskDelay(pdMS_TO_TICKS(10));
    return ERROR_OK;
}

static uint8_t mcp2515_ll_read(MCP2515_Handle h, uint8_t reg)
{
    spi_transaction_t trans = {};
    trans.length = 24;
    trans.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    trans.tx_data[0] = INSTRUCTION_READ;
    trans.tx_data[1] = reg;
    trans.tx_data[2] = 0x00;
    (void)spi_device_transmit(h->spi, &trans);
    return trans.rx_data[2];
}

static void mcp2515_ll_write(MCP2515_Handle h, uint8_t reg, uint8_t value)
{
    spi_transaction_t trans = {};
    trans.length = 24;
    trans.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    trans.tx_data[0] = INSTRUCTION_WRITE;
    trans.tx_data[1] = reg;
    trans.tx_data[2] = value;
    (void)spi_device_transmit(h->spi, &trans);
}

static void mcp2515_ll_bitmod(MCP2515_Handle h, uint8_t reg, uint8_t mask, uint8_t data)
{
    spi_transaction_t trans = {};
    trans.length = 32;
    trans.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    trans.tx_data[0] = INSTRUCTION_BITMOD;
    trans.tx_data[1] = reg;
    trans.tx_data[2] = mask;
    trans.tx_data[3] = data;
    (void)spi_device_transmit(h->spi, &trans);
}

static ERROR_t mcp2515_set_mode(MCP2515_Handle h, uint8_t req)
{
    mcp2515_ll_bitmod(h, MCP_CANCTRL, CANCTRL_REQOP, req);
    for (int i = 0; i < 10; i++) {
        uint8_t stat = mcp2515_ll_read(h, MCP_CANSTAT) & CANSTAT_OPMOD;
        if (stat == req) return ERROR_OK;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return ERROR_FAIL;
}

// ---------------------- Public API ----------------------

ERROR_t MCP2515_CreateOnDevice(spi_device_handle_t spi,
                               gpio_num_t int_gpio,
                               const mcp2515_multi_config_t* cfg,
                               MCP2515_Handle* out_handle)
{
    if (!spi || !cfg || !out_handle) return ERROR_FAIL;
    MCP2515_Context* ctx = (MCP2515_Context*)calloc(1, sizeof(MCP2515_Context));
    if (!ctx) return ERROR_FAIL;
    ctx->spi = spi;
    ctx->int_gpio = int_gpio;
    ctx->can_speed = cfg->can_speed;
    ctx->can_clock = cfg->can_clock;
    ctx->event_sem = xSemaphoreCreateBinary();
    if (!ctx->event_sem) { free(ctx); return ERROR_FAIL; }

    if (int_gpio >= 0) {
        gpio_config_t io_conf = {
            .pin_bit_mask = 1ULL << int_gpio,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_NEGEDGE
        };
        if (gpio_config(&io_conf) != ESP_OK) { vSemaphoreDelete(ctx->event_sem); free(ctx); return ERROR_FAIL; }
        esp_err_t err = gpio_install_isr_service(0);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) { vSemaphoreDelete(ctx->event_sem); free(ctx); return ERROR_FAIL; }
        if (gpio_isr_handler_add(int_gpio, mcp2515_isr_handler, ctx) != ESP_OK) { vSemaphoreDelete(ctx->event_sem); free(ctx); return ERROR_FAIL; }
    }

    if (mcp2515_ll_reset(ctx) != ERROR_OK) { MCP2515_Destroy(ctx); return ERROR_FAIL; }
    // Accept all frames on both RX buffers, enable rollover on RXB0
    mcp2515_ll_write(ctx, RXB0CTRL, RXM_MASK | RXB0_BUKT);
    mcp2515_ll_write(ctx, RXB1CTRL, RXM_MASK);
    // Enable RX interrupts by default; ERRIF also useful for event wake
    mcp2515_ll_write(ctx, MCP_CANINTE, (1u<<0) | (1u<<1) | (1u<<5)); // RX0IF|RX1IF|ERRIF
    *out_handle = ctx;
    return ERROR_OK;
}

ERROR_t MCP2515_CreateOnBus(spi_host_device_t host,
                            const spi_bus_config_t* bus_cfg,
                            const spi_device_interface_config_t* dev_cfg,
                            gpio_num_t int_gpio,
                            const mcp2515_multi_config_t* cfg,
                            MCP2515_Handle* out_handle)
{
    if (!bus_cfg || !dev_cfg) return ERROR_FAIL;
    if (mcp2515_spi_init_bus_if_needed(host, bus_cfg) != ESP_OK) return ERROR_FAIL;
    spi_device_handle_t spi = NULL;
    if (mcp2515_spi_add_device(host, dev_cfg, &spi) != ESP_OK) return ERROR_FAIL;
    ERROR_t rc = MCP2515_CreateOnDevice(spi, int_gpio, cfg, out_handle);
    if (rc != ERROR_OK) {
        (void)mcp2515_spi_remove_device(spi);
    }
    return rc;
}

void MCP2515_Destroy(MCP2515_Handle h)
{
    if (!h) return;
    if (h->int_gpio >= 0) {
        gpio_isr_handler_remove(h->int_gpio);
    }
    if (h->event_sem) vSemaphoreDelete(h->event_sem);
    free(h);
}

ERROR_t MCP2515_Reset(MCP2515_Handle h)
{
    if (!h) return ERROR_FAIL;
    return mcp2515_ll_reset(h);
}

// Bitrate tables (subset) taken from original library
static void bitrate_regs(CAN_SPEED_t s, CAN_CLOCK_t c, uint8_t* cnf1, uint8_t* cnf2, uint8_t* cnf3)
{
    // Defaults invalid
    *cnf1 = *cnf2 = *cnf3 = 0xFF;
    if (c == MCP_16MHZ) {
        switch (s) {
            case CAN_1000KBPS: *cnf1=0x00; *cnf2=0xD0; *cnf3=0x82; break;
            case CAN_500KBPS:  *cnf1=0x00; *cnf2=0xF0; *cnf3=0x86; break;
            case CAN_250KBPS:  *cnf1=0x41; *cnf2=0xF1; *cnf3=0x85; break;
            case CAN_125KBPS:  *cnf1=0x03; *cnf2=0xF0; *cnf3=0x86; break;
            case CAN_100KBPS:  *cnf1=0x03; *cnf2=0xFA; *cnf3=0x87; break;
            case CAN_80KBPS:   *cnf1=0x03; *cnf2=0xFF; *cnf3=0x87; break;
            case CAN_50KBPS:   *cnf1=0x07; *cnf2=0xFA; *cnf3=0x87; break;
            default: break;
        }
    } else if (c == MCP_8MHZ) {
        switch (s) {
            case CAN_1000KBPS: *cnf1=0x00; *cnf2=0x80; *cnf3=0x80; break;
            case CAN_500KBPS:  *cnf1=0x00; *cnf2=0x90; *cnf3=0x82; break;
            case CAN_250KBPS:  *cnf1=0x00; *cnf2=0xB1; *cnf3=0x85; break;
            case CAN_125KBPS:  *cnf1=0x01; *cnf2=0xB1; *cnf3=0x85; break;
            case CAN_100KBPS:  *cnf1=0x01; *cnf2=0xB4; *cnf3=0x86; break;
            default: break;
        }
    } else if (c == MCP_20MHZ) {
        switch (s) {
            case CAN_1000KBPS: *cnf1=0x00; *cnf2=0xD9; *cnf3=0x82; break;
            case CAN_500KBPS:  *cnf1=0x00; *cnf2=0xFA; *cnf3=0x87; break;
            case CAN_250KBPS:  *cnf1=0x41; *cnf2=0xFB; *cnf3=0x86; break;
            case CAN_125KBPS:  *cnf1=0x03; *cnf2=0xFA; *cnf3=0x87; break;
            case CAN_100KBPS:  *cnf1=0x04; *cnf2=0xFA; *cnf3=0x87; break;
            default: break;
        }
    }
}

ERROR_t MCP2515_SetBitrate(MCP2515_Handle h, CAN_SPEED_t speed, CAN_CLOCK_t clock)
{
    if (!h) return ERROR_FAIL;
    if (mcp2515_set_mode(h, CANCTRL_REQOP_CONFIG) != ERROR_OK) return ERROR_FAIL;
    uint8_t c1,c2,c3; bitrate_regs(speed, clock, &c1, &c2, &c3);
    if (c1==0xFF) return ERROR_FAIL;
    mcp2515_ll_write(h, MCP_CNF1, c1);
    mcp2515_ll_write(h, MCP_CNF2, c2);
    mcp2515_ll_write(h, MCP_CNF3, c3);
    h->can_speed = speed; h->can_clock = clock;
    return ERROR_OK;
}

ERROR_t MCP2515_SetNormalMode(MCP2515_Handle h)
{
    if (!h) return ERROR_FAIL;
    return mcp2515_set_mode(h, CANCTRL_REQOP_NORMAL);
}

ERROR_t MCP2515_SetLoopbackMode(MCP2515_Handle h)
{
    if (!h) return ERROR_FAIL;
    return mcp2515_set_mode(h, CANCTRL_REQOP_LOOPBACK);
}

static void prepare_id(uint8_t* buf, bool ext, uint32_t id)
{
    if (ext) {
        uint16_t canid = (uint16_t)(id & 0xFFFF);
        buf[3] = (uint8_t)(canid & 0xFF);        // EID0
        buf[2] = (uint8_t)(canid >> 8);          // EID8
        canid = (uint16_t)(id >> 16);
        buf[1] = (uint8_t)( (canid & 0x03) | ((canid & 0x1C) << 3) | TXB_EXIDE_MASK ); // SIDL
        buf[0] = (uint8_t)(canid >> 5);          // SIDH
    } else {
        buf[0] = (uint8_t)(id >> 3);             // SIDH
        buf[1] = (uint8_t)((id & 0x07) << 5);    // SIDL
        buf[2] = 0; buf[3] = 0;                  // EID8/EID0
    }
}

static uint8_t filter_base(uint8_t idx)
{
    switch (idx) {
        case 0: return RXF0SIDH;
        case 1: return RXF1SIDH;
        case 2: return RXF2SIDH;
        case 3: return RXF3SIDH;
        case 4: return RXF4SIDH;
        case 5: return RXF5SIDH;
        default: return 0xFF;
    }
}

static uint8_t mask_base(uint8_t idx)
{
    switch (idx) {
        case 0: return RXM0SIDH;
        case 1: return RXM1SIDH;
        default: return 0xFF;
    }
}

ERROR_t MCP2515_SetFilter(MCP2515_Handle h, uint8_t filter_idx, bool extended, uint32_t id)
{
    if (!h) return ERROR_FAIL;
    uint8_t base = filter_base(filter_idx);
    if (base == 0xFF) return ERROR_FAIL;
    // Save current mode and switch to config
    uint8_t prev = mcp2515_ll_read(h, MCP_CANSTAT) & CANSTAT_OPMOD;
    if (mcp2515_set_mode(h, CANCTRL_REQOP_CONFIG) != ERROR_OK) return ERROR_FAIL;
    uint8_t buf[4];
    prepare_id(buf, extended, id);
    for (int i=0;i<4;i++) mcp2515_ll_write(h, base + i, buf[i]); // SIDH..EID0
    // Restore previous mode
    (void)mcp2515_set_mode(h, prev);
    return ERROR_OK;
}

ERROR_t MCP2515_SetMask(MCP2515_Handle h, uint8_t mask_idx, bool extended, uint32_t mask)
{
    if (!h) return ERROR_FAIL;
    uint8_t base = mask_base(mask_idx);
    if (base == 0xFF) return ERROR_FAIL;
    uint8_t prev = mcp2515_ll_read(h, MCP_CANSTAT) & CANSTAT_OPMOD;
    if (mcp2515_set_mode(h, CANCTRL_REQOP_CONFIG) != ERROR_OK) return ERROR_FAIL;
    uint8_t buf[4];
    prepare_id(buf, extended, mask);
    for (int i=0;i<4;i++) mcp2515_ll_write(h, base + i, buf[i]); // SIDH..EID0
    (void)mcp2515_set_mode(h, prev);
    return ERROR_OK;
}

ERROR_t MCP2515_SendMessageAfterCtrlCheck(MCP2515_Handle h, const CAN_FRAME* frame)
{
    if (!h || !frame || frame->can_dlc > 8) return ERROR_FAILTX;
    // Choose TXB0 for simplicity
    // Load ID
    uint8_t idbuf[4];
    bool ext = (frame->can_id & (1u<<31)) != 0; // caller may set EFF flag in top bit
    uint32_t id = ext ? (frame->can_id & 0x1FFFFFFF) : (frame->can_id & 0x7FF);
    prepare_id(idbuf, ext, id);
    // Write ID + DLC + data
    // Write SIDH..EID0
    for (int i=0;i<4;i++) mcp2515_ll_write(h, TXB0SIDH + i, idbuf[i]);
    uint8_t dlc = frame->can_dlc & DLC_MASK;
    mcp2515_ll_write(h, TXB0DLC, dlc);
    for (int i=0;i<dlc;i++) mcp2515_ll_write(h, TXB0DATA + i, frame->data[i]);
    // Request to send
    mcp2515_ll_bitmod(h, TXB0CTRL, TXB_TXREQ, TXB_TXREQ);
    // Check error bits
    uint8_t ctrl = mcp2515_ll_read(h, TXB0CTRL);
    if (ctrl & (TXB_ABTF | TXB_MLOA | TXB_TXERR)) return ERROR_FAILTX;
    return ERROR_OK;
}

static uint8_t read_status(MCP2515_Handle h)
{
    spi_transaction_t trans = {};
    trans.length = 16;
    trans.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    trans.tx_data[0] = INSTRUCTION_READ_STATUS;
    trans.tx_data[1] = 0x00;
    (void)spi_device_transmit(h->spi, &trans);
    return trans.rx_data[1];
}

ERROR_t MCP2515_ReadMessageAfterStatCheck(MCP2515_Handle h, CAN_FRAME* frame)
{
    if (!h || !frame) return ERROR_FAIL;
    uint8_t stat = read_status(h);
    uint8_t base = 0;
    if (stat & STAT_RX0IF) base = RXB0SIDH; else if (stat & STAT_RX1IF) base = RXB1SIDH; else return ERROR_NOMSG;

    uint8_t hdr[5];
    for (int i=0;i<5;i++) hdr[i] = mcp2515_ll_read(h, base + i);
    uint32_t id = ((uint32_t)hdr[0] << 3) | (hdr[1] >> 5);
    bool ext = false;
    if (hdr[1] & TXB_EXIDE_MASK) {
        ext = true;
        id = (id << 2) | (hdr[1] & 0x03);
        id = (id << 8) | hdr[2];
        id = (id << 8) | hdr[3];
    }
    uint8_t dlc = hdr[4] & DLC_MASK;
    frame->can_id = ext ? (id | (1u<<31)) : id;
    frame->can_dlc = dlc;
    for (int i=0;i<dlc;i++) frame->data[i] = mcp2515_ll_read(h, (base==RXB0SIDH?RXB0DATA:RXB1DATA) + i);
    // Clear RXnIF
    mcp2515_ll_bitmod(h, MCP_CANINTF, (base==RXB0SIDH)?(1u<<0):(1u<<1), 0);
    return ERROR_OK;
}

void MCP2515_SetEventCallback(MCP2515_Handle h, MCP2515_EventCallback cb, void* userData)
{
    if (!h) return;
    h->cb = cb;
    h->cb_user = userData;
}

uint32_t MCP2515_WaitForEvent(MCP2515_Handle h, uint32_t timeout_ticks)
{
    if (!h) return 0;
    if (xSemaphoreTake(h->event_sem, timeout_ticks) == pdTRUE) {
        uint8_t eflg = mcp2515_ll_read(h, MCP_EFLG);
        uint8_t intf = mcp2515_ll_read(h, MCP_CANINTF);
        uint32_t ev = 0;
        if (intf & ((1u<<0)|(1u<<1))) ev |= MCP2515_EVENT_RX_READY;
        if (eflg) ev |= MCP2515_EVENT_ERROR;
        if (h->cb && ev) h->cb(h, ev, h->cb_user);
        return ev;
    }
    return 0;
}

uint8_t MCP2515_GetErrorFlags(MCP2515_Handle h)
{
    if (!h) return 0;
    return mcp2515_ll_read(h, MCP_EFLG);
}

void MCP2515_ClearRXnOVR(MCP2515_Handle h)
{
    if (!h) return;
    // Clear RX0OVR | RX1OVR and clear interrupts
    // RXnOVR bits are bits 7 and 6 in EFLG; clear by bit-modify
    mcp2515_ll_bitmod(h, MCP_EFLG, (uint8_t)0xC0, 0);
    mcp2515_ll_write(h, MCP_CANINTF, 0);
}

void MCP2515_ClearERRIF(MCP2515_Handle h)
{
    if (!h) return;
    // Clear ERRIF in CANINTF
    mcp2515_ll_bitmod(h, MCP_CANINTF, (1u<<5), 0);
}


