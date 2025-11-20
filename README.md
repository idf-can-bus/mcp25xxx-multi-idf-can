# mcp25xxx-multi-idf-can

Extended CAN driver for ESP32 using multiple MCP25xxx controllers over SPI.

![ESP32 with multiple MCP25xxx CAN controllers](doc/ESP32_MCP2515_CAN_steampunk400x400.png)

High-level multi-device MCP25xxx library for ESP-IDF with a stable public interface and a separate internal backend. Supports multiple MCP25xxx controllers on one or more SPI buses with independent or parallel operation, shared SPI interface and separate CS/INT lines.

**Note:** While designed for multi-device setups, this library is equally well-suited for single MCP25xxx device applications. The unified API provides a clean, consistent interface regardless of the number of devices.

## Features

- ✅ **Scalable design** - works seamlessly with single or multiple MCP25xxx devices
- ✅ **Multiple MCP25xxx devices** on shared or independent SPI buses
- ✅ **Flexible addressing** via numeric IDs, handles, or composite targets
- ✅ **Registry-based** device management with lookup functions
- ✅ **Event-driven or polling** reception modes
- ✅ **Hardware filtering** with configurable acceptance filters and masks
- ✅ **Runtime reconfiguration** of bitrate and operating mode
- ✅ **Standard and extended** CAN frame support (11-bit and 29-bit IDs)
- ✅ **Interrupt support** for efficient message reception
- ✅ **Well Documented** - Full Doxygen documentation with examples
- ✅ **ESP-IDF v5.0+** - Compatible with modern ESP-IDF versions
- ✅ **Multiple ESP32 Variants** - Tested on ESP32, ESP32-S2, ESP32-S3, ESP32-C3, ESP32-C6

## Hardware Requirements

### MCP25xxx CAN Controllers

This library supports MCP25xxx family CAN controllers connected via SPI. The library was tested with **MCP25625** (integrated controller + transceiver) modules.

### Typical Wiring

```
ESP32          MCP25xxx          CAN Bus
GPIO_MISO <---- MISO
GPIO_MOSI ----> MOSI
GPIO_SCLK ----> SCK
GPIO_CS   ----> CS (one per device)
GPIO_INT  <---- INT (optional, one per device)
3.3V      ----> VCC
GND       ----> GND
                 CANH   --------> CAN_H
                 CANL   --------> CAN_L
```

**Note:** Multiple devices on the same SPI bus share MISO, MOSI, and SCLK lines, but each device requires its own unique CS (Chip Select) pin. INT (Interrupt) pins are also individual per device and are used for efficient interrupt-driven message reception (see receive_interrupt example).

**Important Notes:**
- **CAN bus termination:** The general recommendation is to place one 120-ohm termination resistor at each end of a long CAN bus. However, the author's experience with short experimental setups shows that using only one 120-ohm resistor for the entire bus often works better.
- **MCP25xxx modules:** Some MCP25xxx breakout modules already include an onboard 120-ohm termination resistor. Depending on your network topology, you may need to add or remove these onboard resistors so that the bus has the correct number of terminations.
- **SPI buses:** Either SPI2_HOST or SPI3_HOST may be used (SPI1_HOST is reserved for flash on most ESP32 variants)
- **GPIO pins:** Custom GPIO assignments are fully configurable
- **Multiple devices:** Each MCP25xxx device requires a CS (Chip Select) pin; optionally an INT (Interrupt) pin for event-driven reception
- Maximum cable length depends on bitrate (1 Mbps ≤ 40m, 125 kbps ≤ 500m)

### Tested Hardware Configuration

This library has been successfully tested with the following hardware setup:

**Microcontroller:**
- [Waveshare ESP32-S3-Pico](https://www.waveshare.com/esp32-s3-pico.htm) (Model: 23803)
  - [Wiki Documentation](https://www.waveshare.com/wiki/ESP32-S3-Pico)

**CAN Modules:**
- 5x [Adafruit CAN Bus BFF](https://learn.adafruit.com/adafruit-can-bus-bff) modules connected to a single ESP32-S3 via two SPI buses
  - 3 modules on SPI2_HOST configured as receivers (RX)
  - 2 modules on SPI3_HOST configured as transmitters (TX)
  - Uses MCP25625 integrated circuit (combines MCP25xxx CAN controller + MCP2551 CAN transceiver)
  - **Important:** STBY/A1 pin connected to ground on all modules for normal operation mode

**Note:** This demonstrates the library's capability to manage multiple MCP25xxx devices across multiple SPI buses simultaneously. Each SPI bus can support multiple devices sharing MISO/MOSI/SCK lines, with each device requiring its own unique CS pin and optionally an INT pin (used in receive_interrupt example for efficient event-driven reception).

### When to Use MCP25xxx vs Built-in TWAI

#### Understanding CAN Hardware Components

A complete CAN bus interface requires two hardware components:

1. **CAN Controller** - Implements the CAN protocol logic:
   - Creates and decodes CAN frames
   - Handles timing, bit stuffing, and arbitration
   - Manages error detection and filtering
   - Operates at logic level (digital signals)

2. **CAN Transceiver** - Provides the physical layer:
   - Converts logic signals to differential voltage on CANH/CANL lines
   - Converts differential signals back to logic levels
   - Provides noise immunity and ESD protection
   - Common chips: TJA1050, SN65HVD230, MCP2551

**ESP32 with TWAI:**
- Has a built-in **CAN Controller** (called TWAI - Two-Wire Automotive Interface)
- Still requires an **external CAN Transceiver** chip (e.g., TJA1050, SN65HVD230) to connect to the physical CAN bus

**MCP25xxx solutions:**
- **MCP2515**: CAN Controller only (requires separate transceiver like MCP2551)
- **MCP25625**: Integrated solution (Controller + Transceiver in one chip) - used in this project's test configuration

#### When to Choose Each Option

**Use MCP25xxx when:**
- You need multiple independent CAN buses (ESP32 has only one built-in TWAI controller)
- TWAI GPIO pins conflict with other peripherals in your design
- You want to add CAN capability to boards without built-in TWAI support
- You need flexibility in pin assignment (any GPIO can be used for SPI)

**Use built-in TWAI when:**
- Single CAN bus is sufficient for your application
- You want to minimize external components (only transceiver chip needed, no controller)
- Lower latency is critical (direct hardware controller vs SPI communication)
- Cost optimization is important (one less IC required)

For more information about the built-in TWAI option, see [twai-idf-can](https://github.com/idf-can-bus/twai-idf-can).

## Installation

### Manual Installation

Clone into your project's `components` directory:

```bash
cd your_project/components
git clone --recurse-submodules https://github.com/idf-can-bus/mcp25xxx-multi-idf-can.git
```

(Use `--recurse-submodules` because this project includes the submodule [examples-utils-idf-can](https://github.com/idf-can-bus/examples-utils-idf-can.git). You can also initialize submodules later with `git submodule update --init --recursive`.)

### ESP Component Registry (Future)

When published to the ESP Component Registry, install via:
```bash
idf.py add-dependency "mcp25xxx-multi-idf-can"
```

## Project Layout

The component is organized into a small set of well-defined directories:

```text
mcp25xxx-multi-idf-can/
├─ src/                          # Implementation of the MCP25xxx adapter
│   ├─ mcp25xxx_multi.c
│   ├─ mcp25xxx_multi_internal.c
│   └─ mcp25xxx_multi_internal.h  # Internal backend (not exported)
├─ include/                       # Public headers (API and configuration types)
│   └─ mcp25xxx_multi.h
├─ examples/                      # Example applications using this component
│   ├─ config_send.h              # HW config for send example (2 TX devices)
│   ├─ config_receive.h           # HW config for receive examples (3 RX devices)
│   ├─ send/
│   ├─ receive_poll/
│   └─ receive_interrupt/
└─ components/
    └─ examples-utils-idf-can/    # Submodule with shared utilities for examples
```

## API Overview and Naming Conventions

**Public API:** `mcp25xxx_multi.h`
- Configuration types: `mcp_spi_bus_config_t`, `mcp2515_device_config_t`, `mcp2515_bundle_config_t`
- Numeric IDs/handles: `can_bus_id_t`, `can_dev_id_t`, `can_bus_handle_t`, `can_dev_handle_t`, `can_target_t`
- Registry/lifecycle: `canif_register_bundle`, `canif_open_device`, `canif_open_all`
- Messaging: `canif_send_to`, `canif_receive_from`, also `*_id`/`*_target` variants
- Mode/bitrate, events, errors, filters/masks
- Message type: `twai_message_t` (ESP-IDF standard)

**Internal backend:** `mcp2515_multi_internal.h`
- Low-level SPI/MCP25xxx control: `MCP25xxx_*` API
- Not exported to users; subject to change

**Naming conventions:**
- Public facade functions: `canif_*`
- Public configuration types: `mcp_*` (bus/device/bundle)
- Identifiers and messages: `can_*` (`can_message_t`, `can_bus_id_t`, ...)
- Internal backend: `MCP25xxx_*`, `ERROR_t`

See `mcp25xxx_multi.h` for full Doxygen documentation.

## Quick Start

**Note:** These instructions work for both single and multiple MCP25xxx devices. For a single device, simply configure a bundle with `device_count = 1`.

### 1. Include Headers

```c
#include "mcp25xxx_multi.h"
```

### 2. Configure Hardware

Provide bundle configuration (one SPI bus + N devices), ideally as a `const` in a header near your example/app:

```c
extern const mcp2515_bundle_config_t CAN_HW_CFG; // see examples for templates
```

**Note:** Examples use two separate configuration files:
- [examples/config_send.h](examples/config_send.h) for TX-only setup (2 devices on SPI3)
- [examples/config_receive.h](examples/config_receive.h) for RX setup with interrupts (3 devices on SPI2)

### 3. Initialize

```c
canif_multi_init_default(&CAN_HW_CFG); // registers bundle and opens all devices
```

### 4. Send/Receive Messages

```c
can_bus_handle_t bus = canif_bus_default();
for (size_t i = 0; i < canif_bus_device_count(bus); ++i) {
    can_dev_handle_t dev = canif_device_at(bus, i);
    twai_message_t msg = { .identifier = 0x123, .data_length_code = 2, .data = { 0xDE, 0xAD } };
    (void)canif_send_to(dev, &msg);
}
```

---
See examples for detailed usage patterns:
- [examples/send/](./examples/send/main/main.c)
- [examples/receive_poll/](./examples/receive_poll/main/main.c)
- [examples/receive_interrupt/](./examples/receive_interrupt/main/main.c)
---

## Doxygen Documentation

All public headers (`include/mcp25xxx_multi.h` and `components/examples-utils-idf-can/include/examples_utils.h`) are fully documented with Doxygen comments.
To integrate this component into your own documentation, add these include directories to your Doxygen `INPUT` paths.

## Troubleshooting

### No Messages Received

1. Check physical wiring (especially CANH/CANL polarity)
2. Verify termination resistors (see wiring notes above)
3. Ensure matching bitrate on all MCP25xxx devices and CAN nodes
4. Check SPI wiring (MISO, MOSI, SCK, CS pins)
5. Verify MCP25xxx crystal frequency setting matches your hardware
6. Check if STBY pin is grounded (for MCP25625 modules)

### SPI Communication Errors

1. Verify SPI bus configuration (host, speed, DMA)
2. Check CS pin assignments (each device needs unique CS)
3. Ensure GPIO pins are not used by other peripherals
4. Try reducing SPI clock speed (from 10 MHz to 5 MHz)

### Build Errors

1. Ensure ESP-IDF version is 5.0 or newer
2. Check that `driver`, `freertos`, `esp_timer` components are available
3. Verify component is properly included in your project
4. Make sure submodules are initialized (`git submodule update --init --recursive`)

Check ESP-IDF documentation for SPI Master driver details.

## Examples

The library includes three ready-to-use examples in the `examples/` directory:

### Hardware Configuration for Examples

**Important:** Hardware configuration (GPIO pins, SPI speed, MCP25xxx crystal frequency, CAN bitrate) is defined in configuration header files:
- [examples/config_send.h](examples/config_send.h) - Configuration for send example (2 TX devices on SPI3)
- [examples/config_receive.h](examples/config_receive.h) - Configuration for receive examples (3 RX devices on SPI2)

You **must** adapt these configuration files to match your hardware setup before building the examples.

Configuration includes:
- **SPI bus wiring**: MISO, MOSI, SCLK pins
- **Device-specific pins**: CS (Chip Select), INT (Interrupt), optional STBY/RST
- **SPI parameters**: Clock speed (typically 10 MHz), mode, DMA channel
- **MCP25xxx hardware**: Crystal frequency (`MCP25XXX_8MHZ`, `MCP25XXX_16MHZ`, or `MCP25XXX_20MHZ`)
- **CAN parameters**: Bitrate (e.g., `MCP25XXX_500KBPS`, `MCP25XXX_1000KBPS`), loopback mode

### Building Examples

**Build all examples at once:**

```bash
# Using bash script
./build_all_examples.sh

# Using Makefile
make
```

You can also build individual examples using the Makefile targets:

```bash
make send
make receive_poll
make receive_interrupt
```

For CI/CD pipelines, you can run:

```bash
./build_all_examples.sh build
```

This script returns exit code `0` when all examples build successfully and `1` if any example fails.

This setup should be sufficient for most example builds and CI use cases.

### Prerequisites for Building Examples

Before building examples, set up your environment:

```bash
# 1. Activate ESP-IDF environment
. $HOME/esp/esp-idf/export.sh

# 2. Set target chip for ALL examples at once
./set_target_all.sh esp32s3  # Use: esp32, esp32s2, esp32s3, esp32c3, esp32c6, etc.

# 3. Build all examples
./build_all_examples.sh
```

Available targets for `set_target_all.sh` are the same as for the `idf.py set-target` command (for example: esp32, esp32s2, esp32s3, esp32c3, esp32c6, esp32h2, ...).


### Using idf.py Directly

You can work with individual examples using `idf.py` commands:

```bash
idf.py build                    # Build the current example
idf.py -p /dev/ttyUSB0 flash    # Flash the firmware
idf.py -p /dev/ttyUSB0 monitor  # Open serial monitor
```

To combine flashing and monitoring in one step:

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

You can stop the ESP-IDF monitor with `Ctrl+]` or with `Ctrl+T` followed by `X`.

---
**For tests you can use the send example on one ESP32 board and one of the receive examples on another board connected to the same CAN bus.**

Use the `examples/send` project on Board 1 (sender) and one of the `examples/receive_*` projects on Board 2 (receiver). Connect both boards via MCP25xxx transceivers; for short test setups, a single 120Ω termination resistor at one end of the bus is usually sufficient.

---

### 1. Send Example (`examples/send/`)

Demonstrates sending CAN messages to multiple independent buses (polling transmission).

```bash
cd examples/send
idf.py set-target esp32s3  # Set your chip type first!
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

### 2. Receive Polling Example (`examples/receive_poll/`)

Demonstrates receiving CAN messages from multiple buses using polling mode (no interrupts).

```bash
cd examples/receive_poll
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

### 3. Receive Interrupt Example (`examples/receive_interrupt/`)

Demonstrates receiving CAN messages using interrupt-driven reception (most efficient). Uses a producer-consumer pattern with queue buffering to prevent message loss during processing.

```bash
cd examples/receive_interrupt
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Related Projects

My other related CAN bus libraries for ESP-IDF are maintained in the **[idf-can-bus](https://github.com/idf-can-bus)** organization on GitHub.

## Acknowledgments

This project is based on the excellent work of [Microver Electronics](https://github.com/Microver-Electronics/mcp2515-esp32-idf).
Their original [mcp2515-esp32-idf](https://github.com/Microver-Electronics/mcp2515-esp32-idf) project provided the foundation for this extended multi-bus version.

## License

MIT License - see [LICENSE](LICENSE) file for details.

## Author

Ivo Marvan, 2025
