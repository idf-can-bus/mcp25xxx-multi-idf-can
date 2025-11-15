# Building All Examples

This document describes how to build all MCP25xxx-Multi-IDF-CAN examples with a single command.

## Prerequisites

- ESP-IDF v5.0 or newer installed

### Setup ESP-IDF Environment

Before building, you must activate the ESP-IDF environment and set the target chip:

```bash
# 1. Activate ESP-IDF environment
. $HOME/esp/esp-idf/export.sh   # Or your IDF installation path

# 2. Set target chip for ALL examples at once (recommended!)
./set_target_all.sh esp32s3      # For ESP32-S3
# ./set_target_all.sh esp32      # For original ESP32
# ./set_target_all.sh esp32s2    # For ESP32-S2
# ./set_target_all.sh esp32c3    # For ESP32-C3
# ./set_target_all.sh esp32c6    # For ESP32-C6

# Alternatively, set target manually for each example:
# cd examples/send && idf.py set-target esp32s3
# cd ../receive_poll && idf.py set-target esp32s3
# cd ../receive_interrupt && idf.py set-target esp32s3
```

**Important:** Each example needs `set-target` configured before first build!

## Quick Start

**Complete workflow for building all examples:**

```bash
# 1. Activate ESP-IDF
. $HOME/esp/esp-idf/export.sh

# 2. Set target for all examples
./set_target_all.sh esp32s3

# 3. Build all examples
./build_all_examples.sh
```

## Method 1: Using Bash Script (Recommended)

The `build_all_examples.sh` script provides the most flexibility and colored output.

### Build all examples

```bash
./build_all_examples.sh build
```

or simply:

```bash
./build_all_examples.sh
```

### Clean all examples

```bash
./build_all_examples.sh fullclean
```

### Other actions

The script supports any `idf.py` command:

```bash
./build_all_examples.sh menuconfig  # Open menuconfig for each example
./build_all_examples.sh size        # Show size for each example
./build_all_examples.sh reconfigure # Reconfigure all examples
```

**Note:** Commands like `flash` and `monitor` require a serial port and are interactive, so they're not practical for batch processing all examples. Flash/monitor examples individually.

## Method 2: Using Makefile

The Makefile provides standard GNU Make targets.

### Build all examples

```bash
make
```

or:

```bash
make build
```

### Clean all examples

```bash
make clean
```

### Build individual examples

```bash
make send
make receive_poll
make receive_interrupt
```

### Show help

```bash
make help
```

## Method 3: Manual Build (Traditional)

You can still build examples individually:

```bash
cd examples/send
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Hardware Configuration

Examples use different hardware configurations:

- **send example:** Configuration in `examples/config_send.h` (2 TX devices on SPI3)
- **receive examples:** Configuration in `examples/config_receive.h` (3 RX devices on SPI2)

**Before building**, modify these files to match your setup:

### config_send.h (for send example)

```c
.bus = {
    .wiring = {
        .miso_io_num   = GPIO_NUM_15,  // ← Modify for your board
        .mosi_io_num   = GPIO_NUM_16,
        .sclk_io_num   = GPIO_NUM_14,
    },
    .params = {
        .host = SPI3_HOST,  // ← SPI3_HOST or SPI2_HOST
    },
},
.devices = {
    {
        .wiring = {
            .cs_gpio  = GPIO_NUM_11,   // ← CS pin for device 1
        },
        .hw   = { .crystal_frequency = MCP25XXX_16MHZ },
        .can  = { .can_speed = MCP25XXX_1000KBPS },
    },
    // ... more devices
},
.device_count = 2,  // ← Number of MCP25xxx devices
```

### config_receive.h (for receive examples)

```c
.bus = {
    .wiring = {
        .miso_io_num   = GPIO_NUM_37,  // ← Modify for your board
        .mosi_io_num   = GPIO_NUM_38,
        .sclk_io_num   = GPIO_NUM_36,
    },
    .params = {
        .host = SPI2_HOST,  // ← SPI2_HOST or SPI3_HOST
    },
},
.devices = {
    {
        .wiring = {
            .cs_gpio  = GPIO_NUM_33,   // ← CS pin for device 1
            .int_gpio = GPIO_NUM_34,   // ← INT pin for device 1
        },
        .hw   = { .crystal_frequency = MCP25XXX_16MHZ },
        .can  = { .can_speed = MCP25XXX_1000KBPS },
    },
    // ... more devices
},
.device_count = 3,  // ← Number of MCP25xxx devices
```

**Important:** The default configurations are set for ESP32-S3. Change GPIO pins to match your specific chip!

### GPIO Pin Compatibility

Different ESP32 variants have different available GPIO pins:

| Chip | SPI Pins | CS/INT Pins | Notes |
|------|----------|-------------|-------|
| ESP32 | Any GPIO | Any GPIO | Most flexible |
| ESP32-S2 | Limited | Limited | Check datasheet |
| ESP32-S3 | GPIO_14-16, GPIO_36-38 | Any available | Some pins used by Flash/PSRAM (default in config) |
| ESP32-C3 | Limited | Limited | Fewer GPIO pins available |
| ESP32-C6 | Limited | Limited | Fewer GPIO pins available |

**Always check your chip's datasheet** for GPIO availability and SPI peripheral mapping before wiring!

## Troubleshooting

### Error: IDF_PATH is not set

Make sure you've sourced the ESP-IDF environment:

```bash
. $HOME/esp/esp-idf/export.sh
```

Or add it to your `.bashrc` for automatic loading.

### Error: GPIO_NUM_XX undeclared

This means the GPIO pin is not available on your target chip. 

**Solution:**
1. Check which chip you're targeting: `idf.py --version` shows current target
2. Set correct target: `idf.py set-target esp32s3` (or your chip)
3. Modify `examples/config_send.h` or `examples/config_receive.h` to use available GPIO pins
4. See "GPIO Pin Compatibility" table above for recommendations

### Error: Target not set

If you see build errors about missing components or configuration:

```bash
cd examples/send
idf.py set-target esp32s3  # Set your chip type
idf.py build
```

### Build fails for one example

The script will continue building remaining examples and report all failures at the end. Check the output for specific error messages.

### Permission denied when running script

Make the scripts executable:

```bash
chmod +x set_target_all.sh
chmod +x build_all_examples.sh
```

## Examples Overview

### 1. **send** - Multi-Device Sender
- Sends CAN messages from multiple TX devices
- Demonstrates parallel transmission on independent buses
- Uses 2 MCP25xxx devices (configurable)

### 2. **receive_poll** - Polling Receiver
- Receives messages from multiple RX devices using polling
- Simple blocking receive loop for each device
- Good for understanding basic multi-device operation

### 3. **receive_interrupt** - Interrupt Receiver
- Uses interrupt-driven reception from multiple devices
- Producer-consumer pattern with queue buffering
- Prevents message loss during processing
- Recommended for production use

## Build Output

Each example will be built in its own `build/` directory:

```
examples/
├── send/
│   └── build/
│       └── mcp25xxx_send_example.bin
├── receive_poll/
│   └── build/
│       └── mcp25xxx_receive_poll_example.bin
└── receive_interrupt/
    └── build/
        └── mcp25xxx_receive_interrupt_example.bin
```

## Continuous Integration

For CI/CD pipelines, use the bash script:

```bash
# In your CI script
./build_all_examples.sh build
```

The script returns:
- Exit code 0: All examples built successfully
- Exit code 1: One or more examples failed

## Related Documentation

- [Main README](README.md) - Library documentation
- [examples/config_send.h](examples/config_send.h) - Hardware configuration for send example
- [examples/config_receive.h](examples/config_receive.h) - Hardware configuration for receive examples
- [ESP-IDF SPI Master Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html)


