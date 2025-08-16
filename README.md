# TLA202X

TLA202X driver based on efr32mg13p of silicon labs' chipset and gecko sdk's i2c api.

<English>
I rely on a lot of open source help in my work.
So I'm a little bit of a

I'm open-sourcing the ADC device driver in case it helps someone.

I hope it helps those who need it.

<Korean>
저도 일할 때 오픈소스 도움을 많이 받고 있습니다.
그래서 저도 미약하나마

누군가에게 도움이 될 수 있도록 adc device driver를 오픈소스로 공개합니다.

필요한 분들에게 많은 도움이 되길 바랍니다.


## Project Overview

This repository contains the device driver for the TLA2021/TLA2024 Analog-to-Digital Converter (ADC), designed for use with Silicon Labs' EFR32MG13P chipset and leveraging the Gecko SDK's I2C API. This driver facilitates communication with the TLA2021/TLA2024 ADC via I2C to read analog voltage values and convert them into current measurements.

## Features

*   **I2C Communication:** Utilizes the I2C protocol for robust communication with the TLA2021/TLA2024 ADC.
*   **Multi-Channel Support:** Supports reading from up to four analog input channels (AIN0-AIN3).
*   **Configurable Data Rates:** Allows setting various data rates for ADC conversions (128 SPS to 3300 SPS).
*   **Flexible Operating Modes:** Supports both single-shot and continuous conversion modes.
*   **Programmable Gain Amplifier (PGA):** Configurable full-scale range (FSR) from ±0.256V to ±6.144V to optimize measurement resolution.
*   **Voltage to Current Conversion:** Includes logic to convert measured voltage values into current (uA) based on a defined mapping.
*   **Initialization and Process Functions:** Provides clear API for initializing the ADC and continuously processing measurements.

## TLA2021 Driver Details

The core functionality is implemented in `tla2021.c` and `tla2021.h`.

### Supported Device

*   TLA2021 / TLA2024 (12-bit, 4-channel, I2C ADC)

### I2C Address

The default I2C slave address for the TLA202x is `0x48` (shifted to `0x90` for 8-bit addressing in the driver). The address can be configured based on the ADDR pin setting (GND, VDD, SCL).

### Registers

*   `TLA202x_DATA_REG` (0x00): Data Register, used for reading conversion results.
*   `TLA202x_CONFIG_REG` (0x01): Configuration Register, used for setting ADC parameters.

### Configuration Options (Enums)

*   `tla202x_channel_t`: Defines the single channel to read (CHANNEL_0 to CHANNEL_3).
*   `tla202x_rate_t`: Specifies the data rate for conversions (e.g., `TLA202x_RATE_128_SPS`, `TLA202x_RATE_3300_SPS`).
*   `tla202x_mode_t`: Sets the operating mode (`TLA202x_MODE_CONTINUOUS` or `TLA202x_MODE_ONE_SHOT`).
*   `tla202x_mux_t`: Configures the multiplexer for differential or single-ended measurements (e.g., `TLA202x_MUX_AIN0_AIN1`, `TLA202x_MUX_AIN0_GND`).
*   `tla202x_range_t`: Defines the full-scale voltage range (`TLA202x_RANGE_0_256_V` to `TLA202x_RANGE_6_144_V`).

### Key Functions

*   `TLA2021_i2cInit()`: Initializes the I2C peripheral using `I2CSPM_Init()`.
*   `TLA2024_begin()`: Initializes the TLA202x device, performs a reset, and verifies communication.
*   `TLA2024_reset()`: Resets the ADC to its default configuration.
*   `TLA2024_read(uint8_t mem_addr)`: Reads a 16-bit value from the specified register.
*   `TLA2024_write(uint16_t data)`: Writes a 16-bit configuration value to the configuration register.
*   `TLA2024_analogRead()`: Performs an analog-to-digital conversion and returns the raw 12-bit ADC value.
*   `TLA2024_analogReadChannel(tla202x_channel_t channel)`: Reads the raw ADC value for a specific channel.
*   `TLA2024_setFullScaleRange(tla202x_range_t range)`: Sets the PGA full-scale range.
*   `TLA2024_setMuxConfig(tla202x_mux_t option)`: Configures the input multiplexer.
*   `TLA2024_setOperatingMode(tla202x_mode_t mode)`: Sets the ADC operating mode.
*   `TLA2024_setDataRate(tla202x_rate_t rate)`: Sets the ADC data rate.
*   `TLA2024_readVoltage(tla202x_channel_t channel)`: Reads the voltage (in mV) for a given channel, considering the set full-scale range.
*   `voltage2Current(uint32_t voltage)`: Converts a voltage value (mV) to a current value (uA) based on an internal mapping.
*   `tla2021_init()`: High-level initialization function for the driver.
*   `tla2021_process()`: Main processing function that reads all channels, converts voltage to current, and updates a global `g_adc_info_t` structure.

### Dependencies

The driver relies on the following headers/modules, typically part of the Silicon Labs Gecko SDK or a similar embedded development environment:

*   `PLATFORM_HEADER`
*   `rtos_err.h`
*   `os.h`
*   `stack/include/ember-types.h`
*   `stack/include/event.h`
*   `hal/hal.h`
*   `hal/plugin/i2c-driver/i2c-driver.h`
*   `hal/micro/micro.h`
*   `zigbee_common.h`
*   `i2cspm.h`
*   `app/framework/include/af.h`
*   `nx_config.h` (from `tla2021.h`)

### Usage Example

To integrate this driver into your application:

1.  **Initialization:** Call `tla2021_init()` once during your system startup to initialize the I2C and ADC.
2.  **Periodic Processing:** Call `tla2021_process()` periodically (e.g., within a main loop or a dedicated task) to read ADC values and update the global current information.

```c
#include "tla2021.h"
#include "hal/hal.h" // For halCommonDelayMilliseconds

// Global ADC info structure (defined in tla2021.h)
adc_info_t g_adc_info_t;

int main() {
    // ... other system initializations ...

    tla2021_init(); // Initialize the TLA2021 driver

    while (1) {
        tla2021_process(); // Process ADC readings and update current values
        halCommonDelayMilliseconds(1000); // Delay for 1 second (example)
    }

    return 0;
}
```

### License

This project is licensed under the MIT License. See the `LICENSE` file for details.