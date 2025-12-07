# MAX77972 Driver Walkthrough

Implemented the I2C driver for the MAX77972 logic on ESP32.

## Included
- **Core Driver (`max77972.h/cpp`)**: Handles I2C communication and interacts with the chip's registers.
- **Example Project**: A complete ESP-IDF example to initialize the charger and read fuel gauge data.

## Register Map Verification
Using the datasheet:
- **I2C Address**: `0x36` (covers 0x00-0xFF map).
- **Charger Registers**:
  - `nChgConfig` block starts at `0xD0`.
  - `ChargingCurrent` is at `0x28` with LSB ~0.156mA.
  - `ChargingVoltage` is at `0x2A` with LSB 78.125uV (and 10mV/100mV steps logic).
- **Fuel Gauge Registers**:
  - `VCell` (Voltage) is at `0x1A`.
  - `RepSOC` (State of Charge) is at `0x07`.

## Usable API
The driver provides a high-level C++ API:
```cpp
// Initialize
charger.begin(PORT, SDA, SCL);

// Configure
charger.setFastChargeCurrent(1500); // Set 1500mA
charger.setTopOffVoltage(4200);     // Set 4.20V
charger.enableCharger(true);        // Enable Charging

// Read
float soc = charger.getSoC();
float volts = charger.getVoltage();
```

