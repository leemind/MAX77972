#pragma once

#include <Arduino.h>
#include <Wire.h>

// I2C Address (7-bit)
#define MAX77972_I2C_ADDR_DEFAULT 0x36

// --- Register Map ---

// Fuel Gauge / Status / Monitoring
#define MAX77972_REG_STATUS         0x00
#define MAX77972_REG_CHG_MASK_STS   0x01
#define MAX77972_REG_REP_CAP        0x06 // Capacity (mAh LSB=0.5)
#define MAX77972_REG_REP_SOC        0x07 // SOC (% LSB=1/256)
#define MAX77972_REG_DESIGN_CAPACITY  0x18 // Design Capacity mAh - scalar 0.5
#define MAX77972_REG_AVG_VCELL      0x19 // Avg Voltage
#define MAX77972_REG_VCELL          0x1A // Battery Voltage (LSB=78.125uV)
#define MAX77972_REG_TEMP           0x1B // Temperature
#define MAX77972_REG_IIN            0x1C // Input Current (LSB=0.15625mA)
#define MAX77972_REG_FSTAT          0x3D // Fuel Gauge Status

// Smart Power Selector / Charger Config
#define MAX77972_REG_ICHG           0x28 // ChargingCurrent (LSB=0.15625mA)
#define MAX77972_REG_VCHG           0x2A // ChargingVoltage (LSB=78.125uV approx, 10mV steps logic inside)

// Charger Configuration Registers (0xD0 - 0xD5 ...)
#define MAX77972_REG_NCHG_CFG_0     0xD0 // Mode, WDTEN
#define MAX77972_REG_NCHG_CFG_3     0xD3 // CHGIN_ILIM
#define MAX77972_REG_NCHG_CFG_4     0xD4
#define MAX77972_REG_NCHG_CFG_5     0xD5 // ChgEnable
#define MAX77972_REG_CHG_DETAILS_00 0xD6
#define MAX77972_REG_CHG_DETAILS_01 0xD7

// Masks
#define MAX77972_CFG5_CHG_EN_MASK   (1 << 1)

#define MAX77972_STATUS_POR         (1 << 1) // Power-On Reset bit in Status (0x00)
#define MAX77972_FSTAT_DNR          (1 << 0) // Data Not Ready bit in FStat (0x3D)

class MAX77972 {
public:
    MAX77972();

    /**
     * @brief Initialize the driver
     * @param bus I2C bus reference
     * @param address I2C address (7-bit)
     * @return true on success
     */
    bool begin(TwoWire& bus, uint8_t address = MAX77972_I2C_ADDR_DEFAULT);

    // --- Charger Control ---
    
    /**
     * @brief Set Fast Charge Current
     * @param current_ma Current in mA (Valid range: 100mA - 3150mA, Step: ~0.156mA)
     * @return 0 on success
     */
    int setFastChargeCurrent(uint16_t current_ma);

    /**
     * @brief Get Fast Charge Current
     * @return Current in mA, or -1 on error
     */
    int getChargingCurrent();

    /**
     * @brief Set Charging Voltage
     * @param voltage_mv Voltage in mV
     * @return 0 on success
     */
    int setChargingVoltage(int voltage_mv);

    /**
     * @brief Get Top-off Voltage (Termination Voltage)
     * @return Voltage in mV, or -1 on error
     */
    int getChargingVoltage();

    /**
     * @brief Enable or Disable Charger
     * @param enable true to enable, false to disable
     * @return 0 on success
     */
    int enableCharger(bool enable);

    /**
     * @brief Get Charger Status
     * @return Status register value (0x00)
     */
    uint8_t getChargerStatus();

    // --- Fuel Gauge (ModelGauge m5) ---

    /**
     * @brief Get State of Charge
     * @return SOC in percentage (0.0 to 100.0)
     */
    float getSoC();

    /**
     * @brief Get Battery Voltage
     * @return Voltage in mV Let's correct this comment as well
     */
    int getVoltage();

    /**
     * @brief Get Battery Current
     * @return Current in mA (Calculated/Estimated or from register if available - not implemented yet)
     */
    int getCurrent();

    /**
     * @brief Get Remaining Capacity
     * @return Capacity in mAh
     */
    float getCapacity();

    /**
     * @brief Get Design Capacity
     * @return Capacity in mAh, or -1 on error
     */
    int getDesignCapacity();

    /**
     * @brief Set Design Capacity
     * @param capacity_mah Capacity in mAh
     * @return 0 on success
     */
    int setDesignCapacity(uint16_t capacity_mah);

    /**
     * @brief Get Temperature
     * @return Temperature in degrees Celsius
     */
    float getTemperature();

private:
    TwoWire* _i2c;
    uint8_t _address;

    int writeRegister(uint8_t reg, uint8_t data);
    int readRegister(uint8_t reg, uint8_t *data);
    int readRegister16(uint8_t reg, uint16_t *data); 
    int writeRegister16(uint8_t reg, uint16_t data);
    int updateRegister(uint8_t reg, uint8_t mask, uint8_t val);
    
    // Fuel Gauge helpers
    bool waitForDNR(uint16_t timeout_ms = 1000);
};
