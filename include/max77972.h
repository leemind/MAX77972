#pragma once

#include "driver/i2c.h"
#include "esp_err.h"

// I2C Address (7-bit)
#define MAX77972_I2C_ADDR_DEFAULT 0x36

// --- Register Map ---

// Fuel Gauge / Status / Monitoring
#define MAX77972_REG_STATUS         0x00
#define MAX77972_REG_CHG_MASK_STS   0x01
#define MAX77972_REG_REP_CAP        0x06 // Capacity (mAh LSB=0.5)
#define MAX77972_REG_REP_SOC        0x07 // SOC (% LSB=1/256)
#define MAX77972_REG_AVG_VCELL      0x19 // Avg Voltage
#define MAX77972_REG_VCELL          0x1A // Battery Voltage (LSB=78.125uV)
#define MAX77972_REG_TEMP           0x1B // Temperature

// Smart Power Selector / Charger Config
#define MAX77972_REG_ICHG           0x28 // ChargingCurrent (LSB=0.15625mA)
#define MAX77972_REG_VCHG           0x2A // ChargingVoltage (LSB=78.125uV approx, 10mV steps logic inside)

// Charger Configuration Registers (0xD0 - 0xD5 ...)
#define MAX77972_REG_NCHG_CFG_0     0xD0 // Mode, WDTEN
#define MAX77972_REG_NCHG_CFG_3     0xD3 // CHGIN_ILIM
#define MAX77972_REG_NCHG_CFG_4     0xD4
#define MAX77972_REG_NCHG_CFG_5     0xD5 // ChgEnable

// Masks
#define MAX77972_CFG5_CHG_EN_MASK   (1 << 1)

class MAX77972 {
public:
    MAX77972();

    /**
     * @brief Initialize the driver
     * @param port I2C port number
     * @param sda_pin SDA GPIO
     * @param scl_pin SCL GPIO
     * @param address I2C address (7-bit)
     * @return ESP_OK on success
     */
    esp_err_t begin(i2c_port_t port, int sda_pin, int scl_pin, uint8_t address = MAX77972_I2C_ADDR_DEFAULT);

    // --- Charger Control ---
    
    /**
     * @brief Set Fast Charge Current
     * @param current_ma Current in mA (Valid range: 100mA - 3150mA, Step: ~0.156mA)
     * @return ESP_OK on success
     */
    esp_err_t setFastChargeCurrent(uint16_t current_ma);

    /**
     * @brief Set Top-off Voltage (Termination Voltage)
     * @param voltage_mv Voltage in mV (Typical: 4200mV for 4.2V, Step: ~0.078mV internal but ~10mV effective)
     * @return ESP_OK on success
     */
    esp_err_t setTopOffVoltage(uint16_t voltage_mv);

    /**
     * @brief Enable or Disable Charger
     * @param enable true to enable, false to disable
     * @return ESP_OK on success
     */
    esp_err_t enableCharger(bool enable);

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
     * @return Voltage in Volts
     */
    float getVoltage();

    /**
     * @brief Get Battery Current
     * @return Current in mA (Calculated/Estimated or from register if available - not implemented yet)
     */
    float getCurrent();

    /**
     * @brief Get Remaining Capacity
     * @return Capacity in mAh
     */
    float getCapacity();

private:
    i2c_port_t _i2c_port;
    uint8_t _address;

    esp_err_t writeRegister(uint8_t reg, uint8_t data);
    esp_err_t readRegister(uint8_t reg, uint8_t *data);
    esp_err_t readRegister16(uint8_t reg, uint16_t *data); 
    esp_err_t writeRegister16(uint8_t reg, uint16_t data);
    esp_err_t updateRegister(uint8_t reg, uint8_t mask, uint8_t val);
};
