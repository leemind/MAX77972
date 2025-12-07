#include "max77972.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "MAX77972";

MAX77972::MAX77972() : _i2c_port(I2C_NUM_0), _address(MAX77972_I2C_ADDR_DEFAULT) {}

esp_err_t MAX77972::begin(i2c_port_t port, int sda_pin, int scl_pin, uint8_t address) {
    _i2c_port = port;
    _address = address;

    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = scl_pin;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000; // 100kHz standard
    conf.clk_flags = 0;

    esp_err_t err = i2c_param_config(port, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(port, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "MAX77972 initialized on port %d, address 0x%02x", port, address);
    return ESP_OK;
}

esp_err_t MAX77972::writeRegister(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(_i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t MAX77972::readRegister(uint8_t reg, uint8_t *data) {
    if (!data) return ESP_ERR_INVALID_ARG;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd); // Repeated start
    i2c_master_write_byte(cmd, (_address << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(_i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t MAX77972::writeRegister16(uint8_t reg, uint16_t data) {
    // ModelGauge / MAX77972 usually writes LSB then MSB.
    uint8_t LSB = data & 0xFF;
    uint8_t MSB = (data >> 8) & 0xFF;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, LSB, true);
    i2c_master_write_byte(cmd, MSB, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(_i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t MAX77972::readRegister16(uint8_t reg, uint16_t *data) {
    if (!data) return ESP_ERR_INVALID_ARG;

    uint8_t msb, lsb;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_address << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &lsb, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &msb, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(_i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        *data = (msb << 8) | lsb;
    }
    return ret;
}

esp_err_t MAX77972::updateRegister(uint8_t reg, uint8_t mask, uint8_t val) {
    uint8_t curr;
    esp_err_t err = readRegister(reg, &curr);
    if (err != ESP_OK) return err;
    
    uint8_t next = (curr & ~mask) | (val & mask);
    return writeRegister(reg, next);
}

// --- Charger Control ---

esp_err_t MAX77972::setFastChargeCurrent(uint16_t current_ma) {
    // LSB = 0.15625 mA
    // Reg Val = current_ma / 0.15625
    uint16_t reg_val = (uint16_t)((float)current_ma / 0.15625f);
    ESP_LOGI(TAG, "Setting Fast Charge Current: %d mA (Reg: 0x%04x)", current_ma, reg_val);
    return writeRegister16(MAX77972_REG_ICHG, reg_val);
}

esp_err_t MAX77972::setTopOffVoltage(uint16_t voltage_mv) {
    // LSB = 0.078125 mV -> 78.125 uV
    // Reg Val = voltage_mv * 1000 / 78.125
    uint16_t reg_val = (uint16_t)((float)voltage_mv * 1000.0f / 78.125f);
    ESP_LOGI(TAG, "Setting Charge Voltage: %d mV (Reg: 0x%04x)", voltage_mv, reg_val);
    return writeRegister16(MAX77972_REG_VCHG, reg_val);
}

esp_err_t MAX77972::enableCharger(bool enable) {
    ESP_LOGI(TAG, "%s Charger", enable ? "Enabling" : "Disabling");
    // ChgEnable is Bit 1 of nChgConfig5 (0xD5)
    return updateRegister(MAX77972_REG_NCHG_CFG_5, 
                          MAX77972_CFG5_CHG_EN_MASK, 
                          enable ? MAX77972_CFG5_CHG_EN_MASK : 0);
}

uint8_t MAX77972::getChargerStatus() {
    uint8_t status = 0;
    readRegister(MAX77972_REG_STATUS, &status);
    return status;
}

// --- Fuel Gauge ---

float MAX77972::getSoC() {
    uint16_t raw = 0;
    if (readRegister16(MAX77972_REG_REP_SOC, &raw) == ESP_OK) {
        // LSB = 1/256 %
        return (float)raw / 256.0f;
    }
    return -1.0f;
}

float MAX77972::getVoltage() {
    uint16_t raw = 0;
    if (readRegister16(MAX77972_REG_VCELL, &raw) == ESP_OK) {
        // LSB = 78.125 uV
        return (float)raw * 0.000078125f; 
    }
    return -1.0f;
}

float MAX77972::getCurrent() {
    // There is no trivial "Instant Current" register in the map I extracted?
    // Usually ModelGauge has "Current" at 0x0A?
    // I saw "MaxMinCurr" at 0x0A.
    // If "Current" exists it's usually 0x0A or 0x04?
    // For now returning 0.0f as placeholder.
    return 0.0f; 
}

float MAX77972::getCapacity() {
    uint16_t raw = 0;
    if (readRegister16(MAX77972_REG_REP_CAP, &raw) == ESP_OK) {
        // LSB = 0.5 mAh
        return (float)raw * 0.5f; 
    }
    return -1.0f;
}
