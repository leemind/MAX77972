#include "max77972.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "MAX77972";

MAX77972::MAX77972() : _i2c(NULL), _address(MAX77972_I2C_ADDR_DEFAULT) {}

bool MAX77972::begin(TwoWire& bus, uint8_t address) {
    _i2c = &bus;
    _address = address;

    ESP_LOGI(TAG, "Initializing MAX77972 on I2C address 0x%02x", _address);

    // Testing connectivity
    _i2c->beginTransmission(_address);
    int error = _i2c->endTransmission();
    if (error != 0) {
        ESP_LOGE(TAG, "I2C connection failed with error code %d", error);
        return false;
    }

    ESP_LOGI(TAG, "MAX77972 connection successful");
    return true; // Success
}

int MAX77972::writeRegister(uint8_t reg, uint8_t data) {
    if (!_i2c) {
        ESP_LOGE(TAG, "Write failed: I2C bus not initialized");
        return -1;
    }
    
    _i2c->beginTransmission(_address);
    _i2c->write(reg);
    _i2c->write(data);
    int error = _i2c->endTransmission();
    
    if (error != 0) {
        ESP_LOGE(TAG, "Write to reg 0x%02x failed with error %d", reg, error);
        return error;
    }
    
    ESP_LOGD(TAG, "Write reg 0x%02x value 0x%02x success", reg, data);
    return 0;
}

int MAX77972::readRegister(uint8_t reg, uint8_t *data) {
    if (!_i2c || !data) {
        ESP_LOGE(TAG, "Read failed: Invalid argument or I2C not initialized");
        return -1;
    }
    
    _i2c->beginTransmission(_address);
    _i2c->write(reg);
    int error = _i2c->endTransmission(false);
    
    if (error != 0) {
        ESP_LOGE(TAG, "Read reg 0x%02x (address phase) failed with error %d", reg, error);
        return error;
    }
    
    if (_i2c->requestFrom(_address, (uint8_t)1) != 1) {
        ESP_LOGE(TAG, "Read reg 0x%02x (data phase) failed", reg);
        return -1;
    }
    
    *data = _i2c->read();
    ESP_LOGD(TAG, "Read reg 0x%02x value 0x%02x success", reg, *data);
    return 0;
}

int MAX77972::writeRegister16(uint8_t reg, uint16_t data) {
    if (!_i2c) {
        ESP_LOGE(TAG, "Write16 failed: I2C bus not initialized");
        return -1;
    }
    
    uint8_t LSB = data & 0xFF;
    uint8_t MSB = (data >> 8) & 0xFF;

    _i2c->beginTransmission(_address);
    _i2c->write(reg);
    _i2c->write(LSB);
    _i2c->write(MSB);
    int error = _i2c->endTransmission();
    
    if (error != 0) {
        ESP_LOGE(TAG, "Write16 to reg 0x%02x (0x%04x) failed with error %d", reg, data, error);
        return error;
    }
    
    ESP_LOGD(TAG, "Write16 reg 0x%02x value 0x%04x success", reg, data);
    return 0;
}

int MAX77972::readRegister16(uint8_t reg, uint16_t *data) {
    if (!_i2c || !data) {
        ESP_LOGE(TAG, "Read16 failed: Invalid argument or I2C not initialized");
        return -1;
    }
    
    _i2c->beginTransmission(_address);
    _i2c->write(reg);
    int error = _i2c->endTransmission(false);

    if (error != 0) {
        ESP_LOGE(TAG, "Read16 reg 0x%02x (address phase) failed with error %d", reg, error);
        return error;
    }

    if (_i2c->requestFrom(_address, (uint8_t)2) != 2) {
        ESP_LOGE(TAG, "Read16 reg 0x%02x (data phase) failed", reg);
        return -1;
    }
    
    uint8_t lsb = _i2c->read();
    uint8_t msb = _i2c->read();
    *data = (msb << 8) | lsb;
    
    ESP_LOGD(TAG, "Read16 reg 0x%02x value 0x%04x success", reg, *data);
    return 0;
}

int MAX77972::updateRegister(uint8_t reg, uint8_t mask, uint8_t val) {
    uint8_t curr;
    if (readRegister(reg, &curr) != 0) return -1;
    
    uint8_t next = (curr & ~mask) | (val & mask);
    ESP_LOGD(TAG, "Updating reg 0x%02x from 0x%02x to 0x%02x", reg, curr, next);
    return writeRegister(reg, next);
}

// --- Charger Control ---

int MAX77972::setFastChargeCurrent(uint16_t current_ma) {
    uint16_t reg_val = (uint16_t)((float)current_ma / 0.15625f);
    ESP_LOGI(TAG, "Setting Fast Charge Current to %d mA (reg_val: 0x%04x)", current_ma, reg_val);
    return writeRegister16(MAX77972_REG_ICHG, reg_val);
}

int MAX77972::setTopOffVoltage(uint16_t voltage_mv) {
    uint16_t reg_val = (uint16_t)((float)voltage_mv * 1000.0f / 78.125f);
    ESP_LOGI(TAG, "Setting Top-off Voltage to %d mV (reg_val: 0x%04x)", voltage_mv, reg_val);
    return writeRegister16(MAX77972_REG_VCHG, reg_val);
}

int MAX77972::enableCharger(bool enable) {
    ESP_LOGI(TAG, "%s charger", enable ? "Enabling" : "Disabling");
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
    if (readRegister16(MAX77972_REG_REP_SOC, &raw) == 0) {
        return (float)raw / 256.0f;
    }
    return -1.0f;
}

float MAX77972::getVoltage() {
    uint16_t raw = 0;
    if (readRegister16(MAX77972_REG_VCELL, &raw) == 0) {
        return (float)raw * 0.000078125f; 
    }
    return -1.0f;
}

float MAX77972::getCurrent() {
    // Current detection not yet implemented
    return 0.0f; 
}

float MAX77972::getCapacity() {
    uint16_t raw = 0;
    if (readRegister16(MAX77972_REG_REP_CAP, &raw) == 0) {
        return (float)raw * 0.5f; 
    }
    return -1.0f;
}
