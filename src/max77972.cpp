#include "max77972.h"
#include "esp_log.h"
#include <cstdint>
#include <math.h>

static const char *TAG = "MAX77972";

MAX77972::MAX77972() : _i2c(NULL), _address(MAX77972_I2C_ADDR_DEFAULT) {}

bool MAX77972::begin(TwoWire &bus, uint8_t address) {
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

  ESP_LOGI(TAG, "I2C connection successful. Waiting for device to be ready...");

  // 1. Wait for DNR (Data Not Ready) to clear
  if (!waitForDNR(1000)) {
    ESP_LOGW(TAG, "Device took too long to clear DNR bit. Proceeding anyway...");
  }

  // 2. Check for POR (Power-On Reset)
  uint16_t status = 0;
  if (readRegister16(MAX77972_REG_STATUS, &status) == 0) {
    if (status & MAX77972_STATUS_POR) {
      ESP_LOGI(TAG, "Power-On Reset detected. Clearing POR bit...");

      // In a full implementation, we would load the EZ Config model here.
      // For now, we clear the POR bit to allow the algorithm to run.
      updateRegister(MAX77972_REG_STATUS, MAX77972_STATUS_POR, 0);

      // Wait for algorithm to settle
      delay(10);
    }
  } else {
    ESP_LOGE(TAG, "Failed to read Status register.");
    return false;
  }

  ESP_LOGI(TAG, "MAX77972 initialization complete.");
  return true; // Success
}

bool MAX77972::waitForDNR(uint16_t timeout_ms) {
  uint32_t startTime = millis();
  uint16_t fstat = 0;

  while (millis() - startTime < timeout_ms) {
    if (readRegister16(MAX77972_REG_FSTAT, &fstat) == 0) {
      if (!(fstat & MAX77972_FSTAT_DNR)) {
        ESP_LOGI(TAG, "FStat.DNR cleared after %d ms", (int)(millis() - startTime));
        return true;
      }
    }
    delay(10);
  }

  ESP_LOGE(TAG, "Timeout waiting for FStat.DNR to clear (FStat: 0x%04x)", fstat);
  return false;
}

int MAX77972::writeRegister(uint16_t reg, uint8_t data) {
  if (!_i2c) {
    ESP_LOGE(TAG, "Write failed: I2C bus not initialized");
    return -1;
  }

  uint8_t addr_offset = (reg >> 8) & 0xFF;
  uint8_t reg_addr = reg & 0xFF;

  _i2c->beginTransmission(_address + addr_offset);
  _i2c->write(reg_addr);
  _i2c->write(data);
  int error = _i2c->endTransmission();

  if (error != 0) {
    ESP_LOGE(TAG, "Write to reg 0x%04x failed with error %d", reg, error);
    return error;
  }

  return 0;
}

int MAX77972::readRegister(uint16_t reg, uint8_t *data) {
  if (!_i2c || !data) {
    ESP_LOGE(TAG, "Read failed: Invalid argument or I2C not initialized");
    return -1;
  }

  uint8_t addr_offset = (reg >> 8) & 0xFF;
  uint8_t reg_addr = reg & 0xFF;

  _i2c->beginTransmission(_address + addr_offset);
  _i2c->write(reg_addr);
  int error = _i2c->endTransmission(false);

  if (error != 0) {
    ESP_LOGE(TAG, "Read reg 0x%04x (address phase) failed with error %d", reg, error);
    return error;
  }

  if (_i2c->requestFrom((uint8_t)(_address + addr_offset), (uint8_t)1) != 1) {
    ESP_LOGE(TAG, "Read reg 0x%04x (data phase) failed", reg);
    return -1;
  }

  *data = _i2c->read();
  return 0;
}

int MAX77972::writeRegister16(uint16_t reg, uint16_t data) {
  if (!_i2c) {
    ESP_LOGE(TAG, "Write16 failed: I2C bus not initialized");
    return -1;
  }

  uint8_t addr_offset = (reg >> 8) & 0xFF;
  uint8_t reg_addr = reg & 0xFF;

  uint8_t LSB = data & 0xFF;
  uint8_t MSB = (data >> 8) & 0xFF;

  _i2c->beginTransmission(_address + addr_offset);
  _i2c->write(reg_addr);
  _i2c->write(LSB);
  _i2c->write(MSB);
  int error = _i2c->endTransmission();

  if (error != 0) {
    ESP_LOGE(TAG, "Write16 to reg 0x%04x (0x%04x) failed with error %d", reg, data, error);
    return error;
  }

  return 0;
}

int MAX77972::readRegister16(uint16_t reg, uint16_t *data) {
  if (!_i2c || !data) {
    ESP_LOGE(TAG, "Read16 failed: Invalid argument or I2C not initialized");
    return -1;
  }

  uint8_t addr_offset = (reg >> 8) & 0xFF;
  uint8_t reg_addr = reg & 0xFF;

  _i2c->beginTransmission(_address + addr_offset);
  _i2c->write(reg_addr);
  int error = _i2c->endTransmission(false);

  if (error != 0) {
    ESP_LOGE(TAG, "Read16 reg 0x%04x (address phase) failed with error %d", reg, error);
    return error;
  }

  if (_i2c->requestFrom((uint8_t)(_address + addr_offset), (uint8_t)2) != 2) {
    ESP_LOGE(TAG, "Read16 reg 0x%04x (data phase) failed", reg);
    return -1;
  }

  uint8_t lsb = _i2c->read();
  uint8_t msb = _i2c->read();
  *data = (msb << 8) | lsb;

  return 0;
}

int MAX77972::updateRegister(uint16_t reg, uint8_t mask, uint8_t val) {
  uint8_t curr;
  if (readRegister(reg, &curr) != 0)
    return -1;

  uint8_t next = (curr & ~mask) | (val & mask);
  ESP_LOGI(TAG, "Updating register 0x%04x from 0x%02x to 0x%02x with mask 0x%02x", reg, curr, next, mask);
  return writeRegister(reg, next);
}

int MAX77972::updateRegister16(uint16_t reg, uint16_t mask, uint16_t val) {
  uint16_t curr;
  if (readRegister16(reg, &curr) != 0)
    return -1;

  uint16_t next = (curr & ~mask) | (val & mask);
  ESP_LOGI(TAG, "Updating register 0x%04x from 0x%04x to 0x%04x with mask 0x%04x", reg, curr, next, mask);
  return writeRegister16(reg, next);
}

// --- Charger Control ---

// I don't think this is correct - I think you need to set via nIChgCfg1 (0x1CE)
int MAX77972::setFastChargeCurrent(uint16_t current_ma) {
  uint16_t reg_val = (uint16_t)((float)current_ma / 0.15625f);
  ESP_LOGI(TAG, "Setting Fast Charge Current to %d mA (reg_val: 0x%04x)", current_ma, reg_val);
  return writeRegister16(MAX77972_REG_ICHG, reg_val);
}

int MAX77972::getChargingCurrent() {
  uint16_t raw = 0;
  if (readRegister16(MAX77972_REG_ICHG, &raw) == 0) {
    // ESP_LOGI(TAG, "getChargingCurrent raw: 0x%04x", raw);
    return (int)(raw * 0.15625f);
  }
  return -1;
}

int MAX77972::setRoomChargeVoltage(int voltage_mv) {
  uint16_t raw;
  // Read the current configuration from I2C bus + 1, register decomposes 0x1CC
  // to offset 1, reg 0xCC automatically
  if (readRegister16(MAX77972_REG_NVCHG_CFG_1, &raw) != 0) {
    ESP_LOGE(TAG, "Failed to read NVCHG_CFG_1");
    return -1;
  }
  ESP_LOGI(TAG, "Read NVCHG_CFG_1: 0x%04x", raw);

  // Calculate RoomChargeVolt based on: VCHG[Step4][Room] = 3.4V +
  // (RoomChargeVolt * 10mV)
  uint8_t room_cv = (voltage_mv - 3400) / 10;
  if (room_cv < 0)
    room_cv = 0;
  if (room_cv > 255)
    room_cv = 255;

  // WarmChargeVolt is bits 15:12, RoomChargeVolt is bits 11:4, CoolChargeVolt
  // is bits 3:0 Mask out bits 11:4 (0x0FF0) and insert new room_cv
  raw = (raw & 0xF00F) | (room_cv << 4);

  // raw = 0x55A5;

  ESP_LOGI(TAG,
      "Setting Room Charge Voltage to %d mV (RoomChargeVolt: %d, "
      "NVCHG_CFG_1: 0x%04x)",
      voltage_mv, room_cv, raw);

  // Write the updated configuration back to I2C bus + 1
  return writeRegister16(MAX77972_REG_NVCHG_CFG_1, raw);
}

int MAX77972::setRoomChargingCurrent(int charge_ma) {
  uint16_t raw;
  if (readRegister16(MAX77972_REG_NICHG_CFG_1, &raw) != 0) {
    ESP_LOGE(TAG, "Failed to read NICHG_CFG_1");
    return -1;
  }
  ESP_LOGI(TAG, "Read NICHG_CFG_1: 0x%04x", raw);

  uint16_t warm_ma = (0x07 << 11);
  uint16_t room_ma = ((charge_ma - 1) / 50) << 5;
  uint16_t cool_ma = 0x10;

  raw = warm_ma | room_ma | cool_ma;

  // raw = 0b0010001111101000;

  ESP_LOGI(TAG, "Setting Room Charge Current to %d mA, NICHG_CFG_1: 0x%04x", charge_ma, raw);
  ESP_LOGI(TAG, "NICHG_CFG_1: " BYTE_TO_BINARY_PATTERN " " BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(raw >> 8),
      BYTE_TO_BINARY(raw & 0xFF));

  // Write the updated configuration back to I2C bus + 1
  return writeRegister16(MAX77972_REG_NICHG_CFG_1, raw);
}

// Again set this via room charging volrage, not here
int MAX77972::setChargingVoltage(int voltage_mv) {
  uint16_t reg_val;
  if (voltage_mv <= 3400) {
    reg_val = 0x0000;
  } else if (voltage_mv <= 3500) {
    reg_val = 0xAF00;
  } else if (voltage_mv <= 3550) {
    reg_val = 0xB180;
  } else if (voltage_mv == 4050) {
    reg_val = 0xCAB0;
  } else if (voltage_mv >= 4640) {
    reg_val = 0xE801;
  } else {
    // Use the straight formula for everything else (3.6V-4.0V, 4.1V-4.63V)
    reg_val = (uint16_t)(voltage_mv / 0.078125f);
  }

  ESP_LOGI(TAG, "Setting Charging Voltage to %d mV (reg_val: 0x%04x)", voltage_mv, reg_val);
  return writeRegister16(MAX77972_REG_VCHG, reg_val);
}

int MAX77972::getChargingVoltage() {
  uint16_t raw = 0;
  if (readRegister16(MAX77972_REG_VCHG, &raw) == 0) {
    // ESP_LOGI(TAG, "getChargingVoltage raw: 0x%04x", raw);
    if (raw <= 0xAEFF) {
      return 3400;
    } else if (raw >= 0xAF00 && raw <= 0xB17F) {
      return 3500;
    } else if (raw >= 0xB180 && raw <= 0xB3FF) {
      return 3550;
    } else if (raw >= 0xB400 && raw <= 0xCAAF) {
      float mv = raw * 0.078125f;
      return ((int)mv / 100) * 100;
    } else if (raw >= 0xCAB0 && raw <= 0xCCFF) {
      return 4050;
    } else if (raw >= 0xCD00 && raw <= 0xE800) {
      float mv = raw * 0.078125f;
      return ((int)mv / 10) * 10;
    } else {
      return 4640;
    }
  }
  return -1;
}

int MAX77972::enableCharger(bool enable) {
  ESP_LOGI(TAG, "%s charger", enable ? "Enabling" : "Disabling");
  return updateRegister(MAX77972_REG_NCHG_CFG_5, MAX77972_CFG5_CHG_EN_MASK, enable ? MAX77972_CFG5_CHG_EN_MASK : 0);
}

int MAX77972::enableUSB_BC_Detection(bool enable) {
  ESP_LOGI(TAG, "%s USB BC Detection", enable ? "Enabling" : "Disabling");
  return updateRegister16(MAX77972_REG_NCHG_CFG_4, MAX77972_CHGDETEN_MASK, enable ? MAX77972_CHGDETEN_MASK : 0);
}

int MAX77972::enableUSB_CC_Detection(bool enable) {
  ESP_LOGI(TAG, "%s USB CC Detection", enable ? "Enabling" : "Disabling");
  return updateRegister16(MAX77972_REG_NCHG_CFG_5, MAX77972_CCDETEN_MASK, enable ? MAX77972_CCDETEN_MASK : 0);
}

int MAX77972::enableAutoIset(bool enable) {
  ESP_LOGI(TAG, "%s Auto-Iset", enable ? "Enabling" : "Disabling");
  return updateRegister16(MAX77972_REG_NCHG_CFG_4, MAX77972_NO_AUTOISET, enable ? 0 : MAX77972_NO_AUTOISET);
}

int MAX77972::setSDPMaxCurr(uint8_t value) {
  uint16_t current = (value << 13);
  ESP_LOGI(TAG, "Setting SDP Max Curr reg_val: 0x%04x", current);
  return updateRegister16(MAX77972_REG_NCHG_CFG_4, MAX77972_SDP_MAX_CURR_MASK, current);
}

int MAX77972::setCDPMaxCurr(uint8_t value) {
  uint16_t current = (value << 12);
  ESP_LOGI(TAG, "Setting CDP Max Curr reg_val: 0x%04x", current);
  return updateRegister16(MAX77972_REG_NCHG_CFG_4, MAX77972_CDP_MAX_CURR_MASK, current);
}

int MAX77972::setCHGIN_ILIM(uint16_t current_ma) {
  // 0x0 to 0x03 is 100mA
  // 0x04 to 0x7F is (CHGIN_ILIM + 1) * 25mA
  uint16_t value = 0;
  if (current_ma <= 3200) {
    value = (current_ma / 25) - 1;
  }
  ESP_LOGI(TAG, "Setting CHGIN_ILIM reg_val: 0x%04x", value);
  return updateRegister16(MAX77972_REG_NCHG_CFG_3, MAX77972_CHGIN_ILIM_MASK, value);
}

uint8_t MAX77972::getChargerStatus() {
  uint8_t status = 0;
  readRegister(MAX77972_REG_STATUS, &status);
  return status;
}

uint16_t MAX77972::getChargeDetails00() {
  uint16_t details = 0;
  readRegister16(MAX77972_REG_CHG_DETAILS_00, &details);
  return details;
}

uint16_t MAX77972::getChargeDetails01() {
  uint16_t details = 0;
  readRegister16(MAX77972_REG_CHG_DETAILS_01, &details);
  return details;
}

void MAX77972::interpretChargeDetails00(uint16_t details) {
  ESP_LOGI(TAG, "--- Charge Details 00 ---");
  ESP_LOGI(TAG, "AICL_OK    : %d", (details >> 15) & 1);
  ESP_LOGI(TAG, "CHGIN_OK   : %d", (details >> 14) & 1);
  ESP_LOGI(TAG, "CHG_OK     : %d", (details >> 12) & 1);
  ESP_LOGI(TAG, "BAT_OK     : %d", (details >> 11) & 1);
  ESP_LOGI(TAG, "BYP_OK     : %d", (details >> 8) & 1);
  ESP_LOGI(TAG, "CHGIN_DTLS : %d", (details >> 5) & 3); // 2 bits
  ESP_LOGI(TAG, "CHGEN      : %d", (details >> 0) & 1);
}

void MAX77972::interpretChargeDetails01(uint16_t details) {
  ESP_LOGI(TAG, "--- Charge Details 01 ---");
  ESP_LOGI(TAG, "TREG       : %d", (details >> 15) & 1);
  ESP_LOGI(TAG, "BAT_DTLS   : %d", (details >> 14) & 7);  // 3 bits
  ESP_LOGI(TAG, "CHG_DTLS   : %d", (details >> 11) & 15); // 4 bits
  ESP_LOGI(TAG, "BAT_DIS_OC : %d", (details >> 7) & 1);   // 1 bit
  ESP_LOGI(TAG, "BYP_DTLS   : %d", (details >> 3) & 15);  // 4 bits
}

// --- Fuel Gauge ---

float MAX77972::getSoC() {
  uint16_t raw = 0;
  if (readRegister16(MAX77972_REG_REP_SOC, &raw) == 0) {
    // ESP_LOGI(TAG, "getSoC raw: 0x%04x", raw);
    return (float)raw / 256.0f;
  }
  return -1.0f;
}

int MAX77972::getVoltage() {
  uint16_t raw = 0;
  if (readRegister16(MAX77972_REG_VCELL, &raw) == 0) {
    // ESP_LOGI(TAG, "getVoltage raw: 0x%04x", raw);
    return (int)(raw * 0.078125f);
  }
  return -1;
}

int MAX77972::getCurrent() {
  uint16_t raw = 0;
  if (readRegister16(MAX77972_REG_IIN, &raw) == 0) {
    // ESP_LOGI(TAG, "getCurrent raw: 0x%04x", raw);
    return (int)((int16_t)raw * 0.15625f);
  }
  return -1;
}

float MAX77972::getCapacity() {
  uint16_t raw = 0;
  if (readRegister16(MAX77972_REG_REP_CAP, &raw) == 0) {
    return (float)raw * 0.5f;
  }
  return -1.0f;
}

int MAX77972::getDesignCapacity() {
  uint16_t raw = 0;
  if (readRegister16(MAX77972_REG_DESIGN_CAPACITY, &raw) == 0) {
    // ESP_LOGI(TAG, "getDesignCapacity raw: 0x%04x", raw);
    return (int)(raw * 0.5f);
  }
  return -1;
}

int MAX77972::setDesignCapacity(uint16_t capacity_mah) {
  uint16_t reg_val = (uint16_t)(capacity_mah * 2);
  ESP_LOGI(TAG, "Setting Design Capacity to %d mAh (reg_val: 0x%04x)", capacity_mah, reg_val);
  return writeRegister16(MAX77972_REG_DESIGN_CAPACITY, reg_val);
}

float MAX77972::getTemperature() {
  uint16_t raw = 0;
  if (readRegister16(MAX77972_REG_TEMP, &raw) == 0) {
    // ESP_LOGI(TAG, "getTemperature raw: 0x%04x", raw);
    return (float)((int16_t)raw) / 256.0f;
  }
  return -1.0f;
}

int MAX77972::setFactoryShipMode() {
  return updateRegister16(MAX77972_REG_NCHG_CFG_2,MAX77972_FSHIP_MODE_MASK,MAX77972_FSHIP_MODE_MASK);
}

int MAX77972::setDeepShipMode() {
    return updateRegister16(MAX77972_REG_NCHG_CFG_5,MAX77972_DEEPSHIP_MODE_MASK,MAX77972_DEEPSHIP_MODE_MASK);

}
