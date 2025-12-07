#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "max77972.h"
#include "esp_log.h"

static const char *TAG = "EXAMPLE";

// I2C Configuration
#define I2C_MASTER_SCL_IO           22      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0       /*!< I2C master i2c port number */

extern "C" void app_main(void)
{
    MAX77972 charger;

    ESP_LOGI(TAG, "Initializing MAX77972...");
    esp_err_t ret = charger.begin(I2C_NUM_0, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Initialization successful");
    } else {
        ESP_LOGE(TAG, "Initialization failed");
        return;
    }

    // Configure Charger (Example values)
    charger.setFastChargeCurrent(1500); // 1.5A
    charger.setTopOffVoltage(4200);     // 4.2V
    
    // Enable Charger
    charger.enableCharger(true);

    while (1) {
        float soc = charger.getSoC();
        float voltage = charger.getVoltage();
        float capacity = charger.getCapacity();
        uint8_t status = charger.getChargerStatus();

        ESP_LOGI(TAG, "SoC: %.2f %% | Volts: %.4f V | Cap: %.2f mAh | Status: 0x%02x", 
                 soc, voltage, capacity, status);

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
