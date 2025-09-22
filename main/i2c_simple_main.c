#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO    18       // ESP32-S3'te I2C SCL pin numarası
#define I2C_MASTER_SDA_IO    17        // ESP32-S3'te I2C SDA pin numarası
#define I2C_MASTER_NUM       I2C_NUM_0 // I2C port numarası
#define I2C_MASTER_FREQ_HZ   100000   // I2C clock frekansı
#define I2C_MASTER_TX_BUF_DISABLE   0 // I2C TX buffer devre dışı
#define I2C_MASTER_RX_BUF_DISABLE   0 // I2C RX buffer devre dışı
#define BNO055_I2C_ADDRESS   0x28     // BNO055 I2C adresi
#define BNO055_CHIP_ID_ADDR  0x00     // Chip ID register adresi
#define BNO055_CHIP_ID_VALUE 0xA0     // BNO055 sensörünün beklenen Chip ID değeri

static const char *TAG = "BNO055";

esp_err_t i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

esp_err_t bno055_read_chip_id(uint8_t *chip_id) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BNO055_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BNO055_CHIP_ID_ADDR, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BNO055_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, chip_id, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void app_main() {
    ESP_ERROR_CHECK(i2c_master_init());

    uint8_t chip_id;
    if (bno055_read_chip_id(&chip_id) == ESP_OK) {
        ESP_LOGI(TAG, "BNO055 Chip ID: 0x%02X", chip_id);
        if (chip_id == BNO055_CHIP_ID_VALUE) {
            ESP_LOGI(TAG, "Chip ID doğrulandı, sensör çalışıyor.");
        } else {
            ESP_LOGE(TAG, "Chip ID beklenen değerden farklı: 0x%02X", chip_id);
        }
    } else {
        ESP_LOGE(TAG, "Chip ID okunamadı!");
    }
}
