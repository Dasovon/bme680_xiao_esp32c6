#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c.h>
#include "bme68x.h"
#include "esp_log.h"

#define I2C_MASTER_SDA_IO 22       // GPIO6 for SDA (XIAO ESP32C6)
#define I2C_MASTER_SCL_IO 23       // GPIO7 for SCL (XIAO ESP32C6)
#define I2C_MASTER_NUM I2C_NUM_0  // I2C port number
#define I2C_MASTER_FREQ_HZ 400000 // I2C frequency (400 kHz)
// #define BME68X_I2C_ADDR BME68X_I2C_ADDR_LOW // 0x76
#define BME68X_I2C_ADDR BME68X_I2C_ADDR_HIGH // 0x77

static const char *TAG = "BME680";

// I2C read function for BME68x API
int8_t bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME68X_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME68X_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, reg_data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, reg_data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return (ret == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}

// I2C write function for BME68x API
int8_t bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME68X_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, (uint8_t *)reg_data, len, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return (ret == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}

// Delay function for BME68x API
void bme68x_delay_us(uint32_t period, void *intf_ptr) {
    vTaskDelay(pdMS_TO_TICKS(period / 1000));
}

void i2c_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

void app_main(void) {
    struct bme68x_dev bme;
    struct bme68x_conf conf;
    struct bme68x_heatr_conf heatr_conf;
    int8_t rslt;

    // Initialize I2C
    i2c_init();

    // Initialize BME68x device
    bme.intf = BME68X_I2C_INTF;
    bme.read = bme68x_i2c_read;
    bme.write = bme68x_i2c_write;
    bme.delay_us = bme68x_delay_us;
    bme.intf_ptr = NULL; // Not used in this example
    bme.amb_temp = 25;   // Assume ambient temperature is 25°C

    rslt = bme68x_init(&bme);
    if (rslt != BME68X_OK) {
        ESP_LOGE(TAG, "BME68x initialization failed: %d", rslt);
        return;
    }

    // Configure sensor settings
    conf.os_hum = BME68X_OS_16X;
    conf.os_temp = BME68X_OS_2X;
    conf.os_pres = BME68X_OS_1X;
    conf.filter = BME68X_FILTER_OFF;
    conf.odr = BME68X_ODR_NONE;
    rslt = bme68x_set_conf(&conf, &bme);
    if (rslt != BME68X_OK) {
        ESP_LOGE(TAG, "Configuration failed: %d", rslt);
        return;
    }

    // Configure heater for gas measurements
    heatr_conf.enable = BME68X_ENABLE;
    heatr_conf.heatr_temp = 300; // 300°C
    heatr_conf.heatr_dur = 100;  // 100 ms
    rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);
    if (rslt != BME68X_OK) {
        ESP_LOGE(TAG, "Heater configuration failed: %d", rslt);
        return;
    }

    // Set operation mode to forced mode
    rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
    if (rslt != BME68X_OK) {
        ESP_LOGE(TAG, "Set operation mode failed: %d", rslt);
        return;
    }

    while (1) {
        struct bme68x_data data;
        uint8_t n_fields;

        // Trigger a measurement
        rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
        if (rslt != BME68X_OK) {
            ESP_LOGE(TAG, "Trigger measurement failed: %d", rslt);
            continue;
        }

        // Wait for measurement to complete
        uint32_t delay_us = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme) + (heatr_conf.heatr_dur * 1000);
        bme.delay_us(delay_us, bme.intf_ptr);

        // Read sensor data
        rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme);
        if (rslt == BME68X_OK && n_fields > 0) {
            ESP_LOGI(TAG, "Temperature: %.2f °C, Humidity: %.2f %%, Pressure: %.2f hPa, Gas Resistance: %.2f kOhm",
                     data.temperature, data.humidity, data.pressure / 100.0, data.gas_resistance / 1000.0);
        } else {
            ESP_LOGE(TAG, "Data read failed: %d, n_fields: %d", rslt, n_fields);
        }

        vTaskDelay(pdMS_TO_TICKS(3000)); // Wait 3 seconds
    }
}