// Include standard I/O functions like printf()
#include <stdio.h>

// Include FreeRTOS core definitions
#include <freertos/FreeRTOS.h>

// Include FreeRTOS task management (for creating delays and tasks)
#include <freertos/task.h>

// Include I2C driver support for communicating with I2C devices like BME680
#include <driver/i2c.h>

// Include the BME68x sensor API from Bosch
#include "bme68x.h"

// ESP-IDF logging facility to print logs to the console (for debugging)
#include "esp_log.h"

// Define GPIO pins used for I2C communication
#define I2C_MASTER_SDA_IO 22       // SDA (Data line) connected to GPIO22 on XIAO ESP32C6
#define I2C_MASTER_SCL_IO 23       // SCL (Clock line) connected to GPIO23 on XIAO ESP32C6
#define I2C_MASTER_NUM I2C_NUM_0   // Use I2C controller 0 (ESP32 has two I2C controllers: 0 and 1)
#define I2C_MASTER_FREQ_HZ 400000  // Set I2C clock speed to 400kHz (Fast Mode)

// Define the I2C address of the BME68x sensor
// The BME68x sensor has two possible I2C addresses: 0x76 or 0x77
// Here, we use 0x77 (HIGH)
#define BME68X_I2C_ADDR BME68X_I2C_ADDR_HIGH

// Define a tag used for logging
static const char *TAG = "BME680";

// I2C read function to read data from BME68x registers
int8_t bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    // Create an I2C command link (sequence of operations)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Start the I2C communication
    i2c_master_start(cmd);

    // Send the sensor address with write bit (0)
    i2c_master_write_byte(cmd, (BME68X_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);

    // Send the register address we want to read from
    i2c_master_write_byte(cmd, reg_addr, true);

    // Send a repeated start to switch to read mode
    i2c_master_start(cmd);

    // Send the sensor address with read bit (1)
    i2c_master_write_byte(cmd, (BME68X_I2C_ADDR << 1) | I2C_MASTER_READ, true);

    // Read multiple bytes if needed
    if (len > 1) {
        i2c_master_read(cmd, reg_data, len - 1, I2C_MASTER_ACK); // Acknowledge each byte except the last
    }

    // Read the last byte with NACK to end the read
    i2c_master_read_byte(cmd, reg_data + len - 1, I2C_MASTER_NACK);

    // Send stop condition
    i2c_master_stop(cmd);

    // Execute the command link and wait for it to complete
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));

    // Delete the command link to free memory
    i2c_cmd_link_delete(cmd);

    // Return success or communication failure code
    return (ret == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}

// I2C write function to write data to BME68x registers
int8_t bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    // Create an I2C command link
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Start communication
    i2c_master_start(cmd);

    // Send device address with write bit
    i2c_master_write_byte(cmd, (BME68X_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);

    // Send register address to write to
    i2c_master_write_byte(cmd, reg_addr, true);

    // Write data to the register
    i2c_master_write(cmd, (uint8_t *)reg_data, len, true);

    // Stop communication
    i2c_master_stop(cmd);

    // Execute the command
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));

    // Delete the command link
    i2c_cmd_link_delete(cmd);

    // Return status
    return (ret == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}

// Delay function needed by the BME68x API (expects microsecond delays)
// Here, we convert microseconds to milliseconds and use vTaskDelay for timing
void bme68x_delay_us(uint32_t period, void *intf_ptr) {
    vTaskDelay(pdMS_TO_TICKS(period / 1000));
}

// Function to initialize the I2C bus with specified settings
void i2c_init(void) {
    // Configure I2C settings
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,              // We are using the ESP32 as an I2C master
        .sda_io_num = I2C_MASTER_SDA_IO,      // Assign SDA GPIO pin
        .scl_io_num = I2C_MASTER_SCL_IO,      // Assign SCL GPIO pin
        .sda_pullup_en = GPIO_PULLUP_ENABLE,  // Enable internal pull-up resistor for SDA
        .scl_pullup_en = GPIO_PULLUP_ENABLE,  // Enable internal pull-up resistor for SCL
        .master.clk_speed = I2C_MASTER_FREQ_HZ // Set I2C frequency
    };

    // Apply the I2C configuration to the specified port
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));

    // Install the I2C driver to enable communication
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

// Main application entry point
void app_main(void) {
    struct bme68x_dev bme;               // BME68x device structure
    struct bme68x_conf conf;            // Configuration structure for oversampling/filter
    struct bme68x_heatr_conf heatr_conf; // Heater configuration
    int8_t rslt;                         // Result variable to store return status

    // Initialize I2C communication
    i2c_init();

    // Assign required I2C functions and settings to the device structure
    bme.intf = BME68X_I2C_INTF;            // Use I2C interface
    bme.read = bme68x_i2c_read;            // Assign read function
    bme.write = bme68x_i2c_write;          // Assign write function
    bme.delay_us = bme68x_delay_us;        // Assign delay function
    bme.intf_ptr = NULL;                   // Not used in this example
    bme.amb_temp = 25;                     // Estimated ambient temperature (in °C)

    // Initialize the BME68x sensor
    rslt = bme68x_init(&bme);
    if (rslt != BME68X_OK) {
        ESP_LOGE(TAG, "BME68x initialization failed: %d", rslt);
        return;
    }

    // Set oversampling and filter settings
    conf.os_hum = BME68X_OS_16X;        // Humidity oversampling x16
    conf.os_temp = BME68X_OS_2X;        // Temperature oversampling x2
    conf.os_pres = BME68X_OS_1X;        // Pressure oversampling x1
    conf.filter = BME68X_FILTER_OFF;    // No filter
    conf.odr = BME68X_ODR_NONE;         // No output data rate (manual measurement)

    // Apply the sensor configuration
    rslt = bme68x_set_conf(&conf, &bme);
    if (rslt != BME68X_OK) {
        ESP_LOGE(TAG, "Configuration failed: %d", rslt);
        return;
    }

    // Configure the gas heater
    heatr_conf.enable = BME68X_ENABLE;  // Enable the heater
    heatr_conf.heatr_temp = 300;        // Target temperature: 300°C
    heatr_conf.heatr_dur = 100;         // Heating duration: 100 ms

    // Apply heater configuration for forced mode
    rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);
    if (rslt != BME68X_OK) {
        ESP_LOGE(TAG, "Heater configuration failed: %d", rslt);
        return;
    }

    // Set sensor to forced mode (one-shot measurement each time)
    rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
    if (rslt != BME68X_OK) {
        ESP_LOGE(TAG, "Set operation mode failed: %d", rslt);
        return;
    }

    // Infinite loop to repeatedly take sensor measurements
    while (1) {
        struct bme68x_data data;    // Variable to hold sensor readings
        uint8_t n_fields;           // Number of data fields returned

        // Trigger a new measurement
        rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
        if (rslt != BME68X_OK) {
            ESP_LOGE(TAG, "Trigger measurement failed: %d", rslt);
            continue;
        }

        // Calculate how long to wait before data is ready (measurement + heating time)
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

        // Wait for 3 seconds before the next reading
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
