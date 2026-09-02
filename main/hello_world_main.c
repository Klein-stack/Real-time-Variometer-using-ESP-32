#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "MPU6050";

#define TEST_I2C_PORT I2C_NUM_0
#define I2C_MASTER_SCL_IO GPIO_NUM_6
#define I2C_MASTER_SDA_IO GPIO_NUM_11
#define MPU6050_ADDRESS 0x68 //This is the address of MPU6050 when AD0 pin of it is connected to ground
#define PWR_MGMT_REG 0x6B //This is the address of the register whose bit 6 have to be turned 0 for turning on mpu6050
#define ACCEL_XOUT_REG 0x3B
#define ACCEL_SENSIT_PER_LSB 16384
#define ACCEL_DUE_TO_GRAVITY 9.80665

static i2c_master_dev_handle_t dev_handle; 

void mpu6050_init(){
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = TEST_I2C_PORT,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDRESS,
        .scl_speed_hz = 100000
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    uint8_t wake_up_cmd[2] = {PWR_MGMT_REG, 0x00};
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, wake_up_cmd, sizeof(wake_up_cmd), 1000));

}

double mpu_read_data(){

    uint8_t data_reg_addr = ACCEL_XOUT_REG;
    uint8_t data[6];

    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &data_reg_addr, 1 , data, 6, 1000));

    int16_t accel_x = (int16_t)((data[0] << 8) | (data[1])); //because mpu6050 produces data in big endian format (MSB first)
    int16_t accel_y = (int16_t)((data[2] << 8) | (data[3]));
    int16_t accel_z = (int16_t)((data[4] << 8) | (data[5]));

    float accel_z_in_g = accel_z/ACCEL_SENSIT_PER_LSB;
    float accel_z_in_ms2 = accel_z_in_g * ACCEL_DUE_TO_GRAVITY;
    
    return accel_z_in_ms2;
}

void calibrate_mpu(uint8_t num_of_samples){
    ESP_LOGI(TAG, "Calibraitng the acceleratomer... Place the accelerometer level.");
    float sum = 0.0;
    for (int i = 0; i < num_of_samples; i++){
        sum += mpu_read_data();
        vTaskDelay(pdMS_TO_TICKS(5));
    };

    double accel_z_bias = sum / num_of_samples; 
    return accel_z_bias;

}

