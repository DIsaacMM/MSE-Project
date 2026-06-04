/**
 * @file MPU6050.c
 * @brief Simple driver for MPU6050 accelerometer/gyroscope using I2C polling.
 *
 * This module provides basic initialization, register configuration, and data reading
 * functions for the MPU6050 sensor. It relies on the low-level I2C driver (I2C.c)
 * for all I2C transactions. No interrupts or timeouts are used (blocking mode).
 *
 * @authors David Mijares
 */

#include "MPU6050.h"   // Contains MPU6050 address, register definitions, and MPU6050_t struct

/**
 * @brief Initializes the MPU6050 sensor.
 * @details Calls i2c_init() to set up the I2C peripheral, then wakes up the MPU6050
 *          by writing 0 to its power management register (clears the SLEEP bit).
 */
void mpu6050_init(port_t p, uint8_t scl_pin, uint8_t sda_pin, uint8_t mode)
{
    gpio_initPort(p);
    // Alternate Function mode
    gpio_setPinMode(p, scl_pin, mode);
    gpio_setPinMode(p, sda_pin, mode);


    // AF4 for I2C1
    gpio_setAlternateFunction(p, scl_pin, 4);
    gpio_setAlternateFunction(p, sda_pin, 4);

    // Open-drain
    gpio[p]->OTYPER |= (1 << scl_pin);
    gpio[p]->OTYPER |= (1 << sda_pin);

    // Pull-up
    gpio[p]->PUPDR &= ~(3 << (scl_pin * 2));
    gpio[p]->PUPDR |=  (1 << (scl_pin * 2));

    gpio[p]->PUPDR &= ~(3 << (sda_pin * 2));
    gpio[p]->PUPDR |=  (1 << (sda_pin * 2));
    

    i2c_init();


    uint8_t data = 0x00;

    i2c_writeRegDevice(MPU6050_ADDR, PWR_MGMT_1, &data, 1);

}

/**
 * @brief Writes a configuration value to a specific MPU6050 register.
 * @param reg   Internal register address to write to.
 * @param value Byte value to be written.
 */
void mpu6050_config(uint8_t reg, uint8_t value)
{
    // Write a single byte 'value' to register 'reg' of the MPU6050
    i2c_writeRegDevice(MPU6050_ADDR, reg, &value, 1);
}

/**
 * @brief Reads accelerometer and gyroscope data from the MPU6050.
 * @param data Pointer to an MPU6050_t structure where the raw readings will be stored.
 * @details Performs a single I2C burst read of 14 bytes starting from ACCEL_XOUT_H (0x3B).
 *          The 16-bit values are stored in big-endian order (high byte first) and
 *          then combined into the structure's fields.
 */
void mpu6050_readData(MPU6050_t *data)
{
    // Buffer to hold 14 bytes: accelerometer (6) + temperature (2) + gyroscope (6)
    uint8_t mpu6050_data[14];

    // Read all data registers in one I2C transaction
    i2c_readRegDevice(MPU6050_ADDR, ACCEL_XOUT_H, mpu6050_data, 14);

    // Combine high and low bytes for each accelerometer axis (big-endian format)
    data->ax = (int16_t)((mpu6050_data[0] << 8) | mpu6050_data[1]);
    data->ay = (int16_t)((mpu6050_data[2] << 8) | mpu6050_data[3]);
    data->az = (int16_t)((mpu6050_data[4] << 8) | mpu6050_data[5]);

    // Combine bytes for each gyroscope axis (registers 8 to 13)
    data->gx = (int16_t)((mpu6050_data[8] << 8) | mpu6050_data[9]);
    data->gy = (int16_t)((mpu6050_data[10] << 8) | mpu6050_data[11]);
    data->gz = (int16_t)((mpu6050_data[12] << 8) | mpu6050_data[13]);
}