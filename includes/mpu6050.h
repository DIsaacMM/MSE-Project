/**
 * @file MPU6050.h
 * @brief Header file for MPU6050 accelerometer/gyroscope driver.
 *
 * This header defines the I2C address, key register addresses, the data structure
 * for raw sensor readings, and function prototypes for initializing, configuring,
 * and reading data from the MPU6050.
 *
 * @authors David Mijares, Ximena Cedillo, Xavier Clemente
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include "I2C.h"
#include "GPIO.h"

/* MPU6050 I2C address */
#define MPU6050_ADDR 0x68

/* Important register addresses */
#define PWR_MGMT_1    0x6B //- Power Management Register
#define ACCEL_XOUT_H  0x3B //First accelerometer data register

/**
 * @brief Structure to hold raw accelerometer and gyroscope data.
 * @note Values are 16-bit signed integers read directly from the sensor.
 */
typedef struct
{
    int16_t ax;  ///< Accelerometer X-axis value
    int16_t ay;  ///< Accelerometer Y-axis value
    int16_t az;  ///< Accelerometer Z-axis value

    int16_t gx;  ///< Gyroscope X-axis value
    int16_t gy;  ///< Gyroscope Y-axis value
    int16_t gz;  ///< Gyroscope Z-axis value
} MPU6050_t;

/* Function prototypes */
void mpu6050_init(port_t p, uint8_t scl_pin, uint8_t sda_pin, uint8_t mode);
void mpu6050_config(uint8_t reg, uint8_t value);
void mpu6050_readData(MPU6050_t *data);

void i2c_readOneByte(uint8_t device_address,
                     uint8_t register_address,
                     uint8_t *data); 

#endif // MPU6050_H