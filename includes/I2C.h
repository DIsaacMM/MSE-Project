/**
 * @file I2C.h
 * @brief I2C peripheral driver for STM32F411RE (Master mode, polling)
 *
 * This header provides function declarations for initializing the I2C1 peripheral
 * and performing read/write operations with or without register addressing.
 *
 * @authors David Mijares, Ximena Cedillo, Xavier Clemente
 */

#ifndef I2C_H   // If I2C_H is not defined
#define I2C_H   // define

#include <stdint.h>
#include <stm32f4xx.h>   // For STM32F4 register definitions (I2C, RCC, etc.)

/**
 * @def SLC_FREQ
 * @brief Desired I2C clock frequency in Hz.
 *
 * This macro must be defined before including I2C.h, or in your compiler settings.
 * Typical values: 100000 (100 kHz, standard mode) or 400000 (400 kHz, fast mode).
 *
 * Example: #define SLC_FREQ 100000
 */

#define SLC_FREQ 400000    // fast mode for i2c (400 kHz) 


/**
 * @brief Calculate the CCR (Clock Control Register) value for a desired SCL frequency.
 * @param scl_freq_hz Desired SCL clock frequency in Hz.
 * @return Value to be written to the I2Cx->CCR register.
 */
uint32_t i2c_calc_ccr(uint32_t scl_freq_hz);

/**
 * @brief Calculate the TRISE (Rise Time Register) value for a desired SCL frequency.
 * @param scl_freq_hz Desired SCL clock frequency in Hz.
 * @return Value to be written to the I2Cx->TRISE register.
 */
uint32_t i2c_trise(uint32_t scl_freq_hz);

/**
 * @brief Initialize the I2C1 peripheral with default settings.
 *
 * Enables clock, resets control registers, sets CCR and TRISE based on SLC_FREQ,
 * disables all interrupts, and enables the I2C peripheral.
 */
void i2c_init(void);

/**
 * @brief Write multiple bytes to a specific register of an I2C device.
 * @param device_address   7-bit I2C device address.
 * @param register_address Internal register address on the device.
 * @param data             Pointer to the data bytes to be written.
 * @param len              Number of bytes to write.
 *
 * @note Blocking function, no timeouts.
 */
void i2c_writeRegDevice(uint8_t device_address, uint8_t register_address, uint8_t *data, uint32_t len);

/**
 * @brief Write multiple bytes directly to an I2C device (no register address).
 * @param device_address   7-bit I2C device address.
 * @param data             Pointer to the data bytes to be written.
 * @param len              Number of bytes to write.
 */
void i2c_writeDevice(uint8_t device_address, uint8_t *data, uint32_t len);

/**
 * @brief Read multiple bytes from a specific register of an I2C device.
 * @param device_address   7-bit I2C device address.
 * @param register_address Internal register address to read from.
 * @param data             Buffer to store the read data.
 * @param len              Number of bytes to read.
 */
void i2c_readRegDevice(uint8_t device_address, uint8_t register_address, uint8_t *data, uint32_t len);

/**
 * @brief Read multiple bytes directly from an I2C device (no register address).
 * @param device_address   7-bit I2C device address.
 * @param data             Buffer to store the read data.
 * @param len              Number of bytes to read.
 */
void i2c_readDevice(uint8_t device_address, uint8_t *data, uint32_t len);


#endif // I2C_H