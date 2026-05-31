/**
 * @file I2C.c
 * @brief I2C peripheral driver for STM32F411RE (Master mode, polling)
 *
 * This driver provides functions to initialize the I2C1 peripheral,
 * write/read data to/from I2C devices with or without register addressing.
 * All operations are blocking (polling) and do not use interrupts.
 *
 * @authors David Mijares, Ximena Cedillo, Xavier Clemente
 */

#include "I2C.h"

/**
 * @brief Calculate the CCR (Clock Control Register) value for a desired SCL frequency.
 * @param scl_freq_hz Desired SCL clock frequency in Hz (e.g., 100000 for 100 kHz).
 * @return Value to be written to the I2Cx->CCR register.
 * @note This formula is for standard mode (100 kHz) and fast mode (400 kHz)
 *       on STM32F4 devices. Assumes SystemCoreClock is correctly set.
 */
uint32_t i2c_calc_ccr(uint32_t scl_freq_hz)
{
    return SystemCoreClock / (2 * scl_freq_hz);
}

/**
 * @brief Calculate the TRISE (Rise Time Register) value for a desired SCL frequency.
 * @param scl_freq_hz Desired SCL clock frequency in Hz (e.g., 100000 or 400000).
 * @return Value to be written to the I2Cx->TRISE register.
 * @note Uses I2C standard maximum rise times: 1000 ns for 100 kHz, 300 ns for 400 kHz.
 *       The formula is: TRISE = (max_rise_ns * SystemCoreClock / 1e9) + 1.
 */
uint32_t i2c_trise(uint32_t scl_freq_hz)
{
    uint32_t max_rise_ns;

    if (scl_freq_hz <= 100000)
    {
        max_rise_ns = 1000;   // Standard mode
    }
    else if (scl_freq_hz <= 400000)
    {
        max_rise_ns = 300;    // Fast mode
    }
    else
    {
        max_rise_ns = 120;    // Fast mode plus
    }

    // SystemCoreClock is typically in Hz
    // Convert to MHz first to avoid overflow
    uint32_t pclk_mhz = SystemCoreClock / 1000000UL;

    // TRISE = (max_rise_ns * PCLK_MHz / 1000) + 1
    uint32_t trise = ((max_rise_ns * pclk_mhz) / 1000UL) + 1UL;

    return trise;
}

/**
 * @brief Initialize the I2C1 peripheral with default settings.
 * @details
 *   - Enables the clock for I2C1 (APB1 bus).
 *   - Resets CR1 and CR2 registers.
 *   - Sets CCR and TRISE based on the macro SLC_FREQ (Hz).
 *   - Disables all I2C interrupts (event, buffer, error).
 *   - Enables the I2C peripheral.
 *
 * @note The macro SLC_FREQ must be defined (e.g., 100000) in I2C.h or build settings.
 *       SystemCoreClock must be up-to-date (call SystemCoreClockUpdate() before).
 */
void i2c_init(void)
{
    // Enable I2C1 Clock (APB1ENR bit 21)
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // Software reset — limpia cualquier estado colgado del bus
    I2C1->CR1 |=  I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    // CR2: FREQ = frecuencia APB1 en MHz (bits [5:0]) — OBLIGATORIO
    // Sin esto CCR y TRISE no funcionan correctamente
    // STM32F411RE Nucleo @ 16MHz HSI → FREQ = 16
    I2C1->CR2 = 0;
    I2C1->CR2 |= (16U & 0x3F);

    // Set clock speed and rise time
    I2C1->CCR   = i2c_calc_ccr(SLC_FREQ);
    I2C1->TRISE = i2c_trise(SLC_FREQ);

    // Disable all interrupts
    I2C1->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN);

    // Enable the I2C peripheral
    I2C1->CR1 |= I2C_CR1_PE;
}

/**
 * @brief Write multiple bytes to a specific register of an I2C device.
 * @param device_address   7-bit I2C device address.
 * @param register_address Internal register address on the device.
 * @param data             Pointer to the data bytes to be written.
 * @param len              Number of bytes to write.
 *
 * @details Sequence:
 *   - START condition
 *   - Send device address + write bit (0)
 *   - Wait for ADDR flag, clear it by reading SR1 & SR2
 *   - Send register address
 *   - Send each data byte (waiting for TXE after each)
 *   - STOP condition
 *
 * @note This function is blocking and does not implement timeouts or error handling.
 */
#define I2C_TIMEOUT 10000U
#define I2C_WAIT(cond) do {     volatile uint32_t _t = 0;     while(!(cond) && _t < I2C_TIMEOUT) _t++;     if (_t >= I2C_TIMEOUT) { I2C1->CR1 |= I2C_CR1_STOP; return; } } while(0)

void i2c_writeRegDevice(uint8_t device_address, uint8_t register_address, uint8_t *data, uint32_t len)
{
    for (volatile uint32_t t = 0; (I2C1->SR2 & I2C_SR2_BUSY) && t < I2C_TIMEOUT; t++);
    if (I2C1->SR2 & I2C_SR2_BUSY) return;

    I2C1->CR1 |= I2C_CR1_START;
    I2C_WAIT(I2C1->SR1 & I2C_SR1_SB);

    I2C1->DR = (device_address << 1);
    I2C_WAIT(I2C1->SR1 & I2C_SR1_ADDR);

    (void)I2C1->SR1;
    (void)I2C1->SR2;

    I2C1->DR = register_address;
    I2C_WAIT(I2C1->SR1 & I2C_SR1_TXE);

    for(uint32_t i = 0; i < len; i++)
    {
        I2C1->DR = data[i];
        I2C_WAIT(I2C1->SR1 & I2C_SR1_TXE);
    }

    I2C_WAIT(I2C1->SR1 & I2C_SR1_BTF);
    I2C1->CR1 |= I2C_CR1_STOP;
}

/**
 * @brief Write multiple bytes directly to an I2C device (no register address).
 * @param device_address   7-bit I2C device address.
 * @param data             Pointer to the data bytes to be written.
 * @param len              Number of bytes to write.
 *
 * @details Sequence:
 *   - START condition
 *   - Send device address + write bit (0)
 *   - Send all data bytes directly
 *   - STOP condition
 *
 * @note Useful for devices that do not use internal registers (e.g., some sensors).
 */
void i2c_writeDevice(uint8_t device_address, uint8_t *data, uint32_t len)
{
    I2C1->CR1 |= I2C_CR1_START;                     // Generate START
    I2C_WAIT(I2C1->SR1 & I2C_SR1_SB);              // Wait for START sent

    I2C1->DR = (device_address << 1);               // Send address + write bit
    I2C_WAIT(I2C1->SR1 & I2C_SR1_ADDR);            // Wait for address acknowledged

    // Clear ADDR flag
    (void)I2C1->SR1;
    (void)I2C1->SR2;

    for (uint32_t i = 0; i < len; i++)
    {
        I2C1->DR = data[i];                         // Send data byte
        I2C_WAIT(I2C1->SR1 & I2C_SR1_TXE);         // Wait for TX buffer empty
    }

    I2C1->CR1 |= I2C_CR1_STOP;                      // Generate STOP
}

/**
 * @brief Read multiple bytes from a specific register of an I2C device.
 * @param device_address   7-bit I2C device address.
 * @param register_address Internal register address to read from.
 * @param data             Buffer to store the read data.
 * @param len              Number of bytes to read.
 *
 * @details Sequence:
 *   - START condition
 *   - Send device address + write bit (0)
 *   - Send register address
 *   - Repeated START
 *   - Send device address + read bit (1)
 *   - Enable ACK for all but the last byte
 *   - For each byte: wait for RXNE, read DR
 *   - For the last byte: disable ACK and generate STOP before reading
 *
 * @note The STOP is generated before reading the last byte, as required by STM32 I2C.
 */
void i2c_readRegDevice(uint8_t device_address, uint8_t register_address, uint8_t *data, uint32_t len)
{
    // WAIT BUSY
    /* Timeout: si el bus sigue ocupado despues de ~5000 ciclos, salir */
    for (volatile uint32_t t = 0; (I2C1->SR2 & I2C_SR2_BUSY) && t < 5000; t++);
    if (I2C1->SR2 & I2C_SR2_BUSY) return;

    // START
    I2C1->CR1 |= I2C_CR1_START;

    I2C_WAIT(I2C1->SR1 & I2C_SR1_SB);

    // WRITE ADDRESS
    I2C1->DR = (device_address << 1);

    I2C_WAIT(I2C1->SR1 & I2C_SR1_ADDR);

    (void)I2C1->SR1;
    (void)I2C1->SR2;

    // REGISTER ADDRESS
    I2C1->DR = register_address;

    I2C_WAIT(I2C1->SR1 & I2C_SR1_TXE);

    // REPEATED START
    I2C1->CR1 |= I2C_CR1_START;

    I2C_WAIT(I2C1->SR1 & I2C_SR1_SB);

    // READ ADDRESS
    I2C1->DR = (device_address << 1) | 1;

    I2C_WAIT(I2C1->SR1 & I2C_SR1_ADDR);

    // ENABLE ACK
    I2C1->CR1 |= I2C_CR1_ACK;

    (void)I2C1->SR1;
    (void)I2C1->SR2;

    for(uint32_t i = 0; i < len; i++)
    {
        // LAST BYTE
        if(i == (len - 1))
        {
            I2C1->CR1 &= ~I2C_CR1_ACK;
            I2C1->CR1 |= I2C_CR1_STOP;
        }

        I2C_WAIT(I2C1->SR1 & I2C_SR1_RXNE);

        data[i] = I2C1->DR;
    }
}
/**
 * @brief Read multiple bytes directly from an I2C device (no register address).
 * @param device_address   7-bit I2C device address.
 * @param data             Buffer to store the read data.
 * @param len              Number of bytes to read.
 *
 * @details Sequence:
 *   - START condition
 *   - Send device address + write bit (0) – this is a dummy write to set the device?
 *   - Repeated START
 *   - Send device address + read bit (1)
 *   - Enable ACK for all but last byte
 *   - Read bytes, with STOP generated before the last read.
 *
 * @note Some devices require a dummy write before reading (e.g., to set pointer).
 *       For pure read without any register, you can skip the first write phase.
 *       This implementation keeps it for compatibility with devices that need it.
 */
void i2c_readDevice(uint8_t device_address, uint8_t *data, uint32_t len)
{
    // Dummy write phase (some devices need this to set read mode)
    I2C1->CR1 |= I2C_CR1_START;
    I2C_WAIT(I2C1->SR1 & I2C_SR1_SB);

    I2C1->DR = (device_address << 1);               // Write address
    I2C_WAIT(I2C1->SR1 & I2C_SR1_ADDR);
    (void)I2C1->SR1;
    (void)I2C1->SR2;

    // Repeated start and read
    I2C1->CR1 |= I2C_CR1_START;                     // Repeated START
    I2C_WAIT(I2C1->SR1 & I2C_SR1_SB);

    I2C1->DR = (device_address << 1) | 1;           // Read address
    I2C_WAIT(I2C1->SR1 & I2C_SR1_ADDR);
    (void)I2C1->SR1;
    (void)I2C1->SR2;

    I2C1->CR1 |= I2C_CR1_ACK;                       // Enable ACK

    for (uint32_t i = 0; i < len; i++)
    {
        if (i == len - 1)                           // Last byte
        {
            I2C1->CR1 &= ~I2C_CR1_ACK;              // NACK
            I2C1->CR1 |= I2C_CR1_STOP;              // STOP before reading
        }
        I2C_WAIT(I2C1->SR1 & I2C_SR1_RXNE);
        data[i] = I2C1->DR;
    }
}