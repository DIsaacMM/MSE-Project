/**
 * @file Sensors.c
 * @brief High-level sensor driver for ADC-based sensors
 * 
 * This file provides a simplified interface for reading analog sensors
 * using the ADC peripheral. It wraps the low-level ADC driver functions
 * and provides mode selection for single or continuous conversions.
 *
 * @authors David Mijares, Ximena Cedillo, Xavier Clemente
 */

#include "Sensor.h"

/**
 * @brief Initializes a sensor on a specific ADC channel
 * 
 * This function initializes the ADC peripheral and configures it
 * to read from the specified analog channel. Must be called before
 * any conversion can be performed.
 * 
 * @param channel The ADC channel number where the sensor is connected (0 to 18)
 * @return No return value
 * 
 * @note If channel is invalid (>18), function returns without configuration
 */
void sensor_init(uint8_t channel, port_t p, uint8_t pin, uint8_t mode)
{
    // Validate channel number (0-18 are valid, same as regular channels)
    if(channel > 18)
    {
        return;  // Invalid channel - exit without configuring
    }
    
    // Initialize ADC peripheral to default state
    adc_init(); 

    gpio_setPinMode(p, pin, mode);


    // Configure the specified channel for regular ADC conversions
    adc_setChannel(channel); 
}

/**
 * @brief Starts a conversion on a sensor with selectable mode
 * 
 * This function starts an ADC conversion on the specified channel.
 * The user can choose between single conversion (one reading) or
 * continuous conversion (repeated readings until stopped).
 * 
 * @param channel The ADC channel number to convert (0 to 18)
 * @param conversion The conversion mode:
 *        - 0: Single conversion (one reading, then stops)
 *        - 1: Continuous conversion (repeated readings until stopped)
 * @return No return value
 * 
 * @note If channel is invalid (>18), function returns without starting conversion
 * @note For continuous mode, call adc_stopContinuousConversion() to stop
 */
void sensor_startConversion(uint8_t channel, uint8_t conversion)
{
    // Validate channel number (0-18 are valid, same as regular channels)
    if(channel > 18)
    {
        return;  // Invalid channel - exit without configuring
    }
    
    // Select conversion mode based on 'conversion' parameter
    // conversion = 0: Single conversion (one-time reading)
    // conversion = 1: Continuous conversion (continuous readings)
    if(conversion == 0)
    {
        adc_startSingleConversion();           // Start one conversion and stop
    }
    else if(conversion == 1)
    {
        adc_startContinuosConversion(channel); // Start continuous conversions
    }
    // Note: If conversion is neither 0 nor 1, function does nothing
}

/**
 * @brief Reads the last converted value from the sensor
 * 
 * This function returns the digital value from the most recent
 * analog-to-digital conversion performed on the sensor.
 * For single conversion mode, this should be called after conversion completes.
 * For continuous mode, this returns the most recent value at any time.
 * 
 * @return uint32_t The digital conversion result (0-4095 for 12-bit resolution)
 * 
 * @note This function will wait for conversion to complete if called before
 *       the conversion finishes (due to adc_readData() implementation)
 */
uint32_t sensor_readValue(void)
{
    // Read and return the result from the ADC data register
    return adc_readData(); 
}