/**
 * @file Sensor.h
 * @brief High-level sensor driver header file
 * 
 * This file contains function prototypes and type definitions for
 * simplified sensor reading using the ADC peripheral.
 *
 * @authors David Mijares, Ximena Cedillo, Xavier Clemente
 */

#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include "ADC.h"
#include "GPIO.h"


/**
 * @brief Initializes a sensor on a specific ADC channel
 * 
 * This function initializes the ADC peripheral and configures it
 * to read from the specified analog channel.
 * 
 * @param channel The ADC channel number where the sensor is connected (0 to 18)
 * @return No return value
 */
void sensor_init(uint8_t channel, port_t p, uint8_t pin, uint8_t mode);

/**
 * @brief Starts a conversion on a sensor with selectable mode
 * 
 * This function starts an ADC conversion on the specified channel.
 * The user can choose between single conversion (one reading) or
 * continuous conversion (repeated readings until stopped).
 * 
 * @param channel The ADC channel number to convert (0 to 18)
 * @param conversion The conversion mode (0=Single, 1=Continuous)
 * @return No return value
 */
void sensor_startConversion(uint8_t channel, uint8_t conversion);

/**
 * @brief Reads the last converted value from the sensor
 * 
 * This function returns the digital value from the most recent
 * analog-to-digital conversion performed on the sensor.
 * 
 * @return uint32_t The digital conversion result (0-4095 for 12-bit resolution)
 */
uint32_t sensor_readValue();

#endif /* SENSOR_H */