/**
 * @file ADC.h
 * @brief ADC peripheral driver header file for STM32F4xx microcontrollers
 * 
 * This file contains the function prototypes, type definitions, and constants
 * for the Analog-to-Digital Converter (ADC) peripheral driver.
 *
 * @authors David Mijares, Ximena Cedillo, Xavier Clemente
 */

#ifndef ADC_H
#define ADC_H



#include <stdint.h>
#include "stm32f4xx.h" 


/**
 * @brief Initializes the ADC subsystem
 * 
 * This function shall enable clocking for the ADC1 peripheral and configure
 * all ADC registers to their default state after a reset.
 * 
 * @param No parameters
 * @return No return value
 * 
 * @implements FR-1: The system shall enable clocking and configure all ADC 
 *                   peripherals to a default state.
 */
void adc_init(void);

/**
 * @brief Enables the ADC peripheral for conversions
 * 
 * This function shall activate the ADC by setting the ADON bit,
 * allowing the ADC to perform conversions when triggered.
 * 
 * @param No parameters
 * @return No return value
 */
void adc_enableADC(void);

/**
 * @brief Configures a regular ADC channel
 * 
 * This function shall allow selection of a valid ADC channel (0-18)
 * and configure it as the first (and only) channel in the regular
 * conversion sequence.
 * 
 * @param channel The ADC channel number to be configured (0 to 18)
 * @return No return value
 * 
 * @implements FR-3: The system shall allow selection of valid channels.
 */
void adc_setChannel(uint8_t channel);

/**
 * @brief Configures an injected ADC channel
 * 
 * This function shall allow selection of a valid injected ADC channel (0-18)
 * and configure it as the injected channel for conversion when JL=0.
 * 
 * @param channel The ADC injected channel number to be configured (0 to 18)
 * @return No return value
 * 
 * @implements FR-4: The system shall allow selection of valid injected channels.
 */
void setInjectedChannel(uint8_t channel);

/**
 * @brief Starts a single conversion on the configured regular channel
 * 
 * This function shall begin a single ADC conversion and the hardware
 * shall automatically set the status flags when the conversion starts
 * and when it completes.
 * 
 * @param No parameters
 * @return No return value
 * 
 * @implements FR-5: The system shall begin conversion and set status flags.
 */
void adc_startSingleConversion(void);

/**
 * @brief Starts continuous conversion mode on a regular channel
 * 
 * This function shall repeatedly convert input signals on the specified
 * channel until stopped. The ADC will automatically restart a new
 * conversion immediately after each conversion completes.
 * 
 * @param channel The ADC channel number to be converted continuously (0 to 18)
 * @return No return value
 * 
 * @implements FR-6: The system shall repeatedly convert input signals until stopped.
 */
void adc_startContinuosConversion(uint8_t channel);

/**
 * @brief Configures and starts injected channel conversion using external triggers
 * 
 * This function shall configure the ADC to trigger conversion of injected
 * channels based on external events such as timer events or EXTI lines.
 * 
 * @param triggerSource The external event source to trigger the conversion (0 to 15)
 *        Use TriggerSource_t enumeration for standard values
 * @param triggerEdge The polarity of the external trigger (0 to 3)
 *        Use TriggerEdge_t enumeration for standard values
 * @return No return value
 * 
 * @implements FR-7: The system shall trigger conversion based on external events.
 */
void adc_startInjectedConversion(uint8_t triggerSource, uint8_t triggerEdge);

/**
 * @brief Reads the result of the last regular conversion
 * 
 * This function shall return the digital value corresponding to the
 * analog input from the most recent regular channel conversion.
 * 
 * @param No parameters
 * @return uint32_t The digital conversion result (0-4095 for 12-bit resolution)
 * 
 * @implements FR-8: The system shall return the digital value corresponding 
 *                   to the analog input.
 */
uint32_t adc_readData(void);

/**
 * @brief Reads the result of the last injected channel conversion
 * 
 * This function shall return the digital value for the injected channel
 * that was configured. For JL=0 configuration, the result is stored in JDR4.
 * 
 * @param No parameters
 * @return uint32_t The digital conversion result from the injected channel
 * 
 * @implements FR-9: The system shall return the digital value for the 
 *                   specified injected channel.
 */
uint32_t adc_readInjectedChannelData(void);

/**
 * @brief Stops continuous conversion mode
 * 
 * This function disables continuous conversion mode on the regular channel.
 * After calling this function, conversions will stop and the ADC will return
 * to single conversion mode.
 * 
 * @param No parameters
 * @return No return value
 * 
 * @note This function is recommended for use with FR-6 to provide a way
 *       to stop the continuous conversions.
 */
void adc_stopContinuousConversion(void);

/**
 * @brief Checks if a regular conversion is complete
 * 
 * This function reads the End of Conversion (EOC) flag to determine
 * if the most recent regular conversion has finished.
 * 
 * @param No parameters
 * @return uint8_t Returns 1 if conversion is complete, 0 if still in progress
 */
uint8_t adc_isConversionComplete(void);

/**
 * @brief Checks if an injected conversion is complete
 * 
 * This function reads the Injected End of Conversion (JEOC) flag to determine
 * if the most recent injected conversion has finished.
 * 
 * @param No parameters
 * @return uint8_t Returns 1 if conversion is complete, 0 if still in progress
 */
uint8_t adc_isInjectedConversionComplete(void);

/**
 * @brief Clears the regular conversion status flags
 * 
 * This function clears the STRT and EOC flags in the ADC status register.
 * 
 * @param No parameters
 * @return No return value
 */
void adc_clearStatusFlags(void);

#endif /* ADC_H */