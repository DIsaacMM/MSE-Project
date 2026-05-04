/**
 * @file ADC.c
 * @brief ADC peripheral driver for STM32F4xx microcontrollers
 * 
 * This file implements the initialization and control functions for the
 * Analog-to-Digital Converter (ADC) peripheral.
 *
 * @authors David Mijares, Ximena Cedillo, Xavier Clemente
 */

#include "ADC.h"

/**
 * @brief Initializes the ADC subsystem
 * 
 * This function shall enable clocking for the ADC1 peripheral and configure
 * all ADC registers to their default state after a reset.
 * 
 * @param No parameters
 * @return No return value
 * 
 */
void adc_init()
{
    // Initialize ADC Clocks
    // Enable the clock for ADC1 in the APB2 peripheral bus
    // Without this clock, ADC registers cannot be accessed
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; 


    // Reset ADC to ensure it starts from a known default state
    // Set the ADC reset bit in the APB2 reset register
    RCC->APB2RSTR |= RCC_APB2RSTR_ADCRST;       // Reset ADC interface - forces all ADC registers to their reset values
    
    // Clear the reset bit to take the ADC out of reset mode
    // This allows normal ADC operation to begin
    RCC->APB2RSTR &= ~(RCC_APB2RSTR_ADCRST);    // Stops the reset mode in the ADC interface

    // Configure ADC Control Registers to their default states
    
    // CR1 (Control Register 1) = 0x00000000
    // This configures: resolution = 12 bits, analog watchdog disabled, 
    // scan mode disabled, interrupt flags cleared, and AWDCH = 00000 (channel 0 selected for watchdog)
    ADC1->CR1 = 0; // Control Register 1 set to ADC analog input channel 0 
    
    // CR2 (Control Register 2) = 0x00000000
    // This configures: continuous conversion disabled, DMA disabled, 
    // data alignment = right-aligned, software trigger disabled, ADC disabled
    ADC1->CR2 = 0; // Control Register 2 set to disable conversion mode
}

/**
 * @brief Enables the ADC peripheral for conversions
 * 
 * This function shall activate the ADC by setting the ADON bit,
 * allowing the ADC to perform conversions when triggered.
 * 
 * @param No parameters
 * @return No return value
 */
void adc_enableADC()
{
    // Set the ADON (ADC ON) bit in CR2 register
    // This powers up the ADC and enables it to perform conversions
    // When this bit is set, the ADC is activated and ready for conversions
    ADC1->CR2 |= ADC_CR2_ADON; // The ADC is activated for conversions
}

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
 */
void adc_setChannel(uint8_t channel)
{
    // Validate the channel number to prevent invalid configurations
    // STM32F4 ADC supports channels 0 through 18 (19 total channels)
    // Channels 16, 17, 18 are internal (temperature sensor, Vrefint, VBAT)
    if(channel > 18)
    {
        return;  // Exit function without configuring if channel is invalid
    }

    // Configure the 1st conversion in the regular sequence (SQ1)
    // SQ1 bits are located in SQR3 register at positions 0-4 (5 bits total)
    // Clear existing bits to remove any previously configured channel
    ADC1->SQR3 &= ~(31 << 0);       // 31 = 0x1F = binary 11111 - clears bits 0-4
    
    // Set the new channel number in the cleared bits
    // The channel number is placed in bits 0-4 of SQR3
    ADC1->SQR3 |= (channel << 0);   // Configure new channel in the SQ1 field

    // Configure the length of the regular sequence (L bits in SQR1)
    // L bits are located in SQR1 register at positions 20-23 (4 bits total)
    // 0000 = 1 conversion, 0001 = 2 conversions, ... 1111 = 16 conversions
    
    // Clear the existing sequence length configuration
    ADC1->SQR1 &= ~(15 << 20);  // 15 = 0x0F = binary 1111 - clears bits 20-23
    
    // Set L = 0 (binary 0000) which means there is exactly 1 conversion in the sequence
    // This configures the ADC to only convert the channel we just set in SQ1
    ADC1->SQR1 |= (0 << 20);    // Assign L = 0, this means there is 1 conversion
}

/**
 * @brief Configures an injected ADC channel
 * 
 * This function shall allow selection of a valid injected ADC channel (0-18)
 * and configure it as the injected channel for conversion when JL=0.
 * 
 * @param channel The ADC injected channel number to be configured (0 to 18)
 * @return No return value
 * 
 */
void setInjectedChannel(uint8_t channel)
{
    // Validate channel number (0-18 are valid, same as regular channels)
    if(channel > 18)
    {
        return;  // Invalid channel - exit without configuring
    }

    // Configure JSQ4 (4th conversion in injected sequence)
    // When JL=0 (only 1 conversion), the ADC uses only JSQ4
    // JSQ4 bits are located in JSQR register at positions 15-19 (5 bits)
    
    // Clear the existing JSQ4 configuration (bits 15-19)
    ADC1->JSQR &= ~(31 << 15);       // 31 = 0x1F (5 bits) - clears bits 15-19 for JSQ4
    
    // Write the new channel number into JSQ4 field
    ADC1->JSQR |= (channel << 15);   // Place channel number in bits 15-19

    // Configure the injected sequence length (JL bits in JSQR)
    // JL bits are located in JSQR register at positions 20-21 (2 bits total)
    // JL = 00 : 1 conversion, 01: 2 conversions, 10: 3 conversions, 11: 4 conversions
    
    // Clear the existing JL configuration (bits 20-21)
    ADC1->JSQR &= ~(3 << 20);   // 3 = 0b11 (2 bits) - clears bits 20-21
    
    // Set JL = 0 (binary 00) which means exactly 1 conversion
    // With JL=0, the ADC only converts the channel configured in JSQ4
    ADC1->JSQR |= (0 << 20);    // Assign L = 0, this means there is 1 conversion
}

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
 */
void adc_startSingleConversion()
{
    // Ensure the ADC is enabled before attempting a conversion
    // If the ADC is not enabled, the conversion will not start
    adc_enableADC();                // Enable ADC for conversions

    // Start the conversion for the regular channel by setting SWSTART bit
    // SWSTART = Software START - this triggers the ADC to begin conversion
    // The hardware automatically clears this bit after the conversion starts
    ADC1->CR2 |= ADC_CR2_SWSTART;   // Set SWSTART bit to start conversion
    // Note: The status flags (STRT and EOC) are automatically set by hardware
    // STRT indicates conversion has started, EOC indicates conversion is complete
}

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
 */
void adc_startContinuosConversion(uint8_t channel)
{
    // First, configure the channel to be converted
    // This sets up SQ1 in SQR3 and L=0 for single channel sequence
    adc_setChannel(channel);    // Configure the regular channel sequence 

    // Enable continuous conversion mode by setting the CONT bit in CR2
    // When CONT=1, the ADC automatically re-triggers itself after each conversion
    // When CONT=0 (default), the ADC stops after each conversion
    ADC1->CR2 |= ADC_CR2_CONT;  // Set continuous conversion mode 

    // Start the first conversion - after this completes, the ADC will
    // automatically continue converting without additional software triggers
    adc_startSingleConversion(); // Begin the continuous conversion sequence
}

/**
 * @brief Configures and starts injected channel conversion using external triggers
 * 
 * This function shall configure the ADC to trigger conversion of injected
 * channels based on external events such as timer events or EXTI lines.
 * 
 * @param triggerSource The external event source to trigger the conversion (0 to 15)
 *        - 0000: Timer 1 CC4 event      - 1000: Timer 4 CC3 event
 *        - 0001: Timer 1 TRGO event     - 1001: Timer 4 TRGO event
 *        - 0010: Timer 2 CC1 event      - 1010: Timer 5 CC4 event
 *        - 0011: Timer 2 TRGO event     - 1011: Timer 5 TRGO event
 *        - 0100: Timer 3 CC2 event      - 1111: EXTI line15
 *        - 0101: Timer 3 CC4 event      - Others: Reserved
 *        - 0110: Timer 4 CC1 event
 *        - 0111: Timer 4 CC2 event
 * @param triggerEdge The polarity of the external trigger (0 to 3)
 *        - 00: Trigger detection disabled
 *        - 01: Trigger detection on rising edge
 *        - 10: Trigger detection on falling edge
 *        - 11: Trigger detection on both rising and falling edges
 * @return No return value
 * 
 */
void adc_startInjectedConversion(uint8_t triggerSource, uint8_t triggerEdge)
{
    // Validate the trigger source parameter
    // JEXTSEL is 4 bits, so valid range is 0 to 15
    if(triggerSource > 15)  
    {
        return;  // Invalid trigger source - exit without configuring
    }
    
    // Validate the trigger edge/polarity parameter
    // JEXTEN is 2 bits, so valid range is 0 to 3
    if(triggerEdge > 3) 
    {
        return;  // Invalid trigger edge configuration - exit
    }

    // Enable the ADC peripheral before configuring external triggers
    adc_enableADC();
    
    // Clear existing external trigger configuration for injected channels
    // JEXTSEL bits 16-19 (4 bits) - selects which external event triggers conversion
    ADC1->CR2 &= ~(15 << 16);   // Clear JEXTSEL bits 16-19 (15 = 0b1111 = 4 bits)
    
    // JEXTEN bits 20-21 (2 bits) - enables trigger and selects polarity
    ADC1->CR2 &= ~(3 << 20);    // Clear JEXTEN bits 20-21 (3 = 0b11 = 2 bits)
    
    // Configure external trigger source (JEXTSEL[3:0] bits 16-19)
    // Valid trigger sources include:
    // 0000: Timer 1 CC4 event         1000: Timer 4 CC3 event
    // 0001: Timer 1 TRGO event        1001: Timer 4 TRGO event
    // 0010: Timer 2 CC1 event         1010: Timer 5 CC4 event
    // 0011: Timer 2 TRGO event        1011: Timer 5 TRGO event
    // 0100: Timer 3 CC2 event         1100-1110: Reserved
    // 0101: Timer 3 CC4 event         1111: EXTI line15
    // 0110: Timer 4 CC1 event
    // 0111: Timer 4 CC2 event
    ADC1->CR2 |= (triggerSource << 16);   // Write trigger source to JEXTSEL bits 16-19
    
    // Configure external trigger polarity/edge (JEXTEN[1:0] bits 20-21)
    // 00: Trigger detection disabled (no external trigger)
    // 01: Trigger detection on rising edge (0->1 transition)
    // 10: Trigger detection on falling edge (1->0 transition)
    // 11: Trigger detection on both rising and falling edges
    ADC1->CR2 |= (triggerEdge << 20);     // Write polarity configuration to JEXTEN bits 20-21
    
    // Note: No software trigger (JSWSTART) is used here
    // The conversion will start automatically when the selected external event occurs
}

/**
 * @brief Reads the result of the last regular conversion
 * 
 * This function shall return the digital value corresponding to the
 * analog input from the most recent regular channel conversion.
 * 
 * @param No parameters
 * @return uint32_t The digital conversion result (0-4095 for 12-bit resolution)
 * 
 */
uint32_t adc_readData()
{
    // Wait for the End Of Conversion (EOC) flag to be set by hardware
    // EOC flag indicates that the conversion is complete and data is ready
    // The while loop continues as long as EOC is 0 (conversion in progress)
    while(!(ADC1->SR & ADC_SR_EOC));  // Busy-wait until conversion completes
    
    // Read the conversion result from the Data Register (DR)
    // Reading DR automatically clears the EOC flag
    // DR contains the 12-bit (or configured resolution) digital value
    // The value is right-aligned by default (range 0-4095 for 12-bit mode)
    return ADC1->DR; 
}

/**
 * @brief Reads the result of the last injected channel conversion
 * 
 * This function shall return the digital value for the injected channel
 * that was configured. For JL=0 configuration, the result is stored in JDR4.
 * 
 * @param No parameters
 * @return uint32_t The digital conversion result from the injected channel
 * 
 */
uint32_t adc_readInjectedChannelData()
{
    // Wait for the Injected End Of Conversion (JEOC) flag
    // JEOC is set by hardware when an injected channel conversion completes
    // This ensures we don't read old data before a new conversion finishes
    while(!(ADC1->SR & ADC_SR_JEOC));  // Wait for injected conversion to complete
    
    // Clear the JEOC flag by writing 0 to it
    // This must be done manually for injected conversions (unlike regular EOC)
    // Clearing the flag allows detection of the next injected conversion completion
    ADC1->SR &= ~ADC_SR_JEOC;  // Clear the JEOC flag
    
    // Read the injected data register for channel 4 (JDR4)
    // When JL=0 (single conversion configured in setInjectedChannel()),
    // the ADC stores the result in JDR4 (not JDR1, JDR2, or JDR3)
    // This register contains the 12-bit (or configured resolution) digital value
    return ADC1->JDR4; 
}