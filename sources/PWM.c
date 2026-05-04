/**
 * @file PWM.c
 * @brief Implementation of PWM signal generation
 * 
 * This file implements the PWM module functions using the TIM driver.
 * It generates a PWM signal on GPIOA pin 0 using TIM2 channel 1.
 * @authors David Mijares, Ximena Cedillo, Xavier Clemente
 *
 */

#include "PWM.h"

/**
 * @brief Initializes the PWM module
 * 
 * This function performs the following steps:
 * 1. Initializes the GPIO subsystem
 * 2. Enables clock for GPIO port A
 * 3. Configures PA0 as alternate function for TIM2
 * 4. Initializes the timer subsystem
 * 5. Enables clock for TIM2
 * 
 * @note The GPIO alternate function configuration includes both the
 *       MODER register (sets alternate function mode) and the AFR
 *       register (selects the specific alternate function)
 * 
 * @warning Must be called before any other PWM functions
 * 
 * @return No return value
 */

void pwm_init(port_t p, tim_t t, uint8_t pin)
{   
    
    gpio_init(); 
    gpio_initPort(p); 
    gpio_setAlternateFunction(p, pin, ALTERNATE_FUNC_TIM2);  // Connect PIN 0 in GPIOA to the TIM2
    
    //Initialize timer and set frequency
    tim_init(); 
    tim_initTimer(t);

}

/**
 * @brief Configures the PWM signal parameters
 * 
 * This function performs the following steps:
 * 1. Sets the PWM frequency using the timer
 * 2. Configures the channel in PWM mode
 * 3. Calculates the compare value for 50% duty cycle
 * 4. Loads the compare value into the CCR register
 * 
 * @pre pwm_init() must be called before this function
 * @post Timer is configured with the specified frequency and mode
 * 
 * @note The compare value is calculated as 50% of the ARR value
 *       to produce a symmetrical square wave
 * 
 * @return No return value
 */

void pwm_setSignal(tim_t t, channel_t chann, uint32_t frecuency, uint8_t duty_cycle)
{
    tim_setTimerFreq(t, frecuency);
    tim_setTimerCompareMode(t, chann, PWM_MODE);
    
    // Calculate compare value based on duty cycle percentage
    uint16_t compare_value = (TIM[t]->ARR * duty_cycle) / 100;
    
    tim_setTimerCompareChannelValue(t, chann, compare_value);
}

/**
 * @brief Starts PWM signal generation
 * 
 * This function enables the hardware to start generating the PWM signal:
 * 1. Enables the compare channel output
 * 2. Enables the timer counter
 * 
 * @pre pwm_init() and pwm_setSignal() must be called first
 * @post PWM signal is active on the configured GPIO pin
 * 
 * @note Once started, the PWM runs in hardware and does not require
 *       CPU intervention until stopped
 * 
 * @return No return value
 */

void pwm_start(tim_t t, channel_t chann)
{
    tim_enableTimerCompareChannel(t, chann); 
    tim_enableTimer(t); 
}

/**
 * @brief Stops PWM signal generation
 * 
 * This function halts the PWM output:
 * 1. Disables the compare channel output
 * 2. Stops the timer counter
 * 
 * @pre PWM must be previously started
 * @post PWM signal is stopped and GPIO pin returns to high impedance state
 * 
 * @warning Calling this function stops all channels on the same timer
 * 
 * @return No return value
 */

void pwm_stop(tim_t t, channel_t chann)
{
    tim_disableTimerCompareChannel(t, chann); 
    tim_disableTimer(t); 
}