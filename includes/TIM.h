#ifndef TIM_H  //If GPIO_H is not defined
#define TIM_H  // define
/**
 * @file TIM.h
 * @brief Module that generates delays using hardware timers.
 *
 * This module provides functions to configure and use STM32 timers
 * for delays, frequency generation, and compare modes.
 *
 * @authors David Mijares, Ximena Cedillo, Xavier Clemente
 *
 */


#include <stdint.h>
#include "stm32f4xx.h" 

#define TIMER_CLOCK_HZ 16000000 // 16 MHz

typedef enum tim
{
    TIM_1 = 0,
    TIM_2 = 1,
    TIM_3 = 2,
    TIM_4 = 3,
    TIM_5 = 4,
    TIM_9 = 5,
    TIM_10 = 6,
    TIM_11 = 7,
    TIM_SIZE = 8  
} tim_t;

extern TIM_TypeDef* TIM[TIM_SIZE];


/**
 * @brief Initializes the timer pointer array
 * 
 * This function maps logical timer indices to actual hardware timer instances.
 * 
 * @return No return value
 */

void tim_init();

/**
 * @brief Enables the clock for a specific timer
 * 
 * This function activates the RCC clock corresponding to the selected timer.
 * It also initializes the timer pointer array.
 * 
 * @param t Timer index
 * @return No return value
 */

void tim_initTimer(tim_t t);

/**
 * @brief Configures a timer to generate a delay in milliseconds
 * 
 * This function sets the prescaler and auto-reload values to produce
 * a delay based on the specified time in milliseconds.
 * 
 * @param t Timer index
 * @param time_ms Time in milliseconds
 * @return No return value
 */

void tim_setTimerMs(tim_t t, uint32_t time_ms);

/**
 * @brief Configures a timer to generate a specific frequency
 * 
 * This function sets the timer to operate at a desired frequency
 * by adjusting prescaler and auto-reload values.
 * 
 * @param t Timer index
 * @param freq_hz Desired frequency in Hz
 * @return No return value
 */
void tim_setTimerFreq(tim_t t, uint32_t freq_hz);

/**
 * @brief Enables the timer
 * 
 * This function starts the timer counting.
 * 
 * @param t Timer index
 * @return No return value
 */

void tim_enableTimer(tim_t t);

/**
 * @brief Disables the timer
 * 
 * This function stops the timer counting.
 * 
 * @param t Timer index
 * @return No return value
 */

void tim_disableTimer(tim_t t);

/**
 * @brief Waits for the timer to overflow (blocking)
 * 
 * This function continuously polls the update flag until
 * the timer completes its cycle.
 * 
 * @param t Timer index
 * @return No return value
 */

void tim_waitTimer(tim_t t);

/**
 * @brief Sets the compare value for a timer channel
 * 
 * This function writes a value into the corresponding CCR register.
 * 
 * @param t Timer index
 * @param channel Channel number (1 to 4)
 * @param compare_value Value to compare
 * @return No return value
 */

void tim_setTimerCompareChannelValue(tim_t t, uint8_t channel, uint16_t compare_value);
/**
 * @brief Configures the compare mode of a timer channel
 * 
 * This function sets the output compare mode using CCMR registers.
 * 
 * @param t Timer index
 * @param channel Channel number (1 to 4)
 * @param mode Compare mode (0–7)
 * @return No return value
 */

void tim_setTimerCompareMode(tim_t t, uint8_t channel, uint8_t mode);
/**
 * @brief Enables a timer compare channel
 * 
 * This function activates the output of a specific channel.
 * 
 * @param t Timer index
 * @param channel Channel number (1 to 4)
 * @return No return value
 */

void tim_enableTimerCompareChannel(tim_t t, uint8_t channel);

/**
 * @brief Disables a timer compare channel
 * 
 * This function deactivates the output of a specific channel.
 * 
 * @param t Timer index
 * @param channel Channel number (1 to 4)
 * @return No return value
 */

void tim_disableTimerCompareChannel(tim_t t, uint8_t channel);

#endif