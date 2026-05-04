/**
 * @file Timer.c
 * @brief Implementation of timer-based delays
 * 
 * This file implements delay functions using the TIM driver.
 * It uses TIM2 in polling mode to create accurate delays
 * without consuming CPU cycles in software loops.
 *
 * @authors David Mijares, Ximena Cedillo, Xavier Clemente
 *
 */

#include "Timer.h"

/**
 * @brief Initializes the timer module for delay generation
 * 
 * This function performs the following steps:
 * 1. Initializes the TIM driver (maps logical timers to hardware)
 * 2. Enables the clock for TIM2
 * 
 * @note This function must be called before using timer_delay_ms()
 * 
 * @warning Calling this function multiple times has no effect after
 *          the first successful initialization
 * 
 * @return No return value
 */

void timer_init(tim_t t)
{
    tim_init(); 
    
    // Configure timer for the desired delay
    tim_initTimer(t); 
}

/**
 * @brief Generates a blocking delay in milliseconds
 * 
 * This function creates an accurate delay using TIM2 hardware timer.
 * The delay is implemented as follows:
 * 1. Configure TIM2 to generate an update event after DELAY_MS milliseconds
 * 2. Enable the timer to start counting
 * 3. Wait (block) until the timer reaches the configured value
 * 4. Disable the timer to save power
 * 
 * @pre timer_init() must be called before this function
 * 
 * @note The function is blocking - CPU waits for the entire delay duration
 * @note Default delay is 1000ms (1 second) as defined by DELAY_MS
 * 
 * @warning Do not call this function from an interrupt handler as it blocks
 * 
 * @return No return value
 */
void timer_delay_ms(tim_t t, uint16_t delay_ms)
{
    tim_setTimerMs(t, delay_ms); 
    
    // Enable timer and wait for it to complete
    tim_enableTimer(t);
    tim_waitTimer(t);
    tim_disableTimer(t);
}