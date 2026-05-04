#ifndef TIMER_H
#define TIMER_H
/**
 * @file Timer.h
 * @brief Module for generating delays using hardware timers
 * 
 * This module provides functions to generate accurate delays using
 * STM32 hardware timers. It uses TIM2 to create blocking delays
 * in milliseconds.
 * 
 * The module allows initialization and generation of delays without
 * occupying the CPU with software loops.
 *
 * @authors David Mijares, Ximena Cedillo, Xavier Clemente
 *
 */

#include "TIM.h"
#include "GPIO.h"


/**
 * @brief Initializes the timer module for delay generation
 * 
 * This function initializes the TIM driver and configures TIM2
 * for generating delays. It must be called before using delay functions.
 * 
 * @pre None
 * @post Timer module is ready to generate delays
 * 
 * @return No return value
 */

void timer_init(tim_t t);

/**
 * @brief Generates a blocking delay in milliseconds
 * 
 * This function creates a delay by configuring TIM2 for the specified
 * amount of time, then waiting for the timer to complete its cycle.
 * The function blocks execution until the delay time has elapsed.
 * 
 * @pre timer_init() must be called before this function
 * @post Delay has been executed (program waits for specified time)
 * 
 * @return No return value
 * 
 * @note The delay is implemented using hardware timer polling,
 *       which is more accurate than software loops
 */

void timer_delay_ms(tim_t t, uint16_t delay_ms);

#endif