#ifndef PWM_H
#define PWM_H

/**
 * @file PWM.h
 * @brief PWM Driver for ESC Servo PWM Control
 *
 * This module provides the necessary functions
 * to generate PWM signals using STM32 timers.
 *
 * The driver is configured specifically for
 * ESC Servo PWM control.
 *
 * PWM configuration:
 *
 *  Frequency:
 *      50 Hz
 *
 *  Pulse Width:
 *      1000 us -> minimum throttle
 *      2000 us -> maximum throttle
 *
 * This module allows:
 *
 *  - GPIO alternate function configuration
 *  - Timer initialization
 *  - PWM signal configuration
 *  - PWM output enable/disable
 *
 * @authors
 * David Mijares,
 * Ximena Cedillo,
 * Xavier Clemente
 */

// ======================================================
// MODULE DEPENDENCIES
// ======================================================

// Timer low-level driver
#include "TIM.h"

// GPIO low-level driver
#include "GPIO.h"

// Delay driver
#include "Timer.h"

// ======================================================
// CONSTANT DEFINITIONS
// ======================================================

/**
 * @brief TIM2 Alternate Function number
 *
 * AF1 connects the GPIO pin to TIM2.
 */
#define ALTERNATE_FUNC_TIM2 1

/**
 * @brief PWM Mode 1 value
 *
 * Output HIGH when:
 *      CNT < CCRx
 *
 * Output LOW when:
 *      CNT >= CCRx
 */
#define PWM_MODE 6

// ======================================================
// PWM CHANNEL ENUMERATION
// ======================================================

/**
 * @brief PWM timer channels
 *
 * Each timer has up to 4 compare channels.
 */
typedef enum pwm_channel
{
    // Timer Compare Channel 1
    channel_1 = 1,

    // Timer Compare Channel 2
    channel_2 = 2,

    // Timer Compare Channel 3
    channel_3 = 3,

    // Timer Compare Channel 4
    channel_4 = 4,

    // Number of channels
    SIZE = 5

} channel_t;

// ======================================================
// FUNCTION DECLARATIONS
// ======================================================

/**
 * @brief Initialize PWM module
 *
 * This function:
 *
 * 1. Initializes GPIO subsystem
 * 2. Enables selected GPIO port
 * 3. Configures GPIO pin as Alternate Function
 * 4. Connects pin to TIM2
 * 5. Initializes timer subsystem
 * 6. Enables selected timer
 *
 * @param p
 * GPIO port
 *
 * @param t
 * Timer module
 *
 * @param pin
 * GPIO pin number
 *
 * @return
 * No return value
 */
void pwm_init(port_t p, tim_t t, uint8_t pin);

/**
 * @brief Configure PWM signal
 *
 * This function configures:
 *
 *  - Timer prescaler
 *  - Auto-reload register
 *  - PWM Mode 1
 *  - Compare register value
 *  - PWM output channel
 *
 * ESC PWM mapping:
 *
 *      5  -> 1000 us
 *      10 -> 2000 us
 *
 * @param t
 * Timer module
 *
 * @param chann
 * Timer channel
 *
 * @param frecuency
 * PWM frequency
 *
 * @param duty_cycle
 * PWM duty cycle
 *
 * @return
 * No return value
 */
void pwm_setSignal(tim_t t, channel_t chann, uint32_t frecuency, uint8_t duty_cycle);

/**
 * @brief Start PWM signal generation
 *
 * Enables PWM output on selected channel.
 *
 * @param t
 * Timer module
 *
 * @param chann
 * Timer channel
 *
 * @return
 * No return value
 */
void pwm_start(tim_t t, channel_t chann);

/**
 * @brief Stop PWM signal generation
 *
 * Disables:
 *
 *  - PWM output channel
 *  - Timer counter
 *
 * @param t
 * Timer module
 *
 * @param chann
 * Timer channel
 *
 * @return
 * No return value
 */
void pwm_stop(tim_t t, channel_t chann);

#endif