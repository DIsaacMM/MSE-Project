#ifndef PWM_H
#define PWM_H

/**
 * @file PWM.h
 * @brief PWM Driver for ESC Servo PWM Control
 *
 * This module provides the necessary functions to generate
 * PWM signals using STM32 hardware timers.
 *
 * The driver is specifically configured for ESC Servo PWM:
 *
 *  - Frequency: 50 Hz
 *  - Pulse range:
 *      1000 us -> minimum throttle
 *      2000 us -> maximum throttle
 *
 * The PWM signal is generated using the STM32 timer
 * compare channels in PWM Mode 1.
 *
 * This module allows:
 *  - GPIO alternate function initialization
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
 * @brief Alternate Function number for TIM2
 *
 * AF1 connects the GPIO pin to the TIM2 peripheral.
 *
 * Example:
 *  PA0 -> TIM2_CH1
 *  PA1 -> TIM2_CH2
 */
#define ALTERNATE_FUNC_TIM2 1

/**
 * @brief PWM Mode 1 configuration value
 *
 * PWM Mode 1 behavior:
 *
 * Output HIGH:
 *  when CNT < CCRx
 *
 * Output LOW:
 *  when CNT >= CCRx
 *
 * This is the standard PWM mode used
 * for ESC servo control.
 */
#define PWM_MODE 6

// ======================================================
// PWM CHANNEL ENUMERATION
// ======================================================

/**
 * @brief PWM timer channels
 *
 * Each STM32 timer contains up to 4
 * capture/compare channels.
 *
 * These channels correspond to:
 *
 *  CCR1
 *  CCR2
 *  CCR3
 *  CCR4
 */
typedef enum pwm_channel
{
    /**
     * @brief Timer Channel 1
     */
    channel_1 = 1,

    /**
     * @brief Timer Channel 2
     */
    channel_2 = 2,

    /**
     * @brief Timer Channel 3
     */
    channel_3 = 3,

    /**
     * @brief Timer Channel 4
     */
    channel_4 = 4,

    /**
     * @brief Number of channels
     */
    SIZE = 5

} channel_t;

// ======================================================
// FUNCTION DECLARATIONS
// ======================================================

/**
 * @brief Initializes the PWM module
 *
 * This function performs the following steps:
 *
 * 1. Initializes the GPIO subsystem
 * 2. Enables the selected GPIO port
 * 3. Configures the GPIO pin as Alternate Function
 * 4. Connects the GPIO pin to TIM2
 * 5. Initializes the timer subsystem
 * 6. Enables the selected timer
 *
 * @pre
 * None
 *
 * @post
 * GPIO and Timer are configured for PWM operation
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
void pwm_init(
    port_t p,
    tim_t t,
    uint8_t pin
);

/**
 * @brief Configures the PWM signal
 *
 * This function configures:
 *
 *  - Timer prescaler
 *  - Timer auto-reload register
 *  - PWM mode
 *  - Compare value
 *  - PWM output channel
 *
 * The PWM signal is configured for:
 *
 *  Frequency:
 *      50 Hz
 *
 *  Servo Pulse Width:
 *      1000 us -> minimum throttle
 *      2000 us -> maximum throttle
 *
 * Duty cycle mapping:
 *
 *      5  -> 1000 us
 *      10 -> 2000 us
 *
 * @pre
 * pwm_init() must be called first
 *
 * @post
 * PWM signal is fully configured
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
 * Duty cycle percentage
 *
 * @return
 * No return value
 */
void pwm_setSignal(
    tim_t t,
    channel_t chann,
    uint32_t frecuency,
    uint8_t duty_cycle
);

/**
 * @brief Starts PWM generation
 *
 * This function enables the PWM signal
 * generation on the configured timer channel.
 *
 * @pre
 * pwm_init() and pwm_setSignal()
 * must be called first
 *
 * @post
 * PWM signal becomes active
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
void pwm_start(
    tim_t t,
    channel_t chann
);

/**
 * @brief Stops PWM generation
 *
 * This function disables:
 *
 *  - PWM output channel
 *  - Timer counter
 *
 * @pre
 * PWM must already be active
 *
 * @post
 * PWM signal is disabled
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
void pwm_stop(
    tim_t t,
    channel_t chann
);

#endif