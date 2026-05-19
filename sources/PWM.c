/**
 * @file PWM.c
 * @brief PWM Driver for ESC Servo PWM
 *
 * This module implements PWM generation using STM32 timers.
 *
 * The driver is specifically configured for ESC Servo PWM:
 *
 *  - Frequency: 50 Hz
 *  - Pulse width:
 *      1000 us -> minimum throttle
 *      2000 us -> maximum throttle
 *
 * The PWM signal is generated using the timer
 * compare channels configured in PWM Mode 1.
 *
 * The module allows:
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

#include "PWM.h"

/**
 * @brief Initialize PWM GPIO and Timer
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
 * GPIO and Timer are ready for PWM operation
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
void pwm_init(port_t p, tim_t t, uint8_t pin)
{
    // =========================================
    // INITIALIZE GPIO MODULE
    // =========================================

    // Enable GPIO subsystem
    gpio_init();

    // Enable selected GPIO port clock
    gpio_initPort(p);

    // =========================================
    // CONFIGURE GPIO PIN AS TIMER OUTPUT
    // =========================================

    // Connect GPIO pin to TIM2 Alternate Function
    gpio_setAlternateFunction(
        p,
        pin,
        ALTERNATE_FUNC_TIM2
    );

    // =========================================
    // INITIALIZE TIMER MODULE
    // =========================================

    // Enable timer subsystem
    tim_init();

    // Enable selected timer clock
    tim_initTimer(t);
}

/**
 * @brief Configure PWM signal for ESC Servo PWM
 *
 * This function configures:
 *
 *  - Timer prescaler
 *  - Timer auto-reload register
 *  - PWM Mode 1
 *  - Compare register value
 *  - PWM output channel
 *
 * The timer is configured for:
 *
 *  Timer Clock:
 *      16 MHz
 *
 *  Tick Resolution:
 *      1 us
 *
 *  PWM Frequency:
 *      50 Hz
 *
 *  PWM Period:
 *      20 ms
 *
 * ESC expected pulse width:
 *
 *  1000 us -> minimum throttle
 *  2000 us -> maximum throttle
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
 * PWM duty cycle percentage
 *
 * @return
 * No return value
 */
void pwm_setSignal(
    tim_t t,
    channel_t chann,
    uint32_t frecuency,
    uint8_t duty_cycle
)
{
    // Frequency parameter is ignored because
    // ESC Servo PWM always uses 50 Hz
    (void)frecuency;

    // =========================================
    // STOP TIMER BEFORE RECONFIGURATION
    // =========================================

    // Disable timer counter
    TIM[t]->CR1 &= ~(1U << 0);

    // =========================================
    // TIMER BASE CONFIGURATION
    // =========================================
    //
    // Timer clock = 16 MHz
    //
    // PSC = 15
    // 16 MHz / 16 = 1 MHz
    //
    // Tick = 1 us
    //
    // ARR = 19999
    // Period = 20000 us = 20 ms
    //
    // PWM Frequency = 50 Hz
    //
    // =========================================

    // Set timer prescaler
    TIM[t]->PSC = 15;

    // Set auto-reload register
    TIM[t]->ARR = 20000 - 1;

    // =========================================
    // DUTY CYCLE TO MICROSECONDS CONVERSION
    // =========================================

    // Variable that stores pulse width
    uint16_t pulse_us;

    // Minimum throttle
    if (duty_cycle <= 5)
    {
        // 1000 us pulse
        pulse_us = 1000;
    }

    // Maximum throttle
    else if (duty_cycle >= 10)
    {
        // 2000 us pulse
        pulse_us = 2000;
    }

    // Intermediate throttle values
    else
    {
        // Linear interpolation:
        //
        // 5  -> 1000 us
        // 10 -> 2000 us
        //
        pulse_us =
            1000 +
            ((duty_cycle - 5) * 1000) / 5;
    }

    // =========================================
    // PWM MODE 1 CONFIGURATION
    // =========================================

    switch (chann)
    {
        // =====================================
        // CHANNEL 1 CONFIGURATION
        // =====================================

        case channel_1:

            // Clear OC1M bits
            TIM[t]->CCMR1 &= ~(7U << 4);

            // Configure PWM Mode 1
            TIM[t]->CCMR1 |= (6U << 4);

            // Enable preload register
            TIM[t]->CCMR1 |= (1U << 3);

            // Set compare value
            TIM[t]->CCR1 = pulse_us;

            // Enable channel output
            TIM[t]->CCER |= (1U << 0);

            break;

        // =====================================
        // CHANNEL 2 CONFIGURATION
        // =====================================

        case channel_2:

            // Clear OC2M bits
            TIM[t]->CCMR1 &= ~(7U << 12);

            // Configure PWM Mode 1
            TIM[t]->CCMR1 |= (6U << 12);

            // Enable preload register
            TIM[t]->CCMR1 |= (1U << 11);

            // Set compare value
            TIM[t]->CCR2 = pulse_us;

            // Enable channel output
            TIM[t]->CCER |= (1U << 4);

            break;

        // =====================================
        // CHANNEL 3 CONFIGURATION
        // =====================================

        case channel_3:

            // Clear OC3M bits
            TIM[t]->CCMR2 &= ~(7U << 4);

            // Configure PWM Mode 1
            TIM[t]->CCMR2 |= (6U << 4);

            // Enable preload register
            TIM[t]->CCMR2 |= (1U << 3);

            // Set compare value
            TIM[t]->CCR3 = pulse_us;

            // Enable channel output
            TIM[t]->CCER |= (1U << 8);

            break;

        // =====================================
        // CHANNEL 4 CONFIGURATION
        // =====================================

        case channel_4:

            // Clear OC4M bits
            TIM[t]->CCMR2 &= ~(7U << 12);

            // Configure PWM Mode 1
            TIM[t]->CCMR2 |= (6U << 12);

            // Enable preload register
            TIM[t]->CCMR2 |= (1U << 11);

            // Set compare value
            TIM[t]->CCR4 = pulse_us;

            // Enable channel output
            TIM[t]->CCER |= (1U << 12);

            break;

        // Invalid channel
        default:
            return;
    }

    // =========================================
    // ENABLE AUTO-RELOAD PRELOAD
    // =========================================

    // Enable ARR preload
    TIM[t]->CR1 |= (1U << 7);

    // =========================================
    // GENERATE UPDATE EVENT
    // =========================================

    // Force register update
    TIM[t]->EGR |= (1U << 0);

    // =========================================
    // ENABLE TIMER
    // =========================================

    // Start timer counter
    TIM[t]->CR1 |= (1U << 0);
}

/**
 * @brief Start PWM output
 *
 * PWM generation is already enabled
 * inside pwm_setSignal().
 *
 * This function is kept for compatibility
 * with previous implementations.
 *
 * @pre
 * pwm_setSignal() must be called first
 *
 * @post
 * PWM signal remains active
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
void pwm_start(tim_t t, channel_t chann)
{
    // Unused parameters
    (void)t;
    (void)chann;
}

/**
 * @brief Stop PWM output
 *
 * This function disables:
 *
 *  - PWM output channel
 *  - Timer counter
 *
 * @pre
 * PWM signal must already be active
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
void pwm_stop(tim_t t, channel_t chann)
{
    // =========================================
    // DISABLE PWM CHANNEL OUTPUT
    // =========================================

    switch (chann)
    {
        // Disable Channel 1 output
        case channel_1:
            TIM[t]->CCER &= ~(1U << 0);
            break;

        // Disable Channel 2 output
        case channel_2:
            TIM[t]->CCER &= ~(1U << 4);
            break;

        // Disable Channel 3 output
        case channel_3:
            TIM[t]->CCER &= ~(1U << 8);
            break;

        // Disable Channel 4 output
        case channel_4:
            TIM[t]->CCER &= ~(1U << 12);
            break;

        // Invalid channel
        default:
            return;
    }

    // =========================================
    // DISABLE TIMER
    // =========================================

    // Stop timer counter
    TIM[t]->CR1 &= ~(1U << 0);
}