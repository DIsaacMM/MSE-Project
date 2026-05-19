/**
 * @file PWM.c
 * @brief PWM Driver for ESC Servo PWM
 *
 * This module implements PWM generation
 * using STM32 hardware timers.
 *
 * The PWM signal is configured for:
 *
 *  Frequency:
 *      50 Hz
 *
 *  Pulse Width:
 *      1000 us -> minimum throttle
 *      2000 us -> maximum throttle
 *
 * PWM generation uses:
 *
 *  - PWM Mode 1
 *  - Timer Compare Channels
 *  - Hardware timer outputs
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
 * This function:
 *
 * 1. Initializes GPIO subsystem
 * 2. Enables GPIO port
 * 3. Configures GPIO pin as Alternate Function
 * 4. Connects GPIO pin to TIM2
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
void pwm_init(port_t p, tim_t t, uint8_t pin)
{
    // Initialize GPIO subsystem
    gpio_init();

    // Enable GPIO port clock
    gpio_initPort(p);

    // Connect GPIO pin to TIM2 Alternate Function
    gpio_setAlternateFunction(p, pin, ALTERNATE_FUNC_TIM2);

    // Initialize timer subsystem
    tim_init();

    // Enable selected timer
    tim_initTimer(t);
}

/**
 * @brief Configure PWM signal for ESC control
 *
 * This function configures:
 *
 *  - Timer prescaler
 *  - Timer auto-reload register
 *  - PWM Mode 1
 *  - Compare register value
 *  - PWM output channel
 *
 * Timer configuration:
 *
 *  Timer Clock:
 *      16 MHz
 *
 *  Prescaler:
 *      15
 *
 *  Tick Resolution:
 *      1 us
 *
 *  ARR:
 *      19999
 *
 *  PWM Frequency:
 *      50 Hz
 *
 * Duty cycle mapping:
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
void pwm_setSignal(tim_t t, channel_t chann, uint32_t frecuency, uint8_t duty_cycle)
{
    // Frequency parameter unused
    // ESC Servo PWM always uses 50 Hz
    (void)frecuency;

    // ==================================================
    // STOP TIMER
    // ==================================================

    // Disable timer counter
    TIM[t]->CR1 &= ~(1U << 0);

    // ==================================================
    // TIMER CONFIGURATION
    // ==================================================

    // Set prescaler
    // 16 MHz / 16 = 1 MHz
    TIM[t]->PSC = 15;

    // Set auto-reload register
    // 20 ms period -> 50 Hz
    TIM[t]->ARR = 20000 - 1;

    // ==================================================
    // DUTY TO MICROSECONDS CONVERSION
    // ==================================================

    // Variable to store pulse width
    uint16_t pulse_us;

    // Minimum throttle
    if (duty_cycle <= 5)
    {
        pulse_us = 1000;
    }

    // Maximum throttle
    else if (duty_cycle >= 10)
    {
        pulse_us = 2000;
    }

    // Intermediate throttle
    else
    {
        pulse_us = 1000 + ((duty_cycle - 5) * 1000) / 5;
    }

    // ==================================================
    // PWM MODE 1 CONFIGURATION
    // ==================================================

    switch (chann)
    {
        // ==============================================
        // CHANNEL 1
        // ==============================================

        case channel_1:

            // Clear OC1M bits
            TIM[t]->CCMR1 &= ~(7U << 4);

            // Configure PWM Mode 1
            TIM[t]->CCMR1 |= (6U << 4);

            // Enable preload register
            TIM[t]->CCMR1 |= (1U << 3);

            // Set compare value
            TIM[t]->CCR1 = pulse_us;

            // Enable output channel
            TIM[t]->CCER |= (1U << 0);

            break;

        // ==============================================
        // CHANNEL 2
        // ==============================================

        case channel_2:

            // Clear OC2M bits
            TIM[t]->CCMR1 &= ~(7U << 12);

            // Configure PWM Mode 1
            TIM[t]->CCMR1 |= (6U << 12);

            // Enable preload register
            TIM[t]->CCMR1 |= (1U << 11);

            // Set compare value
            TIM[t]->CCR2 = pulse_us;

            // Enable output channel
            TIM[t]->CCER |= (1U << 4);

            break;

        // ==============================================
        // CHANNEL 3
        // ==============================================

        case channel_3:

            // Clear OC3M bits
            TIM[t]->CCMR2 &= ~(7U << 4);

            // Configure PWM Mode 1
            TIM[t]->CCMR2 |= (6U << 4);

            // Enable preload register
            TIM[t]->CCMR2 |= (1U << 3);

            // Set compare value
            TIM[t]->CCR3 = pulse_us;

            // Enable output channel
            TIM[t]->CCER |= (1U << 8);

            break;

        // ==============================================
        // CHANNEL 4
        // ==============================================

        case channel_4:

            // Clear OC4M bits
            TIM[t]->CCMR2 &= ~(7U << 12);

            // Configure PWM Mode 1
            TIM[t]->CCMR2 |= (6U << 12);

            // Enable preload register
            TIM[t]->CCMR2 |= (1U << 11);

            // Set compare value
            TIM[t]->CCR4 = pulse_us;

            // Enable output channel
            TIM[t]->CCER |= (1U << 12);

            break;

        // Invalid channel
        default:
            return;
    }

    // ==================================================
    // ENABLE ARR PRELOAD
    // ==================================================

    // Enable ARR preload
    TIM[t]->CR1 |= (1U << 7);

    // ==================================================
    // GENERATE UPDATE EVENT
    // ==================================================

    // Force register update
    TIM[t]->EGR |= (1U << 0);

    // ==================================================
    // ENABLE TIMER
    // ==================================================

    // Start timer counter
    TIM[t]->CR1 |= (1U << 0);
}

/**
 * @brief Start PWM output
 *
 * PWM is already enabled inside
 * pwm_setSignal().
 *
 * This function remains for compatibility.
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
    // ==================================================
    // DISABLE PWM CHANNEL
    // ==================================================

    switch (chann)
    {
        // Disable Channel 1
        case channel_1:
            TIM[t]->CCER &= ~(1U << 0);
            break;

        // Disable Channel 2
        case channel_2:
            TIM[t]->CCER &= ~(1U << 4);
            break;

        // Disable Channel 3
        case channel_3:
            TIM[t]->CCER &= ~(1U << 8);
            break;

        // Disable Channel 4
        case channel_4:
            TIM[t]->CCER &= ~(1U << 12);
            break;

        // Invalid channel
        default:
            return;
    }

    // ==================================================
    // DISABLE TIMER
    // ==================================================

    // Stop timer counter
    TIM[t]->CR1 &= ~(1U << 0);
}