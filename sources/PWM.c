/**
 * @file PWM.c
 * @brief PWM Driver for ESC Servo PWM
 */

#include "PWM.h"

/**
 * @brief Initialize PWM GPIO and Timer
 */
void pwm_init(port_t p, tim_t t, uint8_t pin)
{
    gpio_init();

    gpio_initPort(p);

    // Connect GPIO pin to TIM2 AF
    gpio_setAlternateFunction(p, pin, ALTERNATE_FUNC_TIM2);

    // Initialize timer module
    tim_init();

    // Enable timer clock
    tim_initTimer(t);
}

/**
 * @brief Configure PWM signal for ESC Servo PWM
 *
 * ESC expects:
 * 50 Hz
 * 1000 us -> minimum throttle
 * 2000 us -> maximum throttle
 *
 * Duty mapping:
 * 5  -> 1000 us
 * 10 -> 2000 us
 */
void pwm_setSignal(
    tim_t t,
    channel_t chann,
    uint32_t frecuency,
    uint8_t duty_cycle
)
{
    (void)frecuency;

    // =========================================
    // STOP TIMER
    // =========================================

    TIM[t]->CR1 &= ~(1U << 0);

    // =========================================
    // TIMER CONFIGURATION
    // =========================================
    //
    // Timer Clock = 16 MHz
    //
    // PSC = 15
    // Tick = 1 us
    //
    // ARR = 19999
    // Period = 20 ms
    // Frequency = 50 Hz
    //
    // =========================================

    TIM[t]->PSC = 15;

    TIM[t]->ARR = 20000 - 1;

    // =========================================
    // DUTY TO MICROSECONDS
    // =========================================

    uint16_t pulse_us;

    if (duty_cycle <= 5)
    {
        pulse_us = 1000;
    }
    else if (duty_cycle >= 10)
    {
        pulse_us = 2000;
    }
    else
    {
        pulse_us =
            1000 +
            ((duty_cycle - 5) * 1000) / 5;
    }

    // =========================================
    // PWM MODE 1 CONFIGURATION
    // =========================================

    switch (chann)
    {
        case channel_1:

            // Clear OC1M bits
            TIM[t]->CCMR1 &= ~(7U << 4);

            // PWM Mode 1
            TIM[t]->CCMR1 |= (6U << 4);

            // Enable preload
            TIM[t]->CCMR1 |= (1U << 3);

            // Compare value
            TIM[t]->CCR1 = pulse_us;

            // Enable output
            TIM[t]->CCER |= (1U << 0);

            break;

        case channel_2:

            TIM[t]->CCMR1 &= ~(7U << 12);

            TIM[t]->CCMR1 |= (6U << 12);

            TIM[t]->CCMR1 |= (1U << 11);

            TIM[t]->CCR2 = pulse_us;

            TIM[t]->CCER |= (1U << 4);

            break;

        case channel_3:

            TIM[t]->CCMR2 &= ~(7U << 4);

            TIM[t]->CCMR2 |= (6U << 4);

            TIM[t]->CCMR2 |= (1U << 3);

            TIM[t]->CCR3 = pulse_us;

            TIM[t]->CCER |= (1U << 8);

            break;

        case channel_4:

            TIM[t]->CCMR2 &= ~(7U << 12);

            TIM[t]->CCMR2 |= (6U << 12);

            TIM[t]->CCMR2 |= (1U << 11);

            TIM[t]->CCR4 = pulse_us;

            TIM[t]->CCER |= (1U << 12);

            break;

        default:
            return;
    }

    // Enable ARR preload
    TIM[t]->CR1 |= (1U << 7);

    // Generate update event
    TIM[t]->EGR |= (1U << 0);

    // Enable timer
    TIM[t]->CR1 |= (1U << 0);
}

/**
 * @brief Start PWM output
 *
 * Timer and channel already enabled
 * in pwm_setSignal()
 */
void pwm_start(tim_t t, channel_t chann)
{
    (void)t;
    (void)chann;
}

/**
 * @brief Stop PWM output
 */
void pwm_stop(tim_t t, channel_t chann)
{
    switch (chann)
    {
        case channel_1:
            TIM[t]->CCER &= ~(1U << 0);
            break;

        case channel_2:
            TIM[t]->CCER &= ~(1U << 4);
            break;

        case channel_3:
            TIM[t]->CCER &= ~(1U << 8);
            break;

        case channel_4:
            TIM[t]->CCER &= ~(1U << 12);
            break;

        default:
            return;
    }

    // Disable timer
    TIM[t]->CR1 &= ~(1U << 0);
}