/**
 * @file main.c
 * @brief ESC PWM test for Motor 1
 *
 * This program generates a PWM signal using TIM2
 * to control a brushless ESC in Servo PWM mode.
 *
 * The ESC is first armed at minimum throttle,
 * then the motor is commanded to spin slowly.
 *
 * @authors
 * David Mijares
 * Aldo De la Torre
 * Jose Paez
 */

#include <stdint.h>

#include "PWM.h"
#include "Sensor.h"
#include "Timer.h"

// ======================================================
// TIMER CONFIGURATION
// ======================================================

// Timer used for PWM generation
#define MOTOR_TIM TIM_2

// Timer used for delays
#define DELAY_TIM TIM_3

// ESC Servo PWM frequency
#define FREQUENCY 50

// ======================================================
// MOTOR 1 CONFIGURATION
// ======================================================

// PA0 -> TIM2_CH1
#define MOTOR_1_PIN 0

// GPIO port
#define MOTOR_1_GPIO A

// Timer channel
#define MOTOR_1_CHANNEL channel_1

// ======================================================
// MOTOR 2 CONFIGURATION
// ======================================================

#define MOTOR_2_PIN 1
#define MOTOR_2_GPIO A
#define MOTOR_2_CHANNEL channel_2

// ======================================================
// MOTOR 3 CONFIGURATION
// ======================================================

#define MOTOR_3_PIN 2
#define MOTOR_3_GPIO A
#define MOTOR_3_CHANNEL channel_3

// ======================================================
// MOTOR 4 CONFIGURATION
// ======================================================

#define MOTOR_4_PIN 3
#define MOTOR_4_GPIO A
#define MOTOR_4_CHANNEL channel_4

// ======================================================
// ESC CONFIGURATION
// ======================================================

// Minimum throttle
// 5 -> 1000 us
#define ESC_MIN_DUTY 5

// Maximum throttle
// 10 -> 2000 us
#define ESC_MAX_DUTY 10

// ESC arming throttle
#define ESC_ARM_DUTY 5

// Low throttle for slow motor spin
#define ESC_SLOW_DUTY 6

/**
 * @brief Main program
 *
 * Program flow:
 *
 * 1. Initialize PWM
 * 2. Send minimum throttle
 * 3. Wait for ESC arming
 * 4. Send low throttle
 * 5. Keep motor spinning slowly
 *
 * @return
 * Never returns
 */
int main(void)
{
    // ==================================================
    // INITIALIZE PWM
    // ==================================================

    // Configure PA0 as PWM output
    pwm_init(MOTOR_1_GPIO, MOTOR_TIM, MOTOR_1_PIN);

    // ==================================================
    // ARM ESC
    // ==================================================

    // Send minimum throttle signal
    pwm_setSignal(MOTOR_TIM, MOTOR_1_CHANNEL, FREQUENCY, ESC_ARM_DUTY);

    // Start PWM generation
    pwm_start(MOTOR_TIM, MOTOR_1_CHANNEL);

    // ==================================================
    // INITIALIZE DELAY TIMER
    // ==================================================

    // Configure TIM3 for delays
    timer_init(DELAY_TIM);

    // Wait for ESC startup
    timer_delay_ms(DELAY_TIM, 3000);

    // ==================================================
    // START MOTOR
    // ==================================================

    // Send low throttle signal
    // so the motor spins slowly
    pwm_setSignal(MOTOR_TIM, MOTOR_1_CHANNEL, FREQUENCY, ESC_SLOW_DUTY);

    // ==================================================
    // MAIN LOOP
    // ==================================================

    while (1)
    {
        // PWM runs in hardware,
        // nothing else is required here
        timer_delay_ms(DELAY_TIM, 1000);
    }

    return 0;
}