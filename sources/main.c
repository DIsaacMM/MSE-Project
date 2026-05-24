/**
 * @file main.c
 * @brief ESC PWM test for 4 Motors
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
#include "Drone.h"

// ======================================================
// TIMER CONFIGURATION
// ======================================================

// Timer used for PWM generation
#define MOTOR_TIM2 TIM_2
#define MOTOR_TIM4 TIM_4


// Timer used for delays
#define DELAY_TIM TIM_3

// ESC Servo PWM frequency
#define FREQUENCY 50

// ======================================================
// MOTOR 1 CONFIGURATION
// ======================================================

#define MOTOR_1_PIN 0
#define MOTOR_1_GPIO A
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

#define MOTOR_3_PIN 6
#define MOTOR_3_GPIO B
#define MOTOR_3_CHANNEL channel_1

// ======================================================
// MOTOR 4 CONFIGURATION
// ======================================================

#define MOTOR_4_PIN 7
#define MOTOR_4_GPIO B
#define MOTOR_4_CHANNEL channel_2

// ======================================================
// ESC CONFIGURATION
// ======================================================

// Minimum throttle
// 5 -> 1000 us
#define ESC_MIN_US 1000

// Maximum throttle
// 10 -> 2000 us
#define ESC_MAX_US 2000


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
    pwm_init(MOTOR_1_GPIO, MOTOR_TIM2, MOTOR_1_PIN);
    pwm_init(MOTOR_2_GPIO, MOTOR_TIM2, MOTOR_2_PIN);
    pwm_init(MOTOR_3_GPIO, MOTOR_TIM4, MOTOR_3_PIN);
    pwm_init(MOTOR_4_GPIO, MOTOR_TIM4, MOTOR_4_PIN);

    // ==================================================
    // ARM ESC
    // ==================================================

    // Send minimum throttle signal
    pwm_setSignal(MOTOR_TIM2, MOTOR_1_CHANNEL, FREQUENCY, ESC_MIN_US);
    pwm_setSignal(MOTOR_TIM2, MOTOR_2_CHANNEL, FREQUENCY, ESC_MIN_US);
    pwm_setSignal(MOTOR_TIM4, MOTOR_3_CHANNEL, FREQUENCY, ESC_MIN_US);
    pwm_setSignal(MOTOR_TIM4, MOTOR_4_CHANNEL, FREQUENCY, ESC_MIN_US);

    // Start PWM generation
    pwm_start(MOTOR_TIM2, MOTOR_1_CHANNEL);
    pwm_start(MOTOR_TIM2, MOTOR_2_CHANNEL);
    pwm_start(MOTOR_TIM4, MOTOR_3_CHANNEL);
    pwm_start(MOTOR_TIM4, MOTOR_4_CHANNEL);

    // ==================================================
    // INITIALIZE DELAY TIMER
    // ==================================================

    // Configure TIM3 for delays
    timer_init(DELAY_TIM);

    // Wait for ESC startup
    timer_delay_ms(DELAY_TIM, 3000);


    // ==================================================
    // MAIN LOOP
    // ==================================================

    while (1)
    {
        for(uint16_t i = ESC_MIN_US; i < 1100; i++)
        {
            pwm_setSignal(MOTOR_TIM2, MOTOR_1_CHANNEL, FREQUENCY, i);
            pwm_setSignal(MOTOR_TIM2, MOTOR_2_CHANNEL, FREQUENCY, i);
            pwm_setSignal(MOTOR_TIM4, MOTOR_3_CHANNEL, FREQUENCY, i);
            pwm_setSignal(MOTOR_TIM4, MOTOR_4_CHANNEL, FREQUENCY, i);
            timer_delay_ms(DELAY_TIM, 20);
        }

        for(uint16_t i = 1000; i > 1000; i--)
        {
            pwm_setSignal(MOTOR_TIM2, MOTOR_1_CHANNEL, FREQUENCY, i);
            pwm_setSignal(MOTOR_TIM2, MOTOR_2_CHANNEL, FREQUENCY, i);
            pwm_setSignal(MOTOR_TIM4, MOTOR_3_CHANNEL, FREQUENCY, i);
            pwm_setSignal(MOTOR_TIM4, MOTOR_4_CHANNEL, FREQUENCY, i);
            timer_delay_ms(DELAY_TIM, 20);
        }

        while (1)
        {
            timer_delay_ms(DELAY_TIM, 1000);
        }        
    }

    return 0;
}

