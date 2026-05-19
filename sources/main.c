/**
 * @file main.c
 * @brief 
 * 
 *
 * @authors David Mijares
 */

#include <stdint.h>
#include "PWM.h"
#include "Sensor.h"
#include "Timer.h" 


// Motor Global Constants
#define MOTOR_TIM TIM_2          // TIM that will be implemented in the main
#define FREQUENCY 50      // Frequency = 50Hz

// Motor 1 Global Constants
#define MOTOR_1_PIN 0          // Pin implemented in the main 
#define MOTOR_1_GPIO A         // PWM's GPIO that will be implemented in the main
#define MOTOR_1_CHANNEL channel_1  // Channel that will be implemented in the main for the TIMx

// Motor 2 Global Constants
#define MOTOR_2_PIN 1           // Pin implemented in the main 
#define MOTOR_2_GPIO A         // PWM's GPIO that will be implemented in the main
#define MOTOR_2_CHANNEL channel_2  // Channel that will be implemented in the main for the TIMx

// Motor 3 Global Constants
#define MOTOR_3_PIN 2           // Pin implemented in the main 
#define MOTOR_3_GPIO A         // PWM's GPIO that will be implemented in the main
#define MOTOR_3_CHANNEL channel_3  // Channel that will be implemented in the main for the TIMx

// Motor 4 Global Constants
#define MOTOR_4_PIN 3           // Pin implemented in the main 
#define MOTOR_4_GPIO A         // PWM's GPIO that will be implemented in the main
#define MOTOR_4_CHANNEL channel_4  // Channel that will be implemented in the main for the TIMx

// Delay Global Constants

#define DELAY_TIM TIM_3

// ESC Global Constants
#define ESC_MIN_DUTY 5
#define ESC_MAX_DUTY 10

// ESC Servo PWM:
//
// 5  -> 1000 us
// 7  -> 1400 us
// 10 -> 2000 us

// Minimum valid throttle
#define ESC_ARM_DUTY 5

// Very low throttle for slow spin
#define ESC_SLOW_DUTY 6

int main(void)
{
    // ==================================================
    // INITIALIZE PWM
    // ==================================================

    pwm_init(
        MOTOR_1_GPIO,
        MOTOR_TIM,
        MOTOR_1_PIN
    );

    // ==================================================
    // ARM ESC
    // ==================================================

    pwm_setSignal(
        MOTOR_TIM,
        MOTOR_1_CHANNEL,
        FREQUENCY,
        ESC_ARM_DUTY
    );

    pwm_start(
        MOTOR_TIM,
        MOTOR_1_CHANNEL
    );

    // ==================================================
    // INITIALIZE DELAY TIMER
    // ==================================================

    timer_init(DELAY_TIM);

    // Wait for ESC arming
    timer_delay_ms(DELAY_TIM, 3000);

    // ==================================================
    // START MOTOR SLOWLY
    // ==================================================

    pwm_setSignal(
        MOTOR_TIM,
        MOTOR_1_CHANNEL,
        FREQUENCY,
        ESC_SLOW_DUTY
    );

    // ==================================================
    // MAIN LOOP
    // ==================================================

    while (1)
    {
        // Keep motor spinning slowly
        timer_delay_ms(DELAY_TIM, 1000);
    }

    return 0;
}