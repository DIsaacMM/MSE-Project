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


// Motor Global Constants
#define MOTOR_TIM TIM_2          // TIM that will be implemented in the main
#define FREQUENCY 1000      // Frequency = 1kHz

// Motor 1 Global Constants
#define MOTOR_1_PIN 0          // Pin implemented in the main 
#define MOTOR_1_GPIO A         // PWM's GPIO that will be implemented in the main
#define MOTOR_1_CHANNEL channel_1  // Channel that will be implemented in the main for the TIMx

// Motor 2 Global Constants
#define MOTOR_2_PIN 1           // Pin implemented in the main 
#define MOTOR_2_GPIO A         // PWM's GPIO that will be implemented in the main
#define MOTOR_2_CHANNEL channel_2  // Channel that will be implemented in the main for the TIMx

// Motor 3 Global Constants
#define MOTOR_3_PIN 3           // Pin implemented in the main 
#define MOTOR_3_GPIO A         // PWM's GPIO that will be implemented in the main
#define MOTOR_3_CHANNEL channel_3  // Channel that will be implemented in the main for the TIMx

// Motor 4 Global Constants
#define MOTOR_4_PIN 3           // Pin implemented in the main 
#define MOTOR_4_GPIO A         // PWM's GPIO that will be implemented in the main
#define MOTOR_4_CHANNEL channel_4  // Channel that will be implemented in the main for the TIMx



int main(void)
{ 
    
    uint8_t duty_cycle1 = 0;
    uint8_t duty_cycle2 = 0;
    uint8_t duty_cycle3 = 0;
    uint8_t duty_cycle4 = 0;
    
    // Initialize Motor 1
    pwm_init(MOTOR_1_GPIO, MOTOR_TIM, MOTOR_1_PIN);
    pwm_setSignal(MOTOR_TIM, MOTOR_1_CHANNEL, FREQUENCY, duty_cycle1);

    // Initialize Motor 2
    pwm_init(MOTOR_2_GPIO, MOTOR_TIM, MOTOR_2_PIN);
    pwm_setSignal(MOTOR_TIM, MOTOR_2_CHANNEL, FREQUENCY, duty_cycle2);

    // Initialize Motor 3
    pwm_init(MOTOR_3_GPIO, MOTOR_TIM, MOTOR_3_PIN);
    pwm_setSignal(MOTOR_TIM, MOTOR_3_CHANNEL, FREQUENCY, duty_cycle3);

    // Initialize Motor 4
    pwm_init(MOTOR_4_GPIO, MOTOR_TIM, MOTOR_4_PIN);
    pwm_setSignal(MOTOR_TIM, MOTOR_4_CHANNEL, FREQUENCY, duty_cycle4);

    while(1)
    {
        duty_cycle1 = 25;
        duty_cycle2 = 25;
        duty_cycle3 = 25;
        duty_cycle4 = 25;

        pwm_setSignal(MOTOR_TIM, MOTOR_1_CHANNEL, FREQUENCY, duty_cycle1);
        pwm_setSignal(MOTOR_TIM, MOTOR_2_CHANNEL, FREQUENCY, duty_cycle2);
        pwm_setSignal(MOTOR_TIM, MOTOR_3_CHANNEL, FREQUENCY, duty_cycle3);
        pwm_setSignal(MOTOR_TIM, MOTOR_4_CHANNEL, FREQUENCY, duty_cycle4);
        // Small delay to prevent overwhelming the system
        for(volatile int i = 0; i < 100; i++) { __NOP(); }
    }
    return 0;  // Never reached
}


