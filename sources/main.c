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
#include "Timer.h"
#include "Drone.h"
#include "PID.h"


// Timer used for delays
#define DELAY_TIM TIM_3

// ESC Servo PWM frequency
#define FREQUENCY 50


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


    PID_Init(); 

    // drone_init(); 

    // timer_init(DELAY_TIM); 
    // timer_delay_ms(DELAY_TIM, 5000);
    // ==================================================
    // MAIN LOOP
    // ==================================================

    while (1)
    {
            PID_Update();
            // m1.setSignal(m1.tim, m1.channel, FREQUENCY, 1050); 
            // m2.setSignal(m2.tim, m2.channel, FREQUENCY, 1050); 
            // m3.setSignal(m3.tim, m3.channel, FREQUENCY, 1050); 
            // m4.setSignal(m4.tim, m4.channel,FREQUENCY, 1050); 
            // timer_delay_ms(DELAY_TIM, 20);

        // for(uint16_t i = ESC_MIN_US; i < 1100; i++)
        // {
        //     pwm_setSignal(MOTOR_TIM2, MOTOR_1_CHANNEL, FREQUENCY, i);
        //     pwm_setSignal(MOTOR_TIM2, MOTOR_2_CHANNEL, FREQUENCY, i);
        //     pwm_setSignal(MOTOR_TIM4, MOTOR_3_CHANNEL, FREQUENCY, i);
        //     pwm_setSignal(MOTOR_TIM4, MOTOR_4_CHANNEL, FREQUENCY, i);
        //     timer_delay_ms(DELAY_TIM, 20);
        // }

        // for(uint16_t i = 1000; i > 1000; i--)
        // {
        //     pwm_setSignal(MOTOR_TIM2, MOTOR_1_CHANNEL, FREQUENCY, i);
        //     pwm_setSignal(MOTOR_TIM2, MOTOR_2_CHANNEL, FREQUENCY, i);
        //     pwm_setSignal(MOTOR_TIM4, MOTOR_3_CHANNEL, FREQUENCY, i);
        //     pwm_setSignal(MOTOR_TIM4, MOTOR_4_CHANNEL, FREQUENCY, i);
        //     timer_delay_ms(DELAY_TIM, 20);
        // }

        // while (1)
        // {
        //     timer_delay_ms(DELAY_TIM, 1000);
        // }        
    }

    return 0;
}
