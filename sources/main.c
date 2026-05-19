/**
 * @file main.c
 * @brief ESC Servo PWM test for Motor 1
 *
 * This application tests the PWM module by generating
 * a Servo PWM signal for a brushless ESC using TIM2.
 *
 * The program performs the following sequence:
 *
 * 1. Initializes PWM output on Motor 1
 * 2. Sends minimum throttle to arm the ESC
 * 3. Waits for ESC initialization
 * 4. Sends a low throttle signal
 * 5. Keeps the motor spinning slowly
 *
 * PWM configuration:
 *
 *  Frequency:
 *      50 Hz
 *
 *  Servo pulse range:
 *      1000 us -> minimum throttle
 *      2000 us -> maximum throttle
 *
 * ESC duty mapping:
 *
 *      5  -> 1000 us
 *      7  -> 1400 us
 *      10 -> 2000 us
 *
 * Timer usage:
 *
 *  TIM2:
 *      PWM generation
 *
 *  TIM3:
 *      Delay generation
 *
 * @authors
 * David Mijares
 */

#include <stdint.h>

// PWM driver
#include "PWM.h"

// Sensor driver
#include "Sensor.h"

// Delay driver
#include "Timer.h"

// ======================================================
// MOTOR TIMER CONFIGURATION
// ======================================================

/**
 * @brief Timer used for PWM generation
 *
 * TIM2 generates the PWM signal for the ESC.
 */
#define MOTOR_TIM TIM_2

/**
 * @brief PWM signal frequency
 *
 * ESC Servo PWM requires:
 *
 *  50 Hz
 *
 * which corresponds to:
 *
 *  20 ms period
 */
#define FREQUENCY 50

// ======================================================
// MOTOR 1 CONFIGURATION
// ======================================================

/**
 * @brief GPIO pin used for Motor 1 PWM output
 *
 * PA0 -> TIM2_CH1
 */
#define MOTOR_1_PIN 0

/**
 * @brief GPIO port used for Motor 1
 */
#define MOTOR_1_GPIO A

/**
 * @brief Timer channel used for Motor 1
 *
 * TIM2 Channel 1
 */
#define MOTOR_1_CHANNEL channel_1

// ======================================================
// MOTOR 2 CONFIGURATION
// ======================================================

/**
 * @brief GPIO pin used for Motor 2 PWM output
 */
#define MOTOR_2_PIN 1

/**
 * @brief GPIO port used for Motor 2
 */
#define MOTOR_2_GPIO A

/**
 * @brief Timer channel used for Motor 2
 */
#define MOTOR_2_CHANNEL channel_2

// ======================================================
// MOTOR 3 CONFIGURATION
// ======================================================

/**
 * @brief GPIO pin used for Motor 3 PWM output
 */
#define MOTOR_3_PIN 2

/**
 * @brief GPIO port used for Motor 3
 */
#define MOTOR_3_GPIO A

/**
 * @brief Timer channel used for Motor 3
 */
#define MOTOR_3_CHANNEL channel_3

// ======================================================
// MOTOR 4 CONFIGURATION
// ======================================================

/**
 * @brief GPIO pin used for Motor 4 PWM output
 */
#define MOTOR_4_PIN 3

/**
 * @brief GPIO port used for Motor 4
 */
#define MOTOR_4_GPIO A

/**
 * @brief Timer channel used for Motor 4
 */
#define MOTOR_4_CHANNEL channel_4

// ======================================================
// DELAY TIMER CONFIGURATION
// ======================================================

/**
 * @brief Timer used for delay generation
 *
 * TIM3 is used exclusively for software delays.
 */
#define DELAY_TIM TIM_3

// ======================================================
// ESC PWM CONFIGURATION
// ======================================================

/**
 * @brief Minimum ESC duty cycle
 *
 * Equivalent to:
 *
 *  1000 us pulse width
 *
 * Used for ESC arming.
 */
#define ESC_MIN_DUTY 5

/**
 * @brief Maximum ESC duty cycle
 *
 * Equivalent to:
 *
 *  2000 us pulse width
 */
#define ESC_MAX_DUTY 10

/**
 * @brief ESC arming duty cycle
 *
 * ESC expects minimum throttle during startup.
 */
#define ESC_ARM_DUTY 5

/**
 * @brief Low throttle duty cycle
 *
 * Used to keep the motor spinning slowly.
 */
#define ESC_SLOW_DUTY 6

/**
 * @brief Main application
 *
 * Application sequence:
 *
 * 1. Initialize PWM output
 * 2. Send minimum throttle
 * 3. Arm ESC
 * 4. Wait for ESC startup
 * 5. Send low throttle
 * 6. Keep motor spinning
 *
 * @return
 * Never returns
 */
int main(void)
{
    // ==================================================
    // INITIALIZE PWM MODULE
    // ==================================================

    /**
     * Configure:
     *
     *  - GPIOA pin 0
     *  - TIM2
     *  - Alternate function
     *  - PWM output
     */
    pwm_init(
        MOTOR_1_GPIO,
        MOTOR_TIM,
        MOTOR_1_PIN
    );

    // ==================================================
    // SEND ESC ARMING SIGNAL
    // ==================================================

    /**
     * Configure PWM signal:
     *
     *  Frequency:
     *      50 Hz
     *
     *  Duty:
     *      ESC minimum throttle
     */
    pwm_setSignal(
        MOTOR_TIM,
        MOTOR_1_CHANNEL,
        FREQUENCY,
        ESC_ARM_DUTY
    );

    // ==================================================
    // START PWM GENERATION
    // ==================================================

    /**
     * Enable PWM signal generation
     */
    pwm_start(
        MOTOR_TIM,
        MOTOR_1_CHANNEL
    );

    // ==================================================
    // INITIALIZE DELAY TIMER
    // ==================================================

    /**
     * Configure TIM3 for delay generation
     */
    timer_init(DELAY_TIM);

    // ==================================================
    // WAIT FOR ESC ARMING
    // ==================================================

    /**
     * Wait 3 seconds for ESC startup
     * and signal recognition
     */
    timer_delay_ms(DELAY_TIM, 3000);

    // ==================================================
    // SEND LOW THROTTLE SIGNAL
    // ==================================================

    /**
     * Increase throttle slightly
     * to make the motor spin slowly
     */
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
        /**
         * Keep motor spinning slowly
         *
         * PWM runs entirely in hardware,
         * therefore no additional CPU
         * interaction is required.
         */
        timer_delay_ms(DELAY_TIM, 1000);
    }

    // Program never reaches this point
    return 0;
}