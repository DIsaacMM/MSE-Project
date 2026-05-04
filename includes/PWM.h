#ifndef PWM_H  //If PWM_H is not defined
#define PWM_H  // define
 /**
 * @file PWM.h
 * @brief 
 * @brief Module for PWM signal generation using hardware timers
 * 
 * This module provides functions to generate PWM signals on GPIO pins
 * using STM32 hardware timers. It uses TIM2 channel 1 to output a
 * PWM signal on GPIOA pin 0.
 * 
 * The module allows initialization, signal configuration, and start/stop
 * of PWM generation. Frequency and duty cycle are configurable.
 * @authors David Mijares, Ximena Cedillo, Xavier Clemente
 *
 */

#include "TIM.h"
#include "GPIO.h"
#include "Timer.h"


#define ALTERNATE_FUNC_TIM2 1   // AF1 configures the GPIO pin to be controlled by TIM2 peripheral
#define PWM_MODE 6              // 6 = PWM mode 1 (active HIGH): output HIGH when CNT < CCRx

/** 
 * @brief PWM channel enumeration
 * 
 * Defines the available timer channels for PWM output.
 * Each timer has 4 capture/compare channels.
 */

typedef enum pwm_channel
{
    channel_1 = 1,   // Channel 1 (CCR1) 
    channel_2 = 2,   // Channel 2 (CCR2)
    channel_3 = 3,   // Channel 3 (CCR3) 
    channel_4 = 4,   // Channel 4 (CCR4) 
    SIZE = 5         // Number of channels
} channel_t; 

/**
 * @brief Initializes the PWM module
 * 
 * This function configures the GPIO pin as alternate function for TIM2,
 * initializes the timer, and prepares the hardware for PWM generation.
 * 
 * @pre None
 * @post GPIO and timer are initialized for PWM output
 * 
 * @return No return value
 */
void pwm_init(port_t p, tim_t t, uint8_t pin);

/**
 * @brief Configures the PWM signal parameters
 * 
 * This function sets the PWM frequency and compare mode, and calculates
 * the compare value based on a 50% duty cycle.
 * 
 * @pre pwm_init() must be called first
 * @post Timer is configured with specified frequency and PWM mode
 * 
 * @return No return value
 */

void pwm_setSignal(tim_t t, channel_t chann, uint32_t frecuency, uint8_t duty_cycle);

/**
 * @brief Starts PWM signal generation
 * 
 * This function enables the compare channel and the timer to begin
 * generating the PWM signal on the configured GPIO pin.
 * 
 * @pre pwm_init() and pwm_setSignal() must be called first
 * @post PWM signal is active on the output pin
 * 
 * @return No return value
 */
void pwm_start(tim_t t, channel_t chann);

/**
 * @brief Stops PWM signal generation
 * 
 * This function disables the compare channel and stops the timer,
 * halting PWM output on the GPIO pin.
 * 
 * @pre PWM must be previously started
 * @post PWM signal is stopped and pin returns to high impedance
 * 
 * @return No return value
 */

void pwm_stop(tim_t t, channel_t chann);

#endif