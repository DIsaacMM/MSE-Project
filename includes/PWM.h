#ifndef PWM_H
#define PWM_H

/**
 * @file PWM.h
 * @brief PWM Driver for ESC Servo PWM Control
 */

#include "TIM.h"
#include "GPIO.h"
#include "Timer.h"

#define ALTERNATE_FUNC_TIM2 1

// PWM MODE 1
#define PWM_MODE 6

typedef enum pwm_channel
{
    channel_1 = 1,
    channel_2 = 2,
    channel_3 = 3,
    channel_4 = 4,
    SIZE = 5
} channel_t;

void pwm_init(port_t p, tim_t t, uint8_t pin);

void pwm_setSignal(
    tim_t t,
    channel_t chann,
    uint32_t frecuency,
    uint8_t duty_cycle
);

void pwm_start(tim_t t, channel_t chann);

void pwm_stop(tim_t t, channel_t chann);

#endif