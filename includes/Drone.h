
#ifndef DRONE_H
#define DRONE_H

#include "PWM.h"
#include "Sensor.h"
#include "Timer.h"

typedef void (*motor_init_t)(port_t, tim_t, uint8_t);     
typedef void (*motor_setSignal_t)(tim_t, channel_t, uint32_t, uint16_t);   
typedef void (*motor_start_t)(tim_t, channel_t); 
typedef void (*motor_stop_t)(tim_t, channel_t); 
typedef struct motor
{
    // Motor Variables
    uint8_t pin;
    port_t gpio;
    channel_t channel; 
    tim_t tim; 
    
    // Motor functions
    motor_init_t init; 
    motor_setSignal_t setSignal; 
    motor_start_t start; 
    motor_stop_t stop; 
}motor_t;

// Global motors
extern motor_t m1; 
extern motor_t m2; 
extern motor_t m3; 
extern motor_t m4; 

void drone_init(uint32_t freq, uint8_t esc_min_duty);

#endif 