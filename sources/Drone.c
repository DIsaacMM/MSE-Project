
#include "Drone.h"


// Motors and there attributes
    motor_t m1 = 
    {
        .pin = 0, 
        .gpio = A,
        .channel = channel_1, 
        .tim = TIM_2, 
        .init = pwm_init, 
        .setSignal = pwm_setSignal, 
        .start = pwm_start, 
        .stop = pwm_stop
    }; 

    motor_t m2 = 
    {
        .pin = 1, 
        .gpio = A,
        .channel = channel_2, 
        .tim = TIM_2, 
        .init = pwm_init, 
        .setSignal = pwm_setSignal, 
        .start = pwm_start, 
        .stop = pwm_stop
    }; 

    motor_t m3 = 
    {
        .pin = 6, 
        .gpio = B,
        .channel = channel_1, 
        .tim = TIM_4, 
        .init = pwm_init, 
        .setSignal = pwm_setSignal, 
        .start = pwm_start, 
        .stop = pwm_stop
    }; 

    motor_t m4 = 
    {
        .pin = 7, 
        .gpio = B,
        .channel = channel_2, 
        .tim = TIM_4, 
        .init = pwm_init, 
        .setSignal = pwm_setSignal, 
        .start = pwm_start, 
        .stop = pwm_stop
    }; 

// Initialize the 4 motors
void drone_init()
{
    // Init PWM for al motors
    m1.init(m1.gpio, m1.tim, m1.pin); 
    m2.init(m2.gpio, m2.tim, m2.pin); 
    m3.init(m3.gpio, m3.tim, m3.pin); 
    m4.init(m4.gpio, m4.tim, m4.pin); 

    // ==================================================
    // ARM ESC
    // ==================================================

    // Send minimum throttle signal 1000us and Frecuency of 50Hz

    m1.setSignal(m1.tim, m1.channel, 50, 1000); 
    m2.setSignal(m2.tim, m2.channel, 50, 1000); 
    m3.setSignal(m3.tim, m3.channel, 50, 1000); 
    m4.setSignal(m4.tim, m4.channel,50, 1000); 

    // Start PWM generation for all motors
    m1.start(m1.tim, m1.channel); 
    m2.start(m2.tim, m2.channel);
    m3.start(m3.tim, m3.channel);  
    m4.start(m4.tim, m4.channel); 

}

void drone_up()
{

}


void drone_down()
{

}

void drone_left()
{

}

void drone_right()
{

}
