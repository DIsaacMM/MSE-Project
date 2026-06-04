
#include "Drone.h"


// Motors and there attributes
    motor_t m1 = {
        .pin = 0, 
        .gpio = A,
        .channel = channel_1, 
        .tim = TIM_2, 
        .init = pwm_init, 
        .setSignal = pwm_setSignal, 
        .start = pwm_start, 
        .stop = pwm_stop
    }; 

    motor_t m2 = {
        .pin = 1, 
        .gpio = A,
        .channel = channel_2, 
        .tim = TIM_2, 
        .init = pwm_init, 
        .setSignal = pwm_setSignal, 
        .start = pwm_start, 
        .stop = pwm_stop
    }; 

    motor_t m3 = {
        .pin = 2, 
        .gpio = A,
        .channel = channel_3, 
        .tim = TIM_2, 
        .init = pwm_init, 
        .setSignal = pwm_setSignal, 
        .start = pwm_start, 
        .stop = pwm_stop
    }; 

    motor_t m4 = {
        .pin = 3, 
        .gpio = A,
        .channel = channel_4, 
        .tim = TIM_2, 
        .init = pwm_init, 
        .setSignal = pwm_setSignal, 
        .start = pwm_start, 
        .stop = pwm_stop
    }; 

// Initialize the 4 motors
void drone_init(uint32_t freq, uint8_t esc_min_duty)
{
    // Motor 1
    m1.init(m1.gpio, m1.tim, m1.pin); 
    m1.setSignal(m1.tim, m1.channel, freq, esc_min_duty); 

    // Motor 2
    m2.init(m2.gpio, m2.tim, m2.pin); 
    m2.setSignal(m2.tim, m2.channel, freq, esc_min_duty); 

    // Motor 3
    m3.init(m3.gpio, m3.tim, m3.pin); 
    m3.setSignal(m3.tim, m3.channel, freq, esc_min_duty); 

    // Motor 4
    m4.init(m4.gpio, m4.tim, m4.pin); 
    m4.setSignal(m4.tim, m4.channel,freq, esc_min_duty); 

    // Start all Motors
    m1.start(m1.tim, m1.channel); 
    m2.start(m2.tim, m2.channel);
    m3.start(m3.tim, m3.channel);  
    m4.start(m4.tim, m4.channel); 
}
// Future Implementations
// void drone_up()
// {

// }


// void drone_down()
// {

// }

// void drone_left()
// {

// }

// void drone_right()
// {

// }
