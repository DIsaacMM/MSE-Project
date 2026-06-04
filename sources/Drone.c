
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
void drone_init()
{
    // Initialize PWM for all Motors 
    m1.init(m1.gpio, m1.tim, m1.pin); 
    m2.init(m2.gpio, m2.tim, m2.pin);
    m3.init(m3.gpio, m3.tim, m3.pin); 
    m4.init(m4.gpio, m4.tim, m4.pin); 

    // Set PWM signal for all motors using 50Hz and the min value for the ESC 1000 us
    m1.setSignal(m1.tim, m1.channel, 50, 1000); 
    m2.setSignal(m2.tim, m2.channel, 50, 1000); 
    m3.setSignal(m3.tim, m3.channel, 50, 1000); 
    m4.setSignal(m4.tim, m4.channel,50, 1000); 

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
