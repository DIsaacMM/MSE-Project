
#include "Drone.h"


// Initialize the 4 motors

void drone_init(tim_t motor_tim, port_t motor1_gpio, uint8_t motor1_pin, channel_t motor1_channel, )
{
    pwm_init(MOTOR_1_GPIO, MOTOR_TIM, MOTOR_1_PIN);
    pwm_setSignal(MOTOR_TIM, MOTOR_1_CHANNEL, FREQUENCY, ESC_ARM_DUTY);
    pwm_start(MOTOR_TIM, MOTOR_1_CHANNEL);
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
