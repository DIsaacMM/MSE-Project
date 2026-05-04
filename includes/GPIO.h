#ifndef GPIO_H  //If GPIO_H is not defined
#define GPIO_H  // define

/**
 * @file GPIO.h
 * @brief Module that manipulates a GPIO for a STM23
 * 
 * The user may use this module with multiple functions to:
 * Initialize a STM32 GPIO port and manipulate a specific pin 
 * 
 * @author David Mijares, Ximena Cedillo, Xavier Clemente
 */

#include <stdint.h>
#include "stm32f4xx.h" 

typedef enum port
 {
    A, 
    B, 
    C, 
    D, 
    E, 
    H,
    GPIOs_SIZE
 }port_t; 


extern GPIO_TypeDef* gpio[GPIOs_SIZE];

/**
 * @brief Initializes the GPIO subsystem
 * 
 * This function shall configure all GPIO ports to a default state. 
 * 
 * @return No return value
 */

void gpio_init(); 

 /**
 * @brief Initializes a specific GPIO port
 * 
 * This function shall enable clocking for the specified port 
 * 
 * @param p The letter or number of the GPIO port that is going to be initialized
 * @return No return value
 */

void gpio_initPort(port_t p); 

 /**
 * @brief Configures the mode of a specific pin.
 * 
 * This function shall allow configuration of pin direction and mode.
 * 
 * @param p The letter or number of the GPIO port that is going to be initialized
 * @param pin unsigned int indicating which pin within the GPIO to modify
 * @param mode unsigned int indicating which mode to assign to the GPIOx pin
 * @return No return value
 */

void gpio_setPinMode(port_t p, uint8_t pin, uint8_t mode);

 /**
 * @brief Sets a pin to logic HIGH.
 * 
 * This function shall drive the specified pin to a high state.
 * 
 * @param p The letter or number of the GPIO port that is going to be initialized
 * @param pin unsigned int indicating which pin within the GPIO to modify
 * @return No return value
 */

void gpio_setPin(port_t p, uint8_t pin);

 /**
 * @brief Clears a pin to logic LOW.
 * 
 * This function shall drive the specified pin to a low state.
 * 
 * @param p The letter or number of the GPIO port that is going to be initialized
 * @param pin unsigned int indicating which pin within the GPIO to modify
 * @return No return value
 */

void gpio_clearPin(port_t p, uint8_t pin);

 /**
 * @brief Toggles the state of a pin.
 * 
 * This function shall invert the current state of the specified pin.
 * 
 * @param p The letter or number of the GPIO port that is going to be initialized
 * @param pin unsigned int indicating which pin within the GPIO to modify
 * @return No return value
 */

void gpio_togglePin(port_t p, uint8_t pin);

 /**
 * @brief Reads the current state of a pin.
 * 
 * This function shall return the digital state of the specified pin.
 * 
 * @param p The letter or number of the GPIO port that is going to be initialized
 * @param pin unsigned int indicating which pin within the GPIO to modify
 * @return The state of the pin (HIGH or LOW)
 */ 

uint8_t gpio_readPin(port_t p, uint8_t pin);

/**
 * @brief Configures the alternate function of a GPIO pin
 * 
 * This function assigns an alternate function (AF) to a specific GPIO pin.
 * Alternate functions allow pins to be used by peripherals such as timers,
 * UART, SPI, I2C, etc.
 * 
 * @param p GPIO port identifier
 * @param pin Pin number (0 to 15)
 * @param alternate_mode Alternate function number (0 to 15)
 * @return No return value
 */

void gpio_setAlternateFunction(port_t p, uint8_t pin, uint8_t alternate_mode); 

#endif