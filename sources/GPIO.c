/**
 * @file GPIO.c
 * @brief Module that manipulates a GPIO and a pin from a STM32
 * 
 * The user may use this module with multiple functions to:
 * Initialize a STM32 GPIO port and manipulate a specific pin 
 *
 * @authors David Mijares, Ximena Cedillo, Xavier Clemente
 *
 */
#include "GPIO.h"

GPIO_TypeDef* gpio[GPIOs_SIZE]; 

/**
 * @brief Initializes the GPIO subsystem
 * 
 * This function shall configure all GPIO ports to a default state. 
 * 
 * @return No return value
 */

void gpio_init()
{
    // Assign the gpio array positions to the STM32 GPIOs
    gpio[0] = GPIOA;
    gpio[1] = GPIOB;
    gpio[2] = GPIOC;
    gpio[3] = GPIOD;
    gpio[4] = GPIOE;
    gpio[5] = GPIOH;
}

 /**
 * @brief Initializes a specific GPIO port
 * 
 * This function shall enable clocking for the specified port 
 * 
 * @param p The letter or number of the GPIO port that is going to be initialized
 * @return No return value
 */

void gpio_initPort(port_t p)
{
    // If not within the range of the Micro's GPIOs, exit
    if(p < 0 || p > 6)
    {
        return;
    }
    else if(p == H || p == 5)
    {
        p = 7; 
    }
    
    gpio_init(); 
    
    // Enable the clock for the selected Port
    RCC->AHB1ENR |= (1UL << p);   
}

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

void gpio_setPinMode(port_t p, uint8_t pin, uint8_t mode)
{
    // If not within the range of the Micro's GPIOs, exit
    if(p < 0 || p > 6)
    {
        return;
    }

    // If mode exceeds the number of options in MODER 
    if(mode > 3)
    {
        return; 
    }

    // If the pin exceeds the number of pins in GPIOx
    if(pin > 15)
    {
        return; 
    }
    
    // First clear what is in the GPIOx MODER
    gpio[p]->MODER &= ~(3 << (pin*2));
    
    // Assign the entered mode to the entered pin
    // Pin is multiplied by two because each pin uses two bits in MODE to indicate which mode to set
    // To modify each pin, you have to consider that each pin uses two bits
    gpio[p]->MODER |= (mode << (pin*2)); 
}

 /**
 * @brief Sets a pin to logic HIGH.
 * 
 * This function shall drive the specified pin to a high state.
 * 
 * @param p The letter or number of the GPIO port that is going to be initialized
 * @param pin unsigned int indicating which pin within the GPIO to modify
 * @return No return value
 */

void gpio_setPin(port_t p, uint8_t pin)
{
    // If not within the range of the Micro's GPIOs, exit
    if(p < 0 || p > 6)
    {
        return;
    }
    // If the pin exceeds the number of pins in GPIOx
    if(pin > 15)
    {
        return; 
    }
    
    // Use the lower Bit set registers to set the indicated pin of GPIOx to 1
    gpio[p]->BSRR = (1 << pin); 
}

 /**
 * @brief Clears a pin to logic LOW.
 * 
 * This function shall drive the specified pin to a low state.
 * 
 * @param p The letter or number of the GPIO port that is going to be initialized
 * @param pin unsigned int indicating which pin within the GPIO to modify
 * @return No return value
 */

void gpio_clearPin(port_t p, uint8_t pin)
{
    // If not within the range of the Micro's GPIOs, exit
    if(p < 0 || p > 6)
    {
        return;
    }
    // If the pin exceeds the number of pins in GPIOx
    if(pin > 15)
    {
        return; 
    }

    // Use the upper Bit set registers to set the indicated pin of GPIOx to 1
    // so 16 is added to the entered pin because that's how BSRR works
    gpio[p]->BSRR = (1 << (pin + 16)); 
}

 /**
 * @brief Toggles the state of a pin.
 * 
 * This function shall invert the current state of the specified pin.
 * 
 * @param p The letter or number of the GPIO port that is going to be initialized
 * @param pin unsigned int indicating which pin within the GPIO to modify
 * @return No return value
 */

void gpio_togglePin(port_t p, uint8_t pin)
{
    // If not within the range of the Micro's GPIOs, exit
    if(p < 0 || p > 6)
    {
        return;
    }
    // If the pin exceeds the number of pins in GPIOx
    if(pin > 15)
    {
        return; 
    }
    // XOR the pin value to change it from 0 to 1 and 1 to 0
    gpio[p]->ODR ^= (1 << pin);
}

 /**
 * @brief Reads the current state of a pin.
 * 
 * This function shall return the digital state of the specified pin.
 * 
 * @param p The letter or number of the GPIO port that is going to be initialized
 * @param pin unsigned int indicating which pin within the GPIO to modify
 * @return The state of the pin (HIGH or LOW)
 */ 

uint8_t gpio_readPin(port_t p, uint8_t pin)
{
    // If within the range of the Micro's GPIOs and pins, exit
    if(0 < p && p < 6 && pin < 16)
    {
        // Check if the selected pin in IDR is 1
        if(gpio[p]->IDR & (1 << pin))
        {
            return 1; // Returns 1 if it is 1
        } 
        else 
        {
            return 0; // Returns 0 if it is 0
        }
    }
    else
    {
        return 0; 
    }
}

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

void gpio_setAlternateFunction(port_t p, uint8_t pin, uint8_t alternate_mode)
{
    // If not within the range of the Micro's GPIOs, exit
    if(p < 0 || p > 6)
    {
        return;
    }

    // If the pin exceeds the number of pins in GPIOx
    if(pin > 15)
    {
        return; 
    }
    
    // First clear what is in the GPIOx MODER
    gpio[p]->MODER &= ~(3 << (pin*2));
    
    // Assign the alternate function mode to the entered pin
    // Pin is multiplied by two because each pin uses two bits in MODE to indicate which mode to set
    // To modify each pin, you have to consider that each pin uses two bits
    gpio[p]->MODER |= (2 << (pin*2));     


    if(pin < 8) 
    {
        gpio[p]->AFR[0] &= ~(15 << (pin * 4));
        gpio[p]->AFR[0] |= (alternate_mode << (pin * 4));
    } 
    else {
        gpio[p]->AFR[1] &= ~(15 << ((pin - 8) * 4));
        gpio[p]->AFR[1] |= (alternate_mode << ((pin - 8) * 4));
    }
}