/**
 * @file TIM.c
 * @brief Module that generates delays using hardware timers.
 *
 * This module provides functions to configure and use STM32 timers
 * for delays, frequency generation, and compare modes.
 *
 * @authors David Mijares, Ximena Cedillo, Xavier Clemente
 *
 */

#include "TIM.h" // Header file with timer definitions and macros

/**
 * @brief Initializes the timer pointer array
 * 
 * This function maps logical timer indices to actual hardware timer instances.
 * 
 * @return No return value
 */

TIM_TypeDef* TIM[TIM_SIZE]; 

void tim_init()
{
    TIM[0] = TIM1;   // index 0 to TIM1
    TIM[1] = TIM2;   // index 1 to TIM2
    TIM[2] = TIM3;   // index 2 to TIM3
    TIM[3] = TIM4;   // index 3 to TIM4
    TIM[4] = TIM5;   // index 4 to TIM5
    TIM[5] = TIM9;   // index 5 to TIM9
    TIM[6] = TIM10;  // index 6 to TIM10
    TIM[7] = TIM11;  // index 7 to TIM11
}


/**
 * @brief Enables the clock for a specific timer
 * 
 * This function activates the RCC clock corresponding to the selected timer.
 * It also initializes the timer pointer array.
 * 
 * @param t Timer index
 * @return No return value
 */

void tim_initTimer(tim_t t)
{
    if (t < 0 || t > 7)                         // Validate timer index range   
    {
        return; 
    }
    else if(t < 5)                              // TIM1–TIM5
    {
        if(t == 0)                              // TIM1
        {
            RCC -> APB2ENR |= (1U << t);        // Enable TIM1 clock (bit 0)
        }
        else                                    // TIM2 - TIM5
        {
            RCC -> APB1ENR |= (1U << (t-1));    // Enable TIMx clock (bit 0 to 3)
        }
    }
    else                                        // TIM9 - TIM11
    {        
        RCC -> APB2ENR |= (1U << (t+11));       // Enable TIMx clock (bit 16 to 18)
    }
}

/**
 * @brief Configures a timer to generate a delay in milliseconds
 * 
 * This function sets the prescaler and auto-reload values to produce
 * a delay based on the specified time in milliseconds.
 * 
 * @param t Timer index
 * @param time_ms Time in milliseconds
 * @return No return value
 */

void tim_setTimerMs(tim_t t, uint32_t time_ms)
{
    if (t < 0 || t > 7 || time_ms == 0)                                 // Validate inputs
    {
        return;
    }
    
    TIM[t]->CR1 &= ~(1U << 0);                                          // Disable timer (clear CEN bit)
    
    uint32_t psc = (TIMER_CLOCK_HZ / 1000U) - 1U;                       // Prescaler for 1ms resolution
    uint32_t arr = time_ms - 1U;                                        // Auto-reload value
    
    if (arr > 65535)                                                    // Limit ARR to 16-bit range
    {
        arr = 65535;
    } 
    if (arr < 1) 
    {
        arr = 1;
    }
    
    TIM[t]->PSC = psc;                                                  // Load prescaler value
    TIM[t]->ARR = arr;                                                  // Load auto-reload value
    TIM[t]->EGR |= (1U << 0);                                           // Generate update event (reload registers)
    TIM[t]->SR &= ~(1U << 0);                                           // Clear update flag
}

/**
 * @brief Configures a timer to generate a specific frequency
 * 
 * This function sets the timer to operate at a desired frequency
 * by adjusting prescaler and auto-reload values.
 * 
 * @param t Timer index
 * @param freq_hz Desired frequency in Hz
 * @return No return value
 */
void tim_setTimerFreq(tim_t t, uint32_t freq_hz)
{
    if (t < 0 || t > 7 || freq_hz == 0 || freq_hz > TIMER_CLOCK_HZ)
    {
        return;
    }
    
    TIM[t]->CR1 &= ~(1U << 0);                                          // Disable timer
    
    uint32_t psc = 0;                                                   // Prescaler
    uint32_t arr = 0;                                                   // Auto-reload
    
    // Select configuration depending on frequency range
    if (freq_hz >= 1000)  // High frequencies
    {
        psc = 0;
        arr = (uint16_t)(TIMER_CLOCK_HZ / freq_hz) - 1;
    }
    else  // Low frequencies
    {
        psc = 15999; 
        arr = (uint16_t)((TIMER_CLOCK_HZ / (psc + 1)) / freq_hz) - 1;
    }
    
    // Limit ARR range
    if (arr > 65535)
    {
        arr = 65535;
    } 
    if (arr < 1) 
    {
        arr = 1;
    }
    
    TIM[t]->PSC = psc;                                                     // Set prescaler
    TIM[t]->ARR = arr;                                                     // Set auto-reload
    TIM[t]->EGR |= (1U << 0);                                              // Apply configuration
    TIM[t]->SR &= ~(1U << 0);                                              // Clear flag
}

/**
 * @brief Enables the timer
 * 
 * This function starts the timer counting.
 * 
 * @param t Timer index
 * @return No return value
 */

void tim_enableTimer(tim_t t)
{
    if (t < 0 || t > 7)
    {
        return; 
    }
    TIM[t]->CR1 |= (1U << 0);  // Set CEN bit (enable counter)
}

/**
 * @brief Disables the timer
 * 
 * This function stops the timer counting.
 * 
 * @param t Timer index
 * @return No return value
 */

void tim_disableTimer(tim_t t)
{
    if (t < 0 || t > 7)
    {
        return; 
    }
    TIM[t]->CR1 &= ~(1U << 0);  // Clear CEN bit (disable counter)
}

/**
 * @brief Waits for the timer to overflow (blocking)
 * 
 * This function continuously polls the update flag until
 * the timer completes its cycle.
 * 
 * @param t Timer index
 * @return No return value
 */

void tim_waitTimer(tim_t t)
{
    if (t < 0 || t > 7)
    {
        return; 
    }
    while (!(TIM[t]->SR & (1<<0)))  // Wait for update flag (UIF)
    {
        __NOP();                    // Idle instruction (no operation)
    }
    TIM[t]->SR &= ~(1U << 0);       // Clear update flag
}

/**
 * @brief Sets the compare value for a timer channel
 * 
 * This function writes a value into the corresponding CCR register.
 * 
 * @param t Timer index
 * @param channel Channel number (1 to 4)
 * @param compare_value Value to compare
 * @return No return value
 */

void tim_setTimerCompareChannelValue(tim_t t, uint8_t channel, uint16_t compare_value)
{
    if (t < 0 || t > 7)
    {
        return; 
    }
    if (channel < 1 || channel > 4)
    {
        return;
    }

    switch (channel)
    {
    case 1:
        TIM[t]->CCR1 = compare_value; 
        break;
    
    case 2:
        TIM[t]->CCR2 = compare_value; 
        break;

    case 3:
        TIM[t]->CCR3 = compare_value; 
        break;
    
    case 4:
        TIM[t]->CCR4 = compare_value; 
        break;
    }
}

/**
 * @brief Configures the compare mode of a timer channel
 * 
 * This function sets the output compare mode using CCMR registers.
 * 
 * @param t Timer index
 * @param channel Channel number (1 to 4)
 * @param mode Compare mode (0–7)
 * @return No return value
 */

void tim_setTimerCompareMode(tim_t t, uint8_t channel, uint8_t mode)
{
    if (t < 0 || t > 7)
    {
        return; 
    }
    else if (mode > 7)
    {
        return; 
    }
    if(channel == 1 || channel == 2)
    {
         
        uint8_t shift = (channel - 1) * 8;

        TIM[t]->CCMR1 &= ~(7 << (shift + 4));   // Clear mode bits
        
        TIM[t]->CCMR1 |= (mode << (shift + 4)); // Set mode
        
        TIM[t]->CCMR1 &= ~(3 << shift);         // Configure as output
    }
    else if(channel == 3 || channel == 4)
    {
        uint8_t shift = (channel - 3) * 8;

        TIM[t]->CCMR2 &= ~(7 << (shift + 4));   // Clear mode bits
        
        TIM[t]->CCMR2 |= (mode << (shift + 4)); // Set mode
        
        TIM[t]->CCMR2 &= ~(3 << shift);         // Configure as output
    }
    else
    {
        return; 
    }
}

/**
 * @brief Enables a timer compare channel
 * 
 * This function activates the output of a specific channel.
 * 
 * @param t Timer index
 * @param channel Channel number (1 to 4)
 * @return No return value
 */

void tim_enableTimerCompareChannel(tim_t t, uint8_t channel)
{
    if (t < 0 || t > 7)
    {
        return; 
    }
    if (channel < 1 || channel > 4)
    {
        return;
    }

    uint8_t shift = (channel - 1) * 4;

    TIM[t]->CCER |= (1 << shift);   // Enable channel output

}

/**
 * @brief Disables a timer compare channel
 * 
 * This function deactivates the output of a specific channel.
 * 
 * @param t Timer index
 * @param channel Channel number (1 to 4)
 * @return No return value
 */

void tim_disableTimerCompareChannel(tim_t t, uint8_t channel)
{
    if (t < 0 || t > 7)
    {
        return; 
    }
    if (channel < 1 || channel > 4)
    {
        return;
    }

    uint8_t shift = (channel - 1) * 4;

    TIM[t]->CCER &= ~(1 << shift); // Disable channel output

}