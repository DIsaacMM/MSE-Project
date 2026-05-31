/**
 * @file PWM.c
 * @brief PWM Driver for ESC Servo PWM
 *
 * This module implements PWM generation
 * using STM32 hardware timers.
 *
 * The PWM signal is configured for:
 *
 *  Frequency:
 *      50 Hz
 *
 *  Pulse Width:
 *      1000 us -> minimum throttle
 *      2000 us -> maximum throttle
 *
 * PWM generation uses:
 *
 *  - PWM Mode 1
 *  - Timer Compare Channels
 *  - Hardware timer outputs
 *
 * @authors
 * David Mijares
 * Aldo De la Torre
 * Jose Paez
 */

#include "PWM.h"

/**
 * @brief Initialize PWM GPIO and Timer
 *
 * This function:
 *
 * 1. Initializes GPIO subsystem
 * 2. Enables GPIO port
 * 3. Configures GPIO pin as Alternate Function
 * 4. Connects GPIO pin to TIM2
 * 5. Initializes timer subsystem
 * 6. Enables selected timer
 *
 * @param p
 * GPIO port
 *
 * @param t
 * Timer module
 *
 * @param pin
 * GPIO pin number
 *
 * @return
 * No return value
 */
void pwm_init(port_t p, tim_t t, uint8_t pin)
{
    // Initialize GPIO subsystem
    gpio_init();

    // Enable selected GPIO port
    gpio_initPort(p);

    // Variable that stores the Alternate Function value
    uint8_t af = 0;

    // =========================================
    // SELECT ALTERNATE FUNCTION
    // =========================================

    // TIM1 and TIM2 use AF1
    if(t == TIM_1 || t == TIM_2)
    {
        af = 1;
    }

    // TIM3, TIM4 and TIM5 use AF2
    else if(t == TIM_3 || t == TIM_4 || t == TIM_5)
    {
        af = 2;
    }

    // TIM8, TIM9, TIM10 and TIM11 use AF3
    else if(t == TIM_9  || t == TIM_10 || t == TIM_11)
    {
        af = 3;
    }


    // Invalid timer
    else
    {
        return;
    }

    // =========================================
    // CONFIGURE GPIO ALTERNATE FUNCTION
    // =========================================

    gpio_setAlternateFunction(p, pin, af);

    // =========================================
    // INITIALIZE TIMER
    // =========================================

    tim_init();

    // Enable selected timer
    tim_initTimer(t);

    /* Configurar timer para PWM 50Hz — solo se hace una vez aquí.
     * PSC=15 → tick=1μs | ARR=19999 → período=20ms=50Hz */
    TIM[t]->CR1  &= ~(1U << 0);    /* detener para configurar         */
    TIM[t]->PSC   = 15;
    TIM[t]->ARR   = 20000 - 1;

    /* Configurar modo PWM Mode 1 en el canal correspondiente */
    /* Canal 1 */
    TIM[t]->CCMR1 &= ~(7U << 4);
    TIM[t]->CCMR1 |=  (6U << 4);
    TIM[t]->CCMR1 |=  (1U << 3);   /* OC1PE preload */
    TIM[t]->CCER  |=  (1U << 0);   /* CC1E output enable */
    TIM[t]->CCR1   = 1000;          /* valor inicial: 1000μs (mínimo) */

    /* Canal 2 */
    TIM[t]->CCMR1 &= ~(7U << 12);
    TIM[t]->CCMR1 |=  (6U << 12);
    TIM[t]->CCMR1 |=  (1U << 11);  /* OC2PE preload */
    TIM[t]->CCER  |=  (1U << 4);   /* CC2E output enable */
    TIM[t]->CCR2   = 1000;

    /* Canal 3 */
    TIM[t]->CCMR2 &= ~(7U << 4);
    TIM[t]->CCMR2 |=  (6U << 4);
    TIM[t]->CCMR2 |=  (1U << 3);   /* OC3PE preload */
    TIM[t]->CCER  |=  (1U << 8);   /* CC3E output enable */
    TIM[t]->CCR3   = 1000;

    /* Canal 4 */
    TIM[t]->CCMR2 &= ~(7U << 12);
    TIM[t]->CCMR2 |=  (6U << 12);
    TIM[t]->CCMR2 |=  (1U << 11);  /* OC4PE preload */
    TIM[t]->CCER  |=  (1U << 12);  /* CC4E output enable */
    TIM[t]->CCR4   = 1000;

    TIM[t]->CR1  |=  (1U << 7);    /* ARPE: ARR preload enable        */
    TIM[t]->EGR  |=  (1U << 0);    /* UG: aplicar PSC/ARR ahora       */
    TIM[t]->SR   &= ~(1U << 0);    /* limpiar flag de update          */
    TIM[t]->CR1  |=  (1U << 0);    /* CEN: iniciar contador           */
}

/**
 * @brief Configure PWM signal for ESC control
 *
 * This function configures:
 *
 *  - Timer prescaler
 *  - Timer auto-reload register
 *  - PWM Mode 1
 *  - Compare register value
 *  - PWM output channel
 *
 * Timer configuration:
 *
 *  Timer Clock:
 *      16 MHz
 *
 *  Prescaler:
 *      15
 *
 *  Tick Resolution:
 *      1 us
 *
 *  ARR:
 *      19999
 *
 *  PWM Frequency:
 *      50 Hz
 *
 * Duty cycle mapping:
 *
 *      5  -> 1000 us
 *      10 -> 2000 us
 *
 * @param t
 * Timer module
 *
 * @param chann
 * Timer channel
 *
 * @param frecuency
 * PWM frequency
 *
 * @param duty_cycle
 * PWM duty cycle
 *
 * @return
 * No return value
 */

void pwm_setSignal(tim_t t, channel_t chann, uint32_t frecuency, uint16_t pulse_us)
{
    (void)frecuency;

    /* Clamp — nunca salir del rango ESC estándar */
    if (pulse_us < 1000) pulse_us = 1000;
    if (pulse_us > 2000) pulse_us = 2000;

    /* Solo actualizar el registro de comparación (CCR).
     * PSC, ARR, CCMR y EGR se configuran UNA SOLA VEZ en pwm_init.
     * Tocarlos aquí puede causar glitches en la señal activa. */
    switch(chann)
    {
        case channel_1:  TIM[t]->CCR1 = pulse_us; break;
        case channel_2:  TIM[t]->CCR2 = pulse_us; break;
        case channel_3:  TIM[t]->CCR3 = pulse_us; break;
        case channel_4:  TIM[t]->CCR4 = pulse_us; break;
        default: return;
    }
}


/**
 * @brief Start PWM output
 *
 * PWM is already enabled inside
 * pwm_setSignal().
 *
 * This function remains for compatibility.
 *
 * @param t
 * Timer module
 *
 * @param chann
 * Timer channel
 *
 * @return
 * No return value
 */
void pwm_start(tim_t t, channel_t chann)
{
    // Unused parameters
    (void)t;
    (void)chann;
}

/**
 * @brief Stop PWM output
 *
 * This function disables:
 *
 *  - PWM output channel
 *  - Timer counter
 *
 * @param t
 * Timer module
 *
 * @param chann
 * Timer channel
 *
 * @return
 * No return value
 */
void pwm_stop(tim_t t, channel_t chann)
{
    // ==================================================
    // DISABLE PWM CHANNEL
    // ==================================================

    switch (chann)
    {
        // Disable Channel 1
        case channel_1:
            TIM[t]->CCER &= ~(1U << 0);
            break;

        // Disable Channel 2
        case channel_2:
            TIM[t]->CCER &= ~(1U << 4);
            break;

        // Disable Channel 3
        case channel_3:
            TIM[t]->CCER &= ~(1U << 8);
            break;

        // Disable Channel 4
        case channel_4:
            TIM[t]->CCER &= ~(1U << 12);
            break;

        // Invalid channel
        default:
            return;
    }

    // ==================================================
    // DISABLE TIMER
    // ==================================================

    // Stop timer counter
    TIM[t]->CR1 &= ~(1U << 0);
}