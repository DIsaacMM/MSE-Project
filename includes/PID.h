/**
 * @file PID.h
 * @brief PID Balance Controller using MPU-6050
 *
 * Módulo PID para estabilización de dron.
 * Compatible con los drivers PWM, Sensor y Timer existentes.
 *
 * Motores:
 *   Motor 1 → TIM2 CH1 (PA0)
 *   Motor 2 → TIM2 CH2 (PA1)
 *   Motor 3 → TIM4 CH1 (PB6)
 *   Motor 4 → TIM4 CH2 (PB7)
 *
 * @authors
 * David Mijares
 * Aldo De la Torre
 * Jose Paez
 */

#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <stddef.h>
#include "PWM.h"
#include "MPU6050.h"
#include "Timer.h"
#include "Drone.h"
/* ===========================================================================
 * CONSTANTES PID — ajusta estos valores para tu dron
 * =========================================================================== */
#define PID_KP              3.55f
#define PID_KI              0.005f
#define PID_KD              2.05f

#define PID_DESIRED_ANGLE   0.0f    /* ángulo objetivo en grados            */
#define PID_THROTTLE        1300.0f /* throttle base (us) enviado a los ESC */

#define PID_INTEGRAL_LIMIT  300.0f  /* anti-windup del integrador           */
#define PID_OUTPUT_LIMIT    1000.0f /* límite del valor PID total           */

#define PID_ESC_MIN         1000    /* pulso mínimo ESC (us)                */
#define PID_ESC_MAX         2000    /* pulso máximo ESC (us)                */

// Timer used for delays
#define DELAY_TIM TIM_3

/**
 * @brief Inicializa el módulo PID.
 *
 * Llama esta función UNA SOLA VEZ en main(), antes del while(1),
 * DESPUÉS de haber inicializado PWM y Timer.
 *
 * Envía pulso mínimo a los 4 motores y espera el armado de los ESC.
 */
void PID_Init(void);

/**
 * @brief Ejecuta un ciclo completo de lectura IMU + cálculo PID + salida PWM.
 *
 * Llama esta función en cada iteración del while(1).
 * El tiempo entre llamadas determina elapsedTime.
 */
void PID_Update(void);

#endif /* PID_H */