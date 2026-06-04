/**
 * pid_controller.h
 * Portado y adaptado desde Betaflight 4.5.1 (GPL-3.0) — src/main/flight/pid.c
 * Target: STM32F411RE Nucleo bare-metal
 *
 * ARQUITECTURA DEL PID PARA DRON:
 * ================================
 * Betaflight usa un PID de "2DOF" (2 degrees of freedom) en modo rata (rate/acro):
 *
 *   setpoint (°/s)  ──┐
 *                     ▼
 *                   error = setpoint - gyroRate
 *                     │
 *          ┌──────────┼──────────┐
 *          │          │          │
 *         [P]        [I]        [D]
 *          │          │          │
 *   Kp*error   Ki*∫error*dt   Kd*d(gyroRate)/dt
 *          │          │          │
 *          └──────────┴──────────┘
 *                     │
 *               pidSum (limitado)
 *                     │
 *               → mixer → motores
 *
 * Tres ejes independientes: ROLL (X), PITCH (Y), YAW (Z)
 *
 * DIFERENCIA CLAVE vs PID clásico:
 *   - El término D se aplica sobre -d(gyroRate)/dt, NO sobre d(error)/dt.
 *     Esto evita los "spikes" del D cuando el setpoint cambia bruscamente
 *     (por ejemplo, cuando mueves el stick rápidamente).
 *     Betaflight: delta = -(gyroRateDterm[axis] - previousGyroRateDterm[axis]) * pidFrequency
 *
 *   - Anti-windup del integrador: el I está limitado a ±itermLimit.
 *
 *   - El término D tiene su propio pipeline de filtros (dterm LPF)
 *     que se aplica ANTES de calcular la derivada.
 *     Aquí usamos PT1 para simplificar (suficiente para F411 a 1kHz).
 *
 * PARÁMETROS TÍPICOS PARA UN 5" (valores iniciales conservadores):
 *   Roll/Pitch: Kp=45, Ki=80, Kd=25  (en escala Betaflight ×1000)
 *   Yaw:        Kp=45, Ki=80, Kd=0
 *   En esta implementación usamos Kp/Ki/Kd directamente como floats.
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "gyro_filter.h"   /* PT1Filter_t, PT2Filter_t */
#include "math_utils.h"     /* floatConstrain */

/* =========================================================
 *  CONSTANTES DE CONFIGURACIÓN
 * ========================================================= */

/* Número de ejes */
#define PID_AXIS_ROLL   0
#define PID_AXIS_PITCH  1
#define PID_AXIS_YAW    2
#define PID_AXIS_COUNT  3

/* Límite del la salida PID total por eje (en unidades de "motor command")
 * Betaflight: PIDSUM_LIMIT = 500, PIDSUM_LIMIT_YAW = 400
 * En nuestro sistema: salida PID va de -500 a +500, luego se suma al throttle base */
#define PID_SUM_LIMIT       500.0f
#define PID_SUM_LIMIT_YAW   400.0f

/* Límite del integrador (anti-windup)
 * Betaflight: itermLimit = 400 (80% de PID_SUM_LIMIT) */
#define PID_ITERM_LIMIT     400.0f
#define PID_ITERM_LIMIT_YAW 300.0f

/* Frecuencia del loop PID y dt */
#define PID_LOOP_HZ     1000
#define PID_DT          (1.0f / PID_LOOP_HZ)

/* Frecuencia de corte del filtro D-term (PT1)
 * Betaflight: dterm_lpf1_static_hz = 75 (modo dinámico)
 * Para empezar, usar 80Hz que es un valor conservador */
#define DTERM_LPF_HZ    80.0f

/* =========================================================
 *  GANANCIAS PID — PUNTO DE PARTIDA CONSERVADOR
 *
 *  Escala Betaflight: los valores en la UI se dividen internamente.
 *  Aquí usamos los valores directamente en rad/s para mayor claridad.
 *
 *  IMPORTANTE: Empezar con valores BAJOS y subir de a poco.
 *  Con hélices puestas, un Kp demasiado alto = oscilaciones = crash.
 *
 *  Proceso de tuning sugerido:
 *  1. Kd=0, Ki=0, subir Kp hasta que empiece a oscilar, bajar 30%
 *  2. Subir Kd hasta que amortigüe las oscilaciones
 *  3. Subir Ki hasta que elimine el error en estado estacionario
 * ========================================================= */

/* Roll (eje X) */
#define PID_KP_ROLL     0.05f    /* Prueba de banco: escala directa °→output */
#define PID_KI_ROLL     0.0f     /* Ki=0 durante prueba de banco             */
#define PID_KD_ROLL     0.0f     /* Kd=0 durante prueba de banco             */

/* Pitch (eje Y) — normalmente igual a Roll */
#define PID_KP_PITCH    0.05f
#define PID_KI_PITCH    0.0f
#define PID_KD_PITCH    0.0f

/* Yaw (eje Z) — generalmente sin D-term */
#define PID_KP_YAW      0.040f
#define PID_KI_YAW      0.060f
#define PID_KD_YAW      0.0f    /* D en yaw causa zumbido con motores brushless */

/* =========================================================
 *  TIPOS DE DATOS
 * ========================================================= */

/** Ganancias de un eje */
typedef struct {
    float Kp;
    float Ki;
    float Kd;
} PidGains_t;

/** Salida PID de un eje — separada por término para debug/blackbox */
typedef struct {
    float P;        /* término proporcional                 */
    float I;        /* término integral (con anti-windup)   */
    float D;        /* término derivativo                   */
    float F;        /* feedforward (0 si no se usa)         */
    float sum;      /* P+I+D+F limitado a ±pidSumLimit      */
} PidAxisData_t;

/** Estado completo del controlador PID */
typedef struct {
    PidGains_t      gains[PID_AXIS_COUNT];
    PidAxisData_t   output[PID_AXIS_COUNT];

    /* Estado interno por eje */
    float   previousGyroRate[PID_AXIS_COUNT];   /* para calcular derivada   */
    PT1Filter_t dtermLpf[PID_AXIS_COUNT];        /* filtro del D-term        */

    /* Límites */
    float   pidSumLimit[PID_AXIS_COUNT];
    float   itermLimit[PID_AXIS_COUNT];

    /* dt del loop */
    float   dt;
    float   pidFrequency;   /* = 1/dt, para escalar la derivada */
} PidState_t;

/* =========================================================
 *  API PÚBLICA
 * ========================================================= */

/**
 * pidInit — inicializar el controlador con ganancias por defecto.
 * Llamar UNA VEZ antes del loop.
 */
void pidInit(PidState_t *pid);

/**
 * pidSetGains — actualizar ganancias en runtime (para tuning).
 * @axis: PID_AXIS_ROLL, PID_AXIS_PITCH o PID_AXIS_YAW
 */
void pidSetGains(PidState_t *pid, int axis, float Kp, float Ki, float Kd);

/**
 * pidResetIterm — resetear el integrador de todos los ejes.
 * Llamar al armar o al aterrizar para evitar windup acumulado.
 */
void pidResetIterm(PidState_t *pid);

/**
 * pidUpdate — calcular una iteración del PID para los 3 ejes.
 *
 * @pid             : estado del controlador
 * @setpointRoll    : setpoint del roll en °/s (de los sticks del RC)
 * @setpointPitch   : setpoint del pitch en °/s
 * @setpointYaw     : setpoint del yaw en °/s
 * @gyroRoll        : velocidad angular medida y filtrada, roll, en °/s
 * @gyroPitch       : idem pitch
 * @gyroYaw         : idem yaw
 *
 * Salida disponible en pid->output[axis].sum (y .P, .I, .D por separado).
 * Rango de salida: [-pidSumLimit, +pidSumLimit] por eje.
 */
void pidUpdate(PidState_t *pid,
               float setpointRoll,  float setpointPitch,  float setpointYaw,
               float gyroRoll,      float gyroPitch,      float gyroYaw);

/**
 * pidConstrainf — clamp de un float entre min y max.
 * Útil para limitar salidas al mixer.
 */
static inline float pidConstrainf(float val, float min, float max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

#endif /* PID_CONTROLLER_H */