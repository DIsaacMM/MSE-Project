/**
 * pid_controller.h
 * Portado y adaptado desde Betaflight 4.5.1 (GPL-3.0) — src/main/flight/pid.c
 * Target: STM32F411RE Nucleo bare-metal
 *
 * CAMBIOS vs version anterior:
 *   - pidUpdate() recibe throttle para TPA interno
 *   - Agregados parametros TPA: TPA_BREAKPOINT, TPA_MIN
 *   - Agregado parametro ITERM_RELAX_THRESHOLD_DPS
 *   - Eliminado ITERM_ZONE_DEG (zona muerta manual en main.c — ya no necesaria)
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "gyro_filter.h"
#include "math_utils.h"

/* =========================================================
 *  CONSTANTES DE CONFIGURACION
 * ========================================================= */

#define PID_AXIS_ROLL   0
#define PID_AXIS_PITCH  1
#define PID_AXIS_YAW    2
#define PID_AXIS_COUNT  3

/* Limites de salida PID por eje */
#define PID_SUM_LIMIT       500.0f
#define PID_SUM_LIMIT_YAW   400.0f

/* Limites del integrador (anti-windup absoluto) */
#define PID_ITERM_LIMIT     400.0f
#define PID_ITERM_LIMIT_YAW 300.0f

/* Frecuencia y dt del loop PID */
#define PID_LOOP_HZ     1000
#define PID_DT          (1.0f / PID_LOOP_HZ)

/* Frecuencia de corte del filtro D-term (PT1)
 * Conservador para 7" con MPU6050 (mas ruido que ICM-42688) */
#define DTERM_LPF_HZ    20.0f

/* ── iterm_relax ──────────────────────────────────────────
 * Congela el integrador cuando el error de rata supera este
 * umbral. Evita que el I acumule transitorios del lazo externo
 * (saltos bruscos en setpoint cuando el angulo cambia rapido).
 * Betaflight: iterm_relax_cutoff — aqui usamos umbral fijo.
 * Ajustar: bajar si el I sigue creciendo en transitorios,
 *          subir si el I no responde suficiente en hover. */
#define ITERM_RELAX_THRESHOLD_DPS   60.0f

/* ── TPA (Throttle PID Attenuation) ──────────────────────
 * Reduce Kp y Kd linealmente entre TPA_BREAKPOINT y ESC_MAX_US.
 * En hover (throttle ~ THROTTLE_BASE) el factor es 1.0 (sin cambio).
 * A plena potencia el factor baja a TPA_MIN.
 * Betaflight default: TPA 0.65 desde throttle 1250.
 * Para el 7" empezar conservador: 0.8 desde 1300. */
#define TPA_BREAKPOINT  1300.0f
#define TPA_MIN         0.80f
#define ESC_MAX_US_F    2000.0f   /* float version para calculos TPA */

/* =========================================================
 *  GANANCIAS PID — PUNTO DE PARTIDA SPECTER 7"
 *
 *  Proceso de tuning sugerido:
 *  1. Kd=0, Ki=0 — subir Kp hasta oscilar, bajar 30%
 *  2. Subir Kd hasta amortiguar el rebote
 *  3. Subir Ki hasta eliminar deriva en hover
 * ========================================================= */

/* Roll (eje X) */
#define PID_KP_ROLL     1.250f
#define PID_KI_ROLL     0.040f    /* Ki=0 durante tuning inicial */
#define PID_KD_ROLL     0.010f

/* Pitch (eje Y) */
#define PID_KP_PITCH    1.250f
#define PID_KI_PITCH    0.040f    /* Ki=0 durante tuning inicial */
#define PID_KD_PITCH    0.010f

/* Yaw (eje Z) */
#define PID_KP_YAW      0.40f    /* Mucha más fuerza inmediata para frenar la rotación */
#define PID_KI_YAW      0.060f   /* Más memoria a largo plazo para vencer desbalances físicos */
#define PID_KD_YAW      0.0f     /* Mantener en 0, muy bien */

/* =========================================================
 *  TIPOS DE DATOS
 * ========================================================= */

typedef struct {
    float Kp;
    float Ki;
    float Kd;
} PidGains_t;

typedef struct {
    float P;      /* termino proporcional                  */
    float I;      /* termino integral (con anti-windup)    */
    float D;      /* termino derivativo                    */
    float F;      /* feedforward (0 si no se usa)          */
    float sum;    /* P+I+D+F limitado a +-pidSumLimit      */
} PidAxisData_t;

typedef struct {
    PidGains_t      gains[PID_AXIS_COUNT];
    PidAxisData_t   output[PID_AXIS_COUNT];

    float           previousGyroRate[PID_AXIS_COUNT];
    PT1Filter_t     dtermLpf[PID_AXIS_COUNT];

    float           pidSumLimit[PID_AXIS_COUNT];
    float           itermLimit[PID_AXIS_COUNT];

    float           dt;
    float           pidFrequency;
} PidState_t;

/* =========================================================
 *  API PUBLICA
 * ========================================================= */

void pidInit(PidState_t *pid);

void pidSetGains(PidState_t *pid, int axis, float Kp, float Ki, float Kd);

void pidResetIterm(PidState_t *pid);

/**
 * pidUpdate — calcular una iteracion del PID para los 3 ejes.
 *
 * @throttle : throttle actual en us (ESC_MIN_US..ESC_MAX_US).
 *             Usado internamente para calcular el factor TPA.
 *             Pasar throttleNow desde main.c.
 */
void pidUpdate(PidState_t *pid,
               float setpointRoll,  float setpointPitch,  float setpointYaw,
               float gyroRoll,      float gyroPitch,      float gyroYaw,
               float throttle);

static inline float pidConstrainf(float val, float min, float max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

#endif /* PID_CONTROLLER_H */