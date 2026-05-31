/**
 * pid_controller.c
 * Portado y adaptado desde Betaflight 4.5.1 (GPL-3.0) — src/main/flight/pid.c
 *
 * Función principal portada: pidController() → pidUpdate()
 *
 * Simplificaciones vs Betaflight original:
 *   - Sin TPA (Throttle PID Attenuation) — se puede agregar después
 *   - Sin anti-gravity (boost de iTerm en throttle rápido)
 *   - Sin iterm_rotation (compensación de rotación de iTerm)
 *   - Sin iterm_relax (supresión de iTerm en inputs rápidos)
 *   - Sin feedforward de RC
 *   - Sin D-max (ganancia variable del D)
 *   - Sin crash recovery
 *   - Sin launch control
 *   - Sin RPM filter
 *   El resultado es el núcleo mínimo funcional y estable para comenzar.
 *
 * El algoritmo D-term usa derivada sobre el proceso (gyro), NO sobre el error.
 * Esto es idéntico a Betaflight y evita el "derivative kick" al mover sticks.
 */

#include "pid_controller.h"

/* =========================================================
 *  INICIALIZACIÓN
 * ========================================================= */

void pidInit(PidState_t *pid)
{
    memset(pid, 0, sizeof(PidState_t));

    pid->dt           = PID_DT;
    pid->pidFrequency = (float)PID_LOOP_HZ;

    /* --- Ganancias por defecto --- */
    pid->gains[PID_AXIS_ROLL].Kp  = PID_KP_ROLL;
    pid->gains[PID_AXIS_ROLL].Ki  = PID_KI_ROLL;
    pid->gains[PID_AXIS_ROLL].Kd  = PID_KD_ROLL;

    pid->gains[PID_AXIS_PITCH].Kp = PID_KP_PITCH;
    pid->gains[PID_AXIS_PITCH].Ki = PID_KI_PITCH;
    pid->gains[PID_AXIS_PITCH].Kd = PID_KD_PITCH;

    pid->gains[PID_AXIS_YAW].Kp   = PID_KP_YAW;
    pid->gains[PID_AXIS_YAW].Ki   = PID_KI_YAW;
    pid->gains[PID_AXIS_YAW].Kd   = PID_KD_YAW;

    /* --- Límites --- */
    pid->pidSumLimit[PID_AXIS_ROLL]  = PID_SUM_LIMIT;
    pid->pidSumLimit[PID_AXIS_PITCH] = PID_SUM_LIMIT;
    pid->pidSumLimit[PID_AXIS_YAW]   = PID_SUM_LIMIT_YAW;

    pid->itermLimit[PID_AXIS_ROLL]   = PID_ITERM_LIMIT;
    pid->itermLimit[PID_AXIS_PITCH]  = PID_ITERM_LIMIT;
    pid->itermLimit[PID_AXIS_YAW]    = PID_ITERM_LIMIT_YAW;

    /* --- Filtros D-term (PT1 lowpass) ---
     * Betaflight: dtermLowpassApplyFn usando PT1 a DTERM_LPF_HZ
     * Filtrar el gyro ANTES de calcular la derivada, no después.
     * Esto reduce el ruido que amplifica el término D. */
    const float kDterm = pt1FilterGain(DTERM_LPF_HZ, PID_DT);
    for (int i = 0; i < PID_AXIS_COUNT; i++) {
        pt1FilterInit(&pid->dtermLpf[i], kDterm);
        pid->previousGyroRate[i] = 0.0f;
    }
}

void pidSetGains(PidState_t *pid, int axis, float Kp, float Ki, float Kd)
{
    if (axis < 0 || axis >= PID_AXIS_COUNT) return;
    pid->gains[axis].Kp = Kp;
    pid->gains[axis].Ki = Ki;
    pid->gains[axis].Kd = Kd;
}

void pidResetIterm(PidState_t *pid)
{
    for (int i = 0; i < PID_AXIS_COUNT; i++) {
        pid->output[i].I = 0.0f;
    }
}

/* =========================================================
 *  ACTUALIZACIÓN DEL PID — NÚCLEO PRINCIPAL
 *  Fuente: betaflight/src/main/flight/pid.c → pidController()
 *
 *  Estructura del loop para CADA eje (idéntica a Betaflight):
 *
 *  1. [Pre-D] Filtrar gyroRate con PT1 para el D-term
 *  2. [P] Kp * errorRate
 *  3. [I] previousI + Ki * dt * errorRate  (anti-windup: clamp)
 *  4. [D] Kd * -(filteredGyro[now] - filteredGyro[prev]) * pidFrequency
 *         — signo negativo: derivada del proceso, no del error
 *         — multiplicar por pidFrequency para normalizar a unidades físicas
 *  5. [Sum] P + I + D  (clamped a ±pidSumLimit)
 * ========================================================= */
void pidUpdate(PidState_t *pid,
               float setpointRoll,  float setpointPitch,  float setpointYaw,
               float gyroRoll,      float gyroPitch,      float gyroYaw)
{
    const float setpoints[PID_AXIS_COUNT] = { setpointRoll, setpointPitch, setpointYaw };
    const float gyroRates[PID_AXIS_COUNT] = { gyroRoll,     gyroPitch,     gyroYaw     };

    /* --- Pre-calcular gyroRate filtrado para D-term ---
     * Betaflight: gyroRateDterm[axis] pasa por dtermNotch + dtermLowpass + dtermLowpass2
     * Aquí: solo PT1 lowpass (suficiente para empezar)
     * El filtro va ANTES de calcular la derivada → menos ruido en D */
    float gyroRateDterm[PID_AXIS_COUNT];
    for (int axis = 0; axis < PID_AXIS_COUNT; axis++) {
        gyroRateDterm[axis] = pt1FilterApply(&pid->dtermLpf[axis], gyroRates[axis]);
    }

    /* --- Loop principal PID por eje --- */
    for (int axis = 0; axis < PID_AXIS_COUNT; axis++) {

        const float gyroRate    = gyroRates[axis];
        const float setpoint    = setpoints[axis];

        /* ── Error ──
         * errorRate = setpoint - gyroRate (referencia - medición)
         * Betaflight pid.c línea: "float errorRate = currentPidSetpoint - gyroRate" */
        const float errorRate = setpoint - gyroRate;

        /* ── Término P ──
         * Betaflight: pidData[axis].P = pidRuntime.pidCoefficient[axis].Kp * errorRate
         * (sin TPA por ahora) */
        pid->output[axis].P = pid->gains[axis].Kp * errorRate;

        /* ── Término I con anti-windup (clamping) ──
         * Betaflight: iTermChange = Ki * dT * itermErrorRate
         *             pidData[axis].I = constrainf(previousIterm + iTermChange, -itermLimit, itermLimit)
         *
         * Anti-windup: el integrador no puede superar ±itermLimit.
         * Esto evita que el I se "cargue" demasiado cuando el dron está
         * incapaz de corregir el error (por ejemplo, al estar en el suelo). */
        const float iTermChange = pid->gains[axis].Ki * pid->dt * errorRate;
        pid->output[axis].I = pidConstrainf(
            pid->output[axis].I + iTermChange,
            -pid->itermLimit[axis],
            +pid->itermLimit[axis]
        );

        /* ── Término D ──
         * Betaflight: delta = -(gyroRateDterm[axis] - previousGyroRateDterm[axis]) * pidFrequency
         *             preTpaD = pidRuntime.pidCoefficient[axis].Kd * delta
         *
         * DERIVADA DEL PROCESO (no del error):
         *   delta = -(current - previous) = previous - current
         *   El signo negativo hace que:
         *     - si el gyro acelera en la dirección del error → D amortigua (negativo)
         *     - si el gyro desacelera → D impulsa (positivo)
         *
         * Multiplicar por pidFrequency (1/dt) normaliza la unidad:
         *   [°/s] / [s] = [°/s²] → proporcional a la aceleración angular */
        pid->output[axis].D = 0.0f;
        if (pid->gains[axis].Kd > 0.0f) {
            const float delta = -(gyroRateDterm[axis] - pid->previousGyroRate[axis])
                                * pid->pidFrequency;
            pid->output[axis].D = pid->gains[axis].Kd * delta;
        }

        /* Guardar gyroRate filtrado para la siguiente iteración */
        pid->previousGyroRate[axis] = gyroRateDterm[axis];

        /* ── Suma PID limitada ──
         * Betaflight: pidData[axis].Sum = P + I + D + F
         *             luego el mixer lo aplica */
        const float rawSum = pid->output[axis].P
                           + pid->output[axis].I
                           + pid->output[axis].D
                           + pid->output[axis].F;

        pid->output[axis].sum = pidConstrainf(rawSum,
                                              -pid->pidSumLimit[axis],
                                              +pid->pidSumLimit[axis]);
    }
}
