/**
 * pid_controller.c
 * Portado y adaptado desde Betaflight 4.5.1 (GPL-3.0) — src/main/flight/pid.c
 *
 * CAMBIOS vs version anterior:
 *   - Anti-windup condicional por saturacion de salida
 *   - iterm_relax: congela I cuando |errorRate| > ITERM_RELAX_THRESHOLD_DPS
 *   - TPA: atenua Kp y Kd linealmente segun throttle
 *   - D se calcula antes que I para que el anti-windup use D actualizado
 */

#include "pid_controller.h"

/* =========================================================
 *  INICIALIZACION
 * ========================================================= */

void pidInit(PidState_t *pid)
{
    memset(pid, 0, sizeof(PidState_t));

    pid->dt           = PID_DT;
    pid->pidFrequency = (float)PID_LOOP_HZ;

    pid->gains[PID_AXIS_ROLL].Kp  = PID_KP_ROLL;
    pid->gains[PID_AXIS_ROLL].Ki  = PID_KI_ROLL;
    pid->gains[PID_AXIS_ROLL].Kd  = PID_KD_ROLL;

    pid->gains[PID_AXIS_PITCH].Kp = PID_KP_PITCH;
    pid->gains[PID_AXIS_PITCH].Ki = PID_KI_PITCH;
    pid->gains[PID_AXIS_PITCH].Kd = PID_KD_PITCH;

    pid->gains[PID_AXIS_YAW].Kp   = PID_KP_YAW;
    pid->gains[PID_AXIS_YAW].Ki   = PID_KI_YAW;
    pid->gains[PID_AXIS_YAW].Kd   = PID_KD_YAW;

    pid->pidSumLimit[PID_AXIS_ROLL]  = PID_SUM_LIMIT;
    pid->pidSumLimit[PID_AXIS_PITCH] = PID_SUM_LIMIT;
    pid->pidSumLimit[PID_AXIS_YAW]   = PID_SUM_LIMIT_YAW;

    pid->itermLimit[PID_AXIS_ROLL]   = PID_ITERM_LIMIT;
    pid->itermLimit[PID_AXIS_PITCH]  = PID_ITERM_LIMIT;
    pid->itermLimit[PID_AXIS_YAW]    = PID_ITERM_LIMIT_YAW;

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
 *  ACTUALIZACION DEL PID — NUCLEO PRINCIPAL
 *
 *  Por eje:
 *  1. [Pre-D]  Filtrar gyro con PT1
 *  2. [TPA]    Calcular factor de atenuacion segun throttle
 *  3. [P]      Kp * tpaFactor * errorRate
 *  4. [D]      Kd * tpaFactor * -(gyroFilt[now] - gyroFilt[prev]) * pidFreq
 *              — calculado ANTES que I para anti-windup correcto
 *  5. [I]      Anti-windup doble:
 *                a) iterm_relax: congela si |errorRate| > umbral
 *                b) clamp absoluto a +-itermLimit
 *                c) condicional: congela si salida proyectada satura
 *  6. [Sum]    P + I + D + F, clamped a +-pidSumLimit
 * ========================================================= */
void pidUpdate(PidState_t *pid,
               float setpointRoll,  float setpointPitch,  float setpointYaw,
               float gyroRoll,      float gyroPitch,      float gyroYaw,
               float throttle)
{
    const float setpoints[PID_AXIS_COUNT] = { setpointRoll, setpointPitch, setpointYaw };
    const float gyroRates[PID_AXIS_COUNT] = { gyroRoll,     gyroPitch,     gyroYaw     };

    /* ── TPA: factor unico para P y D, no afecta I ni yaw ──
     * Entre 1.0 (en o por debajo de TPA_BREAKPOINT) y TPA_MIN (a ESC_MAX_US).
     * El yaw no usa TPA — su Kd ya es 0 y el Kp es muy bajo. */
    float tpaFactor = 1.0f;
    if (throttle > TPA_BREAKPOINT) {
        tpaFactor = 1.0f - (1.0f - TPA_MIN)
                    * (throttle - TPA_BREAKPOINT)
                    / (ESC_MAX_US_F - TPA_BREAKPOINT);
        if (tpaFactor < TPA_MIN) tpaFactor = TPA_MIN;
    }

    /* ── Filtro D-term ── */
    float gyroRateDterm[PID_AXIS_COUNT];
    for (int axis = 0; axis < PID_AXIS_COUNT; axis++) {
        gyroRateDterm[axis] = pt1FilterApply(&pid->dtermLpf[axis], gyroRates[axis]);
    }

    /* ── Loop PID por eje ── */
    for (int axis = 0; axis < PID_AXIS_COUNT; axis++) {

        const float gyroRate = gyroRates[axis];
        const float setpoint = setpoints[axis];
        const float errorRate = setpoint - gyroRate;

        /* ── Termino P (con TPA) ──
         * Betaflight: P = Kp * errorRate * tpaFactor */
        pid->output[axis].P = pid->gains[axis].Kp * tpaFactor * errorRate;

        /* ── Termino D (con TPA) — calculado antes que I ──
         * Derivada del proceso (no del error) para evitar derivative kick.
         * Betaflight: delta = -(gyroFilt[now] - gyroFilt[prev]) * pidFrequency */
        pid->output[axis].D = 0.0f;
        if (pid->gains[axis].Kd > 0.0f) {
            const float delta = -(gyroRateDterm[axis] - pid->previousGyroRate[axis])
                                * pid->pidFrequency;
            pid->output[axis].D = pid->gains[axis].Kd * tpaFactor * delta;
        }
        pid->previousGyroRate[axis] = gyroRateDterm[axis];

        /* ── Termino I con anti-windup triple ──
         *
         * Capa 1 — iterm_relax:
         *   Si el error de rata supera el umbral, el setpoint esta cambiando
         *   bruscamente (transitorio del lazo externo angulo->dps). Congelar
         *   el integrador evita que acumule ese transitorio.
         *   Observable en UART: IR y IP dejan de crecer durante transitorios.
         *   Betaflight: iterm_relax con filtro paso bajo — aqui umbral fijo,
         *   suficiente para bare-metal sin overhead de filtro adicional.
         *
         * Capa 2 — clamp absoluto a +-itermLimit:
         *   El integrador nunca supera su limite propio, sin importar
         *   cuanto tiempo acumule. Proteccion de ultimo recurso.
         *
         * Capa 3 — anti-windup condicional por saturacion de salida:
         *   Si P+I+D ya supera +-pidSumLimit Y el iterm quiere crecer en
         *   esa misma direccion, congelar el integrador. Si el error cambia
         *   de signo se libera inmediatamente para desenrollar sin demora. */

        const bool itermRelaxActive = (errorRate >  ITERM_RELAX_THRESHOLD_DPS ||
                                       errorRate < -ITERM_RELAX_THRESHOLD_DPS);

        if (!itermRelaxActive) {
            const float iTermChange  = pid->gains[axis].Ki * pid->dt * errorRate;
            const float iTermClamped = pidConstrainf(
                pid->output[axis].I + iTermChange,
                -pid->itermLimit[axis],
                +pid->itermLimit[axis]
            );

            const float projectedSum = pid->output[axis].P
                                     + iTermClamped
                                     + pid->output[axis].D
                                     + pid->output[axis].F;

            const bool saturatedHigh = (projectedSum >  pid->pidSumLimit[axis])
                                    && (iTermChange  >  0.0f);
            const bool saturatedLow  = (projectedSum < -pid->pidSumLimit[axis])
                                    && (iTermChange  <  0.0f);

            if (!saturatedHigh && !saturatedLow) {
                pid->output[axis].I = iTermClamped;
            }
            /* Si satura: pid->output[axis].I queda sin cambio (congelado) */
        }
        /* Si iterm_relax activo: pid->output[axis].I queda sin cambio */

        /* ── Suma PID limitada ── */
        const float rawSum = pid->output[axis].P
                           + pid->output[axis].I
                           + pid->output[axis].D
                           + pid->output[axis].F;

        pid->output[axis].sum = pidConstrainf(rawSum,
                                              -pid->pidSumLimit[axis],
                                              +pid->pidSumLimit[axis]);
    }
}