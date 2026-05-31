/**
 * gyro_filter.c
 * Portado desde Betaflight 4.5.1 (GPL-3.0) para STM32F411RE bare-metal
 *
 * Algoritmos originales en:
 *   src/main/common/filter.c          → PT1, PT2, PT3, Biquad
 *   src/main/sensors/gyro.c           → performGyroCalibration(), gyroUpdate()
 *   src/main/sensors/gyro_filter_impl.c → filterGyro() (pipeline)
 *
 * Adaptaciones realizadas:
 *  - Eliminadas todas las dependencias del scheduler, RTOS y HAL de Betaflight
 *  - Eliminados macros FAST_CODE, DEBUG_SET (no existen en bare-metal)
 *  - Eliminado USE_RPM_FILTER y USE_DYN_NOTCH_FILTER (no aplica a STM32F411 sin ESC telemetry)
 *  - sincosf_approx() sustituido por sincosf() estándar de libm
 *  - invSqrt() implementado con Newton-Raphson (mismo que Betaflight)
 *  - La calibración sigue exactamente el algoritmo de performGyroCalibration():
 *      acumula N muestras, calcula media y stddev, rechaza si hay movimiento
 */

#include "gyro_filter.h"
#include "math_utils.h"

/* M_PI no está garantizado en C99 estricto — definirlo si falta */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* =========================================================
 *  UTILIDADES MATEMÁTICAS INTERNAS
 * ========================================================= */

/* fast inverse square root — mismo que Betaflight maths.h */


/* =========================================================
 *  FILTROS PT1 / PT2 / PT3
 *  Fuente: betaflight/src/main/common/filter.c
 * ========================================================= */

/**
 * pt1FilterGain - calcula k para una frecuencia de corte dada.
 * k = omega / (omega + 1),  omega = 2*pi*f_cut*dt
 * Equivalente al coeficiente alpha de un filtro RC digital exponencial.
 */
float pt1FilterGain(float f_cut_hz, float dt)
{
    float omega = 2.0f * (float)M_PI * f_cut_hz * dt;
    return omega / (omega + 1.0f);
}

void pt1FilterInit(PT1Filter_t *f, float k)
{
    f->state = 0.0f;
    f->k     = k;
}

/**
 * pt1FilterApply - EMA (Exponential Moving Average) de primer orden.
 * state(n) = state(n-1) + k * (input - state(n-1))
 */
float pt1FilterApply(PT1Filter_t *f, float input)
{
    f->state += f->k * (input - f->state);
    return f->state;
}

/* PT2: dos PT1 en cascada con corrección de frecuencia para que -3dB sea correcto */
float pt2FilterGain(float f_cut_hz, float dt)
{
    /* shift f_cut para satisfacer condición de -3dB en PT2 */
    return pt1FilterGain(f_cut_hz * CUTOFF_CORRECTION_PT2, dt);
}

void pt2FilterInit(PT2Filter_t *f, float k)
{
    f->state  = 0.0f;
    f->state1 = 0.0f;
    f->k      = k;
}

float pt2FilterApply(PT2Filter_t *f, float input)
{
    f->state1 += f->k * (input    - f->state1);
    f->state  += f->k * (f->state1 - f->state);
    return f->state;
}

/* PT3: tres PT1 en cascada */
float pt3FilterGain(float f_cut_hz, float dt)
{
    return pt1FilterGain(f_cut_hz * CUTOFF_CORRECTION_PT3, dt);
}

void pt3FilterInit(PT3Filter_t *f, float k)
{
    f->state  = 0.0f;
    f->state1 = 0.0f;
    f->state2 = 0.0f;
    f->k      = k;
}

float pt3FilterApply(PT3Filter_t *f, float input)
{
    f->state1 += f->k * (input     - f->state1);
    f->state2 += f->k * (f->state1 - f->state2);
    f->state  += f->k * (f->state2 - f->state);
    return f->state;
}

/* =========================================================
 *  FILTRO BIQUAD IIR
 *  Fuente: betaflight/src/main/common/filter.c
 * ========================================================= */

/**
 * biquadFilterGetNotchQ — calcula Q para un filtro notch dado su centro y cutoff.
 * Q = f0 / (f2 - f1),  f2 = f0²/f1
 * Permite ajustar el ancho de banda del notch.
 */
float biquadFilterGetNotchQ(float centerHz, float cutoffHz)
{
    return centerHz * cutoffHz / (sq(centerHz) - sq(cutoffHz));
}

/**
 * biquadFilterInit — inicializa un filtro biquad IIR.
 * @freqHz      : frecuencia de corte (LPF) o central (notch)
 * @sampleRateHz: frecuencia de muestreo del loop (ej: 1000 Hz)
 * @Q           : factor de calidad (use BIQUAD_Q_BUTTERWORTH = 0.7071 para LPF)
 * @type        : BIQUAD_LPF, BIQUAD_NOTCH, o BIQUAD_BPF
 */
void biquadFilterInit(BiquadFilter_t *f, float freqHz, float sampleRateHz,
                      float Q, BiquadFilterType_e type)
{
    const float omega = 2.0f * (float)M_PI * freqHz / sampleRateHz;
    const float sn    = sinf(omega);
    const float cs    = cosf(omega);
    const float alpha = sn / (2.0f * Q);

    float b0, b1, b2, a0, a1, a2;

    switch (type) {
    case BIQUAD_LPF:
        /* 2do orden Butterworth — Betaflight filter.c FILTER_LPF */
        b1 = 1.0f - cs;
        b0 = b1 * 0.5f;
        b2 = b0;
        a1 = -2.0f * cs;
        a2 = 1.0f - alpha;
        break;

    case BIQUAD_NOTCH:
        /* Notch (band-reject) — Betaflight filter.c FILTER_NOTCH */
        b0 = 1.0f;
        b1 = -2.0f * cs;
        b2 = 1.0f;
        a1 = b1;
        a2 = 1.0f - alpha;
        break;

    case BIQUAD_BPF:
        b0 =  alpha;
        b1 =  0.0f;
        b2 = -alpha;
        a1 = -2.0f * cs;
        a2 =  1.0f - alpha;
        break;

    default:
        /* Fallback: pasar señal sin filtrar */
        f->b0=1; f->b1=0; f->b2=0;
        f->a1=0; f->a2=0;
        f->x1=f->x2=f->y1=f->y2=0;
        return;
    }

    a0 = 1.0f + alpha;

    /* pre-dividir entre a0 para evitar división en cada muestra */
    f->b0 = b0 / a0;
    f->b1 = b1 / a0;
    f->b2 = b2 / a0;
    f->a1 = a1 / a0;
    f->a2 = a2 / a0;

    /* estados iniciales en 0 */
    f->x1 = f->x2 = 0.0f;
    f->y1 = f->y2 = 0.0f;
}

/**
 * biquadFilterApply — forma directa 2 (DF2).
 * Más eficiente numéricamente, adecuado cuando los coeficientes no cambian.
 * Betaflight usa DF2 como función por defecto (biquadFilterApply).
 */
float biquadFilterApply(BiquadFilter_t *f, float input)
{
    const float result = f->b0 * input + f->x1;
    f->x1 = f->b1 * input - f->a1 * result + f->x2;
    f->x2 = f->b2 * input - f->a2 * result;
    return result;
}

/**
 * biquadFilterApplyDF1 — forma directa 1.
 * Ligeramente menos eficiente pero más estable cuando los coeficientes
 * se actualizan dinámicamente (ej. filtros dinámicos adaptativos).
 * Betaflight usa DF1 para los filtros de ganancia variable.
 */
float biquadFilterApplyDF1(BiquadFilter_t *f, float input)
{
    const float result =   f->b0 * input
                         + f->b1 * f->x1
                         + f->b2 * f->x2
                         - f->a1 * f->y1
                         - f->a2 * f->y2;

    f->x2 = f->x1;
    f->x1 = input;
    f->y2 = f->y1;
    f->y1 = result;

    return result;
}

/* =========================================================
 *  CALIBRACIÓN DEL GIROSCOPIO
 *  Fuente: betaflight/src/main/sensors/gyro.c
 *          → performGyroCalibration()
 * ========================================================= */

/**
 * gyroCalibrationStart — resetea la calibración para empezar de nuevo.
 * Llamar al encender o cuando el usuario pide recalibrar.
 */
void gyroCalibrationStart(GyroCalibration_t *calib)
{
    memset(calib, 0, sizeof(GyroCalibration_t));
    calib->cyclesRemaining = GYRO_CALIB_CYCLES;
    calib->complete        = false;
    /* bias[] queda en 0 — no se aplica corrección hasta que termine */
}

bool gyroCalibrationIsComplete(const GyroCalibration_t *calib)
{
    return calib->complete;
}

/**
 * gyroCalibrationUpdate — llamar una vez por ciclo de loop con los raw del sensor.
 *
 * Algoritmo (idéntico a Betaflight performGyroCalibration()):
 *  1. Primer ciclo: resetea acumuladores.
 *  2. Ciclos intermedios: acumula raw y raw² para media y stddev.
 *  3. Último ciclo:
 *     a. Calcula stddev de cada eje.
 *     b. Si stddev > umbral → el dron se movió → reinicia calibración.
 *     c. Si está quieto → bias = suma / N.
 *
 * @return true cuando la calibración terminó satisfactoriamente.
 */
bool gyroCalibrationUpdate(GyroCalibration_t *calib,
                           const int16_t rawX, const int16_t rawY, const int16_t rawZ)
{
    if (calib->complete) return true;

    const float raw[GYRO_AXES] = { (float)rawX, (float)rawY, (float)rawZ };
    const bool isFirst = (calib->cyclesRemaining == GYRO_CALIB_CYCLES);
    const bool isFinal = (calib->cyclesRemaining == 1);

    /* --- 1. Reset en el primer ciclo --- */
    if (isFirst) {
        for (int axis = 0; axis < GYRO_AXES; axis++) {
            calib->sum[axis]   = 0.0f;
            calib->sumSq[axis] = 0.0f;
            calib->bias[axis]  = 0.0f;  /* sin corrección mientras calibra */
        }
    }

    /* --- 2. Acumular muestras --- */
    for (int axis = 0; axis < GYRO_AXES; axis++) {
        calib->sum[axis]   += raw[axis];
        calib->sumSq[axis] += raw[axis] * raw[axis];
    }

    /* --- 3. Evaluar en el último ciclo --- */
    if (isFinal) {
        const float n = (float)GYRO_CALIB_CYCLES;
        bool calFailed = false;

        for (int axis = 0; axis < GYRO_AXES; axis++) {
            /* Desviación estándar por eje
             * stddev = sqrt( E[x²] - E[x]² )
             * Betaflight usa devStandardDeviation() de la librería maths
             * — aquí lo hacemos inline con los acumuladores */
            const float mean    = calib->sum[axis] / n;
            const float variance = (calib->sumSq[axis] / n) - (mean * mean);
            const float stddev  = (variance > 0.0f) ? sqrtf(variance) : 0.0f;

            /* Si el dron se movió durante la calibración → reiniciar */
            if (stddev > GYRO_CALIB_MOVE_THRESHOLD) {
                calFailed = true;
                break;
            }

            calib->bias[axis] = mean;
        }

        if (calFailed) {
            /* reiniciar conteo — Betaflight hace gyroSetCalibrationCycles() de nuevo */
            calib->cyclesRemaining = GYRO_CALIB_CYCLES;
            for (int axis = 0; axis < GYRO_AXES; axis++) {
                calib->sum[axis]   = 0.0f;
                calib->sumSq[axis] = 0.0f;
            }
            return false;
        }

        calib->complete = true;
        return true;
    }

    calib->cyclesRemaining--;
    return false;
}

/* =========================================================
 *  PIPELINE COMPLETO DEL GIROSCOPIO
 *  Fuente: betaflight/src/main/sensors/gyro_filter_impl.c → filterGyro()
 *          + gyro.c → gyroUpdate()
 * ========================================================= */

/**
 * gyroPipelineInit — inicializar filtros y calibración.
 *
 * Orden del pipeline (igual que Betaflight):
 *  RAW sensor → bias subtract → escalar a unidades físicas
 *            → lowpass2 (PT1, downsample/suavizado rápido)
 *            → notch1 (Biquad notch, frecuencia de vibración del motor)
 *            → notch2 (Biquad notch, frecuencia secundaria)
 *            → lowpass1 (PT2, filtro principal de suavizado)
 *            → SALIDA gyroRad[]
 */
void gyroPipelineInit(GyroPipeline_t *pipe)
{
    memset(pipe, 0, sizeof(GyroPipeline_t));

    /* Arranca calibración */
    gyroCalibrationStart(&pipe->calib);

    /* Coeficientes de filtro pre-calculados */
    const float kLpf2 = pt1FilterGain(GYRO_LPF2_HZ, GYRO_DT);
    const float kLpf1 = pt2FilterGain(GYRO_LPF1_HZ, GYRO_DT);

    for (int axis = 0; axis < GYRO_AXES; axis++) {
        /* LPF2 (PT1) — downsample / suavizado de alta frecuencia */
        pt1FilterInit(&pipe->lpf2[axis], kLpf2);

        /* LPF1 (PT2) — filtro principal */
        pt2FilterInit(&pipe->lpf1[axis], kLpf1);

        /* Notch 1 — si la frecuencia es 0, se desactiva */
        if (GYRO_NOTCH1_HZ > 0.0f && GYRO_NOTCH1_CUT_HZ > 0.0f) {
            float Q = biquadFilterGetNotchQ(GYRO_NOTCH1_HZ, GYRO_NOTCH1_CUT_HZ);
            biquadFilterInit(&pipe->notch1[axis], GYRO_NOTCH1_HZ,
                             (float)GYRO_LOOP_HZ, Q, BIQUAD_NOTCH);
            pipe->notch1Active = true;
        }

        /* Notch 2 */
        if (GYRO_NOTCH2_HZ > 0.0f && GYRO_NOTCH2_CUT_HZ > 0.0f) {
            float Q = biquadFilterGetNotchQ(GYRO_NOTCH2_HZ, GYRO_NOTCH2_CUT_HZ);
            biquadFilterInit(&pipe->notch2[axis], GYRO_NOTCH2_HZ,
                             (float)GYRO_LOOP_HZ, Q, BIQUAD_NOTCH);
            pipe->notch2Active = true;
        }
    }
}

/**
 * gyroPipelineUpdate — procesar una muestra del GY-521 (MPU6050).
 *
 * @rawX/Y/Z  : valores int16 directamente de los registros 0x43–0x48 del MPU6050
 * @return    : true si el pipeline está listo (calibración completa)
 *
 * Mientras la calibración no termina:
 *  - Los filtros aún se ejecutan (para que no estén "fríos" al arrancar)
 *  - gyroRad[] y gyroDPS[] tienen valores válidos pero con bias no corregido
 *  - El controlador PID NO debe usarse hasta que retorne true
 *
 * Nota sobre alineación del sensor (del archivo de configuración):
 *  gyro_1_sensor_align = CW90 → rotar X e Y:
 *      nuevoX = -Y,  nuevoY = +X,  Z permanece
 *  Si tu placa NO está rotada, eliminar el bloque de alineación.
 */
bool gyroPipelineUpdate(GyroPipeline_t *pipe,
                        int16_t rawX, int16_t rawY, int16_t rawZ)
{
    /* --- 1. Calibración (bias tracking) --- */
    bool calComplete = gyroCalibrationUpdate(&pipe->calib, rawX, rawY, rawZ);

    /* --- 2. Guardar raw --- */
    pipe->rawADC[AXIS_X] = (float)rawX;
    pipe->rawADC[AXIS_Y] = (float)rawY;
    pipe->rawADC[AXIS_Z] = (float)rawZ;

    /* --- 3. Restar bias (0 hasta que la calibración termine) --- */
    pipe->gyroADC[AXIS_X] = pipe->rawADC[AXIS_X] - pipe->calib.bias[AXIS_X];
    pipe->gyroADC[AXIS_Y] = pipe->rawADC[AXIS_Y] - pipe->calib.bias[AXIS_Y];
    pipe->gyroADC[AXIS_Z] = pipe->rawADC[AXIS_Z] - pipe->calib.bias[AXIS_Z];

    /* --- 4. Alineación del sensor (CW90 según configuración Betaflight) ---
     *  Descomenta si tu placa está montada con esa orientación:
     *
     *  float tmpX = pipe->gyroADC[AXIS_X];
     *  pipe->gyroADC[AXIS_X] = -pipe->gyroADC[AXIS_Y];
     *  pipe->gyroADC[AXIS_Y] =  tmpX;
     */

    /* --- 5. Escalar a grados/segundo --- */
    for (int axis = 0; axis < GYRO_AXES; axis++) {
        pipe->gyroDPS[axis] = pipe->gyroADC[axis] * GYRO_SCALE_DPS;
    }

    /* --- 6. Pipeline de filtros (orden idéntico a Betaflight filterGyro()) ---
     *
     *  gyro_filter_impl.c aplica en este orden:
     *   a) downsample (aquí usamos PT1 lowpass2 como downsample suavizador)
     *   b) notch1
     *   c) notch2
     *   d) lowpass1 (filtro principal)
     *
     *  En Betaflight, el downsample puede ser promedio simple o LP2.
     *  Aquí usamos siempre el LP2 (más suave).
     */
    for (int axis = 0; axis < GYRO_AXES; axis++) {
        float sample = pipe->gyroDPS[axis];

        /* a) Lowpass2 (PT1) — downsample / pre-suavizado */
        sample = pt1FilterApply(&pipe->lpf2[axis], sample);

        /* b) Notch 1 (si activo) */
        if (pipe->notch1Active) {
            sample = biquadFilterApply(&pipe->notch1[axis], sample);
        }

        /* c) Notch 2 (si activo) */
        if (pipe->notch2Active) {
            sample = biquadFilterApply(&pipe->notch2[axis], sample);
        }

        /* d) Lowpass1 principal (PT2) */
        sample = pt2FilterApply(&pipe->lpf1[axis], sample);

        /* Convertir a rad/s para el controlador PID / AHRS */
        pipe->gyroRad[axis] = sample * 0.017453292519943f; /* * pi/180 */
    }

    return calComplete;
}
