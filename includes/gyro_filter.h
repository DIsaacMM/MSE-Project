/**
 * gyro_filter.h
 * Portado desde Betaflight 4.5.1 (GPL-3.0) para STM32F411RE bare-metal
 * Fuente original: src/main/sensors/gyro.c, gyro_filter_impl.c, common/filter.c
 *
 * Módulos incluidos:
 *  - Calibración de bias (offset) del giroscopio MPU6050/GY-521
 *  - Filtro PT1 (lowpass de 1er orden) — equivalente a RC filter digital
 *  - Filtro PT2 (lowpass de 2do orden) — más suave, más lag
 *  - Filtro Biquad IIR (Butterworth LPF + Notch)
 *  - Pipeline completo: RAW → calibración → downsample → notch → lowpass → salida
 *
 * Hardware target: STM32F411RE Nucleo, IMU GY-521 (MPU6050) via I2C
 */

#ifndef GYRO_FILTER_H
#define GYRO_FILTER_H

#include <stdint.h>
#include <stdbool.h>

/* =========================================================
 *  CONSTANTES GLOBALES
 * ========================================================= */

#define GYRO_AXES           3
#define AXIS_X              0
#define AXIS_Y              1
#define AXIS_Z              2

/* Duración de calibración: muestras totales a promediar.
 * A 1kHz de loop, 1000 ciclos = 1 segundo de calibración.
 * Aumentar a 2000 para mayor precisión si el dron está muy quieto. */
#define GYRO_CALIB_CYCLES   2000

/* Umbral de movimiento durante calibración (en raw ADC counts).
 * Si la desviación estándar supera este valor, se reinicia.
 * MPU6050 a ±2000°/s: 1 LSB ≈ 0.061°/s  → umbral ~25 LSB */
#define GYRO_CALIB_MOVE_THRESHOLD  110.0f

/* Factor de escala del MPU6050 en modo ±2000°/s (predeterminado en GY-521):
 * 16.4 LSB por °/s  → dividir raw por 16.4 para obtener °/s
 * O en rad/s: dividir por (16.4 * 57.2958) */
#define GYRO_SCALE_DPS      (1.0f / 16.4f)             /* raw → grados/s  */
#define GYRO_SCALE_RADS     (GYRO_SCALE_DPS * 0.017453292519943f) /* raw → rad/s */

/* Frecuencia de muestreo del loop principal (Hz) — ajustar a tu TIM */
#define GYRO_LOOP_HZ        1000
#define GYRO_DT             (1.0f / GYRO_LOOP_HZ)      /* segundos        */

/* Frecuencias de corte de los filtros (Hz) — valores típicos Betaflight */
#define GYRO_LPF1_HZ        120.0f   /* Lowpass principal (PT2)           */
#define GYRO_LPF2_HZ        200.0f   /* Lowpass de downsample (PT1)       */
#define GYRO_NOTCH1_HZ      0.0f     /* Notch 1: 0 = desactivado          */
#define GYRO_NOTCH1_CUT_HZ  0.0f
#define GYRO_NOTCH2_HZ      0.0f     /* Notch 2: 0 = desactivado          */
#define GYRO_NOTCH2_CUT_HZ  0.0f

/* Corrección de frecuencia de corte para PT2 y PT3 (Betaflight filter.c) */
#define CUTOFF_CORRECTION_PT2   1.553773974f
#define CUTOFF_CORRECTION_PT3   1.961459177f

/* Butterworth Q para biquad */
#define BIQUAD_Q_BUTTERWORTH    0.7071067811865476f   /* 1/sqrt(2) */

/* =========================================================
 *  TIPOS DE DATOS — FILTROS
 * ========================================================= */

/** PT1: filtro lowpass de 1er orden (equivalente RC digital)
 *  state = state + k*(input - state)
 *  k = omega / (omega + 1),  omega = 2*pi*f_cut*dt */
typedef struct {
    float state;
    float k;
} PT1Filter_t;

/** PT2: lowpass de 2do orden (dos PT1 en cascada con corrección de frecuencia) */
typedef struct {
    float state;
    float state1;
    float k;
} PT2Filter_t;

/** PT3: lowpass de 3er orden */
typedef struct {
    float state;
    float state1;
    float state2;
    float k;
} PT3Filter_t;

/** Biquad IIR (forma directa 2) — LPF Butterworth o filtro Notch */
typedef struct {
    float b0, b1, b2;   /* coeficientes numerador   */
    float a1, a2;       /* coeficientes denominador */
    float x1, x2;       /* estados (entradas)       */
    float y1, y2;       /* estados (salidas)        */
} BiquadFilter_t;

typedef enum {
    BIQUAD_LPF   = 0,
    BIQUAD_NOTCH = 1,
    BIQUAD_BPF   = 2
} BiquadFilterType_e;

/* =========================================================
 *  TIPO DE DATO — CALIBRACIÓN
 * ========================================================= */

typedef struct {
    float  sum[GYRO_AXES];       /* acumulador de lecturas raw            */
    float  sumSq[GYRO_AXES];     /* para calcular desviación estándar     */
    float  bias[GYRO_AXES];      /* offset calculado (raw ADC counts)     */
    int32_t cyclesRemaining;     /* ciclos que faltan para terminar       */
    bool    complete;            /* true cuando la calibración terminó    */
} GyroCalibration_t;

/* =========================================================
 *  TIPO DE DATO — PIPELINE COMPLETO DEL GIROSCOPIO
 * ========================================================= */

typedef struct {
    /* --- Calibración --- */
    GyroCalibration_t   calib;

    /* --- Datos procesados por eje --- */
    float   rawADC[GYRO_AXES];      /* lectura raw del sensor (counts)       */
    float   gyroADC[GYRO_AXES];     /* raw - bias (counts)                   */
    float   gyroDPS[GYRO_AXES];     /* en grados/segundo                     */
    float   gyroRad[GYRO_AXES];     /* salida final filtrada (rad/s)         */

    /* --- Filtros por eje --- */
    PT1Filter_t     lpf2[GYRO_AXES];        /* lowpass para downsample        */
    PT2Filter_t     lpf1[GYRO_AXES];        /* lowpass principal              */
    BiquadFilter_t  notch1[GYRO_AXES];      /* notch estático 1               */
    BiquadFilter_t  notch2[GYRO_AXES];      /* notch estático 2               */

    /* --- Configuración runtime --- */
    bool    notch1Active;
    bool    notch2Active;
} GyroPipeline_t;

/* =========================================================
 *  API PÚBLICA
 * ========================================================= */

/* --- Filtros primitivos --- */
float  pt1FilterGain(float f_cut_hz, float dt);
void   pt1FilterInit(PT1Filter_t *f, float k);
float  pt1FilterApply(PT1Filter_t *f, float input);

float  pt2FilterGain(float f_cut_hz, float dt);
void   pt2FilterInit(PT2Filter_t *f, float k);
float  pt2FilterApply(PT2Filter_t *f, float input);

float  pt3FilterGain(float f_cut_hz, float dt);
void   pt3FilterInit(PT3Filter_t *f, float k);
float  pt3FilterApply(PT3Filter_t *f, float input);

float  biquadFilterGetNotchQ(float centerHz, float cutoffHz);
void   biquadFilterInit(BiquadFilter_t *f, float freqHz, float sampleRateHz,
                        float Q, BiquadFilterType_e type);
float  biquadFilterApply(BiquadFilter_t *f, float input);      /* forma directa 2 */
float  biquadFilterApplyDF1(BiquadFilter_t *f, float input);   /* forma directa 1 */

/* --- Calibración --- */
void  gyroCalibrationStart(GyroCalibration_t *calib);
bool  gyroCalibrationIsComplete(const GyroCalibration_t *calib);

/**
 * gyroCalibrationUpdate - llamar cada ciclo con la lectura raw del sensor.
 * Devuelve true cuando la calibración termina.
 * Mientras calibra, bias[] se mantiene en 0 para no aplicar corrección parcial.
 */
bool  gyroCalibrationUpdate(GyroCalibration_t *calib,
                            const int16_t rawX, const int16_t rawY, const int16_t rawZ);

/* --- Pipeline principal --- */

/**
 * gyroPipelineInit - inicializa filtros y arranca calibración.
 * Llamar una sola vez en la inicialización del sistema.
 */
void  gyroPipelineInit(GyroPipeline_t *pipe);

/**
 * gyroPipelineUpdate - procesar UNA muestra raw del MPU6050.
 * rawX/Y/Z son los valores int16 directos del sensor (registro 0x43-0x48).
 * Salida disponible en pipe->gyroRad[] (rad/s) y pipe->gyroDPS[] (°/s).
 * Retorna true si el pipeline está listo (calibración completa).
 */
bool  gyroPipelineUpdate(GyroPipeline_t *pipe,
                         int16_t rawX, int16_t rawY, int16_t rawZ);

#endif /* GYRO_FILTER_H */