/**
 * imu_mahony.h
 * Portado desde Betaflight 4.5.1 (GPL-3.0) — src/main/flight/imu.c
 * Target: STM32F411RE Nucleo bare-metal, IMU GY-521 (MPU6050)
 *
 * ¿QUÉ HACE EL IMU?
 * =================
 * El IMU (Inertial Measurement Unit) resuelve el problema fundamental del vuelo:
 * saber en qué orientación está el dron en cada momento.
 *
 * Problema: El giroscopio mide velocidad angular (°/s), no ángulos absolutos.
 * Si solo integras el giroscopio, acumulas error (drift) indefinidamente.
 * El acelerómetro mide ángulo absoluto, pero es ruidoso con vibraciones.
 *
 * Solución — Filtro de Mahony (AHRS: Attitude and Heading Reference System):
 *
 *   1. PREDICCIÓN (solo giroscopio):
 *      Integra ωx,ωy,ωz para rotar el cuaternión → actualización rápida y precisa
 *
 *   2. CORRECCIÓN (acelerómetro):
 *      Compara el vector de gravedad estimado (de la rotación) vs el medido (acc)
 *      → calcula un error de orientación (producto cruzado)
 *      → lo retroalimenta al giroscopio con ganancia Kp (proporcional) y Ki (integral)
 *
 *   3. SALIDA:
 *      Cuaternión q = {w,x,y,z} → representa la orientación 3D sin gimbal lock
 *      Ángulos de Euler (roll, pitch, yaw) para el controlador PID y para debug
 *
 * Por qué cuaterniones y no matriz de rotación:
 *   - 4 floats vs 9 floats → menos memoria y operaciones
 *   - No hay gimbal lock (singularidades matemáticas a 90° de pitch)
 *   - Normalización trivial: un solo invSqrt()
 *
 * Parámetros clave:
 *   Kp = 0.25 (ganancia proporcional) → qué tan agresivamente corrige el error
 *   Ki = 0.0  (ganancia integral)     → corrección lenta de bias residual del gyro
 *   dt = 0.001 s (a 1kHz)
 *
 * Con solo giroscopio+acelerómetro (sin magnetómetro ni GPS):
 *   ✓ Roll y Pitch: precisos, estables
 *   ✗ Yaw: drifta lentamente (sin referencia absoluta de norte)
 *   → Para estabilización indoor esto es aceptable
 */

#ifndef IMU_MAHONY_H
#define IMU_MAHONY_H

#include <stdint.h>
#include <stdbool.h>

/* =========================================================
 *  CONSTANTES
 * ========================================================= */

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Ganancias del filtro de Mahony
 * Betaflight imu.c:
 *   imu_dcm_kp = 2500  → Kp = 2500/10000 = 0.25
 *   imu_dcm_ki = 0     → Ki = 0
 *
 * Kp más alto: converge más rápido pero más sensible a vibraciones
 * Ki > 0: elimina bias residual del gyro, útil con sensores baratos como el MPU6050 */
#define IMU_DCM_KP          0.25f
#define IMU_DCM_KI          0.0f

/* Al desarmar: Betaflight aumenta Kp×10 para convergencia rápida
 * Usamos la misma lógica: alta ganancia durante 500ms al inicio */
#define IMU_DCM_KP_FAST     (IMU_DCM_KP * 10.0f)

/* Límite de velocidad angular para integrar Ki.
 * Si el dron gira muy rápido, el filtro puede "marearse" (drift en integralFB).
 * Betaflight: SPIN_RATE_LIMIT = 20 °/s  */
#define IMU_SPIN_RATE_LIMIT_RAD  (20.0f * 0.017453292f)  /* 20°/s en rad/s */

/* Factor de escala MPU6050 para el acelerómetro en modo ±2g (predeterminado)
 * 16384 LSB/g  → dividir raw por 16384 para obtener [g] */
#define ACC_SCALE_G         (1.0f / 16384.0f)

/* Tiempo de convergencia rápida al inicio (ms a 1kHz = ciclos) */
#define IMU_FAST_KP_CYCLES  500

/* =========================================================
 *  TIPOS DE DATOS
 * ========================================================= */

/** Cuaternión de orientación: representa la rotación del marco del cuerpo
 *  relativo al marco de tierra. Inicializado a identidad: (1,0,0,0) = sin rotación. */
typedef struct {
    float w, x, y, z;
} Quaternion_t;

/** Matriz de rotación 3×3 derivada del cuaternión.
 *  rMat[2][0..2] = tercer renglón = proyección de Z del cuerpo en tierra.
 *  Betaflight la usa para calcular el error de corrección con el acelerómetro. */
typedef struct {
    float m[3][3];
} Matrix33_t;

/** Ángulos de Euler en grados (×10 para 1 decimal de precisión, igual que Betaflight).
 *  Convertir a grados reales: roll_deg = roll / 10.0f */
typedef struct {
    int16_t roll;    /* balanceo   eje X  (-1800..+1800 = -180°..+180°) */
    int16_t pitch;   /* cabeceo    eje Y  (-900..+900   = -90°..+90°)   */
    int16_t yaw;     /* guiñada    eje Z  (0..3600      = 0°..360°)     */
} EulerAngles_t;

/** Estado completo del AHRS */
typedef struct {
    Quaternion_t    q;              /* cuaternión actual de orientación      */
    Matrix33_t      rMat;           /* matriz de rotación (calculada de q)   */
    EulerAngles_t   attitude;       /* roll/pitch/yaw en decidegrees         */

    /* Integradores del error de Mahony */
    float   integralFBx;
    float   integralFBy;
    float   integralFBz;

    /* Ganancias dinámicas (alta al inicio para convergencia rápida) */
    float   dcmKp;
    float   dcmKi;

    /* Contador de ciclos de convergencia rápida */
    uint32_t fastKpCycles;
    bool     attitudeIsEstablished;
} ImuState_t;

/* =========================================================
 *  API PÚBLICA
 * ========================================================= */

/**
 * imuInit — inicializar el AHRS con cuaternión identidad.
 * Usar ganancia alta (Kp×10) durante los primeros 500 ms para convergencia rápida.
 */
void imuInit(ImuState_t *imu);

/**
 * imuMahonyUpdate — actualizar la estimación de orientación.
 *
 * @imu         : estado del AHRS
 * @dt          : tiempo de ciclo en segundos (ej: 0.001 a 1kHz)
 * @gx,gy,gz    : velocidad angular del giroscopio en RAD/S
 * @ax,ay,az    : lectura del acelerómetro en cualquier unidad (se normaliza internamente)
 *                si ax=ay=az=0 → solo se usa el giroscopio
 * @useAcc      : true si la lectura del acc es válida (magnitud ≈ 1g ± 10%)
 *
 * Llamar una vez por ciclo de loop, después de gyroFilter_update().
 * Salida disponible inmediatamente en imu->attitude.roll/pitch/yaw
 */
void imuMahonyUpdate(ImuState_t *imu, float dt,
                     float gx, float gy, float gz,
                     bool useAcc,
                     float ax, float ay, float az);

/**
 * imuComputeEuler — calcular ángulos de Euler desde el cuaternión actual.
 * Llamada automáticamente por imuMahonyUpdate, pero disponible por si acaso.
 */
void imuComputeEuler(ImuState_t *imu);

/**
 * imuGetRollDeg / PitchDeg / YawDeg — getters en grados flotantes.
 * Convenientes para pasar al PID.
 */
static inline float imuGetRollDeg(const ImuState_t *imu)  { return imu->attitude.roll  * 0.1f; }
static inline float imuGetPitchDeg(const ImuState_t *imu) { return imu->attitude.pitch * 0.1f; }
static inline float imuGetYawDeg(const ImuState_t *imu)   { return imu->attitude.yaw   * 0.1f; }

/**
 * imuIsHealthy — devuelve true cuando la actitud está establecida.
 * No confiar en los ángulos hasta que esto sea true.
 */
static inline bool imuIsHealthy(const ImuState_t *imu) { return imu->attitudeIsEstablished; }

/**
 * imuAccIsHealthy — verifica si la magnitud del acelerómetro es ≈ 1g.
 * Betaflight: rango válido 0.9g – 1.1g (no usar si hay vibración fuerte).
 * @ax,ay,az en [g] (ya escalados por ACC_SCALE_G)
 */
bool imuAccIsHealthy(float ax, float ay, float az);

#endif /* IMU_MAHONY_H */
