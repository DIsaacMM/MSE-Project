/**
 * @file PID.c
 * @brief PID Balance Controller using MPU-6050
 *
 * Implementación del controlador PID para estabilización de dron.
 * Usa los drivers PWM.h, mpu6050.h y Timer.h 
 *
 * Flujo por ciclo (PID_Update):
 *   1. Calcular elapsedTime con Timer
 *   2. Leer acelerómetro y giroscopio del MPU-6050 (mpu6050.h)
 *   3. Calcular ángulos con filtro complementario
 *   4. Calcular P, I, D y sumarlos
 *   5. Calcular PWM de cada motor y enviarlo
 *
 * @authors
 * David Mijares
 * Aldo De la Torre
 * Jose Paez
 */

#include "PID.h"


/* ===========================================================================
 * CONSTANTES MATEMÁTICAS
 * =========================================================================== */
#define RAD_TO_DEG  (180.0f / 3.141592654f)

/* Sensibilidades del MPU-6050 en rango por defecto (±2g / ±250°/s) */
#define ACCEL_SCALE 16384.0f   /* LSB/g   → datasheet MPU-6050 */
#define GYRO_SCALE  131.0f     /* LSB/°/s → datasheet MPU-6050 */

/* Coeficientes del filtro complementario */
#define COMP_GYRO   0.98f
#define COMP_ACCEL  0.02f

/* ===========================================================================
 * VARIABLES INTERNAS (privadas al módulo)
 * =========================================================================== */

/* Ángulos calculados */
static float s_accel_angle[2] = {0.0f, 0.0f};
static float s_gyro_rate[2]   = {0.0f, 0.0f};
static float s_total_angle[2] = {0.0f, 0.0f};

/* Tiempo */
static uint32_t s_time_prev    = 0;
static float    s_elapsed_time = 0.0f;

/* Términos PID */
static float s_error     = 0.0f;
static float s_prev_error= 0.0f;
static float s_pid_p     = 0.0f;
static float s_pid_i     = 0.0f;
static float s_pid_d     = 0.0f;
static float s_pid_out   = 0.0f;

/* ===========================================================================
 * FUNCIONES AUXILIARES PRIVADAS
 * =========================================================================== */

/**
 * @brief Limita un valor float entre [min, max].
 */
static float clampf(float val, float min, float max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}



static float fast_sqrt(float x)
{
    if (x <= 0.0f)
        return 0.0f;

    float guess = x;

    for (uint8_t i = 0; i < 8; i++)
    {
        guess = 0.5f * (guess + x / guess);
    }

    return guess;
}

static float fast_atan2(float y, float x)
{
    const float PI = 3.14159265f;

    if (x == 0.0f)
    {
        if (y > 0.0f) return PI * 0.5f;
        if (y < 0.0f) return -PI * 0.5f;
        return 0.0f;
    }

    float angle;
    float z = y / x;

    if (z < -1.0f || z > 1.0f)
    {
        angle = PI * 0.5f - z / (z * z + 0.28f);

        if (y < 0.0f)
            angle -= PI;
    }
    else
    {
        angle = z / (1.0f + 0.28f * z * z);

        if (x < 0.0f)
        {
            if (y < 0.0f)
                angle -= PI;
            else
                angle += PI;
        }
    }

    return angle;
}


/**
 * @brief Lee el MPU-6050 y calcula los ángulos del filtro complementario.
 *
 * Usa las funciones de mpu6050.h (sensor_readAccel / sensor_readGyro)
 *
 * Convención de ejes:
 *   index 0 → eje X (roll)
 *   index 1 → eje Y (pitch)  ← eje usado por el PID de balance
 */
static void imu_update(void)
{
    /* -----------------------------------------------------------------
     * Leer datos crudos del sensor
     * Ajusta los nombres de función según tu mpu6050.h
     * ----------------------------------------------------------------- */
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw;

    mpu6050_readAccel(&ax_raw, &ay_raw, &az_raw);  /* función de mpu6050.h */
    mpu6050_readGyro (&gx_raw, &gy_raw, NULL);     /* función de mpu6050.h */

    /* -----------------------------------------------------------------
     * Convertir acelerómetro a "g" y calcular ángulos
     * ----------------------------------------------------------------- */
    float ax = ax_raw / ACCEL_SCALE;
    float ay = ay_raw / ACCEL_SCALE;
    float az = az_raw / ACCEL_SCALE;

s_accel_angle[0] = fast_atan2(ay, fast_sqrt(ax * ax + az * az)) * RAD_TO_DEG;

s_accel_angle[1] = fast_atan2(-ax, fast_sqrt(ay * ay + az * az)) * RAD_TO_DEG;

    /* -----------------------------------------------------------------
     * Convertir giroscopio a grados/segundo
     * ----------------------------------------------------------------- */
    s_gyro_rate[0] = gx_raw / GYRO_SCALE;
    s_gyro_rate[1] = gy_raw / GYRO_SCALE;

    /* -----------------------------------------------------------------
     * Filtro complementario
     * Total = 98 % integración giroscopio + 2 % acelerómetro
     * ----------------------------------------------------------------- */
    s_total_angle[0] = COMP_GYRO * (s_total_angle[0] + s_gyro_rate[0] * s_elapsed_time)
                     + COMP_ACCEL * s_accel_angle[0];

    s_total_angle[1] = COMP_GYRO * (s_total_angle[1] + s_gyro_rate[1] * s_elapsed_time)
                     + COMP_ACCEL * s_accel_angle[1];
}


/**
 * @brief Inicializa el módulo PID.
 *
 * IMPORTANTE: Llama esto DESPUÉS de:
 *   - pwm_init() de los 4 motores
 *   - timer_init(DELAY_TIM)
 *
 * Secuencia:
 *   1. Inicializa sensor MPU-6050
 *   2. Envía mínimo a los 4 ESC y arranca PWM
 *   3. Espera armado de ESC (3 s)
 *   4. Inicializa variables de estado
 */
void PID_Init(void)
{
    /* Inicializar sensor MPU-6050 */
    mpu6050_init();   

    // Init al motors
    drone_init(); 

    timer_init(DELAY_TIM); 
    // Wait for ESC startup
    timer_delay_ms(DELAY_TIM, 3000);

    /* Inicializar estado interno */
    s_total_angle[0] = 0.0f;
    s_total_angle[1] = 0.0f;
    s_prev_error     = 0.0f;
    s_pid_i          = 0.0f;

    /* Capturar tiempo inicial usando el contador del timer */
    s_time_prev = timer_get_ms(DELAY_TIM);  /* función de Timer.h */
}

/**
 * @brief Ejecuta un ciclo completo del PID.
 *
 * Llamar en el while(1) del main SIN delays adicionales,
 * ya que el timing lo maneja internamente con timer_get_ms().
 */
void PID_Update(void)
{
    /* =================================================================
     * 1. TIEMPO TRANSCURRIDO
     * ================================================================= */
    uint32_t current_time = timer_get_ms(DELAY_TIM);  /* función de Timer.h */
    s_elapsed_time = (float)(current_time - s_time_prev) / 1000.0f;
    s_time_prev    = current_time;

    /* Protección: si el tiempo es 0 o negativo, saltar ciclo */
    if (s_elapsed_time <= 0.0f) return;

    /* =================================================================
     * 2. LECTURA IMU + FILTRO COMPLEMENTARIO
     * ================================================================= */
    imu_update();

    /* =================================================================
     * 3. CÁLCULO PID
     *
     * Se usa el eje Y (pitch/index 1) para balance.
     * El eje X del IMU debe quedar paralelo al eje de balance.
     * ================================================================= */
    s_error = s_total_angle[1] - PID_DESIRED_ANGLE;

    /* -- Proporcional -- */
    s_pid_p = PID_KP * s_error;

    /* -- Integral (solo actúa en zona de ±3° para evitar windup) -- */
    if (s_error > -3.0f && s_error < 3.0f)
    {
        s_pid_i += PID_KI * s_error;
    }
    s_pid_i = clampf(s_pid_i, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);

    /* -- Derivativo -- */
    s_pid_d = PID_KD * ((s_error - s_prev_error) / s_elapsed_time);

    /* -- Suma total -- */
    s_pid_out = s_pid_p + s_pid_i + s_pid_d;
    s_pid_out = clampf(s_pid_out, -PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);

    /* =================================================================
     * 4. CÁLCULO Y ENVÍO DE PWM A LOS 4 MOTORES
     *
     * Convención:
     *   Motor 1 y 2 (izquierda/derecha en un eje)  → throttle + PID
     *   Motor 3 y 4 (izquierda/derecha en el otro) → throttle - PID
     *
     *   Ajusta el signo según la orientación física de tu dron.
     * ================================================================= */
    float pwm1 = clampf(PID_THROTTLE + s_pid_out, PID_ESC_MIN, PID_ESC_MAX);
    float pwm2 = clampf(PID_THROTTLE - s_pid_out, PID_ESC_MIN, PID_ESC_MAX);
    float pwm3 = clampf(PID_THROTTLE + s_pid_out, PID_ESC_MIN, PID_ESC_MAX);
    float pwm4 = clampf(PID_THROTTLE - s_pid_out, PID_ESC_MIN, PID_ESC_MAX);

    m1.setSignal(m1.tim, m1.channel, 50, (uint16_t)pwm1); 
    m2.setSignal(m2.tim, m2.channel, 50, (uint16_t)pwm2); 
    m3.setSignal(m3.tim, m3.channel, 50, (uint16_t)pwm3); 
    m4.setSignal(m4.tim, m4.channel,50, (uint16_t)pwm4); 
    timer_delay_ms(DELAY_TIM, 20);


    /* =================================================================
     * 5. GUARDAR ERROR PARA EL SIGUIENTE CICLO
     * ================================================================= */
    s_prev_error = s_error;
}