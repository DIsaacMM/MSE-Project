/**
 * @file main.c
 * @brief Prueba de banco PID — Motores apagados en reposo
 *
 * =====================================================================
 *  CONCEPTO DE LA PRUEBA
 * =====================================================================
 *
 *  Los motores están en 1000μs (mínimo / apagados) en reposo.
 *
 *  Cuando el sensor detecta que el dron se inclinó más allá de un
 *  umbral (DEADBAND_DEG), el PID calcula una corrección y la aplica
 *  a los motores por un tiempo corto (RESPONSE_MS), luego vuelve al
 *  mínimo.
 *
 *  Esto permite ver en el UART:
 *    1. ¿El giroscopio detecta la inclinación correctamente?
 *    2. ¿El PID calcula una corrección con sentido?
 *    3. ¿Los motores correctos suben/bajan según la mezcla?
 *    4. ¿La magnitud de la corrección es proporcional al error?
 *
 *  Sin riesgo: los motores solo pulsan brevemente y con poco throttle.
 *
 * =====================================================================
 *  CÓMO HACER LA PRUEBA
 * =====================================================================
 *
 *  1. Conectar terminal serial a 115200 baud (ST-Link VCP, PA2).
 *  2. Encender. Esperar "✓ Calibración completa" (~1 segundo quieto).
 *  3. Inclinar el dron a mano lentamente.
 *  4. Observar en el terminal:
 *
 *     REPOSO:
 *       [IDLE]  ATT R: 0.1  P: 0.2  Y: 0.0  | MOT 1000 1000 1000 1000
 *
 *     AL INCLINAR hacia la derecha (+Roll):
 *       [ACTV]  ATT R: 8.3  P: 0.1  Y: 0.0
 *               PID R:-3.74 P: 0.05 Y: 0.00   ← negativo: quiere ir izq
 *               MOT  1:1039  2:1061  3:1039  4:1061  ← M2/M4 suben
 *
 *  5. Verificar que el sentido es correcto:
 *     Roll derecho (+)  → M2 y M3 deben subir (lado derecho)
 *     Roll izquierdo(-) → M1 y M4 deben subir (lado izquierdo)
 *     Pitch adelante(+) → M1 y M2 deben subir (frente)
 *     Pitch atrás   (-) → M3 y M4 deben subir (traseros)
 *
 * =====================================================================
 *  PARÁMETROS AJUSTABLES
 * =====================================================================
 *
 *  DEADBAND_DEG:  Ángulo mínimo para activar el PID (grados).
 *    Muy bajo → activa con vibraciones del piso.
 *    Muy alto → no detecta inclinaciones pequeñas.
 *    Empezar con 2.0°.
 *
 *  RESPONSE_MS:   Tiempo que los motores responden antes de volver
 *    a mínimo. 200ms es visible en el terminal sin ser peligroso.
 *
 *  TEST_THROTTLE_BASE: Throttle base durante la respuesta.
 *    1100μs = motores apenas girando. Suficiente para ver diferencias.
 *    NO subir de 1150 en prueba de banco.
 *
 *  PID_TEST_AUTHORITY: Cuánto puede variar cada motor sobre la base.
 *    ±60μs con base 1100 → rango [1040, 1160]. Visible y seguro.
 *
 * =====================================================================
 *  HARDWARE
 * =====================================================================
 *    M1: PA0 → TIM2 CH1  (CW,  Frontal-Izquierdo)
 *    M2: PA1 → TIM2 CH2  (CCW, Frontal-Derecho)
 *    M3: PB6 → TIM4 CH1  (CW,  Trasero-Derecho)
 *    M4: PB7 → TIM4 CH2  (CCW, Trasero-Izquierdo)
 *    IMU: PB8 SCL, PB9 SDA → I2C1
 *    DBG: PA2 → USART2 TX → ST-Link VCP 115200 baud
 *    Loop: TIM5 @ 1kHz
 */

#include <stdint.h>
#include <stdbool.h>
#include "PWM.h"
#include "Timer.h"
#include "MPU6050.h"
#include "UART.h"
#include "gyro_filter.h"
#include "imu_mahony.h"
#include "pid_controller.h"

/* =====================================================================
 *  HARDWARE
 * ===================================================================== */
#define MOTOR_TIM_AB    TIM_2
#define MOTOR_TIM_CD    TIM_4
#define M1_CH           channel_1   /* PA0  CW  Frontal-Izq  */
#define M2_CH           channel_2   /* PA1  CCW Frontal-Der  */
#define M3_CH           channel_1   /* PB6  CW  Trasero-Der  */
#define M4_CH           channel_2   /* PB7  CCW Trasero-Izq  */
#define DELAY_TIM       TIM_3
#define I2C_PORT        B
#define I2C_SCL_PIN     8
#define I2C_SDA_PIN     9
#define I2C_PIN_MODE    2

#define ESC_MIN_US  1000
#define ESC_MAX_US  2000

/* =====================================================================
 *  PARÁMETROS DE LA PRUEBA — AJUSTAR AQUÍ
 * ===================================================================== */

/* Ángulo mínimo para activar respuesta (grados).
 * Por debajo de esto: motores en mínimo.
 * Por arriba: PID calcula corrección. */
#define DEADBAND_DEG        0.5f

/* Throttle base cuando el PID responde.
 * 1100 = motores girando muy lento, visible pero seguro en banco.
 * NO subir de 1150 durante pruebas en banco sin hélices. */
#define TEST_THROTTLE_BASE  1100

/* Cuánto puede corregir el PID sobre TEST_THROTTLE_BASE (en μs).
 * ±60 con base 1100 → rango efectivo [1040, 1160].
 * Proporcional al ángulo gracias al PID. */
#define PID_TEST_AUTHORITY  60.0f

/* Tiempo que los motores permanecen activos después de detectar
 * inclinación, antes de volver a 1000μs (ms). */
#define RESPONSE_MS         300

/* Frecuencia de impresión UART (ms) */
#define UART_PRINT_MS       50

/* =====================================================================
 *  ESTADO GLOBAL
 * ===================================================================== */
static GyroPipeline_t   gyroPipeline;
static ImuState_t       imuState;
static PidState_t       pidState;
static MPU6050_t        mpuData;

/* PWM aplicados — guardados para debug */
static uint16_t motorPWM[4];

/* Temporizador de respuesta activa (ms restantes) */
static volatile uint32_t responseTimer = 0;

/* Contadores */
static volatile uint32_t loopCount   = 0;
static uint32_t lastUartPrint        = 0;

/* Flag ISR→main */
static volatile bool pidLoopFlag  = false;
static volatile bool systemReady   = false;

/* =====================================================================
 *  PROTOTIPOS
 * ===================================================================== */
static void flightController_init(void);
static void bench_loop(void);
static void mixer_apply(float pidRoll, float pidPitch, float pidYaw);
static void motors_off(void);
static void tim5_init_1kHz(void);
static void debug_print(bool active, float roll, float pitch, float yaw);
static uint16_t clamp_us(float v);

/* =====================================================================
 *  MAIN
 * ===================================================================== */
int main(void)
{
    uart_init();
    uart_sendLine("========================================");
    uart_sendLine("  Prueba de banco PID — Motores off");
    uart_sendLine("  Inclina el dron y observa la respuesta");
    uart_sendLine("========================================");

    /* Inicializar PWM */
    pwm_init(A, MOTOR_TIM_AB, 0);
    pwm_init(A, MOTOR_TIM_AB, 1);
    pwm_init(B, MOTOR_TIM_CD, 6);
    pwm_init(B, MOTOR_TIM_CD, 7);

    /* Armar ESCs con señal mínima */
    uart_sendLine("Armando ESCs (3s)...");
    motors_off();
    pwm_start(MOTOR_TIM_AB, M1_CH);
    pwm_start(MOTOR_TIM_AB, M2_CH);
    pwm_start(MOTOR_TIM_CD, M3_CH);
    pwm_start(MOTOR_TIM_CD, M4_CH);

    timer_init(DELAY_TIM);
    timer_delay_ms(DELAY_TIM, 3000);

    /* MPU6050 */
    uart_sendLine("Inicializando MPU6050...");
    uart_sendLine("  [1] Bus recovery...");
    mpu6050_init(I2C_PORT, I2C_SCL_PIN, I2C_SDA_PIN, I2C_PIN_MODE);
    uart_sendLine("  [2] mpu6050_init OK");

    /* Verificar comunicacion leyendo WHO_AM_I (reg 0x75, debe devolver 0x68) */
    {
        uint8_t whoami[1] = {0};
        extern void i2c_readRegDevice(uint8_t, uint8_t, uint8_t*, uint32_t);
        i2c_readRegDevice(0x68, 0x75, whoami, 1);
        uart_sendString("  [3] WHO_AM_I = 0x");
        /* Imprimir en hex */
        uint8_t hi = (whoami[0] >> 4) & 0xF;
        uint8_t lo = (whoami[0])      & 0xF;
        uart_sendChar(hi < 10 ? '0'+hi : 'A'+hi-10);
        uart_sendChar(lo < 10 ? '0'+lo : 'A'+lo-10);
        if (whoami[0] == 0x68) {
            uart_sendLine(" <- OK (MPU6050 detectado)");
        } else {
            uart_sendLine(" <- ERROR (esperado 0x68)");
        }
    }

    uart_sendLine("  [4] Configurando registros...");
    mpu6050_config(0x1B, 0x18);   /* Gyro  ±2000 deg/s */
    uart_sendLine("  [5] GYRO_CONFIG OK");
    mpu6050_config(0x1C, 0x00);   /* Accel ±2g         */
    uart_sendLine("  [6] ACCEL_CONFIG OK");
    mpu6050_config(0x1A, 0x02);   /* DLPF  98Hz        */
    uart_sendLine("  [7] DLPF OK");

    /* Flight controller */
    uart_sendLine("  [8] Inicializando flight controller...");
    flightController_init();
    uart_sendLine("  [9] Flight controller OK");

    uart_sendLine(">>> PON EL DRON QUIETO Y PLANO <<<");
    uart_sendLine("Calibrando giroscopio (~1 segundo)...");
    uart_sendLine("  [10] Iniciando TIM5...");

    /* TIM5 se inicia AL FINAL, cuando todo está listo.
     * Si se inicia antes, bench_loop() puede llamar I2C
     * mientras el sistema aún no está completamente inicializado. */
    tim5_init_1kHz();
    uart_sendLine("  [11] TIM5 OK - loop activo");
    systemReady = true;

    /* Esperar calibración — procesar el flag del TIM5 aquí también */
    uint32_t lastCalPrint = 0;
    while (!gyroCalibrationIsComplete(&gyroPipeline.calib)) {
        if (pidLoopFlag) {
            pidLoopFlag = false;
            bench_loop();
        }
        /* Imprimir progreso cada 200ms */
        if ((loopCount - lastCalPrint) >= 200) {
            lastCalPrint = loopCount;
            uart_sendString("CAL: ");
            uart_sendInt(gyroPipeline.calib.cyclesRemaining);
            uart_sendLine(" ciclos restantes");
        }
    }
    uart_sendLine("✓ Calibracion completa!");
    uart_sendLine("Sistema listo. Inclina el dron para probar.");
    uart_sendLine("Formato: [IDLE/ACTV] ATT R P Y | PID R P Y | MOT 1 2 3 4");
    uart_sendLine("----------------------------------------------------------");

    while (1)
    {
        if (pidLoopFlag)
        {
            pidLoopFlag = false;
            if (systemReady) bench_loop();
        }

        if ((loopCount - lastUartPrint) >= UART_PRINT_MS)
        {
            lastUartPrint = loopCount;
            bool active = (responseTimer > 0);
            debug_print(active,
                        imuGetRollDeg(&imuState),
                        imuGetPitchDeg(&imuState),
                        imuGetYawDeg(&imuState));
        }
    }
    return 0;
}

/* =====================================================================
 *  INICIALIZACIÓN
 * ===================================================================== */
static void flightController_init(void)
{
    gyroPipelineInit(&gyroPipeline);
    imuInit(&imuState);
    pidInit(&pidState);

    /* Límites conservadores para prueba de banco */
    for (int i = 0; i < PID_AXIS_COUNT; i++) {
        pidState.pidSumLimit[i] = 500.0f;
        pidState.itermLimit[i]  = 300.0f;
    }
    motors_off();
}

/* =====================================================================
 *  BENCH LOOP — corre a 1kHz desde la ISR
 *
 *  Comportamiento corregido:
 *
 *  DISPARO POR FLANCO (edge-trigger):
 *    Los motores solo se activan en el MOMENTO en que el dron
 *    cruza el umbral de DEADBAND_DEG, no mientras lo sostienes inclinado.
 *
 *    Esto evita que los motores corran indefinidamente si alguien
 *    sujeta el dron inclinado durante varios segundos.
 *
 *  TIEMPO MÁXIMO ABSOLUTO (MAX_RESPONSE_MS):
 *    Aunque haya múltiples disparos seguidos, los motores no pueden
 *    estar activos más de MAX_RESPONSE_MS ms consecutivos en total.
 *    Después necesitan un período de reposo (COOLDOWN_MS) antes
 *    de poder dispararse de nuevo.
 *
 *  Máquina de estados interna:
 *
 *    BENCH_IDLE ──(cruce deadband)──► BENCH_ACTIVE
 *                                          │
 *                              (responseTimer llega a 0
 *                               O maxTimer llega a 0)
 *                                          │
 *                                    BENCH_COOLDOWN
 *                                          │
 *                              (cooldownTimer llega a 0)
 *                                          │
 *                                    BENCH_IDLE ◄──────┘
 * ===================================================================== */

/* Tiempo máximo absoluto que los motores pueden estar ON (ms).
 * Protege contra perturbaciones largas. */
#define MAX_RESPONSE_MS     600

/* Tiempo mínimo de reposo entre activaciones (ms).
 * Evita que el I-term acumule windup entre disparos. */
#define COOLDOWN_MS         100

typedef enum {
    BENCH_IDLE     = 0,
    BENCH_ACTIVE   = 1,
    BENCH_COOLDOWN = 2
} BenchState_t;

static BenchState_t benchState   = BENCH_IDLE;
static bool         wasLevel      = true;  /* el dron estuvo plano antes del disparo */
static uint32_t     maxTimer     = 0;   /* límite absoluto de tiempo activo */
static uint32_t     cooldownTimer = 0;  /* tiempo de enfriamiento */

static void bench_loop(void)
{
    /* 1. Leer sensor */
    mpu6050_readData(&mpuData);

    /* 2. Gyro filter */
    bool calOk = gyroPipelineUpdate(&gyroPipeline,
                                     mpuData.gx,
                                     mpuData.gy,
                                     mpuData.gz);
    if (!calOk) { motors_off(); return; }

    /* 3. IMU Mahony */
    float ax_g = (float)mpuData.ax * ACC_SCALE_G;
    float ay_g = (float)mpuData.ay * ACC_SCALE_G;
    float az_g = (float)mpuData.az * ACC_SCALE_G;

    imuMahonyUpdate(&imuState, 0.001f,
                    gyroPipeline.gyroRad[AXIS_X],
                    gyroPipeline.gyroRad[AXIS_Y],
                    gyroPipeline.gyroRad[AXIS_Z],
                    imuAccIsHealthy(ax_g, ay_g, az_g),
                    ax_g, ay_g, az_g);

    float roll  = imuGetRollDeg(&imuState);
    float pitch = imuGetPitchDeg(&imuState);

    /* 4. Detectar si está fuera del nivel */
    bool outOfLevel = (roll  >  DEADBAND_DEG || roll  < -DEADBAND_DEG ||
                       pitch >  DEADBAND_DEG || pitch < -DEADBAND_DEG);


    /* 5. Calcular PID siempre (mantiene filtros calientes) */
    pidUpdate(&pidState,
              0.0f, 0.0f, 0.0f,
              gyroPipeline.gyroDPS[AXIS_X],
              gyroPipeline.gyroDPS[AXIS_Y],
              gyroPipeline.gyroDPS[AXIS_Z]);

    /* 6. Máquina de estados */
    switch (benchState)
    {
    case BENCH_IDLE:
        motors_off();
        /* Activar cuando esté fuera de nivel Y haya vuelto a plano al menos una vez.
         * wasLevel asegura que el dron pasó por dentro del deadband antes de
         * disparar de nuevo — evita activaciones continuas en la misma posición. */
        if (outOfLevel && wasLevel) {
            wasLevel      = false;
            responseTimer = RESPONSE_MS;
            maxTimer      = MAX_RESPONSE_MS;
            benchState    = BENCH_ACTIVE;
        }
        if (!outOfLevel) wasLevel = true;
        break;

    case BENCH_ACTIVE:
        if (responseTimer > 0) responseTimer--;
        if (maxTimer      > 0) maxTimer--;

        /* Apagar si se cumplió el tiempo de respuesta O el máximo absoluto */
        if (responseTimer == 0 || maxTimer == 0) {
            pidResetIterm(&pidState);
            motors_off();
            cooldownTimer = COOLDOWN_MS;
            benchState    = BENCH_COOLDOWN;
            break;
        }

        /* Durante el tiempo activo: aplicar corrección del PID */
        mixer_apply(pidState.output[PID_AXIS_ROLL].sum,
                    pidState.output[PID_AXIS_PITCH].sum,
                    pidState.output[PID_AXIS_YAW].sum);
        break;

    case BENCH_COOLDOWN:
        motors_off();
        if (cooldownTimer > 0) cooldownTimer--;

        /* Volver a IDLE cuando termine el enfriamiento */
        if (cooldownTimer == 0) {
            benchState = BENCH_IDLE;
        }
        break;
    }
}

/* =====================================================================
 *  MIXER — Quadcopter X
 *
 *  Base fija TEST_THROTTLE_BASE, corrección ±PID_TEST_AUTHORITY.
 *
 *         Roll  Pitch  Yaw
 *  M1 FL:  +     -      +    CW
 *  M2 FR:  -     -      -    CCW
 *  M3 RR:  -     +      +    CW
 *  M4 RL:  +     +      -    CCW
 *
 *  Ejemplo con Roll = +8° (dron ladeado a la derecha):
 *    PID Roll = negativo (quiere corregir hacia la izquierda)
 *    M1 (FL, izquierda): baja  ← roll negativo reduce M1
 *    M2 (FR, derecha):   sube  ← roll negativo aumenta M2
 *    M3 (RR, derecha):   sube
 *    M4 (RL, izquierda): baja
 *    → Lado derecho acelera, levanta ese lado → dron se nivela ✓
 * ===================================================================== */
static void mixer_apply(float pidRoll, float pidPitch, float pidYaw)
{
    const float scale = PID_TEST_AUTHORITY / 500.0f;
    float r = pidRoll  * scale;
    float p = pidPitch * scale;
    float y = pidYaw   * scale;
    float t = (float)TEST_THROTTLE_BASE;

    motorPWM[0] = clamp_us(t + r - p + y);   /* M1 FL CW  */
    motorPWM[1] = clamp_us(t - r - p - y);   /* M2 FR CCW */
    motorPWM[2] = clamp_us(t - r + p + y);   /* M3 RR CW  */
    motorPWM[3] = clamp_us(t + r + p - y);   /* M4 RL CCW */

    pwm_setSignal(MOTOR_TIM_AB, M1_CH, 50, motorPWM[0]);
    pwm_setSignal(MOTOR_TIM_AB, M2_CH, 50, motorPWM[1]);
    pwm_setSignal(MOTOR_TIM_CD, M3_CH, 50, motorPWM[2]);
    pwm_setSignal(MOTOR_TIM_CD, M4_CH, 50, motorPWM[3]);
}

static void motors_off(void)
{
    motorPWM[0] = motorPWM[1] = motorPWM[2] = motorPWM[3] = ESC_MIN_US;
    pwm_setSignal(MOTOR_TIM_AB, M1_CH, 50, ESC_MIN_US);
    pwm_setSignal(MOTOR_TIM_AB, M2_CH, 50, ESC_MIN_US);
    pwm_setSignal(MOTOR_TIM_CD, M3_CH, 50, ESC_MIN_US);
    pwm_setSignal(MOTOR_TIM_CD, M4_CH, 50, ESC_MIN_US);
}

/* =====================================================================
 *  DEBUG UART
 *
 *  [IDLE]  ATT R: 0.1  P: 0.2  Y: 0.0  | MOT 1000 1000 1000 1000
 *  [ACTV]  ATT R: 8.3  P: 0.1  Y: 0.0
 *          PID R:-3.74 P: 0.05 Y: 0.00
 *          MOT  1:1039  2:1061  3:1039  4:1061
 *
 *  Qué verificar:
 *    Roll + → M2/M3 suben, M1/M4 bajan  (corrige hacia izquierda)
 *    Roll - → M1/M4 suben, M2/M3 bajan  (corrige hacia derecha)
 *    Pitch+ → M1/M2 suben, M3/M4 bajan  (corrige hacia atrás)
 *    Pitch- → M3/M4 suben, M1/M2 bajan  (corrige hacia adelante)
 * ===================================================================== */
static void debug_print(bool active, float roll, float pitch, float yaw)
{
    if (!gyroCalibrationIsComplete(&gyroPipeline.calib)) {
        uart_sendString("CAL: ");
        uart_sendInt(gyroPipeline.calib.cyclesRemaining);
        uart_sendString(" ciclos\r\n");
        return;
    }

    /* Estado: IDLE / ACTV (con tiempo restante) / COOL (esperando) */
    if (benchState == BENCH_ACTIVE) {
        uart_sendString("[ACTV ");
        uart_sendInt((int32_t)responseTimer);
        uart_sendString("ms]");
    } else if (benchState == BENCH_COOLDOWN) {
        uart_sendString("[COOL ");
        uart_sendInt((int32_t)cooldownTimer);
        uart_sendString("ms]");
    } else {
        uart_sendString("[IDLE]     ");
    }

    uart_sendString("  ATT R:");
    uart_sendFloat(roll,  1);
    uart_sendString("  P:");
    uart_sendFloat(pitch, 1);
    uart_sendString("  Y:");
    uart_sendFloat(yaw,   1);

    if (active) {
        uart_sendString("\r\n           PID R:");
        uart_sendFloat(pidState.output[PID_AXIS_ROLL].sum,  2);
        uart_sendString("  P:");
        uart_sendFloat(pidState.output[PID_AXIS_PITCH].sum, 2);
        uart_sendString("  Y:");
        uart_sendFloat(pidState.output[PID_AXIS_YAW].sum,   2);
        uart_sendString("\r\n           MOT  1:");
        uart_sendInt(motorPWM[0]);
        uart_sendString("  2:");
        uart_sendInt(motorPWM[1]);
        uart_sendString("  3:");
        uart_sendInt(motorPWM[2]);
        uart_sendString("  4:");
        uart_sendInt(motorPWM[3]);
    }

    uart_sendString("\r\n");
}

/* =====================================================================
 *  UTILIDADES
 * ===================================================================== */
static uint16_t clamp_us(float v)
{
    if (v < (float)ESC_MIN_US) return ESC_MIN_US;
    if (v > (float)ESC_MAX_US) return ESC_MAX_US;
    return (uint16_t)v;
}

static void tim5_init_1kHz(void)
{
    RCC->APB1ENR |= (1U << 3);
    TIM5->CR1  = 0;
    TIM5->PSC  = 15;
    TIM5->ARR  = 999;
    TIM5->EGR  = 1;
    TIM5->SR   = 0;
    TIM5->DIER = (1U << 0);
    NVIC->ISER[50 >> 5] = (1U << (50 & 0x1F));
    NVIC->IP[50] = 0x10;
    TIM5->CR1 |= (1U << 0);
}

/* HardFault handler — imprime mensaje antes de morir */
void HardFault_Handler(void)
{
    extern void uart_sendLine(const char *s);
    uart_sendLine("*** HARDFAULT ***");
    while(1);
}

void TIM5_IRQHandler(void)
{
    if (TIM5->SR & (1U << 0)) {
        TIM5->SR    &= ~(1U << 0);
        pidLoopFlag  = true;
        loopCount++;
    }
}