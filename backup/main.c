/**
 * @file main.c
 * @brief Prueba CON HELICES — 1 EJE (ROLL), 2 motores, CONTINUO
 *        Lazo en cascada: angulo -> setpoint de velocidad (dps) -> PID -> us
 *
 * =====================================================================
 *  LAYOUT DEL ESC NUEVO (4IN1-ESC-DA1)
 * =====================================================================
 *  Disposicion fisica de las salidas del ESC:
 *
 *        IZQ        DER
 *      +-----+    +-----+
 *      | m3  |    | m1  |     <- renglon A
 *      +-----+    +-----+
 *      | m4  |    | m2  |     <- renglon B
 *      +-----+    +-----+
 *
 *  -> m1 y m2 estan AMBOS en la columna DERECHA.
 *  -> Para ROLL hace falta IZQUIERDA vs DERECHA, mismo renglon:
 *         m3 (IZQ)  +  m1 (DER)
 *  -> m1 y m2 juntos darian PITCH, no roll. Por eso NO se usan aqui.
 *
 *  Diferencial de roll con 2 motores:
 *     m3 IZQ = base + corr
 *     m1 DER = base - corr
 *  (Si el signo sale invertido en banco, usar -roll * LEVEL_KP_DPS_PER_DEG)
 *
 * =====================================================================
 *  FUNCIONAMIENTO
 * =====================================================================
 *  Continuo (sin pulso). Una vez LIVE:
 *    1. Lazo externo: angulo de roll -> setpoint de velocidad (dps)
 *         spRoll = roll * LEVEL_KP_DPS_PER_DEG  (limitado a LEVEL_RATE_LIMIT_DPS)
 *    2. Lazo interno: el PID lleva la velocidad real (gyroDPS) al setpoint.
 *    3. Salida del PID -> correccion en us (+-MOTOR_CORR_LIMIT_US).
 *    4. Salida final recortada a [ESC_MIN_US, THROTTLE_CAP_US].
 *
 *  El UART SIEMPRE imprime el PWM enviado a cada ESC.
 *  Tras la calibracion hay un conteo de STARTUP_COUNTDOWN_S antes de "LIVE".
 *
 * =====================================================================
 *  HARDWARE  (definido en Drone.h / Drone.c)
 * =====================================================================
 *    m1: PA0 -> TIM2 CH1   (lado DERECHO,   renglon A)
 *    m3: PB6 -> TIM4 CH1   (lado IZQUIERDO, renglon A)
 *    IMU: PB8 SCL, PB9 SDA -> I2C1
 *    DBG: PA2 -> USART2 TX -> ST-Link VCP 115200 baud
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
#include "Drone.h"

/* =====================================================================
 *  HARDWARE  (los motores vienen de Drone.h: m1, m2, m3, m4)
 *  En esta prueba de ROLL se usan SOLO m3 (IZQ) y m1 (DER).
 * ===================================================================== */
#define FREQUENCY       50
#define DELAY_TIM       TIM_3
#define I2C_PORT        B
#define I2C_SCL_PIN     8
#define I2C_SDA_PIN     9
#define I2C_PIN_MODE    2

#define ESC_MIN_US  1000
#define ESC_MAX_US  2000

/* =====================================================================
 *  PARAMETROS — AJUSTAR AQUI
 * ===================================================================== */
#define TEST_THROTTLE_BASE  1120    /* throttle base continuo (empezar bajo) */
#define THROTTLE_CAP_US     1300    /* tope DURO de salida */
#define STARTUP_COUNTDOWN_S 5       /* conteo (s) antes de que giren los motores */
#define UART_PRINT_MS       50      /* periodo de impresion UART (ms) */

/* --- Lazo externo: angulo -> setpoint de velocidad (dps) --- */
#define LEVEL_KP_DPS_PER_DEG   2.0f
#define LEVEL_RATE_LIMIT_DPS   100.0f
#define ITERM_ZONE_DEG         5.0f
#define MOTOR_CORR_LIMIT_US    60.0f

/* =====================================================================
 *  ESTADO GLOBAL
 * ===================================================================== */
static GyroPipeline_t   gyroPipeline;
static ImuState_t       imuState;
static PidState_t       pidState;
static MPU6050_t        mpuData;

/* PWM aplicados — [0]=m3 (IZQ)  [1]=m1 (DER). Siempre reflejan lo enviado. */
static uint16_t motorPWM[2];

static volatile uint32_t loopCount     = 0;
static uint32_t          lastUartPrint  = 0;

static volatile bool pidLoopFlag = false;
static volatile bool systemReady = false;
static volatile bool motorsLive  = false;

/* =====================================================================
 *  PROTOTIPOS
 * ===================================================================== */
static void flightController_init(void);
static void control_loop(void);
static void mixer_apply(float corrUS);
static void motors_off(void);
static void tim5_init_1kHz(void);
static void debug_print(float roll, float pitch, float yaw);
static uint16_t clamp_cap(float v);
static float clampf(float v, float lo, float hi);

/* =====================================================================
 *  MAIN
 * ===================================================================== */
int main(void)
{
    uart_init();
    uart_sendLine("========================================");
    uart_sendLine("  Prueba CON HELICES — ROLL — m3 + m1");
    uart_sendLine("  Layout ESC: 3-1 / 4-2  (m3=IZQ, m1=DER)");
    uart_sendLine("========================================");

    /* --- Inicializar SOLO m3 (IZQ) y m1 (DER) --- */
    m3.init(m3.gpio, m3.tim, m3.pin);
    m1.init(m1.gpio, m1.tim, m1.pin);

    /* Senal minima ANTES de start — rutina de armado probada */
    m3.setSignal(m3.tim, m3.channel, FREQUENCY, ESC_MIN_US);
    m1.setSignal(m1.tim, m1.channel, FREQUENCY, ESC_MIN_US);

    m3.start(m3.tim, m3.channel);
    m1.start(m1.tim, m1.channel);

    /* Armar — 1000us por 8s (espera pitido da-da-da) */
    uart_sendLine("Armando ESC (8s)... espera pitido da-da-da");
    timer_init(DELAY_TIM);
    timer_delay_ms(DELAY_TIM, 8000);

    /* MPU6050 */
    uart_sendLine("Inicializando MPU6050...");
    mpu6050_init(I2C_PORT, I2C_SCL_PIN, I2C_SDA_PIN, I2C_PIN_MODE);

    {
        uint8_t whoami[1] = {0};
        extern void i2c_readRegDevice(uint8_t, uint8_t, uint8_t*, uint32_t);
        i2c_readRegDevice(0x68, 0x75, whoami, 1);
        uart_sendString("WHO_AM_I = 0x");
        uint8_t hi = (whoami[0] >> 4) & 0xF;
        uint8_t lo = (whoami[0])      & 0xF;
        uart_sendChar(hi < 10 ? '0'+hi : 'A'+hi-10);
        uart_sendChar(lo < 10 ? '0'+lo : 'A'+lo-10);
        uart_sendLine(whoami[0] == 0x68 ? " <- OK" : " <- ERROR (esperado 0x68)");
    }

    mpu6050_config(0x1B, 0x18);   /* Gyro  +-2000 deg/s */
    mpu6050_config(0x1C, 0x00);   /* Accel +-2g         */
    mpu6050_config(0x1A, 0x02);   /* DLPF  98Hz         */
    uart_sendLine("MPU6050 configurado.");

    flightController_init();
    uart_sendLine("Flight controller OK.");

    uart_sendLine(">>> PON EL DRON QUIETO Y PLANO <<<");
    uart_sendLine("Calibrando giroscopio (~1 segundo)...");

    tim5_init_1kHz();
    systemReady = true;

    /* Esperar calibracion (motores en minimo, motorsLive=false) */
    uint32_t lastCalPrint = 0;
    while (!gyroCalibrationIsComplete(&gyroPipeline.calib)) {
        if (pidLoopFlag) { pidLoopFlag = false; control_loop(); }
        if ((loopCount - lastCalPrint) >= 200) {
            lastCalPrint = loopCount;
            uart_sendString("CAL: ");
            uart_sendInt(gyroPipeline.calib.cyclesRemaining);
            uart_sendLine(" ciclos restantes");
        }
    }
    uart_sendLine("OK Calibracion completa!");

    /* ---- Conteo de arranque ---- */
    uart_sendLine("");
    uart_sendLine("Los motores giraran al terminar el conteo.");
    {
        uint32_t liveAt    = loopCount + (uint32_t)STARTUP_COUNTDOWN_S * 1000U;
        int32_t  lastShown = -1;
        while (loopCount < liveAt) {
            if (pidLoopFlag) { pidLoopFlag = false; control_loop(); }
            int32_t secLeft = (int32_t)((liveAt - loopCount + 999U) / 1000U);
            if (secLeft != lastShown) {
                lastShown = secLeft;
                uart_sendString("  MOTORES ON EN ");
                uart_sendInt(secLeft);
                uart_sendLine(" ...");
            }
        }
    }
    motorsLive = true;
    uart_sendLine(">>> LIVE — PID activo en ROLL <<<");
    uart_sendLine("Formato: [ESTADO] ATT R P Y | PID R | MOT L(m3) R(m1)");
    uart_sendLine("----------------------------------------------------------");

    while (1)
    {
        if (pidLoopFlag) {
            pidLoopFlag = false;
            if (systemReady) control_loop();
        }

        if ((loopCount - lastUartPrint) >= UART_PRINT_MS) {
            lastUartPrint = loopCount;
            debug_print(imuGetRollDeg(&imuState),
                        imuGetPitchDeg(&imuState),
                        imuGetYawDeg(&imuState));
        }
    }
    return 0;
}

/* =====================================================================
 *  INICIALIZACION
 * ===================================================================== */
static void flightController_init(void)
{
    gyroPipelineInit(&gyroPipeline);
    imuInit(&imuState);
    pidInit(&pidState);

    for (int i = 0; i < PID_AXIS_COUNT; i++) {
        pidState.pidSumLimit[i] = 500.0f;
        pidState.itermLimit[i]  = 300.0f;
    }
    motors_off();
}

/* =====================================================================
 *  CONTROL LOOP — corre a 1kHz desde la ISR. CONTINUO.
 * ===================================================================== */
static void control_loop(void)
{
    /* 1. Leer sensor */
    mpu6050_readData(&mpuData);

    /* 2. Gyro filter */
    bool calOk = gyroPipelineUpdate(&gyroPipeline,
                                     mpuData.gx, mpuData.gy, mpuData.gz);
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

    float roll = imuGetRollDeg(&imuState);

    /* 4. Lazo externo: angulo -> setpoint de velocidad (dps) */
    float spRoll = clampf(roll * LEVEL_KP_DPS_PER_DEG,
                          -LEVEL_RATE_LIMIT_DPS, LEVEL_RATE_LIMIT_DPS);

    /* Zona muerta del integrador: congela Ki si el angulo es grande */
    bool  inItermZone = (roll > -ITERM_ZONE_DEG && roll < ITERM_ZONE_DEG);
    float savedKi     = pidState.gains[PID_AXIS_ROLL].Ki;
    if (!inItermZone) pidState.gains[PID_AXIS_ROLL].Ki = 0.0f;

    pidUpdate(&pidState, spRoll, 0.0f, 0.0f,
              gyroPipeline.gyroDPS[AXIS_X],
              gyroPipeline.gyroDPS[AXIS_Y],
              gyroPipeline.gyroDPS[AXIS_Z]);

    pidState.gains[PID_AXIS_ROLL].Ki = savedKi;   /* restaurar */

    /* 5. Aplicar (o mantener minimo si aun no LIVE) */
    if (motorsLive) {
        float corrRoll = clampf(pidState.output[PID_AXIS_ROLL].sum,
                                -MOTOR_CORR_LIMIT_US, MOTOR_CORR_LIMIT_US);
        mixer_apply(corrRoll);
    } else {
        motors_off();
    }
}

/* =====================================================================
 *  MIXER — ROLL, 2 motores. Salida recortada a THROTTLE_CAP_US.
 *    m3 IZQ = base + corr
 *    m1 DER = base - corr
 * ===================================================================== */
static void mixer_apply(float corrUS)
{
    float t = (float)TEST_THROTTLE_BASE;

    motorPWM[0] = clamp_cap(t + corrUS);   /* m3 IZQ */
    motorPWM[1] = clamp_cap(t - corrUS);   /* m1 DER */

    m3.setSignal(m3.tim, m3.channel, FREQUENCY, motorPWM[0]);
    m1.setSignal(m1.tim, m1.channel, FREQUENCY, motorPWM[1]);
}

static void motors_off(void)
{
    motorPWM[0] = motorPWM[1] = ESC_MIN_US;
    m3.setSignal(m3.tim, m3.channel, FREQUENCY, ESC_MIN_US);
    m1.setSignal(m1.tim, m1.channel, FREQUENCY, ESC_MIN_US);
}

/* =====================================================================
 *  DEBUG UART — SIEMPRE imprime el PWM enviado a cada ESC
 *
 *  [WAIT]  ATT R: 0.1 P: 0.2 Y: 0.0  PID R: 0.00  MOT L:1000 R:1000
 *  [LIVE]  ATT R: 8.3 P: 0.1 Y: 0.0  PID R:28.40  MOT L:1148 R:1092
 *           L = m3 (IZQ)   R = m1 (DER)
 * ===================================================================== */
static void debug_print(float roll, float pitch, float yaw)
{
    if (!gyroCalibrationIsComplete(&gyroPipeline.calib)) {
        uart_sendString("CAL: ");
        uart_sendInt(gyroPipeline.calib.cyclesRemaining);
        uart_sendString(" ciclos\r\n");
        return;
    }

    uart_sendString(motorsLive ? "[LIVE] " : "[WAIT] ");

    uart_sendString(" ATT R:");
    uart_sendFloat(roll,  1);
    uart_sendString(" P:");
    uart_sendFloat(pitch, 1);
    uart_sendString(" Y:");
    uart_sendFloat(yaw,   1);

    uart_sendString("  PID R:");
    uart_sendFloat(pidState.output[PID_AXIS_ROLL].sum, 2);

    /* SIEMPRE el PWM enviado al ESC */
    uart_sendString("  MOT L(m3):");
    uart_sendInt(motorPWM[0]);
    uart_sendString(" R(m1):");
    uart_sendInt(motorPWM[1]);

    uart_sendString("\r\n");
}

/* =====================================================================
 *  UTILIDADES
 * ===================================================================== */
static uint16_t clamp_cap(float v)
{
    if (v < (float)ESC_MIN_US)      return ESC_MIN_US;
    if (v > (float)THROTTLE_CAP_US) return THROTTLE_CAP_US;
    return (uint16_t)v;
}

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
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