/**
 * @file main.c
 * @brief Test PID — Motor 1 únicamente
 *
 * Motor 1 (PA0, TIM2 CH1, CW Frontal-Izquierdo) corre a THROTTLE_BASE.
 * El PID del eje ROLL ajusta su velocidad según la inclinación.
 *
 * PROTECCIONES IMPLEMENTADAS:
 *  1. motor1PWM volatile  — el compilador no cachea el valor entre
 *                           el bloque PID y el bloque PWM
 *  2. clamp_us en todo    — ningún valor sale del rango 1000-2000μs
 *  3. PWM a 50Hz exacto   — sin delay bloqueante, con loopCount volatile
 *  4. lastPWM inicializado con loopCount — evita desbordamiento uint32_t
 *  5. Zona muerta integrador — Ki solo actúa si |error| < ITERM_ZONE_DEG
 *     (tomado del código de referencia)
 *  6. Watchdog IMU — si el accel no es healthy, se usa solo el gyro
 *  7. ESC siempre recibe señal — motor1PWM nunca queda sin definir
 *  8. LED indica estado exacto del sistema
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

/* ── Hardware ── */
#define MOTOR_TIM2       TIM_2
#define MOTOR_1_CHANNEL  channel_1
#define FREQUENCY        50
#define DELAY_TIM        TIM_3
#define I2C_PORT         B
#define I2C_SCL_PIN      8
#define I2C_SDA_PIN      9
#define I2C_PIN_MODE     2

#define ESC_MIN_US       1000
#define ESC_MAX_US       2000

/* LED verde integrado Nucleo F411RE — PA5 */
#define LED_PORT_ENABLE()   (RCC->AHB1ENR |= (1U << 0))
#define LED_INIT()          do { LED_PORT_ENABLE(); \
                                 GPIOA->MODER &= ~(3U << 10); \
                                 GPIOA->MODER |=  (1U << 10); } while(0)
#define LED_ON()            (GPIOA->BSRR = (1U << 5))
#define LED_OFF()           (GPIOA->BSRR = (1U << 21))

/* ── Parámetros ajustables ── */
#define THROTTLE_BASE    1100        /* velocidad base del motor (μs)     */
#define DEADBAND_DEG     3.0f        /* ángulo mínimo para activar PID    */
#define ITERM_ZONE_DEG   5.0f        /* zona muerta del integrador:       */
                                     /* Ki solo actúa si |error| < 5°    */
                                     /* (del código de referencia Arduino) */
#define UART_PRINT_MS    100         /* frecuencia de debug UART (ms)     */
#define PWM_UPDATE_MS    20          /* frecuencia de señal al ESC (ms)   */

/* ── Estado global ── */
static GyroPipeline_t  gyroPipeline;
static ImuState_t      imuState;
static PidState_t      pidState;
static MPU6050_t       mpuData;

/* volatile: leído en el bloque PID y en el bloque PWM — no cachear */
static volatile uint16_t motor1PWM  = ESC_MIN_US;

static volatile bool     pidLoopFlag = false;
static volatile uint32_t loopCount   = 0;
static uint32_t          lastPrint   = 0;

/* ── TIM5 1kHz ── */
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

/* Clamp con doble protección: float y uint16_t */
static uint16_t clamp_us(float v)
{
    if (v != v) return ESC_MIN_US;          /* NaN → mínimo seguro       */
    if (v < (float)ESC_MIN_US) return ESC_MIN_US;
    if (v > (float)ESC_MAX_US) return ESC_MAX_US;
    return (uint16_t)v;
}

void TIM5_IRQHandler(void)
{
    if (TIM5->SR & (1U << 0)) {
        TIM5->SR   &= ~(1U << 0);
        pidLoopFlag = true;
        loopCount++;
    }
}

int main(void)
{
    uart_init();
    LED_INIT();
    LED_OFF();
    uart_sendLine("=== Test PID Motor 1 ===");

    /* ── PWM motor 1 — señal mínima inmediata ── */
    pwm_init(A, MOTOR_TIM2, 0);
    pwm_setSignal(MOTOR_TIM2, MOTOR_1_CHANNEL, FREQUENCY, ESC_MIN_US);
    pwm_start(MOTOR_TIM2, MOTOR_1_CHANNEL);

    /* ── Armar ESC 5s ── */
    uart_sendLine("Armando ESC (5s)...");
    timer_init(DELAY_TIM);
    timer_delay_ms(DELAY_TIM, 5000);

    /* ── MPU6050 ── */
    mpu6050_init(I2C_PORT, I2C_SCL_PIN, I2C_SDA_PIN, I2C_PIN_MODE);
    mpu6050_config(0x1B, 0x18);   /* Gyro  ±2000°/s */
    mpu6050_config(0x1C, 0x00);   /* Accel ±2g      */
    mpu6050_config(0x1A, 0x02);   /* DLPF  98Hz     */
    uart_sendLine("MPU6050 OK");

    /* ── Inicializar flight controller ── */
    gyroPipelineInit(&gyroPipeline);
    imuInit(&imuState);
    pidInit(&pidState);

    /* ── Iniciar loop 1kHz ── */
    tim5_init_1kHz();

    /* ── Calibración (motores en mínimo) ── */
    uart_sendLine(">>> PON EL DRON QUIETO Y PLANO <<<");
    uart_sendLine("Calibrando...");
    uint32_t lastCalPrint = 0;
    while (!gyroCalibrationIsComplete(&gyroPipeline.calib)) {
        if (pidLoopFlag) {
            pidLoopFlag = false;
            mpu6050_readData(&mpuData);
            gyroPipelineUpdate(&gyroPipeline,
                               mpuData.gx, mpuData.gy, mpuData.gz);
        }
        if ((loopCount - lastCalPrint) >= 200) {
            lastCalPrint = loopCount;
            uart_sendString("CAL: ");
            uart_sendInt(gyroPipeline.calib.cyclesRemaining);
            uart_sendLine("");
        }
    }
    uart_sendLine("✓ Calibrado!");

    /* ── Converger IMU 2s ── */
    uint32_t convStart = loopCount;
    while ((loopCount - convStart) < 2000) {
        if (pidLoopFlag) {
            pidLoopFlag = false;
            mpu6050_readData(&mpuData);
            gyroPipelineUpdate(&gyroPipeline,
                               mpuData.gx, mpuData.gy, mpuData.gz);
            float ax = (float)mpuData.ax * ACC_SCALE_G;
            float ay = (float)mpuData.ay * ACC_SCALE_G;
            float az = (float)mpuData.az * ACC_SCALE_G;
            imuMahonyUpdate(&imuState, 0.001f,
                            gyroPipeline.gyroRad[AXIS_X],
                            gyroPipeline.gyroRad[AXIS_Y],
                            gyroPipeline.gyroRad[AXIS_Z],
                            imuAccIsHealthy(ax, ay, az),
                            ax, ay, az);
        }
    }
    uart_sendLine("✓ IMU listo.");

    /* ── Subir a throttle base con espera no bloqueante ── */
    uart_sendLine("Arrancando motor 1...");
    motor1PWM = THROTTLE_BASE;
    pwm_setSignal(MOTOR_TIM2, MOTOR_1_CHANNEL, FREQUENCY, THROTTLE_BASE);
    {
        uint32_t waitStart = loopCount;
        while ((loopCount - waitStart) < 1000) {
            if (pidLoopFlag) {
                pidLoopFlag = false;
                /* Motor corriendo a THROTTLE_BASE, sin PID aún */
            }
        }
    }

    uart_sendLine("PID activo.");
    uart_sendLine("Roll+ → M1 baja | Roll- → M1 sube");
    uart_sendLine("Formato: Roll | PID_R | M1_PWM");
    uart_sendLine("---------------------------------");

    /* ── Inicializar lastPWM con loopCount actual ──
     * Evita desbordamiento: si loopCount ya es >20 al entrar,
     * la diferencia podría ser enorme y mandar señal inmediata. */
    uint32_t lastPWM = loopCount;

    /* ── Main loop ── */
    while (1)
    {
        /* ── Bloque PID @ 1kHz ── */
        if (pidLoopFlag) {
            pidLoopFlag = false;

            /* 1. Leer sensor */
            mpu6050_readData(&mpuData);
            gyroPipelineUpdate(&gyroPipeline,
                               mpuData.gx, mpuData.gy, mpuData.gz);

            float ax = (float)mpuData.ax * ACC_SCALE_G;
            float ay = (float)mpuData.ay * ACC_SCALE_G;
            float az = (float)mpuData.az * ACC_SCALE_G;
            bool  accOk = imuAccIsHealthy(ax, ay, az);

            imuMahonyUpdate(&imuState, 0.001f,
                            gyroPipeline.gyroRad[AXIS_X],
                            gyroPipeline.gyroRad[AXIS_Y],
                            gyroPipeline.gyroRad[AXIS_Z],
                            accOk, ax, ay, az);

            /* 2. Detectar inclinación */
            float roll = imuGetRollDeg(&imuState);
            bool outOfLevel = (roll >  DEADBAND_DEG ||
                               roll < -DEADBAND_DEG);

            if (outOfLevel) {
                /* 3. PID con zona muerta del integrador
                 * Ki solo actúa si |error| < ITERM_ZONE_DEG
                 * (del código de referencia — evita windup grande) */
                float gyroRoll = gyroPipeline.gyroDPS[AXIS_X];

                /* Zona muerta del integrador: temporalmente ajustar
                 * Ki a 0 si el error angular es grande */
                bool inItermZone = (roll > -ITERM_ZONE_DEG &&
                                    roll <  ITERM_ZONE_DEG);
                if (!inItermZone) {
                    /* Fuera de zona: no integrar, congelar I-term */
                    float savedKi = pidState.gains[PID_AXIS_ROLL].Ki;
                    pidState.gains[PID_AXIS_ROLL].Ki = 0.0f;
                    pidUpdate(&pidState, 0.0f, 0.0f, 0.0f,
                              gyroRoll,
                              gyroPipeline.gyroDPS[AXIS_Y],
                              gyroPipeline.gyroDPS[AXIS_Z]);
                    pidState.gains[PID_AXIS_ROLL].Ki = savedKi;
                } else {
                    pidUpdate(&pidState, 0.0f, 0.0f, 0.0f,
                              gyroRoll,
                              gyroPipeline.gyroDPS[AXIS_Y],
                              gyroPipeline.gyroDPS[AXIS_Z]);
                }

                /* 4. Mixer M1 con clamp y protección NaN */
                const float PID_SCALE = 1000.0f;
                const float tNorm = ((float)THROTTLE_BASE - 1000.0f) / 1000.0f;
                float m1 = tNorm + pidState.output[PID_AXIS_ROLL].sum / PID_SCALE;
                motor1PWM = clamp_us(1000.0f + 1000.0f * m1);
                LED_ON();
            } else {
                /* En nivel: resetear I-term y volver a throttle base */
                pidResetIterm(&pidState);
                motor1PWM = THROTTLE_BASE;
                LED_OFF();
            }
        }

        /* ── Bloque PWM @ 50Hz ──
         * loopCount y motor1PWM son volatile — se leen de RAM,
         * no de un registro cacheado del compilador. */
        if ((loopCount - lastPWM) >= PWM_UPDATE_MS) {
            lastPWM = loopCount;
            pwm_setSignal(MOTOR_TIM2, MOTOR_1_CHANNEL,
                          FREQUENCY, (uint16_t)motor1PWM);
        }

        /* ── Bloque UART debug ── */
        if ((loopCount - lastPrint) >= UART_PRINT_MS) {
            lastPrint = loopCount;
            uart_sendString("Roll:");
            uart_sendFloat(imuGetRollDeg(&imuState), 1);
            uart_sendString("  PID_R:");
            uart_sendFloat(pidState.output[PID_AXIS_ROLL].sum, 2);
            uart_sendString("  M1:");
            uart_sendInt((uint16_t)motor1PWM);
            uart_sendLine("us");
        }
    }
    return 0;
}