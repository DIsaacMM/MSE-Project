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
#include "UART.h"
#include "gyro_filter.h"
#include "imu_mahony.h"
#include "pid_controller.h"
#include "Drone.h"
#include "mpu6050.h"

/* ── Hardware ── */
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
#define LEVEL_KP_DPS_PER_DEG  2.0f   /* ganancia P para nivelar (dps/°)   */
#define LEVEL_RATE_LIMIT_DPS  120.0f /* límite de setpoint para nivelar   */
#define MOTOR_CORR_LIMIT_US 250.0f   /* límite de corrección PID al motor (μs) */
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
static float             rollSetpointDps = 0.0f;
static float             motorCorrectionUS = 0.0f;

/* ── TIM5 1kHz ── */
static void TIM5_PID(void)
{
    tim_init();

    tim_initTimer(TIM_5);
    tim_setTimerFreq(TIM_5, 1000);

    /* Habilitar interrupción por update */
    TIM[TIM_5]->DIER |= TIM_DIER_UIE;

    /* Configurar NVIC para TIM5 */
    NVIC_EnableIRQ(TIM5_IRQn);
    NVIC_SetPriority(TIM5_IRQn, 1);

    /* Arrancar timer */
    tim_enableTimer(TIM_5);
}

/* Clamp con doble protección: float y uint16_t */
static float clamp_us(float v, float min, float max)
{
    if (v != v) return 0.0f;          /* NaN → mínimo seguro       */
    if (v < min) return min;
    if (v > max) return max;
    return v;
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
    uart_sendLine("=== Test PID Motor ===");

    /* ── Initialize Motors ── */
    drone_init(); 

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

    /* ── Iniciar loop 1kHz para el PID ── */

    TIM5_PID();

    /* ── Calibración (motores en mínimo) ── */
    uart_sendLine(">>> PON EL DRON QUIETO Y PLANO <<<");
    uart_sendLine("Calibrando...");
    uint32_t lastCalPrint = 0;
    while (!gyroCalibrationIsComplete(&gyroPipeline.calib)) 
    {
        if (pidLoopFlag) 
        {
            pidLoopFlag = false;
            mpu6050_readData(&mpuData);
            gyroPipelineUpdate(&gyroPipeline, mpuData.gx, mpuData.gy, mpuData.gz);
        }
        if ((loopCount - lastCalPrint) >= 200) 
        {
            lastCalPrint = loopCount;
            uart_sendString("CAL: ");
            uart_sendInt(gyroPipeline.calib.cyclesRemaining);
            uart_sendLine("");
        }
    }
    uart_sendLine("=== Calibrado ===");

    /* ── Converger IMU 2s ── */
    uint32_t convStart = loopCount;
    while ((loopCount - convStart) < 2000) 
    {
        if (pidLoopFlag) 
        {
            pidLoopFlag = false;
            mpu6050_readData(&mpuData);
            gyroPipelineUpdate(&gyroPipeline, mpuData.gx, mpuData.gy, mpuData.gz);
            float ax = (float)mpuData.ax * ACC_SCALE_G;
            float ay = (float)mpuData.ay * ACC_SCALE_G;
            float az = (float)mpuData.az * ACC_SCALE_G;
            imuMahonyUpdate(&imuState, 0.001f, gyroPipeline.gyroRad[AXIS_X], gyroPipeline.gyroRad[AXIS_Y], gyroPipeline.gyroRad[AXIS_Z], imuAccIsHealthy(ax, ay, az), ax, ay, az);
        }
    }
    uart_sendLine("=== IMU listo ===");

    /* ── Subir a throttle base con espera no bloqueante ── */
    uart_sendLine("Arrancando Motores...");
    motor1PWM = THROTTLE_BASE;
    m1.setSignal(m1.tim, m1.channel, FREQUENCY, THROTTLE_BASE); 
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
        if (pidLoopFlag) 
        {
            pidLoopFlag = false;

            /* 1. Leer sensor */
            mpu6050_readData(&mpuData);
            gyroPipelineUpdate(&gyroPipeline, mpuData.gx, mpuData.gy, mpuData.gz);

            float ax = (float)mpuData.ax * ACC_SCALE_G;
            float ay = (float)mpuData.ay * ACC_SCALE_G;
            float az = (float)mpuData.az * ACC_SCALE_G;
            bool  accOk = imuAccIsHealthy(ax, ay, az);

            imuMahonyUpdate(&imuState, 0.001f, gyroPipeline.gyroRad[AXIS_X], gyroPipeline.gyroRad[AXIS_Y], gyroPipeline.gyroRad[AXIS_Z], accOk, ax, ay, az);

            /* 2. Detectar inclinación */
            float roll = imuGetRollDeg(&imuState);
            bool outOfLevel = (roll >  DEADBAND_DEG || roll < -DEADBAND_DEG);

            if (outOfLevel) 
            {
                /* 3. PID con zona muerta del integrador
                 * Ki solo actúa si |error| < ITERM_ZONE_DEG
                 * (del código de referencia — evita windup grande) */
                float gyroRoll = gyroPipeline.gyroDPS[AXIS_X];

                /* Zona muerta del integrador: temporalmente ajustar
                 * Ki a 0 si el error angular es grande */
                bool inItermZone = (roll > -ITERM_ZONE_DEG && roll <  ITERM_ZONE_DEG);
                if (!inItermZone) 
                {
                    /* Fuera de zona: no integrar, congelar I-term */
                    float savedKi = pidState.gains[PID_AXIS_ROLL].Ki;
                    pidState.gains[PID_AXIS_ROLL].Ki = 0.0f;
                    rollSetpointDps = clamp_us(roll * LEVEL_KP_DPS_PER_DEG, -LEVEL_RATE_LIMIT_DPS, +LEVEL_RATE_LIMIT_DPS),
                    pidUpdate(&pidState, rollSetpointDps, 0.0f, 0.0f, gyroRoll, gyroPipeline.gyroDPS[AXIS_Y], gyroPipeline.gyroDPS[AXIS_Z]);
                    pidState.gains[PID_AXIS_ROLL].Ki = savedKi;
                } 
                else 
                {
                    pidUpdate(&pidState, rollSetpointDps, 0.0f, 0.0f, gyroRoll, gyroPipeline.gyroDPS[AXIS_Y], gyroPipeline.gyroDPS[AXIS_Z]);
                }

                /* 4. Mixer M1 con clamp y protección NaN */
                motorCorrectionUS = clamp_us(pidState.output[PID_AXIS_ROLL].sum, -MOTOR_CORR_LIMIT_US, MOTOR_CORR_LIMIT_US);
                motor1PWM = clamp_us((float)THROTTLE_BASE + motorCorrectionUS,ESC_MIN_US, ESC_MAX_US);
                LED_ON();
                rollSetpointDps = 0.0f; /* nivelar → setpoint 0 dps */
                motorCorrectionUS = 0.0f; /* resetear corrección si NaN */
            } 
            else 
            {
                /* En nivel: resetear I-term y volver a throttle base */
                pidResetIterm(&pidState);
                motor1PWM = THROTTLE_BASE;
                LED_OFF();
            }
        }

        /* ── Bloque PWM @ 50Hz ──
         * loopCount y motor1PWM son volatile — se leen de RAM,
         * no de un registro cacheado del compilador. */
        if ((loopCount - lastPWM) >= PWM_UPDATE_MS) 
        {
            lastPWM = loopCount;
            m1.setSignal(m1.tim, m1.channel, FREQUENCY, (uint16_t)motor1PWM); 
        }

        /* ── Bloque UART debug ── */
        if ((loopCount - lastPrint) >= UART_PRINT_MS) 
        {
            lastPrint = loopCount;
            uart_sendString("Roll:");
            uart_sendFloat(imuGetRollDeg(&imuState), 1);
            uart_sendString("  PID_R:");
            uart_sendString("   SP_R:");
            uart_sendFloat(rollSetpointDps, 1);
            uart_sendFloat(pidState.output[PID_AXIS_ROLL].sum, 2);
            uart_sendString("  M1:");
            uart_sendInt((uint16_t)motor1PWM);
            uart_sendLine("us");
        }
    }
    return 0;
}