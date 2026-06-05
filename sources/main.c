/**
 * @file main.c
 * @brief Flight Controller — 4 motores con PID (vuelo), usando Drone.h
 *
 * Layout ESC nuevo (4IN1-ESC-DA1)  — orden 3-1 / 4-2  (FRENTE = ARRIBA):
 *
 *        IZQ        DER
 *      | m3 |     | m1 |      FRENTE   (FL=m3, FR=m1)
 *      | m4 |     | m2 |      ATRAS    (RL=m4, RR=m2)
 *
 * MIXER Quad-X (base + correcciones):
 *            Roll  Pitch  Yaw
 *   m3 FL:    +     -      -
 *   m1 FR:    -     -      +
 *   m4 RL:    +     +      +
 *   m2 RR:    -     +      -
 *
 * GIRO (yaw): diagonales co-rotan -> en AM32, m3 & m2 un sentido,
 *             m1 & m4 el opuesto.  (verificar en banco)
 *
 *   m1: PA0 -> TIM2 CH1   m2: PA1 -> TIM2 CH2
 *   m3: PB6 -> TIM4 CH1   m4: PB7 -> TIM4 CH2
 *
 * PID corre continuo a 1 kHz y escribe motorPWM[].
 * El PWM fisico se envia a los ESC cada PWM_UPDATE_MS (20 ms / 50 Hz).
 *
 * CAMBIOS vs version anterior:
 *   - Corregido signo de spPitch (era +pitch, ahora -pitch)
 *   - Eliminada zona muerta manual del integrador (reemplazada por iterm_relax en PID)
 *   - pidUpdate() recibe throttleNow para TPA
 *   - Rampa de arranque movida al lugar correcto (antes del vuelo activo)
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

/* ── Hardware ── */
#define FREQUENCY        50
#define DELAY_TIM        TIM_3
#define I2C_PORT         B
#define I2C_SCL_PIN      8
#define I2C_SDA_PIN      9
#define I2C_PIN_MODE     2
#define ESC_MIN_US       1000
#define ESC_MAX_US       2000

/* LED (PA5 / LD2) */
#define LED_PORT_ENABLE()   (RCC->AHB1ENR |= (1U << 0))
#define LED_INIT()          do { LED_PORT_ENABLE(); \
                                 GPIOA->MODER &= ~(3U << 10); \
                                 GPIOA->MODER |=  (1U << 10); } while(0)
#define LED_ON()            (GPIOA->BSRR = (1U << 5))
#define LED_OFF()           (GPIOA->BSRR = (1U << 21))

/* ── Parametros ajustables ── */
#define THROTTLE_BASE         1365
#define LEVEL_KP_DPS_PER_DEG  1.0f     /* lazo externo angulo -> dps (mas suave en 7") */
#define LEVEL_RATE_LIMIT_DPS  120.0f   /* limite del setpoint en dps                   */
#define MOTOR_CORR_LIMIT_US   200.0f   /* limite de correccion por motor (reducido 7") */
#define UART_PRINT_MS         100
#define PWM_UPDATE_MS         20       /* envio fisico a ESC (50 Hz)                   */
#define FLIGHT_TIME_MS        15000    /* vuela este tiempo y luego aterriza           */
#define LANDING_TIME_MS       3000    /* duracion del descenso gradual                */
#define PITCH_Trim_deg        6.0f     /* Ajuste fino del trim de pitch (en grados)    */

/* Rampa de arranque */
#define RAMP_STEP_MS          20       /* un paso cada 20 ms                           */

/* ── Filtro PT1 para acelerometro ── */
#define ACCEL_LPF_HZ          15.0f    /* corta vibraciones de motor antes de Mahony */

/* ── Estado global ── */
static GyroPipeline_t  gyroPipeline;
static ImuState_t      imuState;
static PidState_t      pidState;
static MPU6050_t       mpuData;

static volatile uint16_t motorPWM[4] = {
    ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US
};
static volatile bool     pidLoopFlag = false;
static volatile uint32_t loopCount   = 0;
static uint32_t          lastPrint   = 0;

/* Filtros PT1 por eje del acelerometro — NUEVO */
static PT1Filter_t     accelFilterX;
static PT1Filter_t     accelFilterY;
static PT1Filter_t     accelFilterZ;

/* Protocolo de aterrizaje */
static uint32_t          flightStart = 0;
static bool              disarmed    = false;
static float             throttleNow = (float)ESC_MIN_US;

/* Setpoints (dps) y correcciones (us) por eje — para debug */
static float spRoll  = 0.0f, spPitch = 0.0f;
static float corrRoll = 0.0f, corrPitch = 0.0f, corrYaw = 0.0f;

/* ── TIM5 1kHz ── */
static void tim5_init_1kHz(void)
{
    RCC->APB1ENR |= (1U << 3);
    TIM5->CR1 = 0; TIM5->PSC = 15; TIM5->ARR = 999;
    TIM5->EGR = 1; TIM5->SR  = 0;  TIM5->DIER = (1U << 0);
    NVIC->ISER[50 >> 5] = (1U << (50 & 0x1F));
    NVIC->IP[50] = 0x10;
    TIM5->CR1 |= (1U << 0);
}

/* Clamp float con proteccion NaN */
static float clampf(float v, float mn, float mx)
{
    if (v != v) return mn;
    if (v < mn)  return mn;
    if (v > mx)  return mx;
    return v;
}

/* Clamp a uint16_t con proteccion NaN */
static uint16_t clamp_us(float v)
{
    if (v != v)                return ESC_MIN_US;
    if (v < (float)ESC_MIN_US) return ESC_MIN_US;
    if (v > (float)ESC_MAX_US) return ESC_MAX_US;
    return (uint16_t)v;
}

/* Envio fisico de motorPWM[] a los 4 ESC (via Drone.h) */
static void motors_write(void)
{
    m1.setSignal(m1.tim, m1.channel, FREQUENCY, (uint16_t)motorPWM[0]);
    m2.setSignal(m2.tim, m2.channel, FREQUENCY, (uint16_t)motorPWM[1]);
    m3.setSignal(m3.tim, m3.channel, FREQUENCY, (uint16_t)motorPWM[2]);
    m4.setSignal(m4.tim, m4.channel, FREQUENCY, (uint16_t)motorPWM[3]);
}

static void motors_off(void)
{
    motorPWM[0] = motorPWM[1] = motorPWM[2] = motorPWM[3] = ESC_MIN_US;
    motors_write();
}

static void motors_set_all(uint16_t us)
{
    motorPWM[0] = motorPWM[1] = motorPWM[2] = motorPWM[3] = us;
    motors_write();
}

/* Rampa suave de ESC_MIN_US hasta target_us en pasos de 1 us cada RAMP_STEP_MS.
 * Sigue procesando pidLoopFlag para no perder el 1kHz durante la rampa. */
static void motors_ramp_to(uint16_t target_us)
{
    for (uint16_t v = ESC_MIN_US; v < target_us; v++) {
        m1.setSignal(m1.tim, m1.channel, FREQUENCY, v);
        m2.setSignal(m2.tim, m2.channel, FREQUENCY, v);
        m3.setSignal(m3.tim, m3.channel, FREQUENCY, v);
        m4.setSignal(m4.tim, m4.channel, FREQUENCY, v);
        timer_delay_ms(DELAY_TIM, RAMP_STEP_MS);
        /* Drenar el flag del PID para no acumular deuda */
        if (pidLoopFlag) pidLoopFlag = false;
    }
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
    uart_sendLine("=== Flight Controller 4 Motores (Drone.h) ===");
    LED_INIT();
    LED_OFF();

    /* ── Inicializar los 4 motores via Drone.h ── */
    drone_init();

    /* ── Armar ESCs (8s — espera pitido da-da-da) ── */
    uart_sendLine("Armando ESCs (8s)...");
    timer_init(DELAY_TIM);
    timer_delay_ms(DELAY_TIM, 5000);

    /* ── MPU6050 ── */
    mpu6050_init(I2C_PORT, I2C_SCL_PIN, I2C_SDA_PIN, I2C_PIN_MODE);
    mpu6050_config(0x1B, 0x18);   /* Gyro  +-2000 deg/s */
    mpu6050_config(0x1C, 0x00);   /* Accel +-2g         */
    mpu6050_config(0x1A, 0x02);   /* DLPF  98Hz         */
    uart_sendLine("MPU6050 OK");

    /* ── Flight controller ── */
    gyroPipelineInit(&gyroPipeline);
    imuInit(&imuState);
    pidInit(&pidState);
    tim5_init_1kHz();

    float accelK = pt1FilterGain(ACCEL_LPF_HZ, GYRO_DT);
    pt1FilterInit(&accelFilterX, accelK);
    pt1FilterInit(&accelFilterY, accelK);
    pt1FilterInit(&accelFilterZ, accelK);

    /* ── Calibracion ── */
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
    uart_sendLine("Calibrado!");

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
                            imuAccIsHealthy(ax, ay, az), ax, ay, az);
        }
    }
    uart_sendLine("IMU listo.");

    /* ── Rampa de arranque: sube suavemente de 1000 a THROTTLE_BASE ── */
    uart_sendLine("Rampa de arranque...");
    pidResetIterm(&pidState);
    motors_ramp_to(THROTTLE_BASE);
    motors_set_all(THROTTLE_BASE);

    /* ── Espera no bloqueante 1s a THROTTLE_BASE antes de activar PID ── */
    {
        uint32_t waitStart = loopCount;
        while ((loopCount - waitStart) < 1000) {
            if (pidLoopFlag) pidLoopFlag = false;
        }
    }

    LED_ON();
    pidResetIterm(&pidState);      /* reset limpio antes de vuelo activo */
    flightStart = loopCount;

    uart_sendLine("PID activo.");
    uart_sendLine("FASE |R    P    | T |SP_R  SP_P |CR    CP    CY    |m1   m2   m3   m4");
    uart_sendLine("---------------------------------------------------------------------------------");

    uint32_t lastPWM = loopCount;

    /* ── Main loop ── */
    while (1)
    {
        /* ── PID @ 1kHz ── */
        if (pidLoopFlag) {
            pidLoopFlag = false;

            /* 1. Sensor */
            mpu6050_readData(&mpuData);
            gyroPipelineUpdate(&gyroPipeline,
                               mpuData.gx, mpuData.gy, mpuData.gz);
            float ax    = pt1FilterApply(&accelFilterX, (float)mpuData.ax * ACC_SCALE_G);
            float ay    = pt1FilterApply(&accelFilterY, (float)mpuData.ay * ACC_SCALE_G);
            float az    = pt1FilterApply(&accelFilterZ, (float)mpuData.az * ACC_SCALE_G);
            bool accOk  = imuAccIsHealthy(ax, ay, az);
            imuMahonyUpdate(&imuState, 0.001f,
                            gyroPipeline.gyroRad[AXIS_X],
                            gyroPipeline.gyroRad[AXIS_Y],
                            gyroPipeline.gyroRad[AXIS_Z],
                            accOk, ax, ay, az);

            float roll  = imuGetRollDeg(&imuState);
            float pitch = imuGetPitchDeg(&imuState);

            /* 2. Lazo externo: angulo -> setpoint de velocidad angular (dps).
             *    spRoll:  signo negativo — roll +4deg pide -dps para volver al plano.
             *    spPitch: signo negativo — igual convencion que roll (corregido). */
            spRoll  = clampf(-roll  * LEVEL_KP_DPS_PER_DEG,
                             -LEVEL_RATE_LIMIT_DPS, LEVEL_RATE_LIMIT_DPS);
            spPitch = clampf(-(pitch - PITCH_Trim_deg) * LEVEL_KP_DPS_PER_DEG,
                            -LEVEL_RATE_LIMIT_DPS, LEVEL_RATE_LIMIT_DPS);

            /* 3. Protocolo de fases: VUELO -> ATERRIZAJE -> APAGADO */
            uint32_t elapsed = loopCount - flightStart;

            if (disarmed || elapsed >= (FLIGHT_TIME_MS + LANDING_TIME_MS)) {
                disarmed = true;
                motors_off();
                LED_OFF();
            } else {
                /* Throttle base segun la fase */
                if (elapsed < FLIGHT_TIME_MS) {
                    throttleNow = (float)THROTTLE_BASE;
                } else {
                    uint32_t landElapsed = elapsed - FLIGHT_TIME_MS;
                    throttleNow = (float)THROTTLE_BASE
                                  - (float)(THROTTLE_BASE - ESC_MIN_US)
                                    * (float)landElapsed / (float)LANDING_TIME_MS;
                }

                /* 4. PID — recibe throttleNow para TPA interno */
                pidUpdate(&pidState,
                          spRoll, spPitch, 0.0f,
                          gyroPipeline.gyroDPS[AXIS_X],
                          gyroPipeline.gyroDPS[AXIS_Y],
                          gyroPipeline.gyroDPS[AXIS_Z],
                          throttleNow);

                /* 5. Correcciones limitadas */
                corrRoll  = clampf(pidState.output[PID_AXIS_ROLL].sum,
                                   -MOTOR_CORR_LIMIT_US, MOTOR_CORR_LIMIT_US);
                corrPitch = clampf(pidState.output[PID_AXIS_PITCH].sum,
                                   -MOTOR_CORR_LIMIT_US, MOTOR_CORR_LIMIT_US);
                corrYaw   = clampf(pidState.output[PID_AXIS_YAW].sum,
                                   -MOTOR_CORR_LIMIT_US, MOTOR_CORR_LIMIT_US);

                /* 6. Mixer Quad-X — layout 3-1 / 4-2
                 *            Roll  Pitch  Yaw
                 *   m3 FL:    +     -      -
                 *   m1 FR:    -     -      +
                 *   m4 RL:    +     +      +
                 *   m2 RR:    -     +      -   */
                float t = throttleNow;
                motorPWM[0] = clamp_us(t - corrRoll - corrPitch + corrYaw);  /* m1 FR */
                motorPWM[1] = clamp_us(t - corrRoll + corrPitch - corrYaw);  /* m2 RR */
                motorPWM[2] = clamp_us(t + corrRoll - corrPitch - corrYaw);  /* m3 FL */
                motorPWM[3] = clamp_us(t + corrRoll + corrPitch + corrYaw);  /* m4 RL */
            }
        }

        /* ── PWM @ 50Hz ── */
        if ((loopCount - lastPWM) >= PWM_UPDATE_MS) {
            lastPWM = loopCount;
            if (!disarmed) motors_write();
        }

        /* ── UART debug @ 100ms ── */
        if ((loopCount - lastPrint) >= UART_PRINT_MS) {
            lastPrint = loopCount;
            uint32_t el    = loopCount - flightStart;
            const char *ph = disarmed          ? "OFF " :
                             (el < FLIGHT_TIME_MS ? "FLY " : "LAND");

            uart_sendString(ph);
            uart_sendString("R:");      uart_sendFloat(imuGetRollDeg(&imuState),  1);
            uart_sendString("P:");      uart_sendFloat(imuGetPitchDeg(&imuState), 1);
            uart_sendString("Y:");      uart_sendFloat(imuGetYawDeg(&imuState), 1);
            uart_sendString("|T:");    uart_sendInt((uint16_t)throttleNow);
            uart_sendString("|SP_R:"); uart_sendFloat(spRoll,  1);
            uart_sendString("SP_P:");   uart_sendFloat(spPitch, 1);
            uart_sendString("|CR:");   uart_sendFloat(corrRoll,  1);
            uart_sendString("CP:");     uart_sendFloat(corrPitch, 1);
            uart_sendString("CY:");     uart_sendFloat(corrYaw,   1);
            /* Terminos PID individuales para diagnostico */
            uart_sendString("|PR:");   uart_sendFloat(pidState.output[PID_AXIS_ROLL].P,  1);
            uart_sendString("IR:");     uart_sendFloat(pidState.output[PID_AXIS_ROLL].I,  1);
            uart_sendString("DR:");     uart_sendFloat(pidState.output[PID_AXIS_ROLL].D,  1);
            uart_sendString("|PP:");   uart_sendFloat(pidState.output[PID_AXIS_PITCH].P, 1);
            uart_sendString("IP:");     uart_sendFloat(pidState.output[PID_AXIS_PITCH].I, 1);
            uart_sendString("DP:");     uart_sendFloat(pidState.output[PID_AXIS_PITCH].D, 1);
            uart_sendString("|PY:");   uart_sendFloat(pidState.output[PID_AXIS_YAW].P,   1);
            uart_sendString("IY:");     uart_sendFloat(pidState.output[PID_AXIS_YAW].I,   1);
            uart_sendString("DY:");     uart_sendFloat(pidState.output[PID_AXIS_YAW].D,   1);

            uart_sendString("|m1:");   uart_sendInt((uint16_t)motorPWM[0]);
            uart_sendString(" m2:");     uart_sendInt((uint16_t)motorPWM[1]);
            uart_sendString(" m3:");     uart_sendInt((uint16_t)motorPWM[2]);
            uart_sendString(" m4:");     uart_sendInt((uint16_t)motorPWM[3]);
            uart_sendLine("");
        }
    }
    return 0;
}