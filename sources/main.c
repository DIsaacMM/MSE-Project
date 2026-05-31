/**
 * @file main.c
 * @brief Test calibración + IMU a 400kHz
 *
 * Verifica que:
 *  1. El gyro calibra correctamente a 400kHz
 *  2. El IMU Mahony converge a Roll=0 Pitch=0 en reposo
 *  3. Los ángulos cambian correctamente al inclinar el dron
 *
 * Sin motores — solo sensor y cálculos.
 */

#include <stdint.h>
#include <stdbool.h>
#include "Timer.h"
#include "MPU6050.h"
#include "UART.h"
#include "gyro_filter.h"
#include "imu_mahony.h"

#define DELAY_TIM    TIM_3
#define I2C_PORT     B
#define I2C_SCL_PIN  8
#define I2C_SDA_PIN  9
#define I2C_PIN_MODE 2

#define UART_PRINT_MS  100   /* imprimir cada 200ms */

/* TIM5 a 1kHz para el loop de lectura */
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

static GyroPipeline_t  gyroPipeline;
static ImuState_t      imuState;
static MPU6050_t       mpuData;

static volatile bool     pidLoopFlag = false;
static volatile uint32_t loopCount   = 0;
static uint32_t          lastPrint   = 0;

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
    uart_sendLine("=== Test Calibración + IMU @ 400kHz ===");

    timer_init(DELAY_TIM);

    /* MPU6050 */
    mpu6050_init(I2C_PORT, I2C_SCL_PIN, I2C_SDA_PIN, I2C_PIN_MODE);
    mpu6050_config(0x1B, 0x18);   /* Gyro  ±2000°/s */
    mpu6050_config(0x1C, 0x00);   /* Accel ±2g      */
    mpu6050_config(0x1A, 0x02);   /* DLPF  98Hz     */
    uart_sendLine("MPU6050 OK");

    /* Inicializar módulos */
    gyroPipelineInit(&gyroPipeline);
    imuInit(&imuState);

    /* Arrancar loop */
    tim5_init_1kHz();

    uart_sendLine(">>> PON EL DRON QUIETO Y PLANO <<<");
    uart_sendLine("Calibrando...");

    /* Esperar calibración */
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
            uart_sendString(" | bias X:");
            uart_sendFloat(gyroPipeline.calib.bias[0] * GYRO_SCALE_DPS, 2);
            uart_sendString(" Y:");
            uart_sendFloat(gyroPipeline.calib.bias[1] * GYRO_SCALE_DPS, 2);
            uart_sendString(" Z:");
            uart_sendFloat(gyroPipeline.calib.bias[2] * GYRO_SCALE_DPS, 2);
            uart_sendLine(" dps");
        }
    }

    /* Imprimir bias final */
    uart_sendString("✓ Calibrado! Bias final: X=");
    uart_sendFloat(gyroPipeline.calib.bias[0] * GYRO_SCALE_DPS, 3);
    uart_sendString(" Y=");
    uart_sendFloat(gyroPipeline.calib.bias[1] * GYRO_SCALE_DPS, 3);
    uart_sendString(" Z=");
    uart_sendFloat(gyroPipeline.calib.bias[2] * GYRO_SCALE_DPS, 3);
    uart_sendLine(" dps");
    uart_sendLine("Convergiendo IMU... espera 2s plano");

    /* Dar tiempo al IMU para converger */
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

    uart_sendLine("✓ IMU listo. Inclina el dron para verificar angulos.");
    uart_sendLine("Formato: R=roll P=pitch Y=yaw | gyr X Y Z dps | acc healthy");
    uart_sendLine("----------------------------------------------------------");

    /* Loop principal */
    while (1)
    {
        if (pidLoopFlag) {
            pidLoopFlag = false;

            mpu6050_readData(&mpuData);
            gyroPipelineUpdate(&gyroPipeline,
                               mpuData.gx, mpuData.gy, mpuData.gz);

            float ax = (float)mpuData.ax * ACC_SCALE_G;
            float ay = (float)mpuData.ay * ACC_SCALE_G;
            float az = (float)mpuData.az * ACC_SCALE_G;
            bool  healthy = imuAccIsHealthy(ax, ay, az);

            imuMahonyUpdate(&imuState, 0.001f,
                            gyroPipeline.gyroRad[AXIS_X],
                            gyroPipeline.gyroRad[AXIS_Y],
                            gyroPipeline.gyroRad[AXIS_Z],
                            healthy, ax, ay, az);
        }

        if ((loopCount - lastPrint) >= UART_PRINT_MS) {
            lastPrint = loopCount;

            /* Ángulos */
            uart_sendString("R:");
            uart_sendFloat(imuGetRollDeg(&imuState),  1);
            uart_sendString("  P:");
            uart_sendFloat(imuGetPitchDeg(&imuState), 1);
            uart_sendString("  Y:");
            uart_sendFloat(imuGetYawDeg(&imuState),   1);

            /* Gyro filtrado en dps */
            uart_sendString(" | gyr X:");
            uart_sendFloat(gyroPipeline.gyroDPS[0], 1);
            uart_sendString(" Y:");
            uart_sendFloat(gyroPipeline.gyroDPS[1], 1);
            uart_sendString(" Z:");
            uart_sendFloat(gyroPipeline.gyroDPS[2], 1);

            /* Salud del accel */
            uart_sendString(" | acc:");
            float ax = (float)mpuData.ax * ACC_SCALE_G;
            float ay = (float)mpuData.ay * ACC_SCALE_G;
            float az = (float)mpuData.az * ACC_SCALE_G;
            uart_sendChar(imuAccIsHealthy(ax, ay, az) ? '1' : '0');

            uart_sendLine("");
        }
    }
    return 0;
}