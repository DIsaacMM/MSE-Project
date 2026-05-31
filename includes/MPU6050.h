/**
 * @file Sensor.h
 * @brief Driver para MPU-6050 via I2C (bare-metal STM32)
 *
 * Lee acelerómetro y giroscopio del MPU-6050.
 * Usa el periférico I2C1 del STM32 directamente via registros.
 *
 * Conexiones:
 *   MPU-6050 SDA → PB7  (I2C1_SDA, AF4)
 *   MPU-6050 SCL → PB6  (I2C1_SCL, AF4)
 *   MPU-6050 AD0 → GND  (dirección 0x68)
 *   MPU-6050 VCC → 3.3V
 *
 * @authors
 * David Mijares
 * Aldo De la Torre
 * Jose Paez
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

/* ===========================================================================
 * DIRECCIÓN I2C DEL MPU-6050
 * AD0 = GND → 0x68   |   AD0 = VCC → 0x69
 * =========================================================================== */
#define MPU6050_ADDR        0x68

/* ===========================================================================
 * REGISTROS INTERNOS DEL MPU-6050
 * =========================================================================== */
#define MPU6050_REG_PWR_MGMT_1  0x6B   /* Power management — despertar sensor  */
#define MPU6050_REG_SMPLRT_DIV  0x19   /* Sample rate divider                  */
#define MPU6050_REG_CONFIG      0x1A   /* DLPF config                          */
#define MPU6050_REG_GYRO_CFG    0x1B   /* Gyroscope config (full scale)        */
#define MPU6050_REG_ACCEL_CFG   0x1C   /* Accelerometer config (full scale)    */
#define MPU6050_REG_ACCEL_XOUT  0x3B   /* Primer registro de acelerómetro      */
#define MPU6050_REG_GYRO_XOUT   0x43   /* Primer registro de giroscopio        */

/* ===========================================================================
 * ESCALAS (rango por defecto al arrancar)
 *   Acelerómetro : ±2 g    → 16384 LSB/g
 *   Giroscopio   : ±250°/s →   131 LSB/(°/s)
 * =========================================================================== */
#define MPU6050_ACCEL_SCALE     16384.0f
#define MPU6050_GYRO_SCALE        131.0f

/* ===========================================================================
 * API PÚBLICA
 * =========================================================================== */

/**
 * @brief Inicializa I2C1 y despierta el MPU-6050.
 *
 * Llama esta función UNA SOLA VEZ antes de leer datos.
 * Configura:
 *   - GPIO PB6/PB7 como AF4 (I2C1)
 *   - I2C1 en modo estándar (100 kHz) para STM32 a 16 MHz
 *   - MPU-6050: sale del modo sleep, DLPF activo
 */
void mpu6050_init(void);

/**
 * @brief Lee los 3 ejes del acelerómetro en crudo (LSB).
 *
 * @param ax  Puntero donde se guarda el eje X
 * @param ay  Puntero donde se guarda el eje Y
 * @param az  Puntero donde se guarda el eje Z
 */
void mpu6050_readAccel(int16_t *ax, int16_t *ay, int16_t *az);

/**
 * @brief Lee los 3 ejes del giroscopio en crudo (LSB).
 *
 * @param gx  Puntero donde se guarda el eje X (puede ser NULL)
 * @param gy  Puntero donde se guarda el eje Y (puede ser NULL)
 * @param gz  Puntero donde se guarda el eje Z (puede ser NULL)
 */
void mpu6050_readGyro(int16_t *gx, int16_t *gy, int16_t *gz);

#endif /* SENSOR_H */