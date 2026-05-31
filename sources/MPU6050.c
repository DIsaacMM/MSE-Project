/**
 * @file Sensor.c
 * @brief Driver para MPU-6050 via I2C bare-metal (STM32F4 a 16 MHz)
 *
 * Implementación sin HAL. Accede directamente a los registros de:
 *   - RCC   → habilitar clocks de GPIOB e I2C1
 *   - GPIOB → configurar PB6 (SCL) y PB7 (SDA) como AF4 open-drain
 *   - I2C1  → configurar y manejar transacciones I2C
 *
 * Todas las transacciones I2C siguen el protocolo:
 *   START → dirección+W → registro → (repeated START) → dirección+R → datos → STOP
 *
 * @authors
 * David Mijares
 * Aldo De la Torre
 * Jose Paez
 */

#include "MPU6050.h"
#include <stddef.h>   /* NULL */

/* ===========================================================================
 * DEFINICIÓN DE REGISTROS STM32
 * Ajusta la base si usas una variante distinta (F1, F3, L4, etc.)
 * =========================================================================== */

/* ---- RCC ---- */
#define RCC_BASE        0x40023800UL
#define RCC_AHB1ENR     (*(volatile uint32_t *)(RCC_BASE + 0x30))  /* GPIOB clock */
#define RCC_APB1ENR     (*(volatile uint32_t *)(RCC_BASE + 0x40))  /* I2C1 clock  */

#define RCC_AHB1ENR_GPIOBEN  (1U << 1)
#define RCC_APB1ENR_I2C1EN   (1U << 21)

/* ---- GPIOB ---- */
#define GPIOB_BASE      0x40020400UL
#define GPIOB_MODER     (*(volatile uint32_t *)(GPIOB_BASE + 0x00))
#define GPIOB_OTYPER    (*(volatile uint32_t *)(GPIOB_BASE + 0x04))
#define GPIOB_OSPEEDR   (*(volatile uint32_t *)(GPIOB_BASE + 0x08))
#define GPIOB_PUPDR     (*(volatile uint32_t *)(GPIOB_BASE + 0x0C))
#define GPIOB_AFRL      (*(volatile uint32_t *)(GPIOB_BASE + 0x20))  /* pines 0-7 */
#define GPIOB_AFRH      (*(volatile uint32_t *)(GPIOB_BASE + 0x24))  /* pines 8-15 */

/* ---- I2C1 ---- */
#define I2C1_BASE       0x40005400UL
#define I2C1_CR1        (*(volatile uint32_t *)(I2C1_BASE + 0x00))
#define I2C1_CR2        (*(volatile uint32_t *)(I2C1_BASE + 0x04))
#define I2C1_OAR1       (*(volatile uint32_t *)(I2C1_BASE + 0x08))
#define I2C1_CCR        (*(volatile uint32_t *)(I2C1_BASE + 0x1C))
#define I2C1_TRISE      (*(volatile uint32_t *)(I2C1_BASE + 0x20))
#define I2C1_SR1        (*(volatile uint32_t *)(I2C1_BASE + 0x14))
#define I2C1_SR2        (*(volatile uint32_t *)(I2C1_BASE + 0x18))
#define I2C1_DR         (*(volatile uint32_t *)(I2C1_BASE + 0x10))

/* Bits de I2C SR1 */
#define I2C_SR1_SB      (1U << 0)   /* Start bit generado         */
#define I2C_SR1_ADDR    (1U << 1)   /* Dirección enviada/recibida */
#define I2C_SR1_BTF     (1U << 2)   /* Byte transfer finished     */
#define I2C_SR1_RXNE    (1U << 6)   /* RX no vacío                */
#define I2C_SR1_TXE     (1U << 7)   /* TX vacío                   */

/* Bits de I2C CR1 */
#define I2C_CR1_PE      (1U << 0)   /* Peripheral enable  */
#define I2C_CR1_START   (1U << 8)   /* Generate START     */
#define I2C_CR1_STOP    (1U << 9)   /* Generate STOP      */
#define I2C_CR1_ACK     (1U << 10)  /* ACK enable         */
#define I2C_CR1_SWRST   (1U << 15)  /* Software reset     */

/* ===========================================================================
 * PROTOTIPOS PRIVADOS
 * =========================================================================== */
static void i2c_init(void);
static void i2c_start(void);
static void i2c_stop(void);
static void i2c_sendAddr(uint8_t addr, uint8_t rw);
static void i2c_sendByte(uint8_t data);
static uint8_t i2c_readAck(void);
static uint8_t i2c_readNack(void);
static void i2c_clearAddr(void);

static void mpu6050_writeReg(uint8_t reg, uint8_t value);
static void mpu6050_readRegs(uint8_t reg, uint8_t *buf, uint8_t len);

/* ===========================================================================
 * FUNCIONES PRIVADAS — GPIO e I2C
 * =========================================================================== */

/**
 * @brief Configura GPIOB (PB6=SCL, PB7=SDA) e inicializa I2C1 a 100 kHz.
 *
 * Fórmula CCR para modo estándar (100 kHz) con Fpclk1 = 16 MHz:
 *   CCR = Fpclk1 / (2 * Fi2c) = 16_000_000 / 200_000 = 80
 *
 * TRISE = (Fpclk1 / 1_000_000) + 1 = 16 + 1 = 17
 */
static void i2c_init(void)
{
    /* -----------------------------------------------------------------
     * 1. Habilitar clocks de GPIOB e I2C1
     * ----------------------------------------------------------------- */
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC_APB1ENR |= RCC_APB1ENR_I2C1EN;

    /* -----------------------------------------------------------------
     * 2. Configurar PB6 (SCL) y PB7 (SDA)
     *    MODER   = 10 (Alternate Function)
     *    OTYPER  = 1  (Open-drain — obligatorio para I2C)
     *    OSPEEDR = 11 (Very high speed)
     *    PUPDR   = 01 (Pull-up)
     *    AFR     = 4  (I2C1)
     * ----------------------------------------------------------------- */

    /* MODER: pines 6 y 7 → AF (10) */
    GPIOB_MODER &= ~((3U << 12) | (3U << 14));
    GPIOB_MODER |=  ((2U << 12) | (2U << 14));

    /* OTYPER: open-drain en pines 6 y 7 */
    GPIOB_OTYPER |= (1U << 6) | (1U << 7);

    /* OSPEEDR: muy alta velocidad */
    GPIOB_OSPEEDR |= (3U << 12) | (3U << 14);

    /* PUPDR: pull-up en pines 6 y 7 */
    GPIOB_PUPDR &= ~((3U << 12) | (3U << 14));
    GPIOB_PUPDR |=  ((1U << 12) | (1U << 14));

    /* AFRL: AF4 para pines 6 y 7 (cada uno ocupa 4 bits en AFRL) */
    GPIOB_AFRL &= ~((0xFU << 24) | (0xFU << 28));
    GPIOB_AFRL |=  ((4U   << 24) | (4U   << 28));

    /* -----------------------------------------------------------------
     * 3. Resetear y configurar I2C1
     * ----------------------------------------------------------------- */

    /* Software reset para limpiar estado */
    I2C1_CR1 |=  I2C_CR1_SWRST;
    I2C1_CR1 &= ~I2C_CR1_SWRST;

    /* Frecuencia del bus APB1 en MHz (16 MHz → valor 16) */
    I2C1_CR2 = 16U;

    /* CCR = 80 → 100 kHz con Fpclk1 = 16 MHz, modo estándar */
    I2C1_CCR = 80U;

    /* TRISE = 17 → tiempo de subida máximo para 100 kHz */
    I2C1_TRISE = 17U;

    /* Habilitar I2C1 */
    I2C1_CR1 |= I2C_CR1_PE;
}

/** @brief Genera condición START en el bus I2C. */
static void i2c_start(void)
{
    I2C1_CR1 |= I2C_CR1_START;
    while (!(I2C1_SR1 & I2C_SR1_SB));  /* esperar SB=1 */
}

/** @brief Genera condición STOP en el bus I2C. */
static void i2c_stop(void)
{
    I2C1_CR1 |= I2C_CR1_STOP;
}

/**
 * @brief Envía la dirección del esclavo con bit de lectura/escritura.
 * @param addr  Dirección de 7 bits (sin shift)
 * @param rw    0 = escritura, 1 = lectura
 */
static void i2c_sendAddr(uint8_t addr, uint8_t rw)
{
    I2C1_DR = (uint8_t)((addr << 1) | rw);
    while (!(I2C1_SR1 & I2C_SR1_ADDR));  /* esperar ADDR=1 */
}

/** @brief Limpia el flag ADDR leyendo SR1 y SR2. */
static void i2c_clearAddr(void)
{
    (void)I2C1_SR1;
    (void)I2C1_SR2;
}

/**
 * @brief Envía un byte por I2C y espera confirmación.
 * @param data  Byte a enviar
 */
static void i2c_sendByte(uint8_t data)
{
    while (!(I2C1_SR1 & I2C_SR1_TXE));  /* esperar TX vacío */
    I2C1_DR = data;
    while (!(I2C1_SR1 & I2C_SR1_BTF));  /* esperar transfer completo */
}

/** @brief Lee un byte y envía ACK (hay más bytes por recibir). */
static uint8_t i2c_readAck(void)
{
    I2C1_CR1 |= I2C_CR1_ACK;
    while (!(I2C1_SR1 & I2C_SR1_RXNE));
    return (uint8_t)I2C1_DR;
}

/** @brief Lee el último byte y envía NACK + STOP. */
static uint8_t i2c_readNack(void)
{
    I2C1_CR1 &= ~I2C_CR1_ACK;  /* deshabilitar ACK antes del último byte */
    i2c_stop();
    while (!(I2C1_SR1 & I2C_SR1_RXNE));
    return (uint8_t)I2C1_DR;
}

/* ===========================================================================
 * FUNCIONES PRIVADAS — MPU-6050
 * =========================================================================== */

/**
 * @brief Escribe un byte en un registro del MPU-6050.
 *
 * Secuencia:
 *   START → addr+W → reg → value → STOP
 */
static void mpu6050_writeReg(uint8_t reg, uint8_t value)
{
    i2c_start();
    i2c_sendAddr(MPU6050_ADDR, 0);   /* escritura */
    i2c_clearAddr();
    i2c_sendByte(reg);
    i2c_sendByte(value);
    i2c_stop();
}

/**
 * @brief Lee 'len' bytes consecutivos a partir del registro 'reg'.
 *
 * Secuencia:
 *   START → addr+W → reg → repeated START → addr+R → [datos] → STOP
 *
 * @param reg  Registro inicial
 * @param buf  Buffer de destino
 * @param len  Número de bytes a leer (mínimo 1)
 */
static void mpu6050_readRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    /* Fase escritura: apuntar al registro inicial */
    i2c_start();
    i2c_sendAddr(MPU6050_ADDR, 0);
    i2c_clearAddr();
    i2c_sendByte(reg);

    /* Repeated START y cambio a modo lectura */
    i2c_start();
    i2c_sendAddr(MPU6050_ADDR, 1);
    i2c_clearAddr();

    /* Leer bytes: todos con ACK excepto el último con NACK+STOP */
    for (uint8_t i = 0; i < len; i++)
    {
        if (i < (len - 1))
        {
            buf[i] = i2c_readAck();
        }
        else
        {
            buf[i] = i2c_readNack();
        }
    }
}


/**
 * @brief Inicializa I2C y configura el MPU-6050.
 *
 * Configuración aplicada:
 *   PWR_MGMT_1  = 0x00 → salir del modo sleep, usar oscilador interno
 *   SMPLRT_DIV  = 0x07 → sample rate = 1 kHz / (1+7) = 125 Hz
 *   CONFIG      = 0x06 → DLPF a 5 Hz (suaviza ruido)
 *   GYRO_CONFIG = 0x00 → ±250 °/s (máxima sensibilidad)
 *   ACCEL_CONFIG= 0x00 → ±2 g    (máxima sensibilidad)
 */
void mpu6050_init(void)
{
    i2c_init();

    /* Despertar el sensor */
    mpu6050_writeReg(MPU6050_REG_PWR_MGMT_1, 0x00);

    /* Sample rate: 125 Hz */
    mpu6050_writeReg(MPU6050_REG_SMPLRT_DIV, 0x07);

    /* DLPF: 5 Hz de corte → filtra vibraciones de motores */
    mpu6050_writeReg(MPU6050_REG_CONFIG, 0x06);

    /* Giroscopio: ±250 °/s */
    mpu6050_writeReg(MPU6050_REG_GYRO_CFG, 0x00);

    /* Acelerómetro: ±2 g */
    mpu6050_writeReg(MPU6050_REG_ACCEL_CFG, 0x00);
}

/**
 * @brief Lee los 3 ejes del acelerómetro en crudo.
 *
 * Cada eje = 2 bytes (High, Low) → combinar con shift y OR.
 * Registros leídos: 0x3B al 0x40 (6 bytes en total).
 */
void mpu6050_readAccel(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buf[6];
    mpu6050_readRegs(MPU6050_REG_ACCEL_XOUT, buf, 6);

    if (ax) *ax = (int16_t)((buf[0] << 8) | buf[1]);
    if (ay) *ay = (int16_t)((buf[2] << 8) | buf[3]);
    if (az) *az = (int16_t)((buf[4] << 8) | buf[5]);
}

/**
 * @brief Lee los 3 ejes del giroscopio en crudo.
 *
 * Registros leídos: 0x43 al 0x48 (6 bytes en total).
 * Cualquier puntero puede ser NULL si ese eje no se necesita.
 */
void mpu6050_readGyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6];
    mpu6050_readRegs(MPU6050_REG_GYRO_XOUT, buf, 6);

    if (gx) *gx = (int16_t)((buf[0] << 8) | buf[1]);
    if (gy) *gy = (int16_t)((buf[2] << 8) | buf[3]);
    if (gz) *gz = (int16_t)((buf[4] << 8) | buf[5]);
}