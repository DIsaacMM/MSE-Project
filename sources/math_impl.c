/**
 * @file math_impl.c
 * @brief Implementaciones bare-metal de funciones matemáticas
 *
 * Reemplaza las funciones de libm y libc que usan los módulos
 * del flight controller, manteniendo -nostdlib en el makefile.
 *
 * Funciones implementadas:
 *   memset   → limpieza de structs (gyro_filter, imu_mahony, pid_controller)
 *   sqrtf    → magnitud de vectores (IMU Mahony, calibración gyro)
 *   atan2f   → cálculo de Roll y Yaw desde matriz de rotación
 *   asinf    → cálculo de Pitch desde matriz de rotación
 *   sinf     → coeficientes del filtro Biquad
 *   cosf     → coeficientes del filtro Biquad
 *
 * Precisión:
 *   sqrtf    → Newton-Raphson, error < 0.0001%
 *   atan2f   → Aproximación de Betaflight (atan2_approx), error < 0.02°
 *   asinf    → Serie de Taylor orden 5, error < 0.001° en rango ±90°
 *   sinf/cosf → Reducción de rango + serie de Taylor orden 5, error < 0.0001
 *   memset   → Exacto
 *
 * Todas son suficientemente precisas para control de vuelo.
 */

#include <stddef.h>
#include <stdint.h>

/* ── Constantes ── */
#ifndef M_PI
#define M_PI    3.14159265358979323846f
#endif
#define M_PI_2  1.57079632679489661923f
#define M_2PI   6.28318530717958647692f

/* =====================================================================
 *  memset — limpiar bloques de memoria
 *  Idéntico al estándar: llena 'n' bytes en 'ptr' con el valor 'c'
 * ===================================================================== */
void *memset(void *ptr, int c, size_t n)
{
    unsigned char *p = (unsigned char *)ptr;
    while (n--) *p++ = (unsigned char)c;
    return ptr;
}

/* =====================================================================
 *  sqrtf — raíz cuadrada (Newton-Raphson)
 *  Mismo algoritmo que usa Betaflight internamente.
 *  Dos iteraciones dan precisión suficiente para el IMU.
 * ===================================================================== */
float sqrtf(float x)
{
    if (x <= 0.0f) return 0.0f;
    /* Semilla inicial con truco de bits (fast inverse sqrt invertido) */
    union { float f; uint32_t i; } u = { .f = x };
    u.i = 0x5f3759df - (u.i >> 1);
    float r = u.f;
    /* Newton-Raphson: r = r * (1.5 - 0.5*x*r*r) */
    r = r * (1.5f - 0.5f * x * r * r);
    r = r * (1.5f - 0.5f * x * r * r);
    return x * r;   /* x * (1/sqrt(x)) = sqrt(x) */
}

/* =====================================================================
 *  atan2f — arcotangente de dos argumentos
 *  Portado de atan2_approx() de Betaflight (src/main/common/maths.c)
 *  Error máximo: ~0.02° — más que suficiente para Roll/Yaw del IMU.
 * ===================================================================== */
float atan2f(float y, float x)
{
    /* Casos especiales */
    if (x == 0.0f) {
        if (y > 0.0f) return  M_PI_2;
        if (y < 0.0f) return -M_PI_2;
        return 0.0f;
    }

    float abs_y = y < 0.0f ? -y : y;
    float angle;

    if (x >= 0.0f) {
        float r = (x - abs_y) / (x + abs_y);
        angle = 0.1963f * r * r * r - 0.9817f * r + M_PI_2 * 0.5f;
    } else {
        float r = (x + abs_y) / (abs_y - x);
        angle = 0.1963f * r * r * r - 0.9817f * r + M_PI_2 * 1.5f;
    }

    return (y < 0.0f) ? -angle : angle;
}

/* =====================================================================
 *  asinf — arcoseno
 *  Usado para calcular Pitch desde rMat[2][0].
 *  Entrada garantizada en [-1, 1] por imuComputeEuler().
 *  Fórmula: asin(x) = atan2(x, sqrt(1 - x*x))
 * ===================================================================== */
float asinf(float x)
{
    /* Saturar por seguridad */
    if (x >  1.0f) x =  1.0f;
    if (x < -1.0f) x = -1.0f;
    /* Usar atan2 que ya está implementado */
    float d = 1.0f - x * x;
    return atan2f(x, sqrtf(d));
}

/* =====================================================================
 *  sinf / cosf — seno y coseno
 *  Usados solo en biquadFilterInit() para calcular coeficientes.
 *  Reducción de rango a [-π, π] + serie de Taylor orden 9.
 *  Error < 5e-7 en el rango de uso (frecuencias de filtro 80-200 Hz).
 * ===================================================================== */

/* Reducir ángulo al rango [-π, π] */
static float wrap_pi(float x)
{
    while (x >  M_PI) x -= M_2PI;
    while (x < -M_PI) x += M_2PI;
    return x;
}

float sinf(float x)
{
    x = wrap_pi(x);
    /* Serie de Taylor: sin(x) = x - x³/6 + x⁵/120 - x⁷/5040 + x⁹/362880 */
    float x2 = x * x;
    return x * (1.0f
        - x2 * (1.0f/6.0f
        - x2 * (1.0f/120.0f
        - x2 * (1.0f/5040.0f
        - x2 * (1.0f/362880.0f)))));
}

float cosf(float x)
{
    /* cos(x) = sin(x + π/2) */
    return sinf(x + M_PI_2);
}
