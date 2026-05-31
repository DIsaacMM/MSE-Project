#ifndef MATH_UTILS_H
#define MATH_UTILS_H
#include <stdint.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
static inline float sq(float x) { return x * x; }
static inline float invSqrt(float x) {
    float halfx = 0.5f * x;
    union { float f; uint32_t i; } u = { .f = x };
    u.i = 0x5f3759df - (u.i >> 1);
    u.f *= 1.5f - halfx * u.f * u.f;
    u.f *= 1.5f - halfx * u.f * u.f;
    return u.f;
}
#endif

/* Forward declarations de math_impl.c — reemplazan libm y string.h */
#include <stddef.h>
float  sqrtf(float x);
float  sinf(float x);
float  cosf(float x);
float  atan2f(float y, float x);
float  asinf(float x);
void  *memset(void *ptr, int c, size_t n);
