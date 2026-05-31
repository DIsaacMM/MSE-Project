/**
 * imu_mahony.c
 * Portado desde Betaflight 4.5.1 (GPL-3.0) — src/main/flight/imu.c
 *
 * Funciones originales portadas:
 *   imuComputeRotationMatrix()    → imuComputeRotationMatrix() [interna]
 *   imuMahonyAHRSupdate()         → imuMahonyUpdate()
 *   imuUpdateEulerAngles()        → imuComputeEuler()
 *   imuInit()                     → imuInit()
 *   imuIsAccelerometerHealthy()   → imuAccIsHealthy()
 *
 * Eliminaciones vs Betaflight:
 *   - Sin GPS heading correction (COG)
 *   - Sin magnetómetro
 *   - Sin HEADFREE mode
 *   - Sin scheduler / timeUs_t
 *   - Sin macros DEBUG_SET / ARMING_FLAG
 *   - Sin imuCalcKpGain() de estado máquina (simplificado a contador de ciclos)
 *   - sincosf_approx() / atan2_approx() sustituidos por funciones estándar libm
 */

#include "imu_mahony.h"
#include "math_utils.h"

/* =========================================================
 *  UTILIDADES MATEMÁTICAS
 * ========================================================= */



/* =========================================================
 *  MATRIZ DE ROTACIÓN DESDE CUATERNIÓN
 *  Fuente: betaflight/src/main/flight/imu.c → imuComputeRotationMatrix()
 *
 *  Convierte q = {w,x,y,z} en la matriz de rotación 3×3 R.
 *  R transforma vectores del marco del cuerpo al marco de tierra:
 *    v_earth = R * v_body
 *
 *  rMat[2][0..2] (tercer fila) = proyección del eje Z del cuerpo en tierra.
 *  Esto es lo que Mahony usa para calcular el error con el acelerómetro.
 * ========================================================= */
static void imuComputeRotationMatrix(ImuState_t *imu)
{
    const float qw = imu->q.w;
    const float qx = imu->q.x;
    const float qy = imu->q.y;
    const float qz = imu->q.z;

    /* Pre-productos (evitan multiplicaciones repetidas) */
    const float ww = qw*qw, wx = qw*qx, wy = qw*qy, wz = qw*qz;
    const float xx = qx*qx, xy = qx*qy, xz = qx*qz;
    const float yy = qy*qy, yz = qy*qz;
    const float zz = qz*qz;

    imu->rMat.m[0][0] = 1.0f - 2.0f*(yy + zz);
    imu->rMat.m[0][1] = 2.0f*(xy - wz);          /* = 2*(xy + -wz) en Betaflight */
    imu->rMat.m[0][2] = 2.0f*(xz + wy);           /* = 2*(xz - -wy)               */

    imu->rMat.m[1][0] = 2.0f*(xy + wz);           /* = 2*(xy - -wz)               */
    imu->rMat.m[1][1] = 1.0f - 2.0f*(xx + zz);
    imu->rMat.m[1][2] = 2.0f*(yz - wx);           /* = 2*(yz + -wx)               */

    imu->rMat.m[2][0] = 2.0f*(xz - wy);           /* = 2*(xz + -wy)               */
    imu->rMat.m[2][1] = 2.0f*(yz + wx);           /* = 2*(yz - -wx)               */
    imu->rMat.m[2][2] = 1.0f - 2.0f*(xx + yy);

    (void)ww; /* ww no se usa directamente en los términos, pero sí implícitamente */
}

/* =========================================================
 *  INICIALIZACIÓN
 * ========================================================= */

void imuInit(ImuState_t *imu)
{
    memset(imu, 0, sizeof(ImuState_t));

    /* Cuaternión identidad: sin rotación (dron horizontal) */
    imu->q.w = 1.0f;
    imu->q.x = 0.0f;
    imu->q.y = 0.0f;
    imu->q.z = 0.0f;

    /* Integradores de error a cero */
    imu->integralFBx = 0.0f;
    imu->integralFBy = 0.0f;
    imu->integralFBz = 0.0f;

    /* Ganancia alta durante los primeros N ciclos para convergencia rápida
     * Betaflight: imuCalcKpGain() retorna Kp*10 en estado "disarmed" inicial */
    imu->dcmKp          = IMU_DCM_KP_FAST;
    imu->dcmKi          = IMU_DCM_KI;
    imu->fastKpCycles   = IMU_FAST_KP_CYCLES;
    imu->attitudeIsEstablished = false;

    /* Calcular la matriz de rotación inicial (identidad) */
    imuComputeRotationMatrix(imu);
}

/* =========================================================
 *  VERIFICACIÓN DE SALUD DEL ACELERÓMETRO
 *  Fuente: betaflight/src/main/flight/imu.c → imuIsAccelerometerHealthy()
 *
 *  Solo usar el acc para corrección cuando su magnitud está entre 0.9g y 1.1g.
 *  Fuera de ese rango: el dron tiene aceleración lineal (movimiento brusco)
 *  y la lectura no representa solo la gravedad.
 * ========================================================= */
bool imuAccIsHealthy(float ax, float ay, float az)
{
    const float mag = sqrtf(sq(ax) + sq(ay) + sq(az));
    /* Betaflight: 0.9 < accMagnitude < 1.1  (en unidades de g) */
    return (mag > 0.9f) && (mag < 1.1f);
}

/* =========================================================
 *  ACTUALIZACIÓN EULER DESDE CUATERNIÓN
 *  Fuente: betaflight/src/main/flight/imu.c → imuUpdateEulerAngles()
 *
 *  Convierte la matriz de rotación a ángulos de Euler (roll, pitch, yaw).
 *  Usa la convención ZYX (yaw → pitch → roll), estándar en aviación.
 *
 *  Fórmulas (de rMat):
 *    roll  = atan2(rMat[2][1], rMat[2][2])
 *    pitch = (π/2) - acos(-rMat[2][0])    ← equivalente a asin(rMat[2][0])
 *    yaw   = -atan2(rMat[1][0], rMat[0][0])
 *
 *  Los valores se guardan en decidegrees (×10) para mayor precisión con enteros.
 * ========================================================= */
void imuComputeEuler(ImuState_t *imu)
{
    const float R20 = imu->rMat.m[2][0];
    const float R21 = imu->rMat.m[2][1];
    const float R22 = imu->rMat.m[2][2];
    const float R10 = imu->rMat.m[1][0];
    const float R00 = imu->rMat.m[0][0];

    /* roll: rotación alrededor del eje X (izq-der)
     * Betaflight: atan2_approx(rMat[2][1], rMat[2][2]) */
    imu->attitude.roll  = (int16_t)(atan2f(R21, R22) * (1800.0f / M_PI));

    /* pitch: rotación alrededor del eje Y (adelante-atrás)
     * Betaflight: (π/2 - acos(-rMat[2][0])) * (1800/π)
     * Equivalente a: asin(rMat[2][0]) * (1800/π) */
    float sinPitch = -R20;
    /* Saturar en ±1 para evitar NaN en asinf */
    if (sinPitch >  1.0f) sinPitch =  1.0f;
    if (sinPitch < -1.0f) sinPitch = -1.0f;
    imu->attitude.pitch = (int16_t)(asinf(sinPitch) * (1800.0f / M_PI));

    /* yaw: rotación alrededor del eje Z (izquierda-derecha visto desde arriba)
     * Betaflight: -atan2_approx(rMat[1][0], rMat[0][0])
     * Signo negativo: Betaflight usa CW positivo, estándar es CCW positivo */
    int16_t yaw = (int16_t)(-atan2f(R10, R00) * (1800.0f / M_PI));

    /* Normalizar yaw a [0, 3600) decidegrees = [0°, 360°) */
    if (yaw < 0) {
        yaw += 3600;
    }
    imu->attitude.yaw = yaw;
}

/* =========================================================
 *  NÚCLEO DEL ALGORITMO DE MAHONY
 *  Fuente: betaflight/src/main/flight/imu.c → imuMahonyAHRSupdate()
 *
 *  Implementa el filtro de Mahony (PI en espacio de error de orientación):
 *
 *  1. Calcular spin rate para controlar la integración de Ki
 *  2. Calcular error de orientación con el acelerómetro (producto cruzado)
 *  3. Aplicar retroalimentación proporcional (Kp*error) e integral (Ki*∫error)
 *  4. Integrar el cuaternión con los gyro corregidos
 *  5. Normalizar el cuaternión
 *  6. Recalcular la matriz de rotación
 * ========================================================= */
void imuMahonyUpdate(ImuState_t *imu, float dt,
                     float gx, float gy, float gz,
                     bool useAcc,
                     float ax, float ay, float az)
{
    /* --- Gestión de ganancia dinámica (alta al arrancar) ---
     * Betaflight imuCalcKpGain(): en estado disarmed usa Kp*10 para converger rápido.
     * Aquí lo simplificamos: contador descendente de ciclos */
    if (imu->fastKpCycles > 0) {
        imu->fastKpCycles--;
        imu->dcmKp = IMU_DCM_KP_FAST;
    } else {
        imu->dcmKp = IMU_DCM_KP;
    }

    /* --- 1. Velocidad de giro total (rad/s) --- */
    const float spin_rate = sqrtf(sq(gx) + sq(gy) + sq(gz));

    /* --- 2. Error de orientación con el acelerómetro ---
     * Si el acc es válido: comparar vector de gravedad estimado vs medido.
     *
     * Vector de gravedad estimado: la 3ª fila de rMat.
     * (rMat[2][0..2] = proyección de [0,0,1] del cuerpo → tierra)
     *
     * Error = estimado × medido (producto cruzado)
     * Si están alineados → error = 0
     * Si divergen → error apunta en la dirección de corrección */
    float ex = 0.0f, ey = 0.0f, ez = 0.0f;

    if (useAcc) {
        /* Normalizar acelerómetro */
        float recipAccNorm = sq(ax) + sq(ay) + sq(az);
        if (recipAccNorm > 0.01f) {
            recipAccNorm = invSqrt(recipAccNorm);
            ax *= recipAccNorm;
            ay *= recipAccNorm;
            az *= recipAccNorm;

            /* Producto cruzado: estimado (rMat[2]) × medido (acc normalizado)
             * Betaflight imu.c líneas 244-247: */
            ex = (ay * imu->rMat.m[2][2] - az * imu->rMat.m[2][1]);
            ey = (az * imu->rMat.m[2][0] - ax * imu->rMat.m[2][2]);
            ez = (ax * imu->rMat.m[2][1] - ay * imu->rMat.m[2][0]);
        }
    }

    /* --- 3. Integrador de error (Ki) ---
     * Solo integrar si el dron no está girando demasiado rápido.
     * Betaflight: spin_rate < DEGREES_TO_RADIANS(SPIN_RATE_LIMIT) = 20°/s */
    if (imu->dcmKi > 0.0f) {
        if (spin_rate < IMU_SPIN_RATE_LIMIT_RAD) {
            imu->integralFBx += imu->dcmKi * ex * dt;
            imu->integralFBy += imu->dcmKi * ey * dt;
            imu->integralFBz += imu->dcmKi * ez * dt;
        }
    } else {
        /* Ki=0: resetear integrador para evitar windup */
        imu->integralFBx = 0.0f;
        imu->integralFBy = 0.0f;
        imu->integralFBz = 0.0f;
    }

    /* --- 4. Aplicar corrección al giroscopio ---
     * giro_corregido = giro_medido + Kp*error + integralFB */
    gx += imu->dcmKp * ex + imu->integralFBx;
    gy += imu->dcmKp * ey + imu->integralFBy;
    gz += imu->dcmKp * ez + imu->integralFBz;

    /* --- 5. Integrar el cuaternión ---
     * dq/dt = 0.5 * q ⊗ [0, gx, gy, gz]
     *
     * Linealización de primer orden (válida para dt pequeño):
     *   q(t+dt) ≈ q(t) + 0.5*dt * q ⊗ ω
     *
     * Betaflight imu.c líneas 275-283: */
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    const float qw = imu->q.w;
    const float qx = imu->q.x;
    const float qy = imu->q.y;
    const float qz = imu->q.z;

    imu->q.w += (-qx*gx - qy*gy - qz*gz);
    imu->q.x += (+qw*gx + qy*gz - qz*gy);
    imu->q.y += (+qw*gy - qx*gz + qz*gx);
    imu->q.z += (+qw*gz + qx*gy - qy*gx);

    /* --- 6. Normalizar el cuaternión ---
     * Mantiene |q| = 1 para que sea una rotación válida.
     * Un cuaternión no normalizado no representa ninguna rotación. */
    const float recipNorm = invSqrt(sq(imu->q.w) + sq(imu->q.x) +
                                    sq(imu->q.y) + sq(imu->q.z));
    imu->q.w *= recipNorm;
    imu->q.x *= recipNorm;
    imu->q.y *= recipNorm;
    imu->q.z *= recipNorm;

    /* --- 7. Actualizar matriz de rotación y ángulos de Euler --- */
    imuComputeRotationMatrix(imu);
    imuComputeEuler(imu);

    imu->attitudeIsEstablished = true;
}
