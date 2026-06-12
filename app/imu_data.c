//
// Created by SCP on 26-4-12.
//
#include "imu_data.h"



void BMI088_Offset(BMI088_Handle_t *imu) {
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int sample_count = 1500;

    HAL_NVIC_DisableIRQ(EXTI4_IRQn);
    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);

    for (int i = 0; i < sample_count; i++) {
        BMI088_Read_Gyro_Raw(imu);
        BMI088_Data_Convert(imu);

        gx_sum += imu->gyro.x;
        gy_sum += imu->gyro.y;
        gz_sum += imu->gyro.z;
        HAL_Delay(2);
    }

    imu->gyro_offset.x = gx_sum / sample_count;
    imu->gyro_offset.y = gy_sum / sample_count;
    imu->gyro_offset.z = gz_sum / sample_count;

    imu->acc_offset.x = 0.0f;
    imu->acc_offset.y = 0.0f;
    imu->acc_offset.z = 0.0f;

    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}



void Mahony_Update(BMI088_Handle_t *imu, float dt) {
    float q0 = imu->quat.q0, q1 = imu->quat.q1, q2 = imu->quat.q2, q3 = imu->quat.q3;

    float gx = (imu->gyro.x - imu->gyro_offset.x);
    float gy = (imu->gyro.y - imu->gyro_offset.y);
    float gz = (imu->gyro.z - imu->gyro_offset.z);
    float ax = imu->acc.x - imu->acc_offset.x;
    float ay = imu->acc.y - imu->acc_offset.y;
    float az = imu->acc.z - imu->acc_offset.z;

    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm == 0.0f) return;
    ax /= norm; ay /= norm; az /= norm;

    float vx = 2.0f * (q1*q3 - q0*q2);
    float vy = 2.0f * (q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    float ex = (ay*vz - az*vy);
    float ey = (az*vx - ax*vz);
    float ez = (ax*vy - ay*vx);

    imu->integralFBx += ex * Ki * dt;
    imu->integralFBy += ey * Ki * dt;
    imu->integralFBz += ez * Ki * dt;
    gx += Kp * ex + imu->integralFBx;
    gy += Kp * ey + imu->integralFBy;
    gz += Kp * ez + imu->integralFBz;

    imu->quat.q0 += (-q1*gx - q2*gy - q3*gz) * (0.5f * dt);
    imu->quat.q1 += ( q0*gx + q2*gz - q3*gy) * (0.5f * dt);
    imu->quat.q2 += ( q0*gy - q1*gz + q3*gx) * (0.5f * dt);
    imu->quat.q3 += ( q0*gz + q1*gy - q2*gx) * (0.5f * dt);

    norm = sqrtf(imu->quat.q0*imu->quat.q0 + imu->quat.q1*imu->quat.q1 +
                 imu->quat.q2*imu->quat.q2 + imu->quat.q3*imu->quat.q3);
    imu->quat.q0 /= norm; imu->quat.q1 /= norm; imu->quat.q2 /= norm; imu->quat.q3 /= norm;
}


void Quaternion_To_Euler(BMI088_Handle_t *imu) {
    float q0 = imu->quat.q0, q1 = imu->quat.q1, q2 = imu->quat.q2, q3 = imu->quat.q3;

    imu->euler.roll = atan2f(2.0f * (q0*q1 + q2*q3), 1.0f - 2.0f * (q1*q1 + q2*q2)) * 57.29578f;

    float sinp = 2.0f * (q0*q2 - q3*q1);
    if (fabsf(sinp) >= 1.0f)
        imu->euler.pitch = copysignf(1.570796f, sinp) * 57.29578f;
    else
        imu->euler.pitch = asinf(sinp) * 57.29578f;

    imu->euler.yaw = atan2f(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3)) * 57.29578f;
}

void BMI088_Calibrate_Pose(BMI088_Handle_t *imu) {

    imu->Euler_offset.roll_offset = imu->euler.roll;
    imu->Euler_offset.pitch_offset = imu->euler.pitch;
    imu->Euler_offset.yaw_offset = imu->euler.yaw;
}


void Mahony_Initial_Alignment(BMI088_Handle_t *imu) {

    float ax = imu->acc.x;
    float ay = imu->acc.y;
    float az = imu->acc.z;
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if(norm < 0.1f) return;
    ax /= norm; ay /= norm; az /= norm;


    float half_cos = sqrtf(0.5f * (1.0f + az));
    
    imu->quat.q0 = half_cos;
    imu->quat.q1 = -ay / (2.0f * half_cos);
    imu->quat.q2 = ax / (2.0f * half_cos);
    imu->quat.q3 = 0.0f;

    imu->integralFBx = 0.0f;
    imu->integralFBy = 0.0f;
    imu->integralFBz = 0.0f;
}

