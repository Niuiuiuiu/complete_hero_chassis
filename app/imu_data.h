//
// Created by SCP on 26-4-12.
//

#ifndef IMU_DATA_H
#define IMU_DATA_H

#include <stdint.h>
#include "main.h"
#include "bsp_imu.h"
#include <math.h>

#define Kp 0.8f
#define Ki 0.01f

void BMI088_Offset(BMI088_Handle_t *imu);

void Mahony_Update(BMI088_Handle_t *imu, float dt);

void Quaternion_To_Euler(BMI088_Handle_t *imu);

void BMI088_Calibrate_Pose(BMI088_Handle_t *imu);

void Mahony_Initial_Alignment(BMI088_Handle_t *imu);

#endif //IMU_DATA_H
