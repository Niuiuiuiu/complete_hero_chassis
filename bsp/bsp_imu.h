//
// Created by SCP on 26-4-10.
//

#ifndef BSP_IMU_H
#define BSP_IMU_H

#include "main.h"
#include "tim.h"

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} BMI088_RawData_t;


typedef struct {
    float x;
    float y;
    float z;
} BMI088_RealData_t;

typedef struct {
    float pitch; // 俯仰角 (围绕 Y 轴)
    float roll;  // 横滚角 (围绕 X 轴)
    float yaw;   // 偏航角 (围绕 Z 轴)
} EulerAngle_t;

// 四元数结构体
typedef struct {
    float q0; // 实部
    float q1; // 虚部 i
    float q2; // 虚部 j
    float q3; // 虚部 k
} Quaternion_t;


typedef struct {

    SPI_HandleTypeDef *hspi;

    float temperature;

    struct {
        GPIO_TypeDef *port;
        uint16_t pin;
    } acc_cs;

    struct {
        GPIO_TypeDef *port;
        uint16_t pin;
    } gyro_cs;


    BMI088_RawData_t acc_raw;
    BMI088_RawData_t gyro_raw;

    BMI088_RealData_t acc;
    BMI088_RealData_t gyro;

    BMI088_RealData_t acc_offset;
    BMI088_RealData_t gyro_offset;


    float acc_lsb_to_g;
    float gyro_lsb_to_dps;

    Quaternion_t quat;
    EulerAngle_t euler;

    struct {
        float roll_offset;
		float pitch_offset;
		float yaw_offset;
    } Euler_offset;
	
	float integralFBx;
	float integralFBy;
	float integralFBz;
} BMI088_Handle_t;

void BMI088_Init(BMI088_Handle_t *imu_handle);

void BMI088_Start(BMI088_Handle_t *imu_handle);

uint8_t BMI088_Read_Acc(BMI088_Handle_t *imu_handle, uint8_t reg);

uint8_t BMI088_Read_Gyro(BMI088_Handle_t *imu_handle, uint8_t reg);

void BMI088_Write_Acc(BMI088_Handle_t *imu_handle, uint8_t reg, uint8_t data);

void BMI088_Write_Gyro(BMI088_Handle_t *imu_handle, uint8_t reg, uint8_t data);

void BMI088_Read_Acc_Raw(BMI088_Handle_t *imu_handle);

void BMI088_Read_Gyro_Raw(BMI088_Handle_t *imu_handle);

void BMI088_Restart(BMI088_Handle_t *imu_handle);

void BMI088_Data_Convert(BMI088_Handle_t *imu);

void BMI088_Read_Temp(BMI088_Handle_t *imu_handle);

void Temp_Control_Task(BMI088_Handle_t *imu_handle);

#endif //BSP_IMU_H
