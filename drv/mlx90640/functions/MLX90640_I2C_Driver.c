//
// Created by 33140 on 2026/10/19.
//

#include "mlx90640/headers/MLX90640_I2C_Driver.h"
#include "i2c.h"

#define MLX90640_I2C_TIMEOUT_MS 100U

int MLX90640_I2CGeneralReset(void) {
    uint8_t command = 0x06;

    HAL_StatusTypeDef state = HAL_I2C_Master_Transmit(
    &hi2c2, 0x00, &command, 1, 100);

    return state;
}

int MLX90640_I2CWrite(uint8_t slaveAddr,uint16_t writeAddress, uint16_t data) {

    uint8_t tx[2] = {(uint8_t)(data >> 8),(uint8_t)(data & 0xFF)  };
    uint8_t rx[2];
    const uint16_t devAddress = (uint16_t)(slaveAddr << 1);

    // 阻塞读写依赖系统时基处理超时，在中断开启的主循环中调用。
    if (__get_IPSR() != 0U || __get_PRIMASK() != 0U ||
        HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {
        return -1;
    }

    // 先写入高字节、低字节。此适配将 HAL 通信失败统一返回 -1。
    if (HAL_I2C_Mem_Write(&hi2c2,devAddress,writeAddress,
        I2C_MEMADD_SIZE_16BIT,tx, sizeof(tx), MLX90640_I2C_TIMEOUT_MS) != HAL_OK) {
        return -1;
    }

    // 阻塞读回同一地址的两个字节，接收完成后再校验。
    if (HAL_I2C_Mem_Read(&hi2c2,devAddress,writeAddress,
        I2C_MEMADD_SIZE_16BIT,rx,sizeof(rx),MLX90640_I2C_TIMEOUT_MS) != HAL_OK) {
        return -1;
    }

    const uint16_t readback = ((uint16_t)rx[0] << 8) | rx[1];
    return (readback == data) ? 0 : -2;
}

int MLX90640_I2CRead(uint8_t slaveAddr,uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data) {

    if (data == NULL || nMemAddressRead == 0 ||nMemAddressRead > UINT16_MAX / 2U) {return -1;}

    if (HAL_I2C_Mem_Read(&hi2c2,slaveAddr<<1,startAddress,
    I2C_MEMADD_SIZE_16BIT,(uint8_t*)data,nMemAddressRead*2U,MLX90640_I2C_TIMEOUT_MS)!= HAL_OK) {
        return -1;
    }

    // MLX90640 高字节先发送，转换为 STM32 使用的字节序
    for (uint16_t i = 0; i < nMemAddressRead; i++) {
        data[i] = (uint16_t)((data[i] >> 8) | (data[i] << 8));
    }

    return 0;
}

void MLX90640_I2CFreqSet(int freq)
{
    if (freq < 100 || freq > 400) {
        return;
    }

    // 正在通信时不修改配置
    if (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY ||
        __HAL_I2C_GET_FLAG(&hi2c2, I2C_FLAG_BUSY) != RESET) {
        return;
        }

    // HAL 的 ClockSpeed 单位为 Hz
    hi2c2.Init.ClockSpeed = (uint32_t)freq * 1000U;

    if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
        Error_Handler();
    }
}