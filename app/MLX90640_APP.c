//
// Created by 33140 on 2026/10/20.
//

#include <stdint.h>
#include "MLX90640_APP.h"

#include <string.h>
#include <stdlib.h>

#include "mlx90640/headers/MLX90640_API.h"
#include "mlx90640/headers/MLX90640_I2C_Driver.h"
#include "vofa.h"

#define MLX90640_ADDRESS 0x33

static uint16_t eeData[832] , frameData[834] ;
static float imageWork[768];
float  imageData[768];
static uint8_t new_data_flag = 0;
paramsMLX90640 mlx90640_info;

vofa_image_packet_t* mlx90640_image = NULL;

/**
 * @brief 检查是否有新数据
 * @return -1失败 0无新数据 1有新数据
 */
static int MLX90640_APP_check(void) {
    uint16_t buffer;
    if (MLX90640_I2CRead(MLX90640_ADDRESS,MLX90640_STATUS_REG,
    1,&buffer) != 0){ return -1;}
    new_data_flag =(MLX90640_GET_DATA_READY(buffer)!=0);
    return new_data_flag;
}

/**
 * @brief 放在主循环的图像模式读取
 * @return 0无新图像产出 1有新图像产出 负数读取失败
 */
static int MLX90640_Image_Data_process(void) {

    if (!new_data_flag){return 0;}

    new_data_flag = 0;

    static uint8_t image_complete = 0; int status;
    status = MLX90640_GetFrameData(MLX90640_ADDRESS,frameData);
    if (status<0){image_complete= 0;return status;};

    MLX90640_GetImage(frameData , &mlx90640_info ,imageWork);

    image_complete |= (uint8_t)(1U << status); //子页0是01，子页1是10

    if (image_complete != 0x03){return 0;}

    memcpy(imageData , imageWork , sizeof(imageData));
    image_complete = 0;
    return 1;

}

static int MLX90640_Image_frame_config(void) {

    if (mlx90640_image == NULL) {
        mlx90640_image =malloc(sizeof(vofa_image_packet_t) + 768U);

        if (mlx90640_image == NULL) {return -1;}
    }
    vofa_image_packet_config(&mlx90640_image->header , 0 ,
    768,32, 24,Format_Grayscale8);

    return 0;
}

static void image_to_gray8(const float *src, uint8_t *gray)
{
    float min_value = src[0];
    float max_value = src[0];

    // 找出当前帧的最小值、最大值
    for (uint16_t i = 1; i < 768; i++) {
        if (src[i] < min_value) min_value = src[i];
        if (src[i] > max_value) max_value = src[i];
    }

    // 所有像素相同，统一显示黑色，避免除零
    if (max_value == min_value) {
        memset(gray, 0, 768);
        return;
    }

    float scale = 255.0f / (max_value - min_value);

    for (uint16_t i = 0; i < 768; i++) {
        float value = (src[i] - min_value) * scale;

        if (value < 0.0f)   value = 0.0f;
        if (value > 255.0f) value = 255.0f;

        gray[i] = (uint8_t)(value + 0.5f); // 四舍五入
    }
}

int MLX90640_APP_Init(void) {
    int error = 0;
    error = MLX90640_DumpEE(MLX90640_ADDRESS, eeData);
    if (error != 0){return error;}
    error = MLX90640_ExtractParameters(eeData, &mlx90640_info);
    if (error != 0){return error;}
    error = MLX90640_SetRefreshRate(MLX90640_ADDRESS, 0x04);
    if (error != 0) {return error;}
    return MLX90640_Image_frame_config();
}

int MLX90640_APP_Task(void) {

    if (mlx90640_image == NULL) {return -1; } // 图片包未初始化

    int status = MLX90640_APP_check();
    if (status <= 0) {return status; }// 0：无新数据；负数：检查失败

    status = MLX90640_Image_Data_process();
    if (status <= 0) {return status; }  // 0：子页未集齐；负数：读取失败

    image_to_gray8(imageData, mlx90640_image->data);

    HAL_StatusTypeDef tx = vofa_send_image(mlx90640_image);

    if (tx == HAL_OK) {return 1; } // 已启动发送，不代表发送完成
    if (tx == HAL_BUSY) {return 0; } // 串口忙，跳过本帧

    return -1; // 发送启动失败

}

