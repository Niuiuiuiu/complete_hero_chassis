//
// Created by 33140 on 2026/10/20.
//

#ifndef COMPLETE_HERO_CHASSIS_MLX90640_APP_H
#define COMPLETE_HERO_CHASSIS_MLX90640_APP_H

// 完整 32×24 图像的浮点数据，非摄氏温度。
extern float imageData[768];

/**
 * @brief 读取并解析标定参数，设置刷新率为 8 Hz；调用前需初始化 I2C2。
 * @return 0 成功，非 0 为初始化失败的错误码。
 */
int MLX90640_APP_Init(void);


/**
 * @brief 主循环调用：检查新数据，集齐两个子页后转为灰度图并通过 VOFA DMA 发送。
 * @return 1 已启动发送；0 无完整新图像或串口忙；负数表示初始化、通信或发送错误。
 * @note 调用前完成应用初始化、USART1 及发送 DMA 初始化；返回 1 不代表发送已完成。
 */
int MLX90640_APP_Task(void);

#endif //COMPLETE_HERO_CHASSIS_MLX90640_APP_H
