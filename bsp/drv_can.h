#ifndef __DRV_CAN_H
#define __DRV_CAN_H
/* 添加标准整数类型 */
#include <stdint.h>

/* 添加 STM32 HAL 头文件 */
#include "main.h"  // 这通常会间接包含 stm32f4xx_hal_can.h
// 或者直接包含：
// #include "stm32f4xx_hal_can.h"

// 示例定义（请根据实际需求修改）
//#define CAN_FILTER(x)       ((uint32_t)(x))
//#define CAN_FIFO_1          (0x01U)
//#define CAN_STDID           (0x02U)
//#define CAN_DATA_TYPE       (0x04U)

// 滤波器编号
#define CAN_FILTER(x) ((x) << 3)

// 接收队列
#define CAN_FIFO_0 (0 << 2)
#define CAN_FIFO_1 (1 << 2)

//标准帧或扩展帧
#define CAN_STDID (0 << 1)
#define CAN_EXTID (1 << 1)

// 数据帧或遥控帧
#define CAN_DATA_TYPE (0 << 0)
#define CAN_REMOTE_TYPE (1 << 0)


void CAN_Send_Data(CAN_HandleTypeDef *hcan,uint16_t ID,const uint8_t aData[],uint16_t length);
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);





#endif
