//
// Created by jwhww on 2026/1/18.
//

#ifndef JJ_GIMBAL_2025_POWER_METER_H
#define JJ_GIMBAL_2025_POWER_METER_H

// 功率计类型
#define  TYPE_POW_METER_WZY                      (0x01)     // 吴张扬功率计
#define  TYPE_POW_METER_OUYANG                   (0x02)     // 欧阳云翔功率计
//======================================定义使用的功率计类型======================================
#define POW_METER_TYPE                  (TYPE_POW_METER_OUYANG)     // 宏是文本替换
// 防止未定义功率计类型
#if (POW_METER_TYPE != TYPE_POW_METER_WZY) && (POW_METER_TYPE != TYPE_POW_METER_OUYANG)
#error "Invalid POW_METER_TYPE"
#endif
//============================================================================================

#define POWERMETER_CAN_RX_ID                   (0x212)  // 功率计CAN反馈ID
#define POWERMETER_CAN_TX_ID                   (0x213)  // 功率计CAN发报ID

#include "main.h"

// 功率计结构体
typedef struct
{
    // 解算量
    uint16_t canID;
    int16_t x10mV_battary_to_chassis;          // 电池电压
    int16_t x10mA_battary_to_chassis;          // 电池电流
    int16_t x10mV_capacitorBank;               // 电容组电压
    int16_t x10mW_outMaxFormCap_WZY;                    // 当前电容组最大能输出的功率
    int16_t W_realPowerFromChassis_OUYANG;          // 当前底盘真实输出功率

    // 计算量
    float P_x1W_chassis;            // 底盘功率
    float P_x1W_capacitorBank;      // 电容组功率

    uint8_t ifPowerMeter_Ready;    // 是否收到第一帧功率计数据
    uint8_t ifCapacitorBank_Ready; // 是否接入超级电容

    // 功率计类型
    uint8_t type_of_powMeter;

    struct {
        FunctionalState enable_CapacitorBank; // 是否开启超级电容；DISABLE - 关闭，ENABLE - 开启
        int16_t x10mW_limitFromJudgement; // 从判断模块获取的功率限制
        int16_t x10mW_targetPowerToChassis; // 目标底盘功率
    }Tx_data;
    // 时间戳, 单位ms
    uint32_t timestamp;
} powMeter_capacitorBank_t;

extern powMeter_capacitorBank_t powMeter_capBank_info; // 功率计结构体实例

static float LPS_filter(const float a, const float new_value, const float last_value);
void powMeter_data_handle(uint16_t canID, const uint8_t * rx_data);     // 收报处理函数
void powMeter_load_and_Send(uint8_t powMeter_type, CAN_HandleTypeDef *hcan);
void powMeter_WZY_send(CAN_HandleTypeDef *hcan, int16_t limit_from_judgement, uint8_t if_enable_cap);    // 吴张扬功率计发报函数
void powMeter_OUYANG_send(CAN_HandleTypeDef *hcan, int16_t limit_from_judgement, int16_t target_power_to_chassis, uint8_t if_enable_cap);



#endif //JJ_GIMBAL_2025_POWER_METER_H