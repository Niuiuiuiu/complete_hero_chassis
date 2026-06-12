//
// Created by jwhww on 2026/1/18.
//

#include "power_meter.h"

powMeter_capacitorBank_t powMeter_capBank_info = {
    .type_of_powMeter = POW_METER_TYPE,     // 默认吴张扬功率计
    .Tx_data.enable_CapacitorBank = DISABLE,      // 默认关闭
};

//------------------------------------------------------------------------------------------------------
// 函数简介         文件内低通滤波
// 使用示例         LPS_filter()
// 参数介绍         a               低通滤波系数,越大越尖锐，越小越迟钝，常见0.02~0.2或0.5
// 返回参数         无
// 备注信息
//------------------------------------------------------------------------------------------------------
static float LPS_filter(const float a, const float new_value, const float last_value)
{
    //低通滤波
    return a * new_value + (1.f - a) * last_value;
}

//------------------------------------------------------------------------------------------------------
// 函数简介     功率计-收报
// 使用示例     powMeter_data_handle(0x120, rx_data);
// 输入参数     CAN_ID                  CAN ID
// 输入参数     rx_data                 接收数据指针
// 返回参数     无
// 备注信息     小端序
//------------------------------------------------------------------------------------------------------
void powMeter_data_handle(const uint16_t canID, const uint8_t * rx_data)     // 第一个const指的是（const uint8_t），第二个是(
{
    if (canID == POWERMETER_CAN_RX_ID)
    {
        // 解算量
        powMeter_capBank_info.canID = canID;
        powMeter_capBank_info.x10mV_battary_to_chassis = (int16_t)(rx_data[1] << 8 | rx_data[0]);  // 单位10mV
        powMeter_capBank_info.x10mA_battary_to_chassis = (int16_t)(rx_data[3] << 8 | rx_data[2]);  // 单位10mA
        powMeter_capBank_info.x10mV_capacitorBank = (int16_t)(rx_data[5] << 8 | rx_data[4]);  // 单位10mV
        // 最后一个数据不同功率计是不一样的
        if (powMeter_capBank_info.type_of_powMeter == TYPE_POW_METER_WZY) {
            powMeter_capBank_info.x10mW_outMaxFormCap_WZY = (int16_t)(rx_data[7] << 8 | rx_data[6]);  // 单位10mW
            powMeter_capBank_info.W_realPowerFromChassis_OUYANG = -1;
        }
        if (powMeter_capBank_info.type_of_powMeter == TYPE_POW_METER_OUYANG) {
            powMeter_capBank_info.W_realPowerFromChassis_OUYANG = (int16_t)(rx_data[7] << 8 | rx_data[6]);  // 单位10mW
            powMeter_capBank_info.x10mW_outMaxFormCap_WZY = -1;
        }

        // 计算量
        powMeter_capBank_info.P_x1W_chassis = (float)(powMeter_capBank_info.x10mV_battary_to_chassis * powMeter_capBank_info.x10mA_battary_to_chassis) / 10000.0f;
        // 计算量的低通
        static float P_x1W_chassis_last = -100000.f;
        if (P_x1W_chassis_last != -100000.f)
            powMeter_capBank_info.P_x1W_chassis = LPS_filter(0.1f, powMeter_capBank_info.P_x1W_chassis, P_x1W_chassis_last);
        P_x1W_chassis_last = powMeter_capBank_info.P_x1W_chassis;

        if (powMeter_capBank_info.ifPowerMeter_Ready == 0) powMeter_capBank_info.ifPowerMeter_Ready = 1;
        // 时间戳(由于中断内HAL_GetTick值不会变，所以这个时间戳指的是终端发生的时刻)
        powMeter_capBank_info.timestamp = HAL_GetTick();

        // if (powMeter_capBank_info.V_capacitorBank != 0 && powMeter_capBank_info.V_battary !=0 && powMeter_capBank_info.A_battary != 0)
        //     powMeter_capBank_info.ifCapacitorBank_Ready = 1;
        // VOFA监控 - 底盘实时功率
        // Vofa_SendFloatData(1, powMeter_capBank_info.P_x1W_chassis);
    }
}

//------------------------------------------------------------------------------------------------------
// 函数简介     功率计 - 装载
// 使用示例
// 返回参数     无
// 备注信息     小端序；具体见吴张扬功率计说明书
//------------------------------------------------------------------------------------------------------
void powMeter_load_and_Send(uint8_t powMeter_type, CAN_HandleTypeDef *hcan)
{
    CAN_TxHeaderTypeDef TxHeader;
    static uint32_t Tx_mailbox = 0;

    TxHeader.StdId = POWERMETER_CAN_TX_ID;
    TxHeader.ExtId = 0;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 4;
    TxHeader.TransmitGlobalTime = DISABLE;

    uint8_t data[8] = {0};

    if (powMeter_type == TYPE_POW_METER_WZY) {
        data[0] = (uint8_t)(powMeter_capBank_info.Tx_data.x10mW_limitFromJudgement);
        data[1] = (uint8_t)(powMeter_capBank_info.Tx_data.x10mW_limitFromJudgement >> 8);
        data[4] = (uint8_t) ! powMeter_capBank_info.Tx_data.enable_CapacitorBank;   // ！逻辑非运算：等于0则输出int 1，其他值均输出int 0，
    }
    if (powMeter_type == TYPE_POW_METER_OUYANG) {
        data[0] = (uint8_t)(powMeter_capBank_info.Tx_data.x10mW_limitFromJudgement);
        data[1] = (uint8_t)(powMeter_capBank_info.Tx_data.x10mW_limitFromJudgement >> 8);
        data[2] = (uint8_t)(powMeter_capBank_info.Tx_data.x10mW_targetPowerToChassis);
        data[3] = (uint8_t)(powMeter_capBank_info.Tx_data.x10mW_targetPowerToChassis >> 8);
        data[4] = (uint8_t) ! powMeter_capBank_info.Tx_data.enable_CapacitorBank;
    }
    HAL_CAN_AddTxMessage(hcan, &TxHeader, data, &Tx_mailbox);
}

//------------------------------------------------------------------------------------------------------
// 函数简介     吴张扬功率计 - 发报
// 使用示例
// 输入参数     hcan                    CAN句柄指针
// 输入参数     limit_from_judgement    从裁判系统获取的功率限制（单位：10mW）
// 输入参数     if_enable_cap           是否使能电容Bank（0：使能，1：不使能）
// 返回参数     无
// 备注信息     小端序；具体见吴张扬功率计说明书
//------------------------------------------------------------------------------------------------------
void powMeter_WZY_send(CAN_HandleTypeDef *hcan, int16_t limit_from_judgement, uint8_t if_enable_cap)
{
    powMeter_capBank_info.Tx_data.x10mW_limitFromJudgement = limit_from_judgement;
    powMeter_capBank_info.Tx_data.enable_CapacitorBank = if_enable_cap;
    powMeter_load_and_Send(TYPE_POW_METER_WZY, hcan);
}


//------------------------------------------------------------------------------------------------------
// 函数简介     欧阳云翔功率计 - 发报
// 使用示例
// 输入参数     hcan                    CAN句柄指针
// 输入参数     limit_from_judgement    从裁判系统获取的功率限制（单位：10mW）
// 输入参数     target_power_to_chassis 目标功率到底盘（单位：10mW）
// 输入参数     if_enable_cap           是否使能电容Bank（0：使能，1：不使能）
// 返回参数     无
// 备注信息     小端序；具体见飞书超级电容说明书
//------------------------------------------------------------------------------------------------------
void powMeter_OUYANG_send(CAN_HandleTypeDef *hcan, int16_t limit_from_judgement, int16_t target_power_to_chassis, uint8_t if_enable_cap)
{
    powMeter_capBank_info.Tx_data.x10mW_limitFromJudgement = limit_from_judgement;
    powMeter_capBank_info.Tx_data.x10mW_targetPowerToChassis = target_power_to_chassis;
    powMeter_capBank_info.Tx_data.enable_CapacitorBank = if_enable_cap;
    powMeter_load_and_Send(TYPE_POW_METER_OUYANG, hcan);
}