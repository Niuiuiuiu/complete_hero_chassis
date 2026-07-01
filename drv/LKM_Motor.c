#include "lkm_motor.h"
#include "drv_can.h"
#include "main.h"

/**
 * @function: LKM_Motor_Init
 * @brief: 电机初始化
 * @param: motor:电机结构体指针
 * @param: ID:电机ID
 * @retval: None
 */
void LKM_Motor_Init(LKM_Motor* motor,uint8_t ID){
    motor->ID = ID;
    motor->temp = 0;
    motor->DC_vol = 0;
    motor->DC_cur = 0;
    motor->Tor_cur = 0;
    motor->speed = 0;
    motor->encode = 0;
    motor->motorState = off;
    motor->errorState.raw = 0;
}


/**
 * @function: send_motor_order
 * @brief: 发送电机指令(用这个)
 * @param: hcan:CAN句柄
 * @param: motor:电机结构体指针
 * @param: order:指令
 * @retval: None
 */
void send_motor_order(CAN_HandleTypeDef *hcan,LKM_Motor* motor,order_t order){
    uint8_t DATA[8] = {0};
    DATA[0] =order;
    CAN_Send_Data(hcan, 0x140+motor->ID,DATA,8);
}


/**
 * @function: read_motorstate1
 * @brief: 读取电机状态1
 * @param: hcan:CAN句柄
 * @param: motor:电机结构体指针
 * @retval: None
 */
void read_motorstate1(CAN_HandleTypeDef *hcan,LKM_Motor* motor){
    uint8_t DATA[8] = {0};
    DATA[0] = 0x9A;
    CAN_Send_Data(hcan, 0x140+motor->ID,DATA,8);
}


/**
 * @function: process_motorstate1
 * @brief: 处理电机状态1
 * @param: motor:电机结构体指针
 * @param: Rxdata:接收到的数据
 * @retval: None
 */
void process_motorstate1(LKM_Motor* motor,uint8_t *Rxdata){
    motor->temp=(int8_t)Rxdata[1];
    motor->DC_vol=(int16_t)(Rxdata[3]<<8|Rxdata[2]);
    motor->DC_cur=(int16_t)(Rxdata[5]<<8|Rxdata[4]);
    motor->motorState=(motorState_t)Rxdata[6];
    motor->errorState.raw=Rxdata[7];
}


/**
 * @function: read_motorstate2
 * @brief: 读取电机状态2
 * @param: hcan:CAN句柄
 * @param: motor:电机结构体指针
 * @retval: None
 */
void read_motorstate2(CAN_HandleTypeDef *hcan,LKM_Motor* motor){
    uint8_t DATA[8] = {0};
    DATA[0] = 0x9C;
    CAN_Send_Data(hcan, 0x140+motor->ID,DATA,8);
}


/**
 * @function: process_motorstate2
 * @brief: 处理电机状态2
 * @param: motor:电机结构体指针
 * @param: Rxdata:接收到的数据
 * @retval: None
 */
void process_motorstate2(LKM_Motor* motor,uint8_t *Rxdata){
    motor->temp=(int8_t)Rxdata[1];
    motor->Tor_cur=(int16_t)(Rxdata[3]<<8|Rxdata[2]);
    motor->speed=(int16_t)(Rxdata[5]<<8|Rxdata[4]);
    motor->encode=(uint16_t)(Rxdata[7]<<8|Rxdata[6]);
}


/**
 * @function: choose_procession
 * @brief: 选择处理函数
 * @param: motor:电机结构体指针
 * @param: Rxdata:接收到的数据
 * @retval: None
 */
void LKM_choose_procession(LKM_Motor* motor,uint8_t *Rxdata){
    switch(Rxdata[0]){
        case 0x9A:
            process_motorstate1(motor,Rxdata);
            break;
        case 0xA1:
        case 0x9C:
            process_motorstate2(motor,Rxdata);
            break;
        default:
            break;
    }
}


/**
 * @function: motor_off
 * @brief: 电机关闭
 * @param: hcan:CAN句柄
 * @param: motor:电机结构体指针
 * @retval: None
 */
void motor_off(CAN_HandleTypeDef *hcan,LKM_Motor* motor){
    uint8_t DATA[8] = {0};
    DATA[0] = 0x80;
    CAN_Send_Data(hcan, 0x140+motor->ID,DATA,8);
}


/**
 * @function: motor_on
 * @brief: 电机开启
 * @param: hcan:CAN句柄
 * @param: motor:电机结构体指针
 * @retval: None
 */
void motor_on(CAN_HandleTypeDef *hcan,LKM_Motor* motor){
    uint8_t DATA[8] = {0};
    DATA[0] = 0x88;
    CAN_Send_Data(hcan, 0x140+motor->ID,DATA,8);
}


/**
 * @function: motor_stop
 * @brief: 停止电机，但不清除电机运行状态。再次发送控制指令即可控制电机动作。
 * @param: hcan:CAN句柄
 * @param: motor:电机结构体指针
 */
void motor_stop(CAN_HandleTypeDef *hcan,LKM_Motor* motor){
    uint8_t DATA[8] = {0};
    DATA[0] = 0x81;
    CAN_Send_Data(hcan, 0x140+motor->ID,DATA,8);
}


/**
 * @function: motor_send_current
 * @brief: 发送电机电流
 * @param: hcan:CAN句柄
 * @param: motor:电机结构体指针
 * @param: current:电流值
 * @retval: None
 */
void LKM_motor_send_current(CAN_HandleTypeDef *hcan,LKM_Motor* motor,int16_t current){
    uint8_t DATA[8] = {0};
    DATA[0] = 0xA1;
    DATA[4] = (uint8_t)current;
    DATA[5] = (uint8_t)(current>>8);
    CAN_Send_Data(hcan, 0x140+motor->ID,DATA,8);
}


/**
 * @function: motor_restart
 * @brief: 电机重启
 * @param: hcan:CAN句柄
 * @param: motor:电机结构体指针
 * @retval: None
 */
void motor_restart(CAN_HandleTypeDef *hcan,LKM_Motor* motor){
    uint8_t DATA[8] = {0};
    DATA[0] = 0x07;
    CAN_Send_Data(hcan, 0x140+motor->ID,DATA,8);
}