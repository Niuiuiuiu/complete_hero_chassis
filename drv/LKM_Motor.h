#ifndef _LKM_MOTOR_H_
#define _LKM_MOTOR_H_

#include "main.h"


typedef enum {
    LKM_read_motorstate1=0x9A, //读取电机状态1
    LKM_read_motorstate2=0x9C, //读取电机状态2
    LKM_motor_off=0x80,   //电机关闭
    LKM_motor_on=0x88,    //电机启动
    LKM_motor_stop=0x81,  //电机停止
    LKM_motor_restart=0x07, //电机重启
}order_t;

typedef union {
    struct{
    uint8_t low_vol:1;
    uint8_t high_vol:1;
    uint8_t high_temp_driv:1; //驱动过温
    uint8_t high_temp_motor:1; //电机过温
    uint8_t high_cur_motor:1; //电机过流
    uint8_t short_cir:1; //电机短路
    uint8_t locked:1; //电机堵转
    uint8_t unconnnected:1; //输入信号丢失超时
    }bit;
    uint8_t raw;
}errorState_t;

typedef enum {
    on=0x00,
    off=0x10,
}motorState_t;

typedef struct {

    uint8_t ID; //电机ID
    int8_t temp;
    int16_t DC_vol; //母线电压
    int16_t DC_cur; //母线电流
    motorState_t motorState;
    errorState_t errorState;
    int16_t Tor_cur;
    int16_t speed;
    uint16_t encode;

}LKM_Motor;


void LKM_Motor_Init(LKM_Motor* motor,uint8_t ID);
void send_motor_order(CAN_HandleTypeDef *hcan,LKM_Motor* motor,order_t order);
void read_motorstate1(CAN_HandleTypeDef *hcan,LKM_Motor* motor);
void process_motorstate1(LKM_Motor* motor,uint8_t *Rxdata);
void read_motorstate2(CAN_HandleTypeDef *hcan,LKM_Motor* motor);
void process_motorstate2(LKM_Motor* motor,uint8_t *Rxdata);
void LKM_choose_procession(LKM_Motor* motor,uint8_t *Rxdata);
void motor_off(CAN_HandleTypeDef *hcan,LKM_Motor* motor);
void motor_on(CAN_HandleTypeDef *hcan,LKM_Motor* motor);
void motor_stop(CAN_HandleTypeDef *hcan,LKM_Motor* motor);
void motor_restart(CAN_HandleTypeDef *hcan,LKM_Motor* motor);
void LKM_motor_send_current(CAN_HandleTypeDef *hcan,LKM_Motor* motor,int16_t current);


#endif /* _LKM_MOTOR_H_ */