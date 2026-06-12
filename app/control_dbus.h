#ifndef __CONTROL_DBUS_H__
#define __CONTROL_DBUS_H__

#include "dbus.h"
#include "main.h"
#include "M3508.h"
#include "dm_motor.h"
#include "bsp_imu.h"




typedef enum{
    normalcond=3,
    follow_gimbal=1,
}condition_state;


typedef struct
{
    //遥控器通道
	int16_t raw_ch[4];
    int16_t ch[4];
    uint8_t can_send_CH[8];//发送通道
    uint8_t s1;
    uint8_t s2;

    //底盘跟随信息
    uint16_t encode;
    int16_t tran_encode;
    int16_t pre_tran_encode;
    uint16_t tar_encode;
    int16_t tar_tran_encode;
    int error_encode;
    int del_angle;
    float kp_encode;
    float kd_encode;


    //一些速度分量
    int16_t vx1,vy1;
    int16_t vx2,vy2;
    int16_t vw;
}trans_dbus_data;

void dbusctrl_init(trans_dbus_data* dbuscontrol,float kp,float kd);
void trans_dbus(int16_t xx,int16_t yy,int16_t rr,trans_dbus_data* dbusdata);
void dbus_Mec_process(trans_dbus_data* dbusdata,motor_para* pid1,motor_para* pid2,motor_para* pid3,motor_para* pid4,int16_t output);
int16_t follow_gimbal_ctrl(trans_dbus_data* dbusdata,BMI088_Handle_t* imu);
void can_send_dbusdata(trans_dbus_data* dbusdata);
void process_dbus_data(trans_dbus_data* dbusdata,uint8_t* Rx_data);
void process_raw_dbus_data(trans_dbus_data* dbusdata,uint8_t* Rx_data,dm_motor_para* motor1,dm_motor_para* motor3,condition_state* condition);
void process_encode_data(trans_dbus_data* dbusdata,uint8_t* Rx_data);

#endif /* __CONTROL_DBUS_H__ */
