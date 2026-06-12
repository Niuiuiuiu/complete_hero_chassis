#include <math.h>
#include <stdlib.h>
#include "control_dbus.h"
#include "dbus.h"
#include "M3508.h"
#include "bsp_imu.h"

trans_dbus_data dbuscontrol;
condition_state condition_ctrler = normalcond;

/**
 * @brief 初始化遥控器控制数据结构体
 *
 * @param 遥控器控制数据结构体
 * @return 无
 */
void dbusctrl_init(trans_dbus_data* dbuscontrol,float kp,float kd)
{
    for(int i=0;i<4;i++)
    {
        dbuscontrol->raw_ch[i]=1024;
    }
    dbuscontrol->s1=3;
    dbuscontrol->s2=3;
    dbuscontrol->kp_encode=kp;
    dbuscontrol->kd_encode=kd;
	dbuscontrol->tar_tran_encode=4197;
}


/**
 * @brief 映射遥控器数据
 *
 * @param 左摇杆上下
 * @param 左摇杆左右
 * @param 遥控器控制数据结构体
 * @return 无
 */
void trans_dbus(int16_t xx,int16_t yy,int16_t rr,trans_dbus_data* dbusdata)
{
	dbusdata->ch[3]=9.0909*xx-9309.09; //标准
	dbusdata->ch[2]=9.0909*yy-9309.09;	
    dbusdata->ch[0]=9.0909*rr-9309.09;

    if(abs(dbusdata->ch[3])<10){dbusdata->ch[3]=0;}//死区
    if(abs(dbusdata->ch[2])<10){dbusdata->ch[2]=0;}
    if(abs(dbusdata->ch[0])<10){dbusdata->ch[0]=0;}

    if(dbusdata->ch[3]>6000){dbusdata->ch[3]=6000;} //限幅
    if(dbusdata->ch[3]<-6000){dbusdata->ch[3]=-6000;}
    if(dbusdata->ch[2]>6000){dbusdata->ch[2]=6000;}
    if(dbusdata->ch[2]<-6000){dbusdata->ch[2]=-6000;}
}


/**
 * @brief 遥控器麦轮控制
 *
 * @param 遥控器控制数据结构体
 * @param 电机pid参数结构体(ID:5,6,7,8)
 * @return 无
 */
void dbus_Mec_process(trans_dbus_data* dbusdata,motor_para* pid1,motor_para* pid2,motor_para* pid3,motor_para* pid4,int16_t output)
{
    dbusdata->vx1=dbusdata->ch[3]+dbusdata->ch[2];
    dbusdata->vx2=dbusdata->ch[3]-dbusdata->ch[2];

    
    pid1->tar_speed=sqrtf(2.0f)*dbusdata->vx1+output;
	  if(abs(pid1->tar_speed)>4000){pid1->tar_speed=pid1->tar_speed>0?4000:-4000;}        //这样在最大转速下无法旋转
    pid4->tar_speed=-sqrtf(2.0f)*dbusdata->vx1+output;
		if(abs(pid4->tar_speed)>4000){pid4->tar_speed=pid4->tar_speed>0?4000:-4000;}
    pid2->tar_speed=sqrtf(2.0f)*dbusdata->vx2+output;
		if(abs(pid2->tar_speed)>4000){pid2->tar_speed=pid2->tar_speed>0?4000:-4000;}
    pid3->tar_speed=-sqrtf(2.0f)*dbusdata->vx2+output;
		if(abs(pid3->tar_speed)>4000){pid3->tar_speed=pid3->tar_speed>0?4000:-4000;}
    
}


/**
 * @brief 跟随云台控制
 *
 * @param 遥控器控制数据结构体
 * @param IMU数据结构体
 * @return 跟随云台控制输出
 */
int16_t follow_gimbal_ctrl(trans_dbus_data* dbusdata,BMI088_Handle_t* imu)
{
    int16_t output;
    float p_term,d_term;
    p_term=dbusdata->kp_encode*dbusdata->error_encode;
    d_term=dbusdata->kd_encode*(-dbusdata->del_angle/0.01f-imu->gyro.z);
    output=(int16_t)(p_term+d_term);
    return output;
}


/**
 * @brief 将遥控器数据打包成CAN数据
 *
 * @param 遥控器控制数据结构体
 * @return 无
 */
void can_send_dbusdata(trans_dbus_data* dbusdata)
{
    dbusdata->can_send_CH[0] =(uint8_t) ((dbusdata->ch[0] >> 8) & 0xFF);
    dbusdata->can_send_CH[1] = (uint8_t) ((dbusdata->ch[0] & 0xFF));
    dbusdata->can_send_CH[2] = (uint8_t) ((dbusdata->ch[1] >> 8) & 0xFF);
    dbusdata->can_send_CH[3] = (uint8_t) ((dbusdata->ch[1] & 0xFF));
    dbusdata->can_send_CH[4] = (uint8_t) ((dbusdata->ch[2] >> 8) & 0xFF);
    dbusdata->can_send_CH[5] = (uint8_t) ((dbusdata->ch[2] & 0xFF));
    dbusdata->can_send_CH[6] = (uint8_t) ((dbusdata->ch[3] >> 8) & 0xFF);
    dbusdata->can_send_CH[7] = (uint8_t) ((dbusdata->ch[3] & 0xFF));
    
}


/**
 * @brief 处理can接收到处理过的通道数据，更新遥控器控制数据结构体
 *
 * @param 遥控器控制数据结构体
 * @param 接收到的CAN数据
 * @return 无
 */
void process_dbus_data(trans_dbus_data* dbusdata,uint8_t* Rx_data)
{
    dbusdata->ch[0] = (int16_t)(Rx_data[0] << 8 | Rx_data[1]);
    dbusdata->ch[1] = (int16_t)(Rx_data[2] << 8 | Rx_data[3]);
    dbusdata->ch[2] = (int16_t)(Rx_data[4] << 8 | Rx_data[5]);
    dbusdata->ch[3] = (int16_t)(Rx_data[6] << 8 | Rx_data[7]);
}


/**
 * @brief 处理can接收到的原始通道数据，更新遥控器控制数据结构体
 *
 * @param 遥控器控制数据结构体
 * @param 接收到的原始CAN数据
 * @param 机器人状态指针
 * @return 无
 */
void process_raw_dbus_data(trans_dbus_data* dbusdata,uint8_t* Rx_data,dm_motor_para* motor1,dm_motor_para* motor3,condition_state* condition)
{
    static uint8_t prev_s1=3;
    dbusdata->raw_ch[0] = (int16_t)(Rx_data[0] << 8 | Rx_data[1]);
    dbusdata->raw_ch[2] = (int16_t)(Rx_data[2] << 8 | Rx_data[3]);
    dbusdata->raw_ch[3] = (int16_t)(Rx_data[4] << 8 | Rx_data[5]);
    dbusdata->s1 = (uint8_t)(Rx_data[6]);
    if(prev_s1==1 && dbusdata->s1==3){
        motor1->state=landing;
        motor3->state=landing;
    }
    if(motor1->state!=landing){
        motor1->state = (robo_state)(dbusdata->s1);
    }
    if(motor3->state!=landing){
        motor3->state = (robo_state)(dbusdata->s1);
    }
    dbusdata->s2 = (uint8_t)(Rx_data[7]);
    *condition = (condition_state)(dbusdata->s2);
    prev_s1 = (uint8_t)(dbusdata->s1);
}


/**
 * @brief 处理接收到的编码器数据，更新云台电机数据
 *
 * @param 遥控器控制数据结构体
 * @param 接收到的编码器CAN数据
 * @return 无
 */
void process_encode_data(trans_dbus_data* dbusdata,uint8_t* Rx_data)
{
    dbusdata->pre_tran_encode=dbusdata->tran_encode;

//    dbusdata->tar_encode = (uint16_t)(Rx_data[0] << 8 | Rx_data[1]);
//    dbusdata->tar_tran_encode=(int16_t)(dbusdata->tar_encode>32767?dbusdata->tar_encode-65536:dbusdata->tar_encode);
//    if(dbusdata->tar_tran_encode>23000){dbusdata->tar_tran_encode=23000;}
//    if(dbusdata->tar_tran_encode<-16535){dbusdata->tar_tran_encode=-16535;}

    dbusdata->encode = (uint16_t)(Rx_data[2] << 8 | Rx_data[3]);
    dbusdata->tran_encode=(int16_t)(dbusdata->encode>32767?dbusdata->encode-65536:dbusdata->encode);

    dbusdata->error_encode=dbusdata->tar_tran_encode-dbusdata->tran_encode;
    dbusdata->del_angle=(int)((dbusdata->tran_encode-dbusdata->pre_tran_encode)/65536.0f*6.28f);
}

