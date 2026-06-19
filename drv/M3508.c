#include "M3508.h"
#include "drv_can.h"


extern CAN_HandleTypeDef hcan2;

motor_para M35085,M35086,M35087,M35088;

/**
 * @brief M3508电机参数初始化
 * @param PID参数(kpv，kiv，kdv)
 * @param 电机参数结构体指针
 * @return 无
 */
void M3508_init(float kpv,float kiv,float kdv,motor_para* pid1,motor_para* pid2,motor_para* pid3,motor_para* pid4)
{
    pid1->kpv=kpv;
    pid1->kiv=kiv;
    pid1->kdv=kdv;

    pid2->kpv=kpv;
    pid2->kiv=kiv;
    pid2->kdv=kdv;

    pid3->kpv=kpv;
    pid3->kiv=kiv;
    pid3->kdv=kdv;

    pid4->kpv=kpv;
    pid4->kiv=kiv;
    pid4->kdv=kdv;

    pid1->tar_speed=0;
    pid2->tar_speed=0;
    pid3->tar_speed=0;
    pid4->tar_speed=0;
}


/**
 * @brief 计算电流输出值（目标速度）
 *
 * @param 电机信息结构体
 * @return 计算结果
 */
int16_t pid_calculate_v(motor_para* pid)
{
    float pp,ii,dd;
    int16_t output;
		
    pp=pid->kpv*pid->v_error;
    ii=pid->kiv*pid->v_error_sig*0.001f;
    if(ii>5000){ii=5000;}//限幅注意修改
    if (ii<-5000) {ii=-5000;}
    dd=pid->kdv*pid->v_error_del/0.001f;

    output=(int16_t)pp+ii+dd;
	if(output>12000){output=12000;}
    if(output<-12000){output=-12000;}
    return output;
}


/**
 * @brief 计算速度输出值（目标角度）
 *
 * @param 电机信息结构体
 * @return 计算结果
 */
int16_t pid_calculate_a(motor_para* pid)
{
    float pp,ii,dd;
    int16_t output;

    pp=pid->kpa*pid->a_error;
    ii=pid->kia*pid->a_error_sig*0.001f;
    if(ii>5000){ii=5000;}//限幅注意修改
    if (ii<-5000) {ii=-5000;}
    dd=pid->kda*pid->a_error_del/0.001f;

    output=(int16_t)pp+ii+dd;
	if(output>400){output=400;}
	if(output<-400){output=-400;}
    return output;
}


/**
 * @brief 计算输出值(3508) adj_pid_para(5.0f,0.28f,0.02f);
 *
 * @param 电机信息结构体
 * @return 计算结果
 */
int16_t calculate_PID(motor_para* pid)
{
	int16_t output,dif_output;
	float kdd,kpp,kii;
//	if(pid->current<0){kpp=pid->kp*2.9;}
	kpp=pid->kpv;
//	if(pid->current<0){kii=pid->ki*2.2;}
	kii=pid->kiv;
	if(pid->v_error_del>0){kdd=0;}
	else{kdd=pid->kdv;}
	dif_output=((kdd)*(pid->v_error_del)/0.01f);
//	if(dif_output>150){dif_output=150;}
//	if(dif_output<-150){dif_output=-150;}
	if(pid->tar_speed<=0){output=(int16_t)((kpp)*(pid->v_error)+(pid->kiv)*(pid->v_error_sig)*(0.01f)+dif_output);} //一般PID
	else{output=(int16_t)((kpp)*(pid->v_error)+(kii)*(pid->v_error_sig)*(0.01f)+(kdd)*(((pid->speed_now)-(pid->speed_pre))/0.01f));} //微分先行？
	if(output>16384){output=16384;}
	if(output<-16384){output=-16384;}
	return output;
	
	
}


/**
 * @brief 处理参数
 *
 * @param 电机信息结构体
 * @return 无
 */
void process_para(motor_para* pid,uint8_t* Rx_dataa)
{
    pid->v_error_pre=pid->v_error;
	pid->speed_pre=pid->speed_now;
    pid->a_error_pre=pid->a_error;
	pid->tar_encode_pre=pid->tar_encode;

    pid->encode=Rx_dataa[0]<<8|Rx_dataa[1];
    pid->speed_now=Rx_dataa[2]<<8|Rx_dataa[3];
    pid->current=Rx_dataa[4]<<8|Rx_dataa[5];
    pid->temp=Rx_dataa[6];

    pid->v_error=pid->tar_speed-pid->speed_now;
    pid->v_error_sig=pid->v_error_sig+pid->v_error;
    pid->v_error_del=pid->v_error-pid->v_error_pre;

    pid->a_error=pid->tar_encode-pid->encode;
	if(pid->a_error>4096){pid->a_error=pid->tar_encode-pid->encode-8192;}
    if(pid->a_error<-4096){pid->a_error=pid->tar_encode-pid->encode+8192;}
    pid->a_error_sig=pid->a_error_sig+pid->a_error;
    pid->a_error_del=pid->a_error-pid->a_error_pre;

}


/**
 * @brief 发送电流值
 * @param CAN句柄
 * @param 发送ID
 * @param 计算函数指针
 * @param 电机参数结构体指针
 * @return 无
 */
void M3508_currentsend(CAN_HandleTypeDef*hcan,uint16_t sendID,calculate calfunction,motor_para* pid1,motor_para* pid2,motor_para* pid3,motor_para* pid4)
{
    int16_t current[4];
    uint8_t Tx_data[8];

    current[0]=calfunction(pid1);
    current[1]=calfunction(pid2);
    current[2]=calfunction(pid3);
    current[3]=calfunction(pid4);

    Tx_data[0]=current[0]>>8;
    Tx_data[1]=current[0]&0x00ff;
    Tx_data[2]=current[1]>>8;
    Tx_data[3]=current[1]&0x00ff;
    Tx_data[4]=current[2]>>8;
    Tx_data[5]=current[2]&0x00ff;
    Tx_data[6]=current[3]>>8;
    Tx_data[7]=current[3]&0x00ff;

    CAN_Send_Data(hcan,sendID,Tx_data,8);
}
