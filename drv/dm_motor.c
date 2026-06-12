#include "dm_motor.h"
#include "drv_can.h"
#include "main.h"


extern CAN_HandleTypeDef hcan2;
dm_motor_para DM43401,DM43402,DM43403,DM43404;



/** 
 * @brief 初始化电机参数
 * @param 电机结构体
 * @param 位置最大值
 * @param 速度最大值
 * @param 扭矩最大值
 * @param 位置比例增益
 * @param 速度微分增益
 * @param 发送ID
 * @param 接收ID
 * @return 无
 */
void dm_motor_init(dm_motor_para *motor,float pmaxx,float vmaxx,float tmaxx,float kpp,float kdd,uint16_t sID,uint16_t rID)
{
	motor->can_sID=sID;
	motor->can_rID=rID;
	
	motor->VMAX=vmaxx;
	motor->WEL=0.0f;
	
	motor->PMAX=pmaxx;
	motor->POS=0.0f;
	
	motor->Kpp=0.0f;
	motor->Kdv=0.0f;

	motor->kpp_own=kpp;
	motor->kdv_own=kdd;
	
	motor->TMAX=tmaxx;
	motor->Tor=0.0f;   //收到的
	
	motor->T_MOS=0;
	
	motor->T_Rotor=0;

	motor->land_count=1;

	motor->state=normal;

}


/**
 * @brief 使能电机
 * @param hcan CAN编号
 * @param 电机结构体
 * @return 无
 */
void motor_enable(CAN_HandleTypeDef* hcan,dm_motor_para *motor)
{
	uint8_t enabledata[8]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
	CAN_Send_Data(hcan,(uint16_t)motor->can_sID,enabledata,8);
}


/**
 * @brief 失能电机
 * @param hcan CAN编号
 * @param 电机结构体
 * @return 无
 */
void motor_disable(CAN_HandleTypeDef* hcan,dm_motor_para *motor)
{
	uint8_t disabledata[8]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
	CAN_Send_Data(hcan,(uint16_t)motor->can_sID,disabledata,8);
}


/**
 * @brief 清除电机错误
 * @param hcan CAN编号
 * @param 电机结构体
 * @return 无
 */
void clear_error(CAN_HandleTypeDef* hcan,dm_motor_para *motor)
{
	uint8_t clearerrordata[8]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFB};
	CAN_Send_Data(hcan,(uint16_t)motor->can_sID,clearerrordata,8);
}


/**
 * @brief 恢复电机
 * @param hcan CAN编号
 * @param 电机结构体
 * @return 无
 */
void revive_motor(CAN_HandleTypeDef* hcan,dm_motor_para *motor)
{
	if(motor->ERR==0 || motor->ERR==13)
  {
    clear_error(hcan,motor);
    motor_enable(hcan,motor);	
  }
}


/**
 * @brief MIT模式发送数据(给定位置，给定速度，速度范围，位置范围，位置kp，速度kd，扭矩t_ff,扭矩范围)
 *
 * @param 电机信息结构体
 * @return 无
 */
void MIT_senddata(CAN_HandleTypeDef *hcan,dm_motor_para* motor)
{
	uint8_t Tx_data[8];
	uint16_t tran_POS,tran_VEL,tran_Tor,tran_kp,tran_kd;

	tran_POS=(uint16_t)((motor->P_des+motor->PMAX)/(2.0f*motor->PMAX)*65535);
	tran_VEL=(uint16_t)((motor->V_des+motor->VMAX)/(2.0f*motor->VMAX)*4095);
	tran_Tor=(uint16_t)((motor->T_ff+motor->TMAX)/(2.0f*motor->TMAX)*4095);
	tran_kp=(uint16_t)(motor->Kpp/500*4095);
	tran_kd=(uint16_t)(motor->Kdv/5*4095);
	
	if(tran_POS>65535){tran_POS=65535;}
	if(tran_VEL>4095){tran_VEL=4095;}
	if(tran_kp>4095){tran_kp=4095;}
	if(tran_kd>4095){tran_kd=4095;}
	if(tran_Tor>4095){tran_Tor=4095;}
	
	Tx_data[0]=tran_POS>>8;
	Tx_data[1]=tran_POS;
	Tx_data[2]=tran_VEL>>4;
	Tx_data[3]=tran_VEL<<4|tran_kp>>8;
	Tx_data[4]=tran_kp;
	Tx_data[5]=tran_kd>>4;
	Tx_data[6]=tran_kd<<4|tran_Tor>>8;
	Tx_data[7]=tran_Tor;
	
	CAN_Send_Data(hcan,motor->can_sID,Tx_data,8);
	
}


/**
 * @brief 接收数据处理
 * @param 电机信息结构体
 * @param 接收数组
 * @return 无
 */
void Rx_process(dm_motor_para* motor,uint8_t* Rx_data)
{
	uint16_t Rx_POS,Rx_VEL,Rx_Tor;
	
	motor->ERR=Rx_data[0]>>4;
	Rx_POS=(uint16_t)(Rx_data[1]<<8|Rx_data[2]);
	Rx_VEL=(uint16_t)(Rx_data[3]<<4|Rx_data[4]>>4);
	Rx_Tor=(uint16_t)((Rx_data[4]&0x0F)<<8|Rx_data[5]);
	motor->T_MOS=Rx_data[6];
	motor->T_Rotor=Rx_data[7];
	
	motor->POS=(float)((2.0f*motor->PMAX/65535.0f)*Rx_POS-motor->PMAX);
	motor->WEL=(float)((2.0f*motor->VMAX/4095.0f)*Rx_VEL-motor->VMAX);
	motor->Tor=(float)((2.0f*motor->TMAX/4095.0f)*Rx_Tor-motor->TMAX);
}


/**
 * @brief 自己的MIT
 * @param 电机信息结构体
 * @param 发送扭矩项
 * @return 总扭矩
 */
float MIT_calculate_T_ff(dm_motor_para* motor,float torr)	
{
	float kp_term,kd_term,t_ff_term,output;
/*
	if(fabs(imu_ctrl->roll_error)>2.8f)
	{
		if(fabs(motor->POS)>=1.3f)
		{
			if(motor->P_des>0)motor->P_des=1.4f;
			else motor->P_des=-1.4f;
		
		}
		else
		{
			if(motor->P_des>0)motor->P_des=1.0f;
			else motor->P_des=-1.0f;
		}
	}
*/
	
	kp_term=motor->kpp_own*(motor->P_des-motor->POS);
	kd_term=motor->kdv_own*(motor->V_des-motor->WEL);


	
	t_ff_term=torr;


	output=kp_term+kd_term+t_ff_term;
	if(output>motor->TMAX){output=motor->TMAX;}
	if(output<-motor->TMAX){output=-motor->TMAX;}
	return output;
}
