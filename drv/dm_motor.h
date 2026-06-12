#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>


typedef enum{
  normal=3,
  lift_f_leg=1,
  lift_b_leg=2,
  landing=4,
}robo_state;



typedef struct
{
	uint8_t can_sID;
	uint8_t can_rID;
	
	uint8_t ERR;
	
	float VMAX;
	float V_des;
	float WEL;
	
	float PMAX;
	float P_des;
	float POS;
	float pre_POS;
	
	float Kpp;
	float Kdv;

	float kpp_own;
	float kdv_own;
	
	float T_ff;  //拿来发的
	float TMAX;
	float Tor;   //收到的
	
	uint8_t T_MOS;
	
	uint8_t T_Rotor;

	robo_state state;
	uint8_t land_count;
	float start_pos;
	
}dm_motor_para;


void dm_motor_init(dm_motor_para *motor,float pmaxx,float vmaxx,float tmaxx,float kpp,float kdd,uint16_t sID,uint16_t rID);
void motor_enable(CAN_HandleTypeDef* hcan,dm_motor_para *motor);
void motor_disable(CAN_HandleTypeDef* hcan,dm_motor_para *motor);
void clear_error(CAN_HandleTypeDef* hcan,dm_motor_para *motor);
void revive_motor(CAN_HandleTypeDef* hcan,dm_motor_para *motor);
void MIT_senddata(CAN_HandleTypeDef *hcan,dm_motor_para* motor);
void Rx_process(dm_motor_para* motor,uint8_t* Rx_data);
float MIT_calculate_T_ff(dm_motor_para* motor,float torr);

#endif // DM_MOTOR_H
