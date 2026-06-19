#ifndef __M3508_H__
#define __M3508_H__

#include <stdint.h>
#include "main.h"

#define M3508_send_ID_1_4 0x200
#define M3508_send_ID_5_8 0x1FF

typedef struct 
{
    float kpv;
    float kiv;
    float kdv;
    int16_t speed_now;
    int16_t speed_pre;
    int16_t v_error;
    int64_t v_error_sig;
    int16_t v_error_del;
    int16_t v_error_pre;
    int16_t tar_speed;

    int16_t current;
    uint8_t temp;

    float kpa;
    float kia;
    float kda;
    uint16_t encode;
    uint16_t encode_pre;
    int16_t a_error;
    int64_t a_error_sig;
    int16_t a_error_del;
    int16_t a_error_pre;
    uint16_t tar_encode;
	uint16_t tar_encode_pre;


}motor_para;

typedef int16_t (*calculate)(motor_para* pid);

void M3508_init(float kpv,float kiv,float kdv,motor_para* pid1,motor_para* pid2,motor_para* pid3,motor_para* pid4);
int16_t pid_calculate_v(motor_para* pid);
int16_t pid_calculate_a(motor_para* pid);
int16_t calculate_PID(motor_para* pid);
void process_para(motor_para* pid,uint8_t* Rx_dataa);
void M3508_currentsend(CAN_HandleTypeDef*hcan,uint16_t sendID,calculate calfunction,motor_para* pid1,motor_para* pid2,motor_para* pid3,motor_para* pid4);







#endif
