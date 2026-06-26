#ifndef __POWER_CTRL_H__
#define __POWER_CTRL_H__

#include "power_meter.h"
#include "M3508.h"
#include "main.h"

typedef struct{
    float k0,k1,k2,k3,k4,k5;//功率建模系数
    float* power;//真实功率
    int16_t* speed_now;//当前转速
    int16_t* current;//电流
    float power_cal;//计算得到的功率
}power_model_t;

typedef struct {
    float power_limitied; //限制的功率
    powMeter_capacitorBank_t *pwr_info; //功率信息
    float p_kp;
    float p_ki;
    float p_kd;
    float power_error;
    float power_error_last; //上一次的误差
    float power_error_prev; //上上一次的误差
    float power_limited_output; //限幅输出
}power_ctrl;

extern power_ctrl power_ctrler;

void power_ctrl_init(power_ctrl *pwr_ctrl, float power_limitied, powMeter_capacitorBank_t *pwr_info, float p_kp, float p_ki, float p_kd);
float power_ctrl_calc(power_ctrl *pwr_ctrl);
float power_output_clamp(power_ctrl *pwr_ctrl, float power_output,float min,float max);

void power_model_init(power_model_t* pm_info,powMeter_capacitorBank_t* pwr_info,motor_para* motor_info);
void set_power_model_para(power_model_t* pm_info,float k_0,float k_1,float k_2,float k_3,float k_4,float k_5);
void calc_power_model(power_model_t* pm_info);
#endif /* __POWER_CTRL_H__ */