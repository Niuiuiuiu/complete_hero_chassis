#ifndef __POWER_CTRL_H__
#define __POWER_CTRL_H__

#include "power_meter.h"
#include "main.h"


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



#endif /* __POWER_CTRL_H__ */