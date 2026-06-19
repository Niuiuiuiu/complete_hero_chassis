#include "power_ctrl.h"


power_ctrl power_ctrler;

/**
 * @brief 初始化电源控制结构体
 * @param pwr_ctrl 电源控制结构体指针
 * @param power_limitied 电源限制值
 * @param pwr_info 电源信息结构体指针
 * @param p_kp 比例系数
 * @param p_ki 积分系数
 * @param p_kd 微分系数
 * @retval 无
 */
void power_ctrl_init(power_ctrl *pwr_ctrl, float power_limitied, powMeter_capacitorBank_t *pwr_info, float p_kp, float p_ki, float p_kd){
    pwr_ctrl->power_limitied = power_limitied;
    pwr_ctrl->pwr_info = pwr_info;
    pwr_ctrl->p_kp = p_kp;
    pwr_ctrl->p_ki = p_ki;
    pwr_ctrl->p_kd = p_kd;
    pwr_ctrl->power_error_last = 0;
    pwr_ctrl->power_error_prev = 0;
    pwr_ctrl->power_error = 0;
    pwr_ctrl->power_limited_output = 1.0;
}


/**
 * @brief 计算电源控制输出
 * @param pwr_ctrl 电源控制结构体指针
 * @retval 电源控制输出
 */
float power_ctrl_calc(power_ctrl *pwr_ctrl){
    float output,p_term, i_term, d_term;
    if(pwr_ctrl->pwr_info->ifPowerMeter_Ready == 0) return 0;
    pwr_ctrl->power_error_prev =pwr_ctrl->power_error_last;
    pwr_ctrl->power_error_last = pwr_ctrl->power_error;
    pwr_ctrl->power_error = pwr_ctrl->power_limitied - pwr_ctrl->pwr_info->P_x1W_chassis;
    p_term = pwr_ctrl->p_kp *( pwr_ctrl->power_error-pwr_ctrl->power_error_last);
    i_term = pwr_ctrl->p_ki * pwr_ctrl->power_error;
    d_term = pwr_ctrl->p_kd * (pwr_ctrl->power_error - 2*pwr_ctrl->power_error_last + pwr_ctrl->power_error_prev);
    output = p_term + i_term + d_term;
    return output;
}


/**
 * @brief 控制系数输出限幅
 * @param pwr_ctrl 电源控制结构体指针
 * @param power_output 增量式pid计算的结果
 * @param min 最小值
 * @param max 最大值
 * @retval 限幅后的控制系数输出
 */
float power_output_clamp(power_ctrl *pwr_ctrl, float power_output,float min,float max){
    pwr_ctrl->power_limited_output+= power_output;
    return pwr_ctrl->power_limited_output>max?max:pwr_ctrl->power_limited_output<min?min:pwr_ctrl->power_limited_output;
}