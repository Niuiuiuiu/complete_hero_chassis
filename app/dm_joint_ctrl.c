#include "dm_joint_ctrl.h"
#include "dm_motor.h"
#include "main.h"

joint_ctrl_para imu_ctrler;


/**
 * @brief 处理控制参数
 * @param ctrl_para 控制参数结构体指针
 * @param imu IMU数据结构体指针
 * @param motor 达妙电机1，2，3，4的参数结构体指针
 * @return 无
 */
void process_ctrl_para(joint_ctrl_para* ctrl_para,BMI088_Handle_t* imu,dm_motor_para* motor1,dm_motor_para* motor2,dm_motor_para* motor3,dm_motor_para* motor4)
{
   ctrl_para->last_pitch_error=ctrl_para->pitch_error;
   ctrl_para->last_roll_error=ctrl_para->roll_error;
   ctrl_para->last_h_error=ctrl_para->h_error;

   ctrl_para->pitch_error=ctrl_para->tar_pitch-imu->euler.pitch;
   ctrl_para->pitch_sig_error=ctrl_para->pitch_sig_error+ctrl_para->pitch_error;
   ctrl_para->pitch_delta_error=ctrl_para->pitch_error-ctrl_para->last_pitch_error;

   ctrl_para->roll_error=ctrl_para->tar_roll-imu->euler.roll;
   ctrl_para->roll_sig_error=ctrl_para->roll_sig_error+ctrl_para->roll_error;
   ctrl_para->roll_delta_error=ctrl_para->roll_error-ctrl_para->last_roll_error;

   ctrl_para->h_now=LENGTH_ARM *(cos(motor1->POS)+cos(motor2->POS)+cos(motor3->POS)+cos(motor4->POS))/4.0f;
   ctrl_para->h_error=ctrl_para->tar_h-ctrl_para->h_now;
   ctrl_para->h_delta_error=ctrl_para->h_error-ctrl_para->last_h_error;
}


/**
 * @brief 计算俯仰角控制输出
 * @param ctrl_para 控制参数结构体指针
 * @return 俯仰角控制输出
 */
float pitch_ctrl(joint_ctrl_para* ctrl_para)
{
    float pitch_output;
    pitch_output = ctrl_para->kp_pitch * ctrl_para->pitch_error +ctrl_para->ki_pitch*ctrl_para->pitch_sig_error*0.005f+ctrl_para->kd_pitch * ctrl_para->pitch_delta_error/0.005f;
    return pitch_output;
}


/**
 * @brief 计算横滚角控制输出
 * @param ctrl_para 控制参数结构体指针
 * @return 横滚角控制输出
 */
float roll_ctrl(joint_ctrl_para* ctrl_para)
{
    float roll_output;
    roll_output = ctrl_para->kp_roll * ctrl_para->roll_error +ctrl_para->ki_roll*ctrl_para->roll_sig_error*0.005f+ ctrl_para->kd_roll * ctrl_para->roll_delta_error/0.005f;
    return roll_output;
}


/**
 * @brief 计算高度控制输出
 * @param ctrl_para 控制参数结构体指针
 * @return 高度控制输出
 */
float height_ctrl(joint_ctrl_para* ctrl_para)
{
    float h_output;
    h_output = ctrl_para->kp_h * ctrl_para->h_error + ctrl_para->kd_h * ctrl_para->h_delta_error/0.005f;
    return h_output;
}


/**
 * @brief 将力转换为扭矩
 * @param motor 电机参数结构体指针
 * @param force 作用在机械臂末端的力
 * @return 转换后的扭矩
 */
float force_to_torque(dm_motor_para* motor, float force)
{
    float output,sin_clamp,limi_factor;
    if(fabs(motor->POS)>1.19&&force<0){          //防抬腿输出限幅
        limi_factor=(1.29f-fabs(motor->POS))/0.1f;
        if(limi_factor<0.0f){limi_factor=0.0f;}
        force=force*limi_factor;
    }
    sin_clamp=fabs(sin(motor->POS))<0.38f?(sin(motor->POS)>0?0.38f:-0.38f):sin(motor->POS);
    output=force*LENGTH_ARM/sin_clamp*-1.0f;
	if(output>motor->TMAX){output=motor->TMAX;}
	if(output<-motor->TMAX){output=-motor->TMAX;}
    return output;
}


/**
 * @brief 匀速落腿
 * @param hcan CAN编号
 * @param motor 电机参数结构体指针
 * @param state 机器人状态枚举指针
 * @return 无
 */
void const_land_leg(dm_motor_para* motor)
{
    float tar_pos=motor->POS>0?1.0f:-1.0f;
    motor->kdv_own=0.4f;
    if(motor->land_count==1) motor->start_pos=motor->POS;
    motor->P_des=motor->start_pos-(motor->start_pos-tar_pos)/200.0f*motor->land_count;
    motor->V_des=0;    //设置速度目标
    motor->T_ff=MIT_calculate_T_ff(motor,0.0f);  //使用MIT发送
    motor->kdv_own=4.0f;
    motor->land_count++;
    if(motor->land_count>200){
        motor->land_count=1;
        motor->state=normal;
    }
}