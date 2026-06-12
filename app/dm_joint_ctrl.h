#ifndef DM_JOINT_CTRL_H
#define DM_JOINT_CTRL_H

#include "dm_motor.h"
#include "control_dbus.h"
#include "imu_data.h"
#include "main.h"



      

#define LENGTH_ARM 1.4f





typedef struct
{
    float kp_pitch;
	  float ki_pitch;
    float kd_pitch;
    float tar_pitch;
    float pitch_error;
		float pitch_sig_error;
    float last_pitch_error;
    float pitch_delta_error;
    
    float kp_roll;
	  float ki_roll;
    float kd_roll;
    float tar_roll;
    float roll_error;
		float roll_sig_error;
    float last_roll_error;
    float roll_delta_error;
    
    float kp_h;
    float kd_h;

    float tar_h;
    float h_now;
    float h_error;
    float last_h_error;
    float h_delta_error;
    

}joint_ctrl_para;

typedef float (*attitude_ctrl_func)(joint_ctrl_para* ctrl_para);
void process_ctrl_para(joint_ctrl_para* ctrl_para,BMI088_Handle_t* imu,dm_motor_para* motor1,dm_motor_para* motor2,dm_motor_para* motor3,dm_motor_para* motor4);
float pitch_ctrl(joint_ctrl_para* ctrl_para);
float roll_ctrl(joint_ctrl_para* ctrl_para);
float force_to_torque(dm_motor_para* motor, float force);
float height_ctrl(joint_ctrl_para* ctrl_para);
void const_land_leg(dm_motor_para* motor);

#endif // DM_JOINT_CTRL_H
