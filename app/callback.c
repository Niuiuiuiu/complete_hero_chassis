#include "control_dbus.h"
#include "dbus.h"
#include "M3508.h"
#include "drv_can.h"
#include "dm_motor.h"
#include "imu_data.h"
#include "bsp_imu.h"
#include "dm_joint_ctrl.h"
#include "power_meter.h"
#include "power_ctrl.h"
#include "can.h"
#include "main.h"

extern BMI088_Handle_t imu1;
extern joint_ctrl_para imu_ctrler;
extern motor_para M35085,M35086,M35087,M35088;
extern dm_motor_para DM43401,DM43402,DM43403,DM43404;
extern dbus_struct dbus_ctrl_data;
extern trans_dbus_data dbuscontrol;
extern condition_state condition_ctrler;
extern powMeter_capacitorBank_t powMeter_capBank_info;
extern power_ctrl power_ctrler;
extern power_model_t power_model;
extern motion_state motion_ctrler;
extern float USART_I[8];



/**
 * @brief 外部中断函数
 * @param GPIO引脚
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(! imu1.imu_ready) return;
  if (GPIO_Pin == GPIO_PIN_4) {

    BMI088_Read_Acc_Raw(&imu1);
  }
  else if (GPIO_Pin == GPIO_PIN_5) {

    BMI088_Read_Gyro_Raw(&imu1);
  }
}


/**
 * @brief 定时器中断回调函数
 * @param 定时器句柄
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    
		if (htim->Instance == TIM13) // 判断是否为 TIM13 的中断
    {
			static uint8_t count13=0;
			if(count13>=100){count13=0;}
			count13++;
			if(count13%4==1){
				revive_motor(&hcan2,&DM43401);
				revive_motor(&hcan2,&DM43403); 
			}
			else if(count13%4==3){
				revive_motor(&hcan2,&DM43404);
				revive_motor(&hcan2,&DM43402);  
			}
			else{                                       //                                            
				M3508_currentsend(&hcan2,M3508_send_ID_5_8,power_output_clamp(&power_ctrler,power_ctrl_calc(&power_ctrler),0.0,1.0),calculate_PID,&M35085,&M35086,&M35087,&M35088);
			}
    }
		
		if (htim->Instance == TIM12)
		{
			static uint8_t count=0;
			if(count>100){count=0;}
			if(count%2==1)
			{

				if(DM43401.state==lift_f_leg)
                {
                    DM43401.P_des=1.8f;
                    DM43403.P_des=-1.5f;
                    DM43401.T_ff=MIT_calculate_T_ff(&DM43401,0.0f);          
                    DM43403.T_ff=MIT_calculate_T_ff(&DM43403,0.0f);
                }
                else if(DM43401.state==landing)
                {
                    const_land_leg(&DM43401);
					DM43403.P_des=-0.4f;
                    DM43403.T_ff=MIT_calculate_T_ff(&DM43403,0.0f);
                }
                else if(DM43401.state==lift_b_leg)
                {
                    DM43403.P_des=-1.6f;
                    DM43401.T_ff=force_to_torque(&DM43401,height_ctrl(&imu_ctrler)-pitch_ctrl(&imu_ctrler)-roll_ctrl(&imu_ctrler) );          
                    DM43403.T_ff=MIT_calculate_T_ff(&DM43403,0.0f);
                }
                else{                              //height_ctrl(&imu_ctrler)-pitch_ctrl(&imu_ctrler)-roll_ctrl(&imu_ctrler)
                    DM43401.T_ff=force_to_torque(&DM43401,height_ctrl(&imu_ctrler)-pitch_ctrl(&imu_ctrler)-roll_ctrl(&imu_ctrler));  //1  k_rol
				    DM43403.T_ff=force_to_torque(&DM43403,height_ctrl(&imu_ctrler)+pitch_ctrl(&imu_ctrler)-roll_ctrl(&imu_ctrler));  //-1  k_rol
                }                                 //height_ctrl(&imu_ctrler)+pitch_ctrl(&imu_ctrler)-roll_ctrl(&imu_ctrler)
				MIT_senddata(&hcan2,&DM43401);   
				MIT_senddata(&hcan2,&DM43403);				
				
			}
			else
			{	

				if(DM43404.state==lift_f_leg)
                {
                    DM43402.P_des=1.5f;
                    DM43404.P_des=-1.8f;
                    DM43402.T_ff=MIT_calculate_T_ff(&DM43402,0.0f);          
                    DM43404.T_ff=MIT_calculate_T_ff(&DM43404,0.0f);
                }
				else if(DM43404.state==landing){
					DM43402.P_des=0.4f;
                    DM43402.T_ff=MIT_calculate_T_ff(&DM43402,0.0f);
                    const_land_leg(&DM43404);
                }
                else if(DM43404.state==lift_b_leg)
                {
                    DM43402.P_des=1.6f;
                    DM43404.T_ff=force_to_torque(&DM43404,height_ctrl(&imu_ctrler)-pitch_ctrl(&imu_ctrler)+roll_ctrl(&imu_ctrler) );         
                    DM43402.T_ff=MIT_calculate_T_ff(&DM43402,0.0f);
                }
				else{                                  //height_ctrl(&imu_ctrler)+pitch_ctrl(&imu_ctrler)+roll_ctrl(&imu_ctrler) 
                    DM43402.T_ff=force_to_torque(&DM43402,height_ctrl(&imu_ctrler)+pitch_ctrl(&imu_ctrler)+roll_ctrl(&imu_ctrler) ); //-1 -1 1
				    DM43404.T_ff=force_to_torque(&DM43404,height_ctrl(&imu_ctrler)-pitch_ctrl(&imu_ctrler)+roll_ctrl(&imu_ctrler) );   //-1 1 -1
                }                                     //height_ctrl(&imu_ctrler)-pitch_ctrl(&imu_ctrler)+roll_ctrl(&imu_ctrler)
				MIT_senddata(&hcan2,&DM43402);       
				MIT_senddata(&hcan2,&DM43404);
				
			}
			
			count++;
		}
		if(htim->Instance == TIM14) {
            static uint8_t count14=0;
            if(count14>100){count14=0;}
			__disable_irq();
			BMI088_Data_Convert(&imu1);
			__enable_irq();

			Mahony_Update(&imu1, 0.001f);

			Quaternion_To_Euler(&imu1);
	  
			BMI088_Read_Temp(&imu1);

            
            if(count14%5==0){
              motion_state_ctrl(&imu_ctrler,&motion_ctrler,&M35085,&M35086,&M35087,&M35088);
			  process_ctrl_para(&imu_ctrler,&imu1,&DM43401,&DM43402,&DM43403,&DM43404); //不是scp的库
             }
            
             
			USART_I[0] = imu1.temperature;
			USART_I[1] = imu1.euler.pitch;
			USART_I[2] = imu1.euler.yaw;
			USART_I[3] = imu1.euler.roll;
            count14++;
        }
}


/**
 * @brief CAN接收中断回调函数
 *
 * @param hcan CAN编号
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
//		__disable_irq();
		CAN_RxHeaderTypeDef Rxheader;
		uint8_t Rx_data[8];
		HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO1,&Rxheader,Rx_data);//can1can2要注意改
		switch(Rxheader.StdId)
	{
		case 0x205:
			process_para(&M35085,Rx_data);
			break;
		case 0x206:
			process_para(&M35086,Rx_data);
			break;
		case 0x207:
			process_para(&M35087,Rx_data);
			break;
		case 0x208:
			process_para(&M35088,Rx_data);
			break;
    case 0x141:
      Rx_process(&DM43401,Rx_data);
      break;
    case 0x142:
      Rx_process(&DM43402,Rx_data);
      break;
    case 0x143:
      Rx_process(&DM43403,Rx_data);
      break;
    case 0x144:
      Rx_process(&DM43404,Rx_data);
      break;
    case POWERMETER_CAN_RX_ID:
      powMeter_data_handle(Rxheader.StdId, Rx_data);
      break;
//    case 0x05:
//      process_dbus_data(&dbuscontrol,Rx_data);
//      break;
    default:
      break;
	}
//	__enable_irq();
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
		CAN_RxHeaderTypeDef Rxheader;
		uint8_t Rx_data[8];
		HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&Rxheader,Rx_data);//can1can2要注意改
		switch(Rxheader.StdId)
	{
//		case 0x205:
//			process_para(&M35085,Rx_data);
//			break;
//		case 0x206:
//			process_para(&M35086,Rx_data);
//			break;
//		case 0x207:
//			process_para(&M35087,Rx_data);
//			break;
//		case 0x208:
//			process_para(&M35088,Rx_data);
//			break;
//    case 0x141:
//      Rx_process(&DM43401,Rx_data);
//      break;
//    case 0x142:
//      Rx_process(&DM43402,Rx_data);
//      break;
//    case 0x143:
//      Rx_process(&DM43403,Rx_data);
//      break;
//    case 0x144:
//      Rx_process(&DM43404,Rx_data);
//      break;
    case 0x05:
      process_raw_dbus_data(&dbuscontrol,Rx_data,&DM43401,&DM43404,&condition_ctrler);
      break;
    case 0x06:
      process_encode_data(&dbuscontrol,Rx_data);
      break;
    default:
      break;
	}
}