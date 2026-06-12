#include "drv_can.h"
#include "M3508.h"
#include "dm_motor.h"
#include "control_dbus.h"
#include "main.h"
#include "power_meter.h"


//extern CAN_HandleTypeDef hcan1;

extern motor_para M35085,M35086,M35087,M35088;
extern dm_motor_para DM43401,DM43402,DM43403,DM43404;
extern trans_dbus_data dbuscontrol;
extern robo_state state_ctrler;
extern condition_state condition_ctrler;


/**
 * @brief 发送报文
 *
 * @param hcan CAN编号
 * @param 报文ID
 * @param 发送的数据
 * @param 数据位数
 */
void CAN_Send_Data(CAN_HandleTypeDef *hcan,uint16_t ID,const uint8_t aData[],uint16_t length)
{
	
	CAN_TxHeaderTypeDef tx_header;
	tx_header.StdId=ID;
	tx_header.ExtId=0;
	tx_header.IDE=0;
	tx_header.RTR=0;
	tx_header.DLC=length;
	uint32_t timeout = 500;  //还剩几个邮箱可以用
//  while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0 && --timeout);
//	if(timeout)
//	{
		uint32_t mailbox;
		HAL_CAN_AddTxMessage(hcan,&tx_header,aData,&mailbox);
//	}

}


/**
 * @brief 配置CAN的滤波器
 *
 * @param hcan CAN编号
 * @param Object_Para 编号 | FIFOx | ID类型 | 帧类型
 * @param ID ID
 * @param Mask_ID 屏蔽位(0x3ff, 0x1fffffff)
 */
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID)
{
  CAN_FilterTypeDef can_filter_init_structure;

  // 检测关键传参
  assert_param(hcan != NULL);

  if ((Object_Para & 0x02))
  {
    // 标准帧
    // 掩码后ID的高16bit
    can_filter_init_structure.FilterIdHigh = ID << 3 >> 16;
    // 掩码后ID的低16bit
    can_filter_init_structure.FilterIdLow = ID << 3 | ((Object_Para & 0x03) << 1);
    // ID掩码值高16bit
    //can_filter_init_structure.FilterMaskIdHigh = Mask_ID << 3 << 16;
		can_filter_init_structure.FilterMaskIdHigh = Mask_ID << 3 >> 16;
    // ID掩码值低16bit
    can_filter_init_structure.FilterMaskIdLow = Mask_ID << 3 | ((Object_Para & 0x03) << 1);
  }
  else
  {
    // 扩展帧
    // 掩码后ID的高16bit
    can_filter_init_structure.FilterIdHigh = ID << 5;
    // 掩码后ID的低16bit
    can_filter_init_structure.FilterIdLow = ((Object_Para & 0x03) << 1);
    // ID掩码值高16bit
    can_filter_init_structure.FilterMaskIdHigh = Mask_ID << 5;
    // ID掩码值低16bit
    can_filter_init_structure.FilterMaskIdLow = ((Object_Para & 0x03) << 1);
  }

  // 滤波器序号, 0-27, 共28个滤波器, can1是0~13, can2是14~27
  can_filter_init_structure.FilterBank = Object_Para >> 3;
  // 滤波器绑定FIFOx, 只能绑定一个
  can_filter_init_structure.FilterFIFOAssignment = (Object_Para >> 2) & 0x01;
  // 使能滤波器
  can_filter_init_structure.FilterActivation = ENABLE;
  // 滤波器模式, 设置ID掩码模式
  can_filter_init_structure.FilterMode = CAN_FILTERMODE_IDMASK;
  // 32位滤波
  can_filter_init_structure.FilterScale = CAN_FILTERSCALE_32BIT;
    //从机模式选择开始单元
  can_filter_init_structure.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(hcan, &can_filter_init_structure);
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