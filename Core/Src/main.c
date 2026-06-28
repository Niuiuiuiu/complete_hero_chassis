/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern DMA_HandleTypeDef hdma_usart3_rx;
BMI088_Handle_t imu1;
extern joint_ctrl_para imu_ctrler;
extern motor_para M35085,M35086,M35087,M35088;
extern dm_motor_para DM43401,DM43402,DM43403,DM43404;
extern dbus_struct dbus_ctrl_data;
extern trans_dbus_data dbuscontrol;
extern condition_state condition_ctrler;
extern powMeter_capacitorBank_t powMeter_capBank_info;
extern power_ctrl power_ctrler;
extern power_model_t power_model;
float USART_I[8]={0};
float power_send[3] = {0};
uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_TIM10_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  BMI088_Full_Init(&imu1);
	
	dm_motor_init(&DM43401,12.5,10,18,50.0f,4.0f,0x01,0x141);    //达妙的MIT暂时给60 
  dm_motor_init(&DM43402,12.5,10,18,35.0f,4.0f,0x02,0x142);    //自己的MIT暂时给40 偏软
  dm_motor_init(&DM43403,12.5,10,18,35.0f,4.0f,0x03,0x143);
  dm_motor_init(&DM43404,12.5,10,18,50.0f,4.0f,0x04,0x144);
  DM43401.P_des=2.0f;
  DM43402.P_des=1.0f;
  DM43403.P_des=-1.0f;
  DM43404.P_des=-2.0f;

  motor_enable(&hcan2,&DM43401);
  motor_enable(&hcan2,&DM43402);
  motor_enable(&hcan2,&DM43403);
  motor_enable(&hcan2,&DM43404);
	
	imu_ctrler.tar_h = 0.76f; //dm
	imu_ctrler.tar_roll = 0.0f;
  imu_ctrler.tar_pitch = 0.0f;

	imu_ctrler.kp_h = 40.0f;
  imu_ctrler.kd_h = 1.0f;
  imu_ctrler.kp_pitch = 0.4f;   //0.3
	imu_ctrler.ki_pitch = 0.0f;   //0.22f
  imu_ctrler.kd_pitch = 0.05f;   // 0.1f
  imu_ctrler.kp_roll =0.4f;   //0.3
	imu_ctrler.ki_roll = 0.0f;   //0.22f
  imu_ctrler.kd_roll = 0.02f;  // 0.09f

  M3508_init(2.7f,0.1f,0.02f,&M35085,&M35086,&M35087,&M35088);
  power_ctrl_init(&power_ctrler,60,&powMeter_capBank_info,0.25f,0.11f,0.0f);
  power_model_init(&power_model,&powMeter_capBank_info,&M35085);
  dbusctrl_init(&dbuscontrol,0.2f,0.4f);
 
  CAN_Filter_Mask_Config(&hcan1,CAN_FILTER(13)|CAN_FIFO_0|CAN_STDID|CAN_DATA_TYPE,0x05,0x00);//需要根据电机id修改
	CAN_Filter_Mask_Config(&hcan2,CAN_FILTER(14)|CAN_FIFO_1|CAN_STDID|CAN_DATA_TYPE,0x200,0x700);//需要根据电机id修改
  CAN_Filter_Mask_Config(&hcan2,CAN_FILTER(15)|CAN_FIFO_1|CAN_STDID|CAN_DATA_TYPE,0x140,0x700);//需要根据电机id修改
	
  HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO1_MSG_PENDING);
	
  HAL_CAN_Start(&hcan1);
  HAL_CAN_Start(&hcan2);
	//dbus_init(&huart3, &hdma_usart3_rx);
	
	__HAL_TIM_SET_COUNTER(&htim13, 1250);

	
	HAL_TIM_Base_Start_IT(&htim13);
	HAL_TIM_Base_Start_IT(&htim12);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    trans_dbus(dbuscontrol.raw_ch[3],dbuscontrol.raw_ch[2],dbuscontrol.raw_ch[0],&dbuscontrol);
    if(condition_ctrler==follow_gimbal){
        dbus_Mec_process(&dbuscontrol,&M35085,&M35086,&M35087,&M35088,follow_gimbal_ctrl(&dbuscontrol,&imu1));
    }
    else{
        dbus_Mec_process(&dbuscontrol,&M35085,&M35086,&M35087,&M35088,0);
    }
    
		power_send[0]=powMeter_capBank_info.P_x1W_chassis;
		HAL_UART_Transmit(&huart6, (uint8_t*)power_send, sizeof(power_send), HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart6, tail, 4, HAL_MAX_DELAY);
		HAL_Delay(25);
		Temp_Control_Task(&imu1); 
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
