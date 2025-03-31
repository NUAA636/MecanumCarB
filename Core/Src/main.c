/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
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
float temp2=0,temp4=0;
int delta2=0,delta4=0,cnt2=0,cnt4=0;
int speed2=0,angle2=0,target_angle2=0,real_target_speed2=0,target_speed2=0,pwm_out2=0,sum_angle2=0;
int speed4=0,angle4=0,target_angle4=0,real_target_speed4=0,target_speed4=0,pwm_out4=0,sum_angle4=0;
int PID_mode=0;
const fp32 PID_speed2[3]={0.12,0.01,0};
const fp32 PID_position2[3]={4.0,0.0026,0};
const fp32 PID_speed4[3]={0.12,0.01,0};
const fp32 PID_position4[3]={4.0,0.0028,0};
pid_type_def Position2,Speed2;
pid_type_def Position4,Speed4;
volatile uint8_t rx_flag3 = 0;
volatile uint8_t rx_data3;
static CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[1];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
{	
		
	if(htim->Instance == TIM4)
	{
		cnt2 = __HAL_TIM_GET_COUNTER(&htim2);
		cnt4 = __HAL_TIM_GET_COUNTER(&htim3);	
		
		if(cnt2>=30000)	{cnt2=cnt2-65535;}
		temp2 = ( cnt2 * 360 * 100 ) / (4 * 13 * 34 ) ;
		speed2 = (int)temp2;
		sum_angle2 = sum_angle2 + (int)temp2*0.01;
				
		if(cnt4>=30000)	{cnt4=cnt4-65535;}
		temp4 = ( cnt4 * 360 * 100 ) / (4 * 13 * 34 ) ;
		speed4 = (int)temp4;
		sum_angle4 = sum_angle4 + (int)temp4*0.01;
			
		if(delta2<=30 && delta2>=-30 && PID_mode==1)
		{
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
		}else
		{	
			target_speed2=PID_calc(&Position2,0,delta2);
			if(PID_mode==0)	{target_speed2=real_target_speed2;}
			pwm_out2=PID_calc(&Speed2,speed2,target_speed2);
			if(pwm_out2>0)
			{
				HAL_GPIO_WritePin(GPIOB, BIN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, BIN2_Pin, GPIO_PIN_RESET);
			}else
			{
				HAL_GPIO_WritePin(GPIOB, BIN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, BIN2_Pin, GPIO_PIN_SET);
				pwm_out2=(-1)*pwm_out2;
			}
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,pwm_out2);
		}
		
		if(delta4<=30 && delta4>=-30 && PID_mode==1)
		{
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
		}else
		{
			target_speed4=PID_calc(&Position4,0,delta4);
			if(PID_mode==0)	{target_speed4=real_target_speed4;}
			pwm_out4=PID_calc(&Speed4,speed4,target_speed4);
			if(pwm_out4>0)
			{
				HAL_GPIO_WritePin(GPIOA, DIN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, DIN2_Pin, GPIO_PIN_RESET);
			}else
			{
				HAL_GPIO_WritePin(GPIOA, DIN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, DIN2_Pin, GPIO_PIN_SET);
				pwm_out4=(-1)*pwm_out4;
			}
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,pwm_out4);
		}
				
		__HAL_TIM_SetCounter(&htim2,0);
		__HAL_TIM_SetCounter(&htim3,0);
	}
	
//	if(speed2!=0)
//	{
//		char buffer[64];
//		sprintf(buffer, "%d,%d,%d\n",target_speed2,speed2,-pwm_out2);
//		HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);	
//	}串口调试
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART3)
	{
		rx_flag3=1;
		HAL_UART_Receive_IT(&huart3, (uint8_t *)&rx_data3, 1);
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
	{
		if (RxHeader.StdId == 0x123 && RxData[0] == 0x00)
		{
			PID_mode=1;
			target_angle2=0;target_angle4=0;
			target_speed2=0;target_speed4=0;
			sum_angle2=0;sum_angle4=0;
			delta2=0;delta4=0;
		}
	} 
}
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim4);
	__HAL_TIM_SetCounter(&htim2,0);
	__HAL_TIM_SetCounter(&htim3,0);
	PID_init(&Position2,PID_POSITION,PID_position2,500.0f,1000.0f);
  PID_init(&Speed2,PID_POSITION,PID_speed2,30.0f,1000.0f);
	PID_init(&Position4,PID_POSITION,PID_position4,500.0f,1000.0f);
  PID_init(&Speed4,PID_POSITION,PID_speed4,30.0f,1000.0f);
	HAL_UART_Receive_IT(&huart3, (uint8_t *)&rx_data3, 1);
	
	CAN_FilterTypeDef filter;
  filter.FilterIdHigh = 0x123 << 5;  // 标准ID左移5位对齐
  filter.FilterIdLow = 0;
  filter.FilterMaskIdHigh = 0x7FF << 5;  // 掩码精确匹配
  filter.FilterMaskIdLow = 0;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(&hcan, &filter);
	HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(rx_flag3)
		{
			rx_flag3=0;
			
			if(rx_data3 == 0x00)	{PID_mode=0;target_angle2=0;target_angle4=0;real_target_speed2=0;real_target_speed4=0;sum_angle2=0;sum_angle4=0;}//停止
			
			else if(rx_data3 == 0x01)	{PID_mode=0;real_target_speed2=500;real_target_speed4=-500;target_angle2=0;target_angle4=0;sum_angle2=0;sum_angle4=0;}//定速前进
			else if(rx_data3 == 0xF1)	{PID_mode=1;target_angle2=1440;target_angle4=-1440;sum_angle2=0;sum_angle4=0;}//定位移前进
			
			else if(rx_data3 == 0x02)	{PID_mode=0;real_target_speed2=-500;real_target_speed4=500;target_angle2=0;target_angle4=0;sum_angle2=0;sum_angle4=0;}//定速后退
			else if(rx_data3 == 0xF2)	{PID_mode=1;target_angle2=-1440;target_angle4=1440;sum_angle2=0;sum_angle4=0;}//定位移后退
			
			else if(rx_data3 == 0x03)	{PID_mode=0;real_target_speed2=-500;real_target_speed4=500;target_angle2=0;target_angle4=0;sum_angle2=0;sum_angle4=0;}//定速左移
			else if(rx_data3 == 0xF3)	{PID_mode=1;target_angle2=-1440;target_angle4=1440;sum_angle2=0;sum_angle4=0;}//定位移左移
			
			else if(rx_data3 == 0x04)	{PID_mode=0;real_target_speed2=500;real_target_speed4=-500;target_angle2=0;target_angle4=0;sum_angle2=0;sum_angle4=0;}//定速右移
			else if(rx_data3 == 0xF4)	{PID_mode=1;target_angle2=1440;target_angle4=-1440;sum_angle2=0;sum_angle4=0;}//定位移右移
			
			else if(rx_data3 == 0x05)	{PID_mode=0;real_target_speed2=-500;real_target_speed4=-500;target_angle2=0;target_angle4=0;sum_angle2=0;sum_angle4=0;}//定速左转弯
			else if(rx_data3 == 0xF5)	{PID_mode=1;target_angle2=-720;target_angle4=-720;sum_angle2=0;sum_angle4=0;}//定位移左转弯
			
			else if(rx_data3 == 0x06)	{PID_mode=0;real_target_speed2=500;real_target_speed4=500;target_angle2=0;target_angle4=0;sum_angle2=0;sum_angle4=0;}//定速右转弯
			else if(rx_data3 == 0xF6)	{PID_mode=1;target_angle2=720;target_angle4=720;sum_angle2=0;sum_angle4=0;}//定位移右转弯
			
			else if(rx_data3 == 0x07)	{PID_mode=0;target_angle2=0;target_angle4=0;real_target_speed2=0;real_target_speed4=0;sum_angle2=0;sum_angle4=0;}//定速左前
			else if(rx_data3 == 0xF7)	{PID_mode=0;target_angle2=0;target_angle4=0;real_target_speed2=0;real_target_speed4=0;sum_angle2=0;sum_angle4=0;}//定位移左前
			
			else if(rx_data3 == 0x08)	{PID_mode=0;real_target_speed2=-500;real_target_speed4=500;target_angle2=0;target_angle4=0;sum_angle2=0;sum_angle4=0;}//定速左后
			else if(rx_data3 == 0xF8)	{PID_mode=1;target_angle2=-1440;target_angle4=1440;sum_angle2=0;sum_angle4=0;}//定位移左后
			
			else if(rx_data3 == 0x09)	{PID_mode=0;real_target_speed2=500;real_target_speed4=-500;target_angle2=0;target_angle4=0;sum_angle2=0;sum_angle4=0;}//定速右前
			else if(rx_data3 == 0xF9)	{PID_mode=1;target_angle2=1440;target_angle4=-1440;sum_angle2=0;sum_angle4=0;}//定位移右前
			
			else if(rx_data3 == 0x0A)	{PID_mode=0;target_angle2=0;target_angle4=0;real_target_speed2=0;real_target_speed4=0;sum_angle2=0;sum_angle4=0;}//定速右后
			else if(rx_data3 == 0xFA)	{PID_mode=0;target_angle2=0;target_angle4=0;real_target_speed2=0;real_target_speed4=0;sum_angle2=0;sum_angle4=0;}//定位移右后
	
		}
			
		if(PID_mode)
		{
			delta2=target_angle2-sum_angle2;
			delta4=target_angle4-sum_angle4;
		}
		else	{delta2=0;delta4=0;}
		
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

#ifdef  USE_FULL_ASSERT
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
