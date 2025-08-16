/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
#include "Macnuam.h"
#include "math.h"
#include "usart.h"
#include "oled.h"
#include <stdint.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	Forward,
	Back,
	Right,
	Left,
	Rrotate,
	Lrotate,
	Stop,
}Status;

typedef enum {
	Fast,
	Normal,
	Low
}SpeedSet;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart1_rx;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
Speed myspeed;
int vx,vy,vz;

Motor RB;
Motor RF;
Motor LB;
Motor LF;

int timecount;
int starttime;

int8_t receivedata[8];

int times[27] = {         2000,   390,       200 ,    450 ,   600,    420,    120,    320,     120,    846,    6       , 100,   468,     500,    66,       200,     1080,    1000   ,280      ,700,      800 ,  900 ,  1000,   900  ,  1000,900  ,  10000};
Status states[27] = {     Stop,   Forward,   Stop,    Right,  Stop,   Right,  Stop,   Forward, Stop,   Left,   Rrotate ,Stop,  Forward, Stop,   Back,     Stop,     Right,  Stop   ,Forward  ,Rrotate,  Stop , Left,  Stop,   Left  , Stop,Left  , Stop} ;
SpeedSet speedsets[27] = {Normal, Normal,    Normal,  Normal, Normal, Normal, Normal, Fast,    Normal, Normal, Normal  ,Normal,Fast,    Normal, Normal,   Normal,   Normal, Normal ,Low      ,Normal,   Normal,Normal,Normal, Normal, Normal,Normal, Normal};

int phase = -1;
Status state = Stop;
SpeedSet speedset = Normal;
	
int distance;
uint32_t capture_rising, capture_falling;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId MotorHandle;
osThreadId MainTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
float GetDistance(void);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void MotorFunc(void const * argument);
void MainFunc(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Motor */
  osThreadDef(Motor, MotorFunc, osPriorityRealtime, 0, 512);
  MotorHandle = osThreadCreate(osThread(Motor), NULL);

  /* definition and creation of MainTask */
  osThreadDef(MainTask, MainFunc, osPriorityHigh, 0, 512);
  MainTaskHandle = osThreadCreate(osThread(MainTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	HAL_TIM_IC_Start_IT(&htim17, TIM_CHANNEL_1);
  /* Infinite loop */
  for(;;)
  {
		osSignalWait(0x01, osWaitForever);
    uint32_t time_diff = (capture_falling - capture_rising + 10000) % 10000;
    distance = (float)time_diff * 0.034 / 2;
		OLED_ShowNum(80,2,distance,3,16);
		OLED_ShowNum(16,2,abs(myspeed.RB) * 50 * 2 * 3.14 * 0.03 / 30.0f / 52.0f, 3, 16);
		OLED_ShowNum(16,4,receivedata[0],3,16);
		OLED_ShowNum(80,4,receivedata[1],3,16);
    osDelay(20);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_MotorFunc */
/**
* @brief Function implementing the Motor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorFunc */
void MotorFunc(void const * argument)
{
  /* USER CODE BEGIN MotorFunc */
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_2);
	
	RB.encoder = htim2;
	RF.encoder = htim3;
	LB.encoder = htim4;
	LF.encoder = htim5;
	
	RB.PWM = htim1;
	RF.PWM = htim1;
	LB.PWM = htim1;
	LF.PWM = htim1;
	
	RB.PWMchannel = TIM_CHANNEL_1;
	RF.PWMchannel = TIM_CHANNEL_2;
	LB.PWMchannel = TIM_CHANNEL_3;
	LF.PWMchannel = TIM_CHANNEL_4;
	
	RB.ENG1 = GPIOB;
	RB.ENP1 = GPIO_PIN_2;
	RB.ENG2 = GPIOE;
	RB.ENP2 = GPIO_PIN_7;
	
	RF.ENG1 = GPIOE;
	RF.ENP1 = GPIO_PIN_8;
	RF.ENG2 = GPIOE;
	RF.ENP2 = GPIO_PIN_10;
	
	LB.ENG1 = GPIOB;
	LB.ENP1 = GPIO_PIN_10;
	LB.ENG2 = GPIOB;
	LB.ENP2 = GPIO_PIN_11;
	
	LF.ENG1 = GPIOE;
	LF.ENP1 = GPIO_PIN_15;
	LF.ENG2 = GPIOE;
	LF.ENP2 = GPIO_PIN_12;
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,50);
	
		HAL_UART_Receive_IT(&huart1, (uint8_t *)receivedata, 2);
	
  /* Infinite loop */
  for(;;)
  {
//		if(receivedata[0] == 0x00)
//		{
//			Calculate(myspeed.RB, &RB, 1);
//		  Calculate(myspeed.RF, &RF, 1);
//		  Calculate(myspeed.LB, &LB, 1);
//		  Calculate(myspeed.LF, &LF, 1);
//		}
//		else if (receivedata[0] == 0x01 || receivedata[0] == 0x11)
//		{
//			float vyset = (float)receivedata[2] * (float)0x0B/0xff;
//			float vzset = (float)receivedata[1] * (float)0x0D/90;
//			if(receivedata[1] <30 && receivedata[1] >-30)
//			{
//				Macnuam(0x05, (int)vyset, (int)vzset, &myspeed);
//			}
//			else
//			{
//				if(receivedata[1] >=30)
//				  Macnuam(0x03, (int)vyset, 0x04, &myspeed);
//				else
//					Macnuam(0x03, (int)vyset, -0x04, &myspeed);
//			}

//			Calculate(myspeed.RB, &RB, 0);
//		  Calculate(myspeed.RF, &RF, 0);
//		  Calculate(myspeed.LB, &LB, 0);
//		  Calculate(myspeed.LF, &LF, 0);
//		}
//		else if(receivedata[0] == 0x02)
//		{
//			float vyset = receivedata[2] * (float)0x06/0xff;
//			float vzset = receivedata[1] * (float)0x06/0xff;
//			Macnuam(-0x02, (int)vyset, (int)vzset, &myspeed);
//			Calculate(myspeed.RB, &RB, 0);
//		  Calculate(myspeed.RF, &RF, 0);
//		  Calculate(myspeed.LB, &LB, 0);
//		  Calculate(myspeed.LF, &LF, 0);
//		}
//		else
//		{
//			Calculate(myspeed.RB, &RB, 1);
//		  Calculate(myspeed.RF, &RF, 1);
//		  Calculate(myspeed.LB, &LB, 1);
//		  Calculate(myspeed.LF, &LF, 1);
//		}
		if(state == Stop )
		{
			Macnuam(0, 0, 0, &myspeed);
			Calculate(myspeed.RB, &RB, 1);
			Calculate(myspeed.RF, &RF, 1);
			Calculate(myspeed.LB, &LB, 1);
			Calculate(myspeed.LF, &LF, 1);
		}
		else if(state == Forward)
		{
			if(speedset == Normal)
			{
				Macnuam(0x0A, 0, 0, &myspeed);
			}
			else if(speedset == Fast)
			{
				Macnuam(0x0F, 0, 0, &myspeed);
			}
			else
			{
				Macnuam(0x03, 0, 0, &myspeed);
			}
				Calculate(myspeed.RB, &RB, 0);
				Calculate(myspeed.RF, &RF, 0);
				Calculate(myspeed.LB, &LB, 0);
				Calculate(myspeed.LF, &LF, 0);
		}
		else if(state == Back)
		{
			if(speedset == Normal)
			{
				Macnuam(-0x07, 0, 0, &myspeed);
			}
			else if(speedset == Fast)
			{
				Macnuam(-0x0B, 0, 0, &myspeed);
			}
			else
			{
				Macnuam(-0x05, 0, 0, &myspeed);
			}
				Calculate(myspeed.RB, &RB, 0);
				Calculate(myspeed.RF, &RF, 0);
				Calculate(myspeed.LB, &LB, 0);
				Calculate(myspeed.LF, &LF, 0);
		}
		else if(state == Right)
		{
			if(speedset == Normal)
			{
				Macnuam(0, 0X06, 0, &myspeed);
			}
			else if(speedset == Fast)
			{
				Macnuam(0, 0X0D, 0, &myspeed);
			}
			else
			{
				Macnuam(0, 0X03, 0, &myspeed);
			}
				Calculate(myspeed.RB, &RB, 0);
				Calculate(myspeed.RF, &RF, 0);
				Calculate(myspeed.LB, &LB, 0);
				Calculate(myspeed.LF, &LF, 0);
		}
		else if(state == Left)
		{
			if(speedset == Normal)
			{
				Macnuam(0, -0X06, 0, &myspeed);
			}
			else if(speedset == Fast)
			{
				Macnuam(0, -0X0D, 0, &myspeed);
			}
			else
			{
				Macnuam(0, -0X03, 0, &myspeed);
			}
				Calculate(myspeed.RB, &RB, 0);
				Calculate(myspeed.RF, &RF, 0);
				Calculate(myspeed.LB, &LB, 0);
				Calculate(myspeed.LF, &LF, 0);
		}
		else if(state == Rrotate)
		{
			if(speedset == Normal)
			{
				Macnuam(0, 0, 0X07, &myspeed);
			}
			else if(speedset == Fast)
			{
				Macnuam(0, 0, 0X08, &myspeed);
			}
			else
			{
				Macnuam(0, 0, 0X04, &myspeed);
			}
				Calculate(myspeed.RB, &RB, 0);
				Calculate(myspeed.RF, &RF, 0);
				Calculate(myspeed.LB, &LB, 0);
				Calculate(myspeed.LF, &LF, 0);
		}
		else if(state == Lrotate)
		{
			if(speedset == Normal)
			{
				Macnuam(0, 0, -0X06, &myspeed);
			}
			else if(speedset == Fast)
			{
				Macnuam(0, 0, -0X08, &myspeed);
			}
			else
			{
				Macnuam(0, 0, -0X04, &myspeed);
			}
				Calculate(myspeed.RB, &RB, 0);
				Calculate(myspeed.RF, &RF, 0);
				Calculate(myspeed.LB, &LB, 0);
				Calculate(myspeed.LF, &LF, 0);
		}
		
	osDelay(2);
  }
  /* USER CODE END MotorFunc */
}

/* USER CODE BEGIN Header_MainFunc */
/**
* @brief Function implementing the MainTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MainFunc */
void MainFunc(void const * argument)
{
  /* USER CODE BEGIN MainFunc */
	
	OLED_Init();
	OLED_Clear();
	OLED_ShowCHinese(96,0,6);//?
	OLED_ShowCHinese(112,0,7);//?
	OLED_ShowCHinese(0,0,8);//I
	OLED_ShowCHinese(16,0,9);//R
	OLED_ShowCHinese(32,0,10);//O
	OLED_ShowCHinese(48,0,11);//B
	OLED_ShowCHinese(64,0,12);//O
	OLED_ShowCHinese(80,0,13);//T
	
	OLED_ShowChar(0,2,83,16);//SPEED
	OLED_ShowChar(0,4,77,16);//M NUM1
	OLED_ShowChar(64,2,68,16);//DISTANCE
	OLED_ShowChar(64,4,78,16);//N NUM2
	
	OLED_ShowChar(8,2,58,16);//:
	OLED_ShowChar(72,4,58,16);//:
	OLED_ShowChar(72,2,58,16);//:
	OLED_ShowChar(8,4,58,16);//:
	
  /* Infinite loop */
  for(;;)
  {		

		timecount++;
		
		//start........
    if(starttime == 0)
	{
		phase = (phase+1)%27;
		starttime = times[phase];
		state = states[phase];
		speedset = speedsets[phase];
	}
	else
	{
		starttime--;
	}
	
	if(phase == 0 || phase == 4|| phase == 17)
	{
		if(starttime <= 600)
		{
			if((starttime / 100) % 2)
				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,800);
			else
				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
		}
	}
		
		
    osDelay(2);
  }
  /* USER CODE END MainFunc */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	UNUSED(huart);
	
	HAL_UART_Receive_IT(&huart1, (uint8_t *)receivedata, 2);
	
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim17)
	{
        if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET)
        {
            capture_rising = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        }
        else
        {
            capture_falling = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            osSignalSet(defaultTaskHandle, 0x01);
        }
	}
}
/* USER CODE END Application */
