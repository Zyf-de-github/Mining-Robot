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
#include "math.h"
#include <stdlib.h>
#include "bsp_rc.h"
#include "remote_control.h"
#include "stm32f4xx_it.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct{
	int now;
	int last;
	int vref;
	int err;
	int pout;
	int iout;
	int out;
	
	TIM_HandleTypeDef encoder;  //读取电机编码器回传数据
	GPIO_TypeDef *ENG1;					//in1通道
  uint16_t ENP1;							//
	GPIO_TypeDef *ENG2;					//in2通道
  uint16_t ENP2;							//
	
}Motor;

typedef struct{
    int RB;
    int RF;
    int LB;
    int LF;
		int UP;
}Speed;



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define Stop_State 0
#define Forward_State 1
#define Back_State 2
#define Right_State 3
#define Left_State 4
#define Rrotate_State 5
#define Lrotate_State 6


#define Normal 0
#define Fast 1
#define Slow 2

#define out_control 70
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

int state=Stop_State;
int up_state=Stop_State;
int speedset=Slow;
extern Massage msg;

Speed myspeed;
Motor RB;
Motor RF;
Motor LB;
Motor LF;
Motor UP;

extern const RC_ctrl_t *local_rc_ctrl;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
osThreadId MotorHandle;
osThreadId GimbalHandle;
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

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

void MotorFunc(void const * argument);
void GimbalFunc(void const * argument);
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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	
  osThreadDef(Motor, MotorFunc, osPriorityNormal, 0, 512);
  MotorHandle = osThreadCreate(osThread(Motor), NULL);
	
	osThreadDef(Gimbal, GimbalFunc, osPriorityNormal, 0, 512);
  GimbalHandle = osThreadCreate(osThread(Gimbal), NULL);
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
  * @brief  底盘任务线程
  * @param  None
  * @retval None
  */
void Macnuam(int vx, int vy, int vz, Speed *speed)
{
  speed->RB = -vx + vy - vz;
  speed->RF = vx - vy - vz;
  speed->LB = -vx - vy + vz;
  speed->LF = vx + vy + vz;
}
void Calculate(int vset, Motor *motor, int breakkill)
{
	motor->last = motor->now;
	motor->now = __HAL_TIM_GET_COUNTER(&motor->encoder);
	motor->vref = motor->now - motor->last;
	if(motor->vref > 0x7fff)
	{
		motor->vref -= 0xffff;
	}
	else if(motor->vref < -0x7fff)
	{
		motor->vref += 0xffff;
	}
	motor->err = vset - motor->vref;
	
	motor->pout = 15 * motor->err;
	
	motor->iout += 2 * motor->err;
	if(motor->iout > 100)
	{
		motor->iout = 100;
	}
	else if(motor->iout < -100)
	{
		motor->iout = -100;
	}
	
	motor->out = motor->iout + motor->pout;
	if(motor->out > 980)
	{
		motor->out = 980;
	}
	else if(motor->out < -980)
	{
		motor->out = -980;
	}
	
	if(breakkill==1)
	{
		HAL_GPIO_WritePin(motor->ENG1,motor->ENP1,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(motor->ENG2,motor->ENP2,GPIO_PIN_RESET);
	}
	else if(breakkill==2)
	{
		HAL_GPIO_WritePin(motor->ENG1,motor->ENP1,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(motor->ENG2,motor->ENP2,GPIO_PIN_SET);
	}
	else if(motor->out > out_control)
	{
		HAL_GPIO_WritePin(motor->ENG1,motor->ENP1,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(motor->ENG2,motor->ENP2,GPIO_PIN_SET);
	}
	else if(motor->out < -out_control)
	{
		HAL_GPIO_WritePin(motor->ENG1,motor->ENP1,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(motor->ENG2,motor->ENP2,GPIO_PIN_RESET);
	}
	else if(motor->out>=-out_control&&motor->out<=out_control)
	{
		HAL_GPIO_WritePin(motor->ENG1,motor->ENP1,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(motor->ENG2,motor->ENP2,GPIO_PIN_RESET);
	}
//	__HAL_TIM_SET_COMPARE(&motor->PWM, motor->PWMchannel, abs(motor->out));

}
void MotorFunc(void const * argument)
{
  /* USER CODE BEGIN MotorFunc */
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_2);
	
	RF.encoder = htim1;
	RB.encoder = htim3;
	LF.encoder = htim2;
	LB.encoder = htim4;
	
	RF.ENG1 = GPIOC;
	RF.ENP1 = GPIO_PIN_3;
	RF.ENG2 = GPIOC;
	RF.ENP2 = GPIO_PIN_2;
	
	RB.ENG1 = GPIOC;
	RB.ENP1 = GPIO_PIN_5;
	RB.ENG2 = GPIOC;
	RB.ENP2 = GPIO_PIN_4;
	
	LF.ENG1 = GPIOC;
	LF.ENP1 = GPIO_PIN_0;
	LF.ENG2 = GPIOC;
	LF.ENP2 = GPIO_PIN_1;
	
	LB.ENG1 = GPIOC;
	LB.ENP1 = GPIO_PIN_11;
	LB.ENG2 = GPIOC;
	LB.ENP2 = GPIO_PIN_10;


  /* Infinite loop */
  for(;;)
  {
		//底盘控制
		if(local_rc_ctrl->rc.s[0]==2)
		{
			Macnuam(local_rc_ctrl->rc.ch[3]/66, local_rc_ctrl->rc.ch[0]/66, local_rc_ctrl->rc.ch[2]/66, &myspeed);
		}
		else if(local_rc_ctrl->rc.s[0]==3)
		{
			Macnuam(msg.x*10, -msg.y*10, msg.yaw*10, &myspeed);
		}
		Calculate(myspeed.RB, &RB, 0);
		Calculate(myspeed.RF, &RF, 0);
		Calculate(myspeed.LB, &LB, 0);
		Calculate(myspeed.LF, &LF, 0);
		
    osDelay(3);
  }
}







/**
  * @brief  云台任务线程
  * @param  None
  * @retval None
  */
#define open_angle 20   		//夹爪开
#define close_angle 115   	//夹爪关
#define mid_angle  70 			//正中
#define down_angle 155	 		//正下

int angle_1=open_angle;
int angle_2=mid_angle;
int total_encoder=100;
int encoder_init=0;
void Servo_SetAngle_1(int angle)
{
    if(angle > 180) angle = 180;
    uint32_t ccr = 25 + ((uint32_t)angle * (125 - 25)) / 180;
//    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, ccr);//25-125
	  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, ccr);
//	  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, ccr);
}
void Servo_SetAngle_2(int angle)
{
    if(angle > 180) angle = 180;
    uint32_t ccr = 25 + ((uint32_t)angle * (125 - 25)) / 180;
//    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, ccr);//25-125
//	  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, ccr);
	  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, ccr);
}
void Calculate2(int vset, Motor *motor)
{
	motor->last = motor->now;
	motor->now = __HAL_TIM_GET_COUNTER(&motor->encoder);
	motor->vref = motor->now - motor->last;
	if(motor->vref > 0x7fff)
	{
		motor->vref -= 0xffff;
	}
	else if(motor->vref < -0x7fff)
	{
		motor->vref += 0xffff;
	}
	motor->err = vset - motor->vref;
	
	motor->pout = 15 * motor->err;
	
	motor->iout += 2 * motor->err;
	if(motor->iout > 100)
	{
		motor->iout = 100;
	}
	else if(motor->iout < -100)
	{
		motor->iout = -100;
	}
	
	motor->out = motor->iout + motor->pout;
	if(motor->out > 980)
	{
		motor->out = 980;
	}
	else if(motor->out < -980)
	{
		motor->out = -980;
	}
	__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, abs(motor->out));
	if(motor->out > out_control)
	{
		HAL_GPIO_WritePin(motor->ENG1,motor->ENP1,GPIO_PIN_RESET);
//	  HAL_GPIO_WritePin(motor->ENG2,motor->ENP2,GPIO_PIN_SET);
	}
	else if(motor->out < -out_control)
	{
		HAL_GPIO_WritePin(motor->ENG1,motor->ENP1,GPIO_PIN_SET);
//	  HAL_GPIO_WritePin(motor->ENG2,motor->ENP2,GPIO_PIN_RESET);
	}
}
void Macnuam2(int vx, int vy, int vz, Speed *speed)
{
  speed->UP = vx + vy - vz;
}
void GimbalFunc(void const * argument)
{//2底下 3中间
	up_state=Stop_State;
	HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_2);
	
	UP.encoder = htim8;
	
	UP.ENG1 = GPIOB;
	UP.ENP1 = GPIO_PIN_5;
//	UP.ENG2 = GPIOB;
//	UP.ENP2 = GPIO_PIN_4;
	
	Servo_SetAngle_1(angle_1);//夹爪
	Servo_SetAngle_2(angle_2);//俯仰
	for(;;)
	{		
		//夹爪电机控制
		if(local_rc_ctrl->rc.s[1]==2)
		{
			angle_1=open_angle;
		}
		else if(local_rc_ctrl->rc.s[1]==3)
		{
			angle_1=close_angle;
		}
		else
		{
			angle_1=angle_1;
		}
		//俯仰电机控制
		if(local_rc_ctrl->rc.ch[4]==660)
		{
			if(angle_2<down_angle)
			{
				angle_2+=1;
			}
			else
			{
				angle_2=angle_2;
			}
		}
		else if(local_rc_ctrl->rc.ch[4]==-660)
		{
			if(angle_2>mid_angle)
			{
				angle_2-=1;
			}
			else
			{
				angle_2=angle_2;
			}
		}
		else
		{
			angle_2=angle_2;
		}
		//命令发送
		Servo_SetAngle_1(angle_1);
		Servo_SetAngle_2(angle_2);

		
//		//机械臂控制
//		if(encoder_init)
//		{
//			if(total_encoder+UP.vref>1350)
//			{
//				total_encoder=total_encoder;
//				Macnuam2(0, 0, 0, &myspeed);
//				Calculate2(myspeed.UP, &UP);
//			}
//			else if(total_encoder+UP.vref<140)
//			{
//				total_encoder=total_encoder;
//				Macnuam2(0, 0, 0, &myspeed);
//				Calculate2(myspeed.UP, &UP);
//			}
//			else
//			{
				if(local_rc_ctrl->rc.s[0]==2)
				{
					Macnuam2(local_rc_ctrl->rc.ch[1]/132, 0, 0, &myspeed);
				}
				else if(local_rc_ctrl->rc.s[0]==3)
				{
					Macnuam2(msg.z*10, 0, 0, &myspeed);
				}
//				total_encoder+=UP.vref;
				Calculate2(myspeed.UP, &UP);
//			}
//		}
//		else
//		{
//			if(local_rc_ctrl->rc.ch[1]/132>0)
//			{
//			Macnuam2(local_rc_ctrl->rc.ch[1]/132, 0, 0, &myspeed);
//			}
//		total_encoder+=UP.vref;
//			Calculate2(myspeed.UP, &UP);
//			if(total_encoder>150)
//			{
//				encoder_init=1;
//			}
//	}

		
		osDelay(8);
	}
}

/* USER CODE END Application */
