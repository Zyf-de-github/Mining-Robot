/* Macnuam.c - Macnuam 1.0 */
#include "Macnuam.h"

void Macnuam(int vx, int vy, int vz, Speed *speed)
{
  speed->RB = vx + vy - vz;
  speed->RF = vx - vy - vz;
  speed->LB = vx - vy + vz;
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
	
	motor->pout = 120 * motor->err;
	
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
	
	if(breakkill)
	{
		HAL_GPIO_WritePin(motor->ENG1,motor->ENP1,GPIO_PIN_RESET);
	  	HAL_GPIO_WritePin(motor->ENG2,motor->ENP2,GPIO_PIN_RESET);
	}
	else if(motor->out > 0)
	{
		HAL_GPIO_WritePin(motor->ENG1,motor->ENP1,GPIO_PIN_RESET);
	  	HAL_GPIO_WritePin(motor->ENG2,motor->ENP2,GPIO_PIN_SET);
	}
	else if(motor->out < 0)
	{
		HAL_GPIO_WritePin(motor->ENG1,motor->ENP1,GPIO_PIN_SET);
	  	HAL_GPIO_WritePin(motor->ENG2,motor->ENP2,GPIO_PIN_RESET);
	}
	__HAL_TIM_SET_COMPARE(&motor->PWM, motor->PWMchannel, abs(motor->out));

}

int maxset(int ref, int set)
{
  if (ref > set)
    return set;
  else if (ref < -set)
    return -set;
  else
    return ref;
}