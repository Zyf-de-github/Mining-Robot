/* Macnuam.h - Macnuam 1.0*/
#include "tim.h"
#include "math.h"
#include <stdlib.h> 

typedef struct{
    int RB;
    int RF;
    int LB;
    int LF;
}Speed;

typedef struct{
	int now;
	int last;
	int vref;
	int err;
	int pout;
	int iout;
	int out;
	
	TIM_HandleTypeDef encoder;
	TIM_HandleTypeDef PWM;
	uint32_t PWMchannel;
	GPIO_TypeDef *ENG1;
  uint16_t ENP1;
	GPIO_TypeDef *ENG2;
  uint16_t ENP2;
	
}Motor;

extern void Macnuam(int vx, int vy, int vz, Speed *speed);
extern void Calculate(int vset, Motor *motor, int breakkill);