#ifndef __CONTROL_H
#define __CONTROL_H

#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx.h"
#include "usart.h"
#include "tim.h"
#include "main.h"


/* 最大比较值 */
#define PWM_MAX_PERIOD_COUNT              (PWM_PERIOD_COUNT - 30)    //如果PWM弄成了满的，一些驱动板就会出现问题（硬件上的原因）
#define PWM2_MAX_PERIOD_COUNT              (PWM2_PERIOD_COUNT - 30)


/****************左边电机引脚初始化**************/
/* 使能输出 */
#define MOTOR_L_ENABLE()      HAL_TIM_PWM_Start(&htim13,TIM_CHANNEL_1);  

/* 禁用输出 */
#define MOTOR_L_DISABLE()     HAL_TIM_PWM_Stop(&htim13,TIM_CHANNEL_1);



/****************右边电机引脚初始化**************/      //当两个轮子转向不一致的时候，要同时调这里的宏定义和encode.c编码器捕获脉冲的正负
/* 设置速度（占空比）2 */     
//#define SET_R_PWM(pwm_r)       __HAL_TIM_SET_COMPARE(&htim14,TIM_CHANNEL_1,pwm_r)    // 设置比较寄存器的值 

/* 使能输出2 */
#define MOTOR_R_ENABLE()                  HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);   

/* 禁用输出2 */
#define MOTOR_R_DISABLE()                 HAL_TIM_PWM_Stop(&htim14,TIM_CHANNEL_1);   
    


/* 编码器物理分辨率 */
#define ENCODER_RESOLUTION                     13		//编码器线数

/* 经过倍频之后的总分辨率 */
#define ENCODER_TOTAL_RESOLUTION             (ENCODER_RESOLUTION * 4)  /* 4倍频后的总分辨率 */

/* 减速电机减速比 */
#define REDUCTION_RATIO  30

/*在sysTick里调用PID计算的周期，以毫秒为单位*/
#define SPEED_PID_PERIOD  20    //这个要看定时器7的中断周期
#define TARGET_SPEED_MAX  100  //

/*轮子半径*/
#define RADIUS 	3.35    
#define Perimeter 	(RADIUS*2*3.14) 


float calculation_pulse(float distance_cm);
void Location_Speed_control(void);
//左轮
void Set_Motor_L_Forward(void);
void Set_Motor_L_Back(void);
void Set_Motor_L_Stop(void);
float speed_pid_L_control(void);
float location_pid_L_control(void);
void Set_PWM_L(int PWM_L);

//右轮
void Set_Motor_R_Forward(void);
void Set_Motor_R_Back(void);
void Set_Motor_R_Stop(void);
float speed_pid_R_control(void);
float location_pid_R_control(void);
void Set_PWM_R(int PWM_R);
#endif 

