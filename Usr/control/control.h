#ifndef __CONTROL_H
#define __CONTROL_H

#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx.h"
#include "usart.h"
#include "tim.h"
#include "main.h"


/* ���Ƚ�ֵ */
#define PWM_MAX_PERIOD_COUNT              (PWM_PERIOD_COUNT - 30)    //���PWMŪ�������ģ�һЩ������ͻ�������⣨Ӳ���ϵ�ԭ��
#define PWM2_MAX_PERIOD_COUNT              (PWM2_PERIOD_COUNT - 30)


/****************��ߵ�����ų�ʼ��**************/
/* ʹ����� */
#define MOTOR_L_ENABLE()      HAL_TIM_PWM_Start(&htim13,TIM_CHANNEL_1);  

/* ������� */
#define MOTOR_L_DISABLE()     HAL_TIM_PWM_Stop(&htim13,TIM_CHANNEL_1);



/****************�ұߵ�����ų�ʼ��**************/      //����������ת��һ�µ�ʱ��Ҫͬʱ������ĺ궨���encode.c�������������������
/* �����ٶȣ�ռ�ձȣ�2 */     
//#define SET_R_PWM(pwm_r)       __HAL_TIM_SET_COMPARE(&htim14,TIM_CHANNEL_1,pwm_r)    // ���ñȽϼĴ�����ֵ 

/* ʹ�����2 */
#define MOTOR_R_ENABLE()                  HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);   

/* �������2 */
#define MOTOR_R_DISABLE()                 HAL_TIM_PWM_Stop(&htim14,TIM_CHANNEL_1);   
    


/* ����������ֱ��� */
#define ENCODER_RESOLUTION                     13		//����������

/* ������Ƶ֮����ֱܷ��� */
#define ENCODER_TOTAL_RESOLUTION             (ENCODER_RESOLUTION * 4)  /* 4��Ƶ����ֱܷ��� */

/* ���ٵ�����ٱ� */
#define REDUCTION_RATIO  30

/*��sysTick�����PID��������ڣ��Ժ���Ϊ��λ*/
#define SPEED_PID_PERIOD  20    //���Ҫ����ʱ��7���ж�����
#define TARGET_SPEED_MAX  100  //

/*���Ӱ뾶*/
#define RADIUS 	3.35    
#define Perimeter 	(RADIUS*2*3.14) 


float calculation_pulse(float distance_cm);
void Location_Speed_control(void);
//����
void Set_Motor_L_Forward(void);
void Set_Motor_L_Back(void);
void Set_Motor_L_Stop(void);
float speed_pid_L_control(void);
float location_pid_L_control(void);
void Set_PWM_L(int PWM_L);

//����
void Set_Motor_R_Forward(void);
void Set_Motor_R_Back(void);
void Set_Motor_R_Stop(void);
float speed_pid_R_control(void);
float location_pid_R_control(void);
void Set_PWM_R(int PWM_R);
#endif 

