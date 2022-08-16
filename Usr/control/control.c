#include "control.h"
#include "usart.h"
#include "tim.h"
#include "pid.h"
#include "encoder.h"
#include "protocol.h"

int  PWM_L = 0, PWM_R = 0;
float Vel_L = 0, Vel_R = 0;
extern long EncoderData_L,EncoderData_R;
extern _pid pid_speed_L, pid_speed_R;    
extern _pid pid_location_L, pid_location_R;
extern long EncoderData_L_SUM,EncoderData_R_SUM;

unsigned char location_control_count = 0;  //执行频率不需要那么高的用这个事件计数，用在中断中


void Location_Speed_control(void)          //原地转向可以直接 调用这个
{
		//mpu_dmp_get_data放在这会影响到编码器脉冲的获取。

		location_control_count++;
		if(location_control_count >= 2)
		{
				location_control_count = 0; 

				Vel_L = location_pid_L_control();
				Vel_R = location_pid_R_control();
		}

		set_pid_target(&pid_speed_L, Vel_L); //每次都必须有位置环的值     //调试速度环PID时取消这两句
		set_pid_target(&pid_speed_R, Vel_R); //每次都必须有位置环的值

		PWM_L = speed_pid_L_control();    
		PWM_R = speed_pid_R_control();  

}

float calculation_pulse(float distance_cm)
{
	float pulses = (distance_cm / Perimeter) * ENCODER_TOTAL_RESOLUTION * REDUCTION_RATIO;
	return pulses;
}

//左轮 L:21.04805 
void Set_Motor_L_Forward(void){
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,GPIO_PIN_SET);
}
void Set_Motor_L_Back(void){
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,GPIO_PIN_RESET);
}
void Set_Motor_L_Stop(void){
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0|GPIO_PIN_1,GPIO_PIN_RESET);
}

float speed_pid_L_control(void)  
{
    float cont_val = 0.0;                       // 当前控制值
    double actual_speed = 0;
	  actual_speed = (EncoderData_L*1000*60)/(ENCODER_TOTAL_RESOLUTION*REDUCTION_RATIO*SPEED_PID_PERIOD);  //rpm
		
//	USART2Printf("actual_speed_L:%lf\r\n",actual_speed);
	
    cont_val = speed_pid_L_realize(&pid_speed_L, actual_speed);    // 进行 PID 计算
	
		if(cont_val>1600)
			cont_val = 1600;
		else if(cont_val<-1600)
			cont_val = -1600;
//	printf("cont_val:%.2f",cont_val);
		
		int temp = actual_speed;
	
//  #if defined(PID_ASSISTANT_EN)
//    set_computer_value(SEND_FACT_CMD, CURVES_CH1, &temp, 1);                // 给通道 1 发送实际值
//  #endif

		return cont_val;
}

float location_pid_L_control(void)  
{
		float cont_val = 0.0; 
		int32_t actual_location;
	
	  actual_location =  EncoderData_L_SUM;   //1圈 = 2464个脉冲 = 56*11*4  //这里位置用圈数代替。

    cont_val = location_pid_L_realize(&pid_location_L, actual_location);   
	
		USART2Printf("EncoderData_L_SUM:%ld\t cont_val:%f\r\n", EncoderData_L_SUM, cont_val);
	
	  // 改成串级PID后，位置换的cont_val对应目标速度
	
	  /* 目标速度上限处理 */
		if (cont_val > TARGET_SPEED_MAX)
		{
			cont_val = TARGET_SPEED_MAX;
		}
		else if (cont_val < -TARGET_SPEED_MAX)
		{
			cont_val = -TARGET_SPEED_MAX;
		}
		
//		#if defined(PID_ASSISTANT_EN)
//			set_computer_value(SEND_FACT_CMD, CURVES_CH3, &actual_location, 1);                // 给通道 1 发送实际值
//		#endif
	return cont_val;
}

void Set_PWM_L(int PWM_L){
	if(PWM_L == 0){
		__HAL_TIM_SetCompare(&htim13,TIM_CHANNEL_1,PWM_L);
//		Set_Motor_L_Stop();
	}else if(PWM_L > 0){
		Set_Motor_L_Forward();
		__HAL_TIM_SetCompare(&htim13,TIM_CHANNEL_1,PWM_L);
	}else{
		Set_Motor_L_Back();
		__HAL_TIM_SetCompare(&htim13,TIM_CHANNEL_1,-PWM_L);
	}
}



//右轮
void Set_Motor_R_Forward(void){
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_3,GPIO_PIN_SET);
}
void Set_Motor_R_Back(void){
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_3,GPIO_PIN_RESET);
}
void Set_Motor_R_Stop(void){
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2|GPIO_PIN_3,GPIO_PIN_RESET);
}

float speed_pid_R_control(void)  
{
    float cont_val = 0.0;                       // 当前控制值
    double actual_speed = 0;
	  actual_speed = (EncoderData_R*1000*60)/(ENCODER_TOTAL_RESOLUTION*REDUCTION_RATIO*SPEED_PID_PERIOD);  //rpm
		
//	USART2Printf("actual_speed_R:%lf\r\n",actual_speed);
	
    cont_val = speed_pid_R_realize(&pid_speed_R, actual_speed);    // 进行 PID 计算
	
		if(cont_val>1600)
			cont_val = 1600;
		else if(cont_val<-1600)
			cont_val = -1600;
//	USART2Printf("cont_val_R:%.2f\r\n",cont_val);
		
		
//  #if defined(PID_ASSISTANT_EN)
//    set_computer_value(SEND_FACT_CMD, CURVES_CH1, &actual_speed, 1);                // 给通道2发送实际值
//  #endif

		return cont_val;
}

float location_pid_R_control(void)  
{
	float cont_val = 0.0; 
	int32_t actual_location;
	
	  actual_location =  EncoderData_R_SUM;   //1圈 = 2464个脉冲 = 56*11*4  //这里位置用圈数代替。

    cont_val = location_pid_R_realize(&pid_location_R, actual_location);   
	  
		USART2Printf("EncoderData_R_SUM:%ld\t cont_val:%f\r\n", EncoderData_L_SUM, cont_val);

	  // 改成串级PID后，位置换的cont_val对应目标速度
	
	    	/* 目标速度上限处理 */
      if (cont_val > TARGET_SPEED_MAX)
      {
        cont_val = TARGET_SPEED_MAX;
      }
      else if (cont_val < -TARGET_SPEED_MAX)
      {
        cont_val = -TARGET_SPEED_MAX;
      }
			
//			#if defined(PID_ASSISTANT_EN)
//				set_computer_value(SEND_FACT_CMD, CURVES_CH4, &actual_location, 1);                // 给通道 1 发送实际值
//			#endif
	return cont_val;
}

void Set_PWM_R(int PWM_R){
	if(PWM_R == 0){
		__HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,PWM_R);
	}else if(PWM_R > 0){
		Set_Motor_R_Forward();
		__HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,PWM_R);
	}else{
		Set_Motor_R_Back();
		__HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,-PWM_R);
	}
}

