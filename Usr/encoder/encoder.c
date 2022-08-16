#include "encoder.h"
#include "stm32f4xx.h"
#include "tim.h"//包含tim头文件
#include "usart.h"

long EncoderData_L, EncoderData_R;//编码器数值
int Direction_L,Direction_R;//电机方向
long EncoderData_L_SUM = 0,EncoderData_R_SUM = 0;
//如果编码器有时候捕获的数值会跳变的话，可以在cubemx的编码器设置里面加上滤波，可以设置1~16的值


/*******************实际运行时读取编码器数值************************/   //当两个轮子转向不一致的时候，要同时调这里编码器捕获脉冲的正负和control.h里的PWM通道宏定义
void Read_Encoder(void)//读取电机脉冲
{
		Direction_L = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);  
		EncoderData_L = __HAL_TIM_GET_COUNTER(&htim3);	
		Direction_R = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4); 
		EncoderData_R = __HAL_TIM_GET_COUNTER(&htim4);
//		printf("Direction_L:%d,Direction_R:%d\r\n",Direction_L,Direction_R);//方向

		if(Direction_L == 0)
			EncoderData_L = EncoderData_L;
		else if(Direction_L == 1 && EncoderData_L > 5000)
			EncoderData_L = __HAL_TIM_GET_COUNTER(&htim3) - 65536;
		else
			EncoderData_L = 0;
		
		if(Direction_R == 0)
			EncoderData_R = EncoderData_R;
		else if(Direction_R == 1 && EncoderData_R > 5000)
			EncoderData_R = __HAL_TIM_GET_COUNTER(&htim4) - 65536;
		else
			EncoderData_R = 0;
//		USART2Printf("EncoderData_L:%d,EncoderData_R:%d\r\n",EncoderData_L,EncoderData_R);//编码器的值，脉冲数
		
		__HAL_TIM_SetCounter(&htim3,0);//清空计数器
		__HAL_TIM_SetCounter(&htim4,0);//清空计数器
		
		EncoderData_L_SUM += EncoderData_L;//位置外环使用的脉冲累积
	  EncoderData_R_SUM += EncoderData_R;//位置外环使用的脉冲累积
		
}
