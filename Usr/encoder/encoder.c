#include "encoder.h"
#include "stm32f4xx.h"
#include "tim.h"//����timͷ�ļ�
#include "usart.h"

long EncoderData_L, EncoderData_R;//��������ֵ
int Direction_L,Direction_R;//�������
long EncoderData_L_SUM = 0,EncoderData_R_SUM = 0;
//�����������ʱ�򲶻����ֵ������Ļ���������cubemx�ı�����������������˲�����������1~16��ֵ


/*******************ʵ������ʱ��ȡ��������ֵ************************/   //����������ת��һ�µ�ʱ��Ҫͬʱ��������������������������control.h���PWMͨ���궨��
void Read_Encoder(void)//��ȡ�������
{
		Direction_L = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);  
		EncoderData_L = __HAL_TIM_GET_COUNTER(&htim3);	
		Direction_R = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4); 
		EncoderData_R = __HAL_TIM_GET_COUNTER(&htim4);
//		printf("Direction_L:%d,Direction_R:%d\r\n",Direction_L,Direction_R);//����

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
//		USART2Printf("EncoderData_L:%d,EncoderData_R:%d\r\n",EncoderData_L,EncoderData_R);//��������ֵ��������
		
		__HAL_TIM_SetCounter(&htim3,0);//��ռ�����
		__HAL_TIM_SetCounter(&htim4,0);//��ռ�����
		
		EncoderData_L_SUM += EncoderData_L;//λ���⻷ʹ�õ������ۻ�
	  EncoderData_R_SUM += EncoderData_R;//λ���⻷ʹ�õ������ۻ�
		
}
