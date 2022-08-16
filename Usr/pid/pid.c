#include "pid.h"
#include "usart.h"
#include "control.h"
//����ȫ�ֱ���

_pid pid_speed_L, pid_speed_R;    
_pid pid_location_L, pid_location_R;

extern int  PWM_L, PWM_R;
extern int EncoderData_L,EncoderData_L;
extern _pid pid_speed_L, pid_speed_R;    
extern _pid pid_location_L, pid_location_R;
extern long EncoderData_L_SUM, EncoderData_R_SUM;



/**
  * @brief  PID������ʼ��
	*	@note 	��
  * @retval ��
  */
void PID_param_init()
{
	/* λ����س�ʼ������ */
    pid_location_L.target_val=0.0;				
    pid_location_L.actual_val=0.0;
    pid_location_L.err=0.0;
    pid_location_L.err_last=0.0;
    pid_location_L.integral=0.0;
  
		pid_location_L.Kp = 0.3;
		pid_location_L.Ki = 0.0;
		pid_location_L.Kd = 0.0;
  
  	/* �ٶ���س�ʼ������ */
    pid_speed_L.target_val=0.0;				
    pid_speed_L.actual_val=0.0;
    pid_speed_L.err=0.0;
    pid_speed_L.err_last=0.0;
    pid_speed_L.integral=0.0;
  
		pid_speed_L.Kp = 12;
		pid_speed_L.Ki = 1.8;
		pid_speed_L.Kd = 4.0;
		
			/* λ����س�ʼ������ */
    pid_location_R.target_val=0.0;				
    pid_location_R.actual_val=0.0;
    pid_location_R.err=0.0;
    pid_location_R.err_last=0.0;
    pid_location_R.integral=0.0;
  
		pid_location_R.Kp = 0.3;
		pid_location_R.Ki = 0.0;
		pid_location_R.Kd = 0.0;
  
  	/* �ٶ���س�ʼ������ */
    pid_speed_R.target_val=0.0;				
    pid_speed_R.actual_val=0.0;
    pid_speed_R.err=0.0;
    pid_speed_R.err_last=0.0;
    pid_speed_R.integral=0.0;
    
		pid_speed_R.Kp = 12.0;
		pid_speed_R.Ki = 1.8;
		pid_speed_R.Kd = 4.0;
		
//#if defined(PID_ASSISTANT_EN)
//    float pid_temp[3] = {pid_speed_L.Kp, pid_speed_L.Ki, pid_speed_L.Kd};
//    set_computer_value(SEND_P_I_D_CMD, CURVES_CH1, pid_temp, 3);     // ��ͨ�� 1 ���� P I D ֵ
//#endif
}

/**
  * @brief  ����Ŀ��ֵ
  * @param  val		Ŀ��ֵ
	*	@note 	��
  * @retval ��
  */
void set_pid_target(_pid *pid, float temp_val)
{
  pid->target_val = temp_val;    // ���õ�ǰ��Ŀ��ֵ
}

/**
  * @brief  ��ȡĿ��ֵ
  * @param  ��
	*	@note 	��
  * @retval Ŀ��ֵ
  */
float get_pid_target(_pid *pid)
{
  return pid->target_val;    // ���õ�ǰ��Ŀ��ֵ
}

void set_p_i_d(_pid *pid, float p, float i, float d)
{
  	pid->Kp = p;    // ���ñ���ϵ�� P
		pid->Ki = i;    // ���û���ϵ�� I
		pid->Kd = d;    // ����΢��ϵ�� D
}

/**
  * @brief  λ��PID�㷨ʵ��
  * @param  actual_val:ʵ��ֵ
	*	@note 	��
  * @retval ͨ��PID���������
  */
float location_pid_L_realize(_pid *pid, float actual_val)  //λ�û����Kp����Ҳ����
{
		/*����Ŀ��ֵ��ʵ��ֵ�����*/
    pid->err=pid->target_val-actual_val;
	
    /* �趨�ջ����� */   //�⻷�������Բ�Ҫ 
    if((pid->err >= -10) && (pid->err <= 10)) 
    {
      pid->err = 0;
      pid->integral = 0;
			
			EncoderData_L_SUM = 0;
			pid->target_val = 0;  
    }
    
    pid->integral += pid->err;    // ����ۻ�

		/*PID�㷨ʵ��*/
    pid->actual_val = pid->Kp*pid->err
		                  +pid->Ki*pid->integral
		                  +pid->Kd*(pid->err-pid->err_last);
  
		/*����*/
    pid->err_last=pid->err;
    
		/*���ص�ǰʵ��ֵ*/
    return pid->actual_val;
}
float location_pid_R_realize(_pid *pid, float actual_val)  //λ�û����Kp����Ҳ����
{
		/*����Ŀ��ֵ��ʵ��ֵ�����*/
  pid->err=pid->target_val-actual_val;
	
	USART2Printf("pid->err:%f\t actual_val:%f\r\n",pid->err,actual_val);
	
    /* �趨�ջ����� */   //�⻷�������Բ�Ҫ 
    if((pid->err >= -10) && (pid->err <= 10)) 
    {
      pid->err = 0;
      pid->integral = 0;
			
			EncoderData_R_SUM = 0;
			pid->target_val = 0;  
    }
    
    pid->integral += pid->err;    // ����ۻ�

		/*PID�㷨ʵ��*/
    pid->actual_val = pid->Kp*pid->err
		                  +pid->Ki*pid->integral
		                  +pid->Kd*(pid->err-pid->err_last);
  
		/*����*/
    pid->err_last=pid->err;
    
		/*���ص�ǰʵ��ֵ*/
    return pid->actual_val;
}

/**
  * @brief  �ٶ�PID�㷨ʵ��
  * @param  actual_val:ʵ��ֵ
	*	@note 	��
  * @retval ͨ��PID���������
  */
float speed_pid_L_realize(_pid *pid, float actual_val)
{

		/*����Ŀ��ֵ��ʵ��ֵ�����*/
    pid->err = pid->target_val-actual_val;

//    if((pid->err<0.5f ) && (pid->err>-0.5f))  //��1��ô�����������1���ӣ�λ�ò�Ϊ1�����ӵ��ܳ� 
//		{
//      pid->err = 0.0f;
//		}
	
    pid->integral += pid->err;    // ����ۻ�
	
	
	  /*�����޷�*/
	   	 if (pid->integral >= 1200) {pid->integral = 1200;}
      else if (pid->integral < -1200)  {pid->integral = -1200;}

		/*PID�㷨ʵ��*/
    pid->actual_val = pid->Kp*pid->err
		                  +pid->Ki*pid->integral
		                   +pid->Kd*(pid->err-pid->err_last);
  
		/*����*/
    pid->err_last=pid->err;
//    
//		USART2Printf("actual_val:%.2f\t target_val:%.2f\r\n",actual_val,pid->target_val);
//		USART2Printf("pid->Kp:%.2f\t pid->Ki:%.2f\r\n",pid->Kp,pid->Ki);	
//				USART2Printf("pid->Kp:%.2f\t target_val:%.2f\r\n",pid->Kp,pid->target_val);	
		/*���ص�ǰʵ��ֵ*/
    return pid->actual_val;
}
float speed_pid_R_realize(_pid *pid, float actual_val)
{

		/*����Ŀ��ֵ��ʵ��ֵ�����*/
    pid->err = pid->target_val-actual_val;

//    if((pid->err<0.5f ) && (pid->err>-0.5f))  //��1��ô�����������1���ӣ�λ�ò�Ϊ1�����ӵ��ܳ� 
//		{
//      pid->err = 0.0f;
//		}
	
    pid->integral += pid->err;    // ����ۻ�
	
	
	  /*�����޷�*/
	   	 if (pid->integral >= 1200) {pid->integral = 1200;}
      else if (pid->integral < -1200)  {pid->integral = -1200;}

		/*PID�㷨ʵ��*/
    pid->actual_val = pid->Kp*pid->err
		                  +pid->Ki*pid->integral
		                   +pid->Kd*(pid->err-pid->err_last);
  
		/*����*/
    pid->err_last=pid->err;
//    
//		USART2Printf("actual_val:%.2f\t target_val:%.2f\r\n",actual_val,pid->target_val);
//		USART2Printf("pid->Kp:%.2f\t pid->Ki:%.2f\r\n",pid->Kp,pid->Ki);	
//				USART2Printf("pid->Kp:%.2f\t target_val:%.2f\r\n",pid->Kp,pid->target_val);	
		/*���ص�ǰʵ��ֵ*/
    return pid->actual_val;
}
