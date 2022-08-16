#include "pid.h"
#include "usart.h"
#include "control.h"
//定义全局变量

_pid pid_speed_L, pid_speed_R;    
_pid pid_location_L, pid_location_R;

extern int  PWM_L, PWM_R;
extern int EncoderData_L,EncoderData_L;
extern _pid pid_speed_L, pid_speed_R;    
extern _pid pid_location_L, pid_location_R;
extern long EncoderData_L_SUM, EncoderData_R_SUM;



/**
  * @brief  PID参数初始化
	*	@note 	无
  * @retval 无
  */
void PID_param_init()
{
	/* 位置相关初始化参数 */
    pid_location_L.target_val=0.0;				
    pid_location_L.actual_val=0.0;
    pid_location_L.err=0.0;
    pid_location_L.err_last=0.0;
    pid_location_L.integral=0.0;
  
		pid_location_L.Kp = 0.3;
		pid_location_L.Ki = 0.0;
		pid_location_L.Kd = 0.0;
  
  	/* 速度相关初始化参数 */
    pid_speed_L.target_val=0.0;				
    pid_speed_L.actual_val=0.0;
    pid_speed_L.err=0.0;
    pid_speed_L.err_last=0.0;
    pid_speed_L.integral=0.0;
  
		pid_speed_L.Kp = 12;
		pid_speed_L.Ki = 1.8;
		pid_speed_L.Kd = 4.0;
		
			/* 位置相关初始化参数 */
    pid_location_R.target_val=0.0;				
    pid_location_R.actual_val=0.0;
    pid_location_R.err=0.0;
    pid_location_R.err_last=0.0;
    pid_location_R.integral=0.0;
  
		pid_location_R.Kp = 0.3;
		pid_location_R.Ki = 0.0;
		pid_location_R.Kd = 0.0;
  
  	/* 速度相关初始化参数 */
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
//    set_computer_value(SEND_P_I_D_CMD, CURVES_CH1, pid_temp, 3);     // 给通道 1 发送 P I D 值
//#endif
}

/**
  * @brief  设置目标值
  * @param  val		目标值
	*	@note 	无
  * @retval 无
  */
void set_pid_target(_pid *pid, float temp_val)
{
  pid->target_val = temp_val;    // 设置当前的目标值
}

/**
  * @brief  获取目标值
  * @param  无
	*	@note 	无
  * @retval 目标值
  */
float get_pid_target(_pid *pid)
{
  return pid->target_val;    // 设置当前的目标值
}

void set_p_i_d(_pid *pid, float p, float i, float d)
{
  	pid->Kp = p;    // 设置比例系数 P
		pid->Ki = i;    // 设置积分系数 I
		pid->Kd = d;    // 设置微分系数 D
}

/**
  * @brief  位置PID算法实现
  * @param  actual_val:实际值
	*	@note 	无
  * @retval 通过PID计算后的输出
  */
float location_pid_L_realize(_pid *pid, float actual_val)  //位置环光个Kp好像也可以
{
		/*计算目标值与实际值的误差*/
    pid->err=pid->target_val-actual_val;
	
    /* 设定闭环死区 */   //外环死区可以不要 
    if((pid->err >= -10) && (pid->err <= 10)) 
    {
      pid->err = 0;
      pid->integral = 0;
			
			EncoderData_L_SUM = 0;
			pid->target_val = 0;  
    }
    
    pid->integral += pid->err;    // 误差累积

		/*PID算法实现*/
    pid->actual_val = pid->Kp*pid->err
		                  +pid->Ki*pid->integral
		                  +pid->Kd*(pid->err-pid->err_last);
  
		/*误差传递*/
    pid->err_last=pid->err;
    
		/*返回当前实际值*/
    return pid->actual_val;
}
float location_pid_R_realize(_pid *pid, float actual_val)  //位置环光个Kp好像也可以
{
		/*计算目标值与实际值的误差*/
  pid->err=pid->target_val-actual_val;
	
	USART2Printf("pid->err:%f\t actual_val:%f\r\n",pid->err,actual_val);
	
    /* 设定闭环死区 */   //外环死区可以不要 
    if((pid->err >= -10) && (pid->err <= 10)) 
    {
      pid->err = 0;
      pid->integral = 0;
			
			EncoderData_R_SUM = 0;
			pid->target_val = 0;  
    }
    
    pid->integral += pid->err;    // 误差累积

		/*PID算法实现*/
    pid->actual_val = pid->Kp*pid->err
		                  +pid->Ki*pid->integral
		                  +pid->Kd*(pid->err-pid->err_last);
  
		/*误差传递*/
    pid->err_last=pid->err;
    
		/*返回当前实际值*/
    return pid->actual_val;
}

/**
  * @brief  速度PID算法实现
  * @param  actual_val:实际值
	*	@note 	无
  * @retval 通过PID计算后的输出
  */
float speed_pid_L_realize(_pid *pid, float actual_val)
{

		/*计算目标值与实际值的误差*/
    pid->err = pid->target_val-actual_val;

//    if((pid->err<0.5f ) && (pid->err>-0.5f))  //差1这么多可以吗？运行1分钟，位置差为1个轮子的周长 
//		{
//      pid->err = 0.0f;
//		}
	
    pid->integral += pid->err;    // 误差累积
	
	
	  /*积分限幅*/
	   	 if (pid->integral >= 1200) {pid->integral = 1200;}
      else if (pid->integral < -1200)  {pid->integral = -1200;}

		/*PID算法实现*/
    pid->actual_val = pid->Kp*pid->err
		                  +pid->Ki*pid->integral
		                   +pid->Kd*(pid->err-pid->err_last);
  
		/*误差传递*/
    pid->err_last=pid->err;
//    
//		USART2Printf("actual_val:%.2f\t target_val:%.2f\r\n",actual_val,pid->target_val);
//		USART2Printf("pid->Kp:%.2f\t pid->Ki:%.2f\r\n",pid->Kp,pid->Ki);	
//				USART2Printf("pid->Kp:%.2f\t target_val:%.2f\r\n",pid->Kp,pid->target_val);	
		/*返回当前实际值*/
    return pid->actual_val;
}
float speed_pid_R_realize(_pid *pid, float actual_val)
{

		/*计算目标值与实际值的误差*/
    pid->err = pid->target_val-actual_val;

//    if((pid->err<0.5f ) && (pid->err>-0.5f))  //差1这么多可以吗？运行1分钟，位置差为1个轮子的周长 
//		{
//      pid->err = 0.0f;
//		}
	
    pid->integral += pid->err;    // 误差累积
	
	
	  /*积分限幅*/
	   	 if (pid->integral >= 1200) {pid->integral = 1200;}
      else if (pid->integral < -1200)  {pid->integral = -1200;}

		/*PID算法实现*/
    pid->actual_val = pid->Kp*pid->err
		                  +pid->Ki*pid->integral
		                   +pid->Kd*(pid->err-pid->err_last);
  
		/*误差传递*/
    pid->err_last=pid->err;
//    
//		USART2Printf("actual_val:%.2f\t target_val:%.2f\r\n",actual_val,pid->target_val);
//		USART2Printf("pid->Kp:%.2f\t pid->Ki:%.2f\r\n",pid->Kp,pid->Ki);	
//				USART2Printf("pid->Kp:%.2f\t target_val:%.2f\r\n",pid->Kp,pid->target_val);	
		/*返回当前实际值*/
    return pid->actual_val;
}
