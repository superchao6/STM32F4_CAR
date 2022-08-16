/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "usart.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "tim.h"
#include "pid.h"
#include "control.h"
#include "encoder.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t  ESP32_Buffer[255];

extern uint8_t USART2_flag;
extern uint8_t  USART2_Buffer[255];//收到的数据存放处
extern uint8_t USART3_flag;
extern uint8_t  USART3_Buffer[255];//收到的数据存放处

extern char one[5],two[5],three[5],four[5],five[5],six[5];
extern char one_k210[5],two_k210[5],three_k210[5];

extern int  PWM_L, PWM_R;
extern long EncoderData_L,EncoderData_L;
extern int Direction_L,Direction_R;
extern _pid pid_speed_L, pid_speed_R;    
extern _pid pid_location_L, pid_location_R;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
float pitch,roll,yaw;//data of MPU6050
uint8_t mpu_res;//Get callback data of MPU6050.If it is 0,geting datas of MPU6050 is successful!

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);

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

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityIdle, 0, 256);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, StartTask03, osPriorityIdle, 0, 256);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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
		//	Set_Motor_L_Forward();
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);	//open left encoder
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);	//open right encoder
	HAL_TIM_PWM_Start(&htim13,TIM_CHANNEL_1);	//output left pwm
	HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);	//output right pwm
//	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0|GPIO_PIN_2,GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1|GPIO_PIN_3,GPIO_PIN_RESET);
//	__HAL_TIM_SetCompare(&htim13, TIM_CHANNEL_1, 0);//set right pwm
//	__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 0);//set right pwm
	
  /* Infinite loop */
  for(;;)
  {		

//		receiving_process();   //解析野火上位机通信协议,调试电机位置速度串级PID时用
		
    osDelay(5);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
		osDelay(100);
//		mpu_res = mpu_dmp_get_data(&pitch,&roll,&yaw);//If mpu_res is 0,geting datas of MPU6050 is successful!
//		if(!mpu_res){
////			printf("read mpu6050 success!\r\n");
////			printf("角度：%.2f\t    %.2f\t   %.2f\r\n",pitch,roll,yaw);
//		}
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
		if(USART2_flag == 1){
//			printf("recive buff is %s\r\n",ESP32_Buffer);
			json_Parse((char *)&ESP32_Buffer);
//			printf("one:%s\r\n",one);
//			printf("one:%.2f\r\n",atof(one));
			set_pid_target(&pid_location_L,calculation_pulse(atof(one)));
			set_pid_target(&pid_location_R,calculation_pulse(atof(two)));
			USART2_flag = 0;
//			memset(USART2_Buffer,0x00,255);
		}
		if(USART3_flag == 1){
			printf("recive buff is %s\r\n",USART3_Buffer);
//			json_Parse_K210((char *)&USART3_Buffer);
//			printf("one_k210:%s\r\n",one_k210);
			USART3_flag = 0;
			memset(USART3_Buffer,0x00,255);
		}
    osDelay(2);
  }
  /* USER CODE END StartTask03 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
