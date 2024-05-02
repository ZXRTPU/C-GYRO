/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
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
#include "rc_potocal.h"
#include "arm_math.h"
#include "INS_task.h"
#include "exchange.h"
#include "Chassis_task.h"
#include "super_cap.h"
#include  "UI_task.h"
#include "stm32f4xx_it.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId Chassis_taskHandle;
osThreadId myTask02Handle;
osThreadId super_capHandle;
osThreadId UI_taskHandle;
/* USER CODE END Variables */
osThreadId INSTaskHandle;
osThreadId defaultTaskHandle;
osThreadId ChassistaskHandle;
osThreadId UItaskHandle;
osThreadId ExchangeTaskHandle;
osThreadId GimbaltaskHandle;
osThreadId DaemontaskHandle;
osThreadId ShoottaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartINSTask(void const * argument);
void StartDefaultTask(void const * argument);
extern void Chassis_task(void const * argument);
void StartUItask(void const * argument);
extern void Exchange_task(void const * argument);
extern void Gimbal_task(void const * argument);
void StartDaemontask(void const * argument);
extern void Shoot_task(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
    /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize)
{
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
    *ppxTimerTaskStackBuffer = &xTimerStack[0];
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
    /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

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
  /* definition and creation of INSTask */
  osThreadDef(INSTask, StartINSTask, osPriorityNormal, 0, 1024);
  INSTaskHandle = osThreadCreate(osThread(INSTask), NULL);

  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Chassistask */
  osThreadDef(Chassistask, Chassis_task, osPriorityNormal, 0, 512);
  ChassistaskHandle = osThreadCreate(osThread(Chassistask), NULL);

  /* definition and creation of UItask */
  osThreadDef(UItask, StartUItask, osPriorityNormal, 0, 512);
  UItaskHandle = osThreadCreate(osThread(UItask), NULL);

  /* definition and creation of ExchangeTask */
  osThreadDef(ExchangeTask, Exchange_task, osPriorityNormal, 0, 128);
  ExchangeTaskHandle = osThreadCreate(osThread(ExchangeTask), NULL);

  /* definition and creation of Gimbaltask */
  osThreadDef(Gimbaltask, Gimbal_task, osPriorityRealtime, 0, 512);
  GimbaltaskHandle = osThreadCreate(osThread(Gimbaltask), NULL);

  /* definition and creation of Daemontask */
  osThreadDef(Daemontask, StartDaemontask, osPriorityNormal, 0, 128);
  DaemontaskHandle = osThreadCreate(osThread(Daemontask), NULL);

  /* definition and creation of Shoottask */
  osThreadDef(Shoottask, Shoot_task, osPriorityNormal, 0, 256);
  ShoottaskHandle = osThreadCreate(osThread(Shoottask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartINSTask */
/**
 * @brief Function implementing the INSTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartINSTask */
void StartINSTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartINSTask */
    INS_Init();
    /* Infinite loop */
    for (;;)
    {
        INS_Task();
        osDelay(1);
    }
  /* USER CODE END StartINSTask */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
* @brief Function implementing the defaultTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOH,GPIO_PIN_10,GPIO_PIN_SET);
	uint8_t TIM1_flag = 1;//��֪��bug
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartUItask */
/**
* @brief Function implementing the UItask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUItask */
void StartUItask(void const * argument)
{
  /* USER CODE BEGIN StartUItask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartUItask */
}

/* USER CODE BEGIN Header_StartDaemontask */
/**
* @brief Function implementing the Daemontask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDaemontask */
void StartDaemontask(void const * argument)
{
  /* USER CODE BEGIN StartDaemontask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDaemontask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
//void StartDefaultTask(void const * argument)
//{
  /* USER CODE BEGIN StartDefaultTask */
	//HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_SET);
	//HAL_GPIO_WritePin(GPIOH,GPIO_PIN_10,GPIO_PIN_SET);
	//uint8_t TIM1_flag = 1;//��֪��bug
  /* Infinite loop */
  //for(;;)
  //{
   // osDelay(1);
  //}
  /* USER CODE END StartDefaultTask */
//}
/* USER CODE END Application */
