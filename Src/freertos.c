/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "chassis_task.h"
#include "detect_task.h"
#include "remote_control.h"
#include "iwdg.h"
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

/* USER CODE END Variables */
osThreadId GimbalTaskHandle;
osThreadId INSTaskHandle;
osThreadId DetectTaskHandle;
osThreadId PowerMeasureTasHandle;
osThreadId UITaskHandle;
osThreadId ChassisTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartGimbalTask(void const * argument);
void StartINSTask(void const * argument);
void StartDetectTask(void const * argument);
void StartPowerMeasureTask(void const * argument);
void StartUITask(void const * argument);
void StartChassisTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

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

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
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
  /* GimbalTask — disabled (chassis-only) */
  // osThreadDef(GimbalTask, StartGimbalTask, osPriorityHigh, 0, 512);
  // GimbalTaskHandle = osThreadCreate(osThread(GimbalTask), NULL);

  /* INSTask — disabled (no IMU in chassis-only config) */
  // osThreadDef(INSTask, StartINSTask, osPriorityAboveNormal, 0, 512);
  // INSTaskHandle = osThreadCreate(osThread(INSTask), NULL);

  /* definition and creation of DetectTask */
  osThreadDef(DetectTask, StartDetectTask, osPriorityBelowNormal, 0, 512);
  DetectTaskHandle = osThreadCreate(osThread(DetectTask), NULL);

  /* definition and creation of PowerMeasureTas */
  osThreadDef(PowerMeasureTas, StartPowerMeasureTask, osPriorityNormal, 0, 512);
  PowerMeasureTasHandle = osThreadCreate(osThread(PowerMeasureTas), NULL);

  /* definition and creation of UITask */
  osThreadDef(UITask, StartUITask, osPriorityNormal, 0, 512);
  UITaskHandle = osThreadCreate(osThread(UITask), NULL);

  /* definition and creation of ChassisTask */
  osThreadDef(ChassisTask, StartChassisTask, osPriorityAboveNormal, 0, 512);
  ChassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartGimbalTask */
/**
  * @brief  Function implementing the GimbalTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartGimbalTask */
/* GimbalTask — disabled (chassis-only configuration) */
// void StartGimbalTask(void const * argument)
// {
//   for(;;)
//   {
//     osDelay(1);
//   }
// }

/* USER CODE BEGIN Header_StartINSTask */
/**
* @brief Function implementing the INSTask thread.
* @param argument: Not used
* @retval None
*/
// /* USER CODE END Header_StartINSTask */
// void StartINSTask(void const * argument)
// {
//   /* USER CODE BEGIN StartINSTask */
//   INS_Init();
//   /* Infinite loop */
//   for(;;)
//   {
//     INS_Task();
//     osDelay(1);
//   }
//   /* USER CODE END StartINSTask */
// }

/* USER CODE BEGIN Header_StartDetectTask */
/**
* @brief Function implementing the DetectTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDetectTask */
void StartDetectTask(void const * argument)
{
  /* USER CODE BEGIN StartDetectTask */
  Detect_Task_Init();
  /* Infinite loop */
  for(;;)
  {
    Detect_Task();
    osDelay(DETECT_TASK_PERIOD);
  }
  /* USER CODE END StartDetectTask */
}

/* USER CODE BEGIN Header_StartPowerMeasureTask */
/**
* @brief Function implementing the PowerMeasureTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPowerMeasureTask */
void StartPowerMeasureTask(void const * argument)
{
  /* USER CODE BEGIN StartPowerMeasureTask */
  /* Infinite loop */
  for (;;)
  {
    HAL_IWDG_Refresh(&hiwdg);

    /* Key combo F+E held > 1.5s triggers system reset */
    if ((remote_control.key_code & Key_F) && (remote_control.key_code & Key_E))
    {
      resetCount++;
      if (resetCount * DETECT_TASK_PERIOD > 1500)
      {
        __set_FAULTMASK(1);
        HAL_NVIC_SystemReset();
      }
    }
    else
    {
      resetCount = 0;
    }

    osDelay(DETECT_TASK_PERIOD);
  }
  /* USER CODE END StartPowerMeasureTask */
}

/* USER CODE BEGIN Header_StartUITask */
/**
* @brief Function implementing the UITask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUITask */
void StartUITask(void const * argument)
{
  /* USER CODE BEGIN StartUITask */
  /* Infinite loop */
  for(;;)
  {
    /* LED indication based on chassis/RC status */
    osDelay(50);
  }
  /* USER CODE END StartUITask */
}

/* USER CODE BEGIN Header_StartChassisTask */
/**
* @brief Function implementing the ChassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartChassisTask */
void StartChassisTask(void const * argument)
{
  /* USER CODE BEGIN StartChassisTask */
  /* One-time initialization */
  Chassis_Init();

  /* Infinite loop */
  for(;;)
  {
    Chassis_Control();
    osDelay(CHASSIS_TASK_PERIOD);
  }
  /* USER CODE END StartChassisTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
