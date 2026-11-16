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
osThreadId InsTaskHandle;
osThreadId MotorTaskHandle;
osThreadId ChassisTaskHandle;
osThreadId GimbalTaskHandle;
osThreadId ShootTaskHandle;
osThreadId CmdTaskHandle;
osThreadId UsartTaskHandle;
osThreadId TransmissionTasHandle;
osThreadId RefereeTaskHandle;
osThreadId SpecialTaskHandle;
osThreadId PowerTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartInsTask(void const * argument);
void StartMotorTask(void const * argument);
void StartChassisTask(void const * argument);
void StartGimbalTask(void const * argument);
void StartShootTask(void const * argument);
void StartCmdTask(void const * argument);
void StartUsartTask(void const * argument);
void StartTransmissionTask(void const * argument);
void StartRefereeTask(void const * argument);
void StartSpecialTask(void const * argument);
void StartPowerTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
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
  /* definition and creation of InsTask */
  osThreadDef(InsTask, StartInsTask, osPriorityRealtime, 0, 2048);
  InsTaskHandle = osThreadCreate(osThread(InsTask), NULL);

  /* definition and creation of MotorTask */
  osThreadDef(MotorTask, StartMotorTask, osPriorityHigh, 0, 1024);
  MotorTaskHandle = osThreadCreate(osThread(MotorTask), NULL);

  /* definition and creation of ChassisTask */
  osThreadDef(ChassisTask, StartChassisTask, osPriorityAboveNormal, 0, 2048);
  ChassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

  /* definition and creation of GimbalTask */
  osThreadDef(GimbalTask, StartGimbalTask, osPriorityAboveNormal, 0, 2048);
  GimbalTaskHandle = osThreadCreate(osThread(GimbalTask), NULL);

  /* definition and creation of ShootTask */
  osThreadDef(ShootTask, StartShootTask, osPriorityNormal, 0, 2048);
  ShootTaskHandle = osThreadCreate(osThread(ShootTask), NULL);

  /* definition and creation of CmdTask */
  osThreadDef(CmdTask, StartCmdTask, osPriorityNormal, 0, 2048);
  CmdTaskHandle = osThreadCreate(osThread(CmdTask), NULL);

  /* definition and creation of UsartTask */
  osThreadDef(UsartTask, StartUsartTask, osPriorityNormal, 0, 2048);
  UsartTaskHandle = osThreadCreate(osThread(UsartTask), NULL);

  /* definition and creation of TransmissionTas */
  osThreadDef(TransmissionTas, StartTransmissionTask, osPriorityBelowNormal, 0, 2048);
  TransmissionTasHandle = osThreadCreate(osThread(TransmissionTas), NULL);

  /* definition and creation of RefereeTask */
  osThreadDef(RefereeTask, StartRefereeTask, osPriorityBelowNormal, 0, 2048);
  RefereeTaskHandle = osThreadCreate(osThread(RefereeTask), NULL);

  /* definition and creation of SpecialTask */
  osThreadDef(SpecialTask, StartSpecialTask, osPriorityLow, 0, 2048);
  SpecialTaskHandle = osThreadCreate(osThread(SpecialTask), NULL);

  /* definition and creation of PowerTask */
  osThreadDef(PowerTask, StartPowerTask, osPriorityLow, 0, 1024);
  PowerTaskHandle = osThreadCreate(osThread(PowerTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartInsTask */
/**
  * @brief  Function implementing the InsTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartInsTask */
__weak void StartInsTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartInsTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartInsTask */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorTask */
__weak void StartMotorTask(void const * argument)
{
  /* USER CODE BEGIN StartMotorTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMotorTask */
}

/* USER CODE BEGIN Header_StartChassisTask */
/**
* @brief Function implementing the ChassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartChassisTask */
__weak void StartChassisTask(void const * argument)
{
  /* USER CODE BEGIN StartChassisTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartChassisTask */
}

/* USER CODE BEGIN Header_StartGimbalTask */
/**
* @brief Function implementing the GimbalTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGimbalTask */
__weak void StartGimbalTask(void const * argument)
{
  /* USER CODE BEGIN StartGimbalTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartGimbalTask */
}

/* USER CODE BEGIN Header_StartShootTask */
/**
* @brief Function implementing the ShootTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartShootTask */
__weak void StartShootTask(void const * argument)
{
  /* USER CODE BEGIN StartShootTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartShootTask */
}

/* USER CODE BEGIN Header_StartCmdTask */
/**
* @brief Function implementing the CmdTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCmdTask */
__weak void StartCmdTask(void const * argument)
{
  /* USER CODE BEGIN StartCmdTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartCmdTask */
}

/* USER CODE BEGIN Header_StartUsartTask */
/**
* @brief Function implementing the UsartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUsartTask */
__weak void StartUsartTask(void const * argument)
{
  /* USER CODE BEGIN StartUsartTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartUsartTask */
}

/* USER CODE BEGIN Header_StartTransmissionTask */
/**
* @brief Function implementing the TransmissionTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTransmissionTask */
__weak void StartTransmissionTask(void const * argument)
{
  /* USER CODE BEGIN StartTransmissionTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTransmissionTask */
}

/* USER CODE BEGIN Header_StartRefereeTask */
/**
* @brief Function implementing the RefereeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRefereeTask */
__weak void StartRefereeTask(void const * argument)
{
  /* USER CODE BEGIN StartRefereeTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartRefereeTask */
}

/* USER CODE BEGIN Header_StartSpecialTask */
/**
* @brief Function implementing the SpecialTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSpecialTask */
__weak void StartSpecialTask(void const * argument)
{
  /* USER CODE BEGIN StartSpecialTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSpecialTask */
}

/* USER CODE BEGIN Header_StartPowerTask */
/**
* @brief Function implementing the PowerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPowerTask */
__weak void StartPowerTask(void const * argument)
{
  /* USER CODE BEGIN StartPowerTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartPowerTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
