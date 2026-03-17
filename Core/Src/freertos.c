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
#include "Ahrs_Task.h"
#include "Motor_Task.h"
#include "posture_task.h"
#include "remoctrl_task.h"
#include "process_task.h"
#include "jump_task.h"
#include "Standup_Task.h"
#include "stdio.h"
#include "VMC.h"
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
extern angle_leg Leg;
/* USER CODE END Variables */
/* Definitions for Detect */

//  osThreadId_t PrintHandle;
//  const osThreadAttr_t Print_angle = {
//     .name = "Print",
//     .priority = (osPriority_t)osPriorityNormal6,
//     .stack_size = 128 * 4};
//	osThreadId_t DetectHandle;
// const osThreadAttr_t Detect_attributes = {
//     .name = "Detect",
//     .priority = (osPriority_t)osPriorityNormal5,
//     .stack_size = 128 * 4};
/* Definitions for AHRS */
osThreadId_t AHRSHandle;
const osThreadAttr_t AHRS_attributes = {
    .name = "AHRS",
    .priority = (osPriority_t)osPriorityNormal4,  //4
    .stack_size = 512 * 4};
/* Definitions for RemoCtrl */
osThreadId_t RemoCtrlHandle;
const osThreadAttr_t RemoCtrl_attributes = {
    .name = "RemoCtrl",
    .priority = (osPriority_t)osPriorityNormal3,  //3
    .stack_size = 512 * 4};
/* Definitions for Motor */
osThreadId_t MotorHandle;
const osThreadAttr_t Motor_attributes = {
    .name = "Motor",
    .priority = (osPriority_t)osPriorityNormal2,  //2
    .stack_size = 512 * 4};
osThreadId_t StandupHandle;
const osThreadAttr_t Standup_attributes = {
    .name = "Standup",
    .priority = (osPriority_t)osPriorityNormal1,
    .stack_size = 128 * 4};
osThreadId_t JumpHandle;
const osThreadAttr_t Jump_attributes = {
    .name = "Jump",
    .priority = (osPriority_t)osPriorityNormal1,
    .stack_size = 128 * 4};
/* Definitions for Posture */
osThreadId_t PostureHandle;
const osThreadAttr_t Posture_attributes = {
    .name = "Posture",
    .priority = (osPriority_t)osPriorityNormal,
    .stack_size = 512 * 4};
/* Definitions for Process */
osThreadId_t ProcessHandle;
const osThreadAttr_t Process_attributes = {
    .name = "Process",
    .priority = (osPriority_t)osPriorityBelowNormal7,
    .stack_size = 512 * 4};
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

//void Detect_Task(void *argument);
//void Print_Task(void *argument);
// void AHRS_Task(void *argument);
// void RemoCtrl_Task(void *argument);
// void Motor_Task(void *argument);
// void Posture_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
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
//	PrintHandle = osThreadNew(Print_Task, NULL, &Print_angle);
//  /* creation of Detect */
//	DetectHandle = osThreadNew(Detect_Task, NULL, &Detect_attributes);
  /* creation of AHRS */
  AHRSHandle = osThreadNew(AHRS_Task, NULL, &AHRS_attributes);

  /* creation of RemoCtrl */
  RemoCtrlHandle = osThreadNew(RemoCtrl_Task, NULL, &RemoCtrl_attributes);

  /* creation of Motor */
  MotorHandle = osThreadNew(Motor_Task, NULL, &Motor_attributes);
//	MotorHandle = osThreadNew(Motor_Task_X, NULL, &Motor_attributes);
//	MotorHandle = osThreadNew(Motor_Task_Individual, NULL, &Motor_attributes);
  /* creation of Posture */
  PostureHandle = osThreadNew(Posture_Task, NULL, &Posture_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  StandupHandle = osThreadNew(Standup_Task, NULL, &Standup_attributes);
  JumpHandle = osThreadNew(Jump_Task, NULL, &Jump_attributes);
  ProcessHandle = osThreadNew(Process_Task, NULL, &Process_attributes);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_Detect_Task */
// /**
//   * @brief  Function implementing the Detect thread.
//   * @param  argument: Not used
//   * @retval None
//   */
// /* USER CODE END Header_Detect_Task */
// void Print_Task(void *argument)
// {
//   /* USER CODE BEGIN Detect_Task */
//   /* Infinite loop */
//	 while(1){
//		 printf("%f,%f,%f,%f\n",Aim_Angle[6],Aim_Angle[7],(180-Leg.zeta1),Leg.zeta4);
//		 osThreadSuspend(PrintHandle);
//	 }

//   /* USER CODE END Detect_Task */
// }
// void Detect_Task(void *argument)
// {
//   /* USER CODE BEGIN Detect_Task */
//   /* Infinite loop */
//   for(;;)
//   {
//     osDelay(1);
//   }
//   /* USER CODE END Detect_Task */
// }

// /* USER CODE BEGIN Header_AHRS_Task */
// /**
// * @brief Function implementing the AHRS thread.
// * @param argument: Not used
// * @retval None
// */
// /* USER CODE END Header_AHRS_Task */
// void AHRS_Task(void *argument)
// {
//   /* USER CODE BEGIN AHRS_Task */
//   /* Infinite loop */
//   for(;;)
//   {
//     osDelay(1);
//   }
//   /* USER CODE END AHRS_Task */
// }

// /* USER CODE BEGIN Header_RemoCtrl_Task */
// /**
// * @brief Function implementing the RemoCtrl thread.
// * @param argument: Not used
// * @retval None
// */
// /* USER CODE END Header_RemoCtrl_Task */
// void RemoCtrl_Task(void *argument)
// {
//   /* USER CODE BEGIN RemoCtrl_Task */
//   /* Infinite loop */
//   for(;;)
//   {
//     osDelay(1);
//   }
//   /* USER CODE END RemoCtrl_Task */
// }

// /* USER CODE BEGIN Header_Motor_Task */
// /**
// * @brief Function implementing the Motor thread.
// * @param argument: Not used
// * @retval None
// */
// /* USER CODE END Header_Motor_Task */
// void Motor_Task(void *argument)
// {
//   /* USER CODE BEGIN Motor_Task */
//   /* Infinite loop */
//   for(;;)
//   {
//     osDelay(1);
//   }
//   /* USER CODE END Motor_Task */
// }

// /* USER CODE BEGIN Header_Posture_Task */
// /**
// * @brief Function implementing the Posture thread.
// * @param argument: Not used
// * @retval None
// */
// /* USER CODE END Header_Posture_Task */
// void Posture_Task(void *argument)
// {
//   /* USER CODE BEGIN Posture_Task */
//   /* Infinite loop */
//   for(;;)
//   {
//     osDelay(1);
//   }
//   /* USER CODE END Posture_Task */
// }

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
