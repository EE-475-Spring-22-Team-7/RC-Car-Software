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
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum ir_state
{
  idle,
  object_ahead,
  object_side,
  orient_vehicle,
}ir_sensor_state;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
bool front_sensor_triggered;
bool fall_sensor_triggered;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId ir_sensor_TaskHandle;
osThreadId ir_sensor_state_TaskHandle;
osMessageQId myExampleQueue01Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
unsigned int bit_toggle(unsigned int, int);
void ir_sensor_detection(void const * argument);
void ir_sensor_state_machine(void const* argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

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
  front_sensor_triggered = false;
  fall_sensor_triggered = false;
  ir_sensor_state = idle;
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

  /* Create the queue(s) */
  /* definition and creation of myExampleQueue01 */
  osMessageQDef(myExampleQueue01, 4, uint16_t);
  myExampleQueue01Handle = osMessageCreate(osMessageQ(myExampleQueue01), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(IR_sensor_checking, ir_sensor_detection, osPriorityNormal, 0, 128);
  ir_sensor_TaskHandle = osThreadCreate(osThread(IR_sensor_checking),NULL);
  osThreadDef(IR_sensor_state_changes, ir_sensor_state_machine, osPriorityNormal, 0, 128);
  ir_sensor_TaskHandle = osThreadCreate(osThread(IR_sensor_state_changes),NULL);
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
  /* Infinite loop */
  /* Acts as the IDLE task. DONT TOUCH */ 
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void ir_sensor_detection(void const * argument)
{
  while(1)
  {
    fall_sensor_triggered = !HAL_GPIO_ReadPin(IR_DOWN_GPIO_Port, IR_DOWN_Pin);
    front_sensor_triggered = fall_sensor_triggered
                            & HAL_GPIO_ReadPin(IR_FRONT_GPIO_Port, IR_FRONT_Pin)
                            & HAL_GPIO_ReadPin(IR_FRONT_L_GPIO_Port, IR_FRONT_L_Pin)
                            & HAL_GPIO_ReadPin(IR_FRONT_R_GPIO_Port, IR_FRONT_R_Pin);
    osDelay(1);
  }
}

void ir_sensor_state_machine(void const * argument)
{
  while(1)
  {
    switch(ir_sensor_state)
    {
      case idle:
        if(front_sensor_triggered)
        {
          if(fall_sensor_triggered)
          {
            // HALT
          } else {
            ir_sensor_state = object_ahead;
          }
        }
      case object_ahead:
        // HALT
        // TODO: Rotate function
      case object_side:
        // TODO:
      case orient_vehicle:
        // TODO


    }
  }
}
/* Utilities */
// bit toggle
unsigned int bit_toggle(unsigned int reg, int bitnum){
  return reg = reg ^ (1<<bitnum);
}


/* USER CODE END Application */
