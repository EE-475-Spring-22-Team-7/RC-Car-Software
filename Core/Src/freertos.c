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
#include "tim.h"
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
sys_state system_status = NORMAL;

vehicle_direction vehicle_motion_dir = HALT;

motors leftmotors;
motors rightmotors;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId demoForwardHandle;
osMessageQId myExampleQueue01Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void set_motor_speed(motors * motor_set, uint16_t new_pulse);
void halt_motors(motors * motor_set);
void forward_motors(motors * motor_set);
void reverse_motors(motors * motor_set);
void set_motors_direction(motors * motor_set,direction dir);
void forward_vehicle(motors * motors_set1, motors * motors_set2);
void reverse_vehicle(motors * motors_set1, motors * motors_set2);
void halt_vehicle(motors * motors_set1, motors * motors_set2);
void rotate_right_vehicle(motors * motors_set1, motors * motors_set2);
void rotate_left_vehicle(motors * motors_set1, motors * motors_set2);
void set_vehicle_motion(motors * motors_set1, motors * motors_set2, vehicle_direction dir);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartForwardTask(void const * argument);

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
  leftmotors.htim = &htim3;
  leftmotors.motor_Channel = TIM_CHANNEL_3;
  leftmotors.pulse = 0;
  leftmotors.motors_dir = HALTED;
  leftmotors.motors_GPIOx = GPIOE;
  leftmotors.H1_GPIO_Pin = MOTORS_LEFT_H1_Pin;
  leftmotors.H2_GPIO_Pin = MOTORS_LEFT_H2_Pin;
  
  rightmotors.htim = &htim3;
  rightmotors.motor_Channel = TIM_CHANNEL_4;
  rightmotors.pulse = 0;
  rightmotors.motors_dir = HALTED;
  rightmotors.motors_GPIOx = GPIOE;
  rightmotors.H1_GPIO_Pin = MOTORS_RIGHT_H1_Pin;
  rightmotors.H2_GPIO_Pin = MOTORS_RIGHT_H2_Pin;
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

  /* definition and creation of demoForward */
  osThreadDef(demoForward, StartForwardTask, osPriorityNormal, 0, 128);
  demoForwardHandle = osThreadCreate(osThread(demoForward), NULL);

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
  /* Infinite loop */
  /* Acts as the IDLE task. DONT TOUCH */ 
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartForwardTask */
/**
* @brief Function implementing the demoForward thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartForwardTask */
void StartForwardTask(void const * argument)
{
  /* USER CODE BEGIN StartForwardTask */
  /* Infinite loop */
  
  // Initialize motor speed for demo
  // will not start moving until we set vehicle motion
  set_motor_speed(&leftmotors,13999);
  set_motor_speed(&rightmotors,13999);
  
  for(;;)
  {
    switch(system_status){
      case NORMAL:
        set_vehicle_motion(&leftmotors,&rightmotors,MOVE_FORWARD);
        
        //delay task - effectively forward motion for 5 seconds
        osDelay(3000);
        
        // Halt vehicle
        set_vehicle_motion(&leftmotors,&rightmotors,HALT);
        
        // delay task - effectively halt motion for 2 seconds
        osDelay(1000);
        break;
      case FAULT:
        // halt vehicle on a fault detection (NOT FROM OBJECT DETECTION)
        set_vehicle_motion(&leftmotors,&rightmotors,HALT);
        break;
      case AI:
        // command vehicle halt from object detection task
        // From object detection task, suspend and resume this task using 
        // osThreadSuspend() and osThreadResume() to prevent this task from 
        // running when there is an obstacle and to resume running when obstacle
        // is cleared. 
        
        // just in case will set delay here
        osDelay(10000); // 10 sec delay
        break;
      default:
        // do nothing
        break;
      
    }
  }
  /* USER CODE END StartForwardTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */




/************* MOTION UTILITIES ***********************************************/
/* Of all of these functions, only a few functions are of
 * particular interest and use. The rest are called within
 * these particular functions. The functions are:
 * - set_vehicle_motion()
 * - set_motor_speed()
*/




/** @brief Set the motor speed
  * @param motor_set: motors handle
  * @param new_pulse: Value to be assigned to output compare register. Must be 
  *     less than/equal to TIM period
  * @retval none
 */
void set_motor_speed(motors * motor_set, uint16_t new_pulse)
//void set_motor_speed(TIM_HandleTypeDef *htim, uint32_t Channel,uint16_t pulse)
{
  // Check that pulse argument not greater than period
  if(new_pulse > motor_set->htim->Init.Period)
  {
    // do nothing or implement error handler
    return;
  }
  
  // set motor speed
  __HAL_TIM_SET_COMPARE(motor_set->htim,motor_set->motor_Channel,new_pulse);
  
  // Update motor struct
  motor_set->pulse = new_pulse;
}


/** @brief Halt motors
  * @param motor_set: motors handle
  *     *NOTE* only sets H-Bridge pins low. Pulse value is conserved
  * @retval none
 */
void halt_motors(motors * motor_set)
{
  // Set H_bridge pins all low
  HAL_GPIO_WritePin(motor_set->motors_GPIOx,motor_set->H1_GPIO_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(motor_set->motors_GPIOx,motor_set->H2_GPIO_Pin,GPIO_PIN_RESET);
  
  // Update motor struct direction
  motor_set->motors_dir = HALTED;
}

/** @brief Set motors to forward motion
  * @param motor_set: motors handle
  * @retval none
 */
void forward_motors(motors * motor_set)
{
  // Set H-Bridge pins
  HAL_GPIO_WritePin(motor_set->motors_GPIOx,motor_set->H1_GPIO_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(motor_set->motors_GPIOx,motor_set->H2_GPIO_Pin,GPIO_PIN_RESET);
  
  // Update motor struct direction
  motor_set->motors_dir = FORWARD;
}

/** @brief Set motors to reverse motion
  * @param motor_set: motors handle
  * @retval none
 */
void reverse_motors(motors * motor_set)
{
  // Set H-Bridge pins
  HAL_GPIO_WritePin(motor_set->motors_GPIOx,motor_set->H1_GPIO_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(motor_set->motors_GPIOx,motor_set->H2_GPIO_Pin,GPIO_PIN_SET);
  
  // Update motor struct direction
  motor_set->motors_dir = REVERSE;
}


/** @brief Set direction of motors
  * @param motor_set: motors handle
  * @param dir: direction of type enum
  * @retval none
 */
void set_motors_direction(motors * motor_set,direction dir)
{
//  // Check that direction argument is different than current direction
//  if(dir == motor_set->motors_dir)
//  {
//    // do nothing and exit
//    return;
//  }
//  else
//  {
//    // otherwise halt before switching directions
//    halt_motors(motor_set);
//  }
  
  switch(dir){
    case FORWARD:
      forward_motors(motor_set);
      break; 
    case REVERSE:
      reverse_motors(motor_set);
      break;  
    case HALTED:
      halt_motors(motor_set);
      break;
    default:
      // halt anyways
      halt_motors(motor_set);
  }
      
}

/** @brief set the vehicles motion to forward
  * @param motors_set1: motors handle
  * @param motors_set2: motors handle
  * @retval none
 */
void forward_vehicle(motors * motors_set1, motors * motors_set2)
{
  // Set both sets of motors to forward motion
  set_motors_direction(motors_set1,FORWARD);
  set_motors_direction(motors_set2,FORWARD);
}

/** @brief set the vehicles motion to reverse
  * @param motors_set1: motors handle
  * @param motors_set2: motors handle
  * @retval none
 */
void reverse_vehicle(motors * motors_set1, motors * motors_set2)
{
  // Set both sets of motors to reverse motion
  set_motors_direction(motors_set1,REVERSE);
  set_motors_direction(motors_set2,REVERSE);
}

/** @brief halt the vehicles motion
  * @param motors_set1: motors handle
  * @param motors_set2: motors handle
  * @retval none
 */
void halt_vehicle(motors * motors_set1, motors * motors_set2)
{
  // Set both sets of motors to reverse motion
  set_motors_direction(motors_set1,HALTED);
  set_motors_direction(motors_set2,HALTED);
}

/** @brief set the vehicles motion to rotate ("steering")
  * @param motors_set1: motors handle
  * @param motors_set2: motors handle
  * @retval none
 */
void rotate_right_vehicle(motors * motors_set1, motors * motors_set2)
{
  // TO DO!!! 
  // Set a default pulse value
  // set a length of time to rotate for example how long to run to rotate 90 degrees
  
  // Set sets of motors to opposite directions
  set_motors_direction(motors_set1,FORWARD);
  set_motors_direction(motors_set2,REVERSE);  
}

/** @brief set the vehicles motion to rotate ("steering")
  * @param motors_set1: motors handle
  * @param motors_set2: motors handle
  * @retval none
 */
void rotate_left_vehicle(motors * motors_set1, motors * motors_set2)
{
  // TO DO!!! 
  // Set a default pulse value
  // set a length of time to rotate for example how long to run to rotate 90 degrees
  
  // Set sets of motors to opposite directions
  set_motors_direction(motors_set1,REVERSE);
  set_motors_direction(motors_set2,FORWARD);  
}

/** @brief set the vehicles motion wrapper function
  * @param motors_set1: motors handle
  * @param motors_set2: motors handle
  * @param dir: The desired motion
  * @retval none
 */
void set_vehicle_motion(motors * motors_set1, motors * motors_set2, vehicle_direction dir)
{
//  // Check current vehicle motion direction
//  if(dir == vehicle_motion_dir)
//  {
//    // do nothing and return
//    return;
//  }
//  else
//  {
//    // halt before switching motion
//    halt_vehicle(motors_set1,motors_set2);
//  }
//  
  switch(dir)
  {
    case HALT:
      // do nothing since it halts above
      halt_vehicle(motors_set1,motors_set2);
      break;
    case MOVE_FORWARD:
      forward_vehicle(motors_set1,motors_set2);
      break;
    case MOVE_REVERSE:
      reverse_vehicle(motors_set1,motors_set2);
      break;
    case ROTATE_RIGHT:
      // NOT IMPLEMENTED YET
      break;
    case ROTATE_LEFT:
      // not implemented yet
      break;
    default:
      halt_vehicle(motors_set1, motors_set2); 
  }
}

/* USER CODE END Application */
