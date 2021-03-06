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
#include "usart.h"
#include <stdbool.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// led system status delay times
#define NORMAL_LED_DELAY_TIME   2000
#define FAULT_LED_DELAY_TIME    1000
#define AI_LED_DELAY_TIME       1000

// definition of vehicles speeds in meters/sec
// *NOTE!! Ensure these values < max speed
#define SLOW_VEHICLE_SPEED      6
#define MEDIUM_VEHICLE_SPEED    8
#define FAST_VEHICLE_SPEED      10

// stuff for photoelectric encoder sensors
#define TIMCLOCK                84000000
#define PSCALAR                 0
#define numval                  10
#define SENSOR_SAMPLING_TIME    50

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MIN(a,b) ((a) < (b) ? (a) : (b))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
bool front_sensor_triggered = false;
bool fall_sensor_triggered = false;
bool left_sensor_triggered = false;
bool right_sensor_triggered = false;
bool AI_mutex = false;
bool halt_AI = false;
sys_state system_status = NORMAL;
int veh_direction = 0;

vehicle_direction vehicle_motion_dir = HALT;
obj_detection_state obj_detect_state = IDLE;
motors leftmotors;
motors rightmotors;

command_HandleTypeDef command;
command_HandleTypeDef * ptcommand;
command_HandleTypeDef AIcommand;

uint16_t default_rot_pulse      = 15999;
uint16_t rot_time               = 400;

float wheel_radius              = 0.033;

// DMA and rpm sensor stuff
uint8_t rightriseCaptured = 0;
uint8_t leftriseCaptured = 0;
uint32_t riseData[numval];
uint32_t left_riseData[numval];
float right_frequency = 0;
float left_frequency = 0;
float right_encoder_risetime = 0;
float left_encoder_risetime = 0;
int8_t isMeasured = 0;

// PID stuff
float MEA_VAL_rightmotors = 0.0;
float MEA_VAL_leftmotors = 0.0;
bool message_sent = false;

// VC stuff
bool VCmessage_received = false;
uint8_t data_receive[12];

osThreadId ir_sensor_TaskHandle;
osThreadId ir_sensor_state_TaskHandle;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId vehicleMotionHandle;
osThreadId voiceCommandHandle;
osThreadId vehicleMotion_PHandle;
osThreadId statusLEDSHandle;
osThreadId pidEncodersHandle;
osMessageQId myCommandQueue01Handle;
osMessageQId myCommandQueue02Handle;
osSemaphoreId VCBinarySem01Handle;

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
void set_vehicle_speed(motors * motors_set1, motors * motors_set2, uint16_t pulse);

float PID_controller(float REF_Val, float MEA_Val, float gain_arr[],
                     float * I_error, float MEA_Val_previous, float sample_delta);
float read_RPMSensor(void);
float controller_Saturation(float u, float u_sat[]);
void set_motorDuty(float u, float voltage_range[], motors * motors_set);
void read_EncoderSensors(float * left, float * right);

void leds_flash(led_color color, uint32_t pulses,uint32_t delay);

void ir_sensor_detection(void const * argument);
void ir_sensor_state_machine(void const* argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartVehicleMotionTask(void const * argument);
void StartVoiceCommandTask(void const * argument);
void StartvehicleMotion_PIDControl(void const * argument);
void StartstatusLEDSTask(void const * argument);
void StartpidEncoderTask(void const * argument);

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

  /* TIM2 Channel 2 is set to rising edge, so it will store the data in 'riseData' */
  HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_2, riseData, numval);
	
  /* TIM2 Channel 4 is set to rising edge, so it will store the data in 'left_riseData' */
  HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_4, left_riseData, numval);

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of VCBinarySem01 */
  osSemaphoreDef(VCBinarySem01);
  VCBinarySem01Handle = osSemaphoreCreate(osSemaphore(VCBinarySem01), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of myCommandQueue01 */
  osMessageQDef(myCommandQueue01, 2, uint32_t);
  myCommandQueue01Handle = osMessageCreate(osMessageQ(myCommandQueue01), NULL);

  /* definition and creation of myCommandQueue02 */
  osMessageQDef(myCommandQueue02, 2, uint32_t);
  myCommandQueue02Handle = osMessageCreate(osMessageQ(myCommandQueue02), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  osThreadDef(IR_sensor_checking, ir_sensor_detection, osPriorityNormal, 0, 128);
  ir_sensor_TaskHandle = osThreadCreate(osThread(IR_sensor_checking),NULL);
  osThreadDef(IR_sensor_state_changes, ir_sensor_state_machine, osPriorityNormal, 0, 128);
  ir_sensor_TaskHandle = osThreadCreate(osThread(IR_sensor_state_changes),NULL);
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of vehicleMotion */
  osThreadDef(vehicleMotion, StartVehicleMotionTask, osPriorityAboveNormal, 0, 128);
  vehicleMotionHandle = osThreadCreate(osThread(vehicleMotion), NULL);

  /* definition and creation of voiceCommand */
  osThreadDef(voiceCommand, StartVoiceCommandTask, osPriorityAboveNormal, 0, 128);
  voiceCommandHandle = osThreadCreate(osThread(voiceCommand), NULL);

  /* definition and creation of vehicleMotion_P */
  osThreadDef(vehicleMotion_P, StartvehicleMotion_PIDControl, osPriorityAboveNormal, 0, 256);
  vehicleMotion_PHandle = osThreadCreate(osThread(vehicleMotion_P), NULL);

  /* definition and creation of statusLEDS */
  osThreadDef(statusLEDS, StartstatusLEDSTask, osPriorityBelowNormal, 0, 128);
  statusLEDSHandle = osThreadCreate(osThread(statusLEDS), NULL);

  /* definition and creation of pidEncoders */
  osThreadDef(pidEncoders, StartpidEncoderTask, osPriorityIdle, 0, 128);
  pidEncodersHandle = osThreadCreate(osThread(pidEncoders), NULL);

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

/* USER CODE BEGIN Header_StartVehicleMotionTask */
/**
* @brief Function implementing the vehicleMotion thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartVehicleMotionTask */
void StartVehicleMotionTask(void const * argument)
{
  /* USER CODE BEGIN StartVehicleMotionTask */
  /* Infinite loop */
  
  osEvent message_event;
  command_HandleTypeDef * ptcommand_message;
  vehicle_direction command_veh_direction;

  for(;;)
  {
    
    // task only executes if message is received 
    message_event = osMessageGet(myCommandQueue01Handle,osWaitForever);
    if(message_event.status == osEventMessage){
      // message received is pointer to struct not struct itself
      ptcommand_message = (command_HandleTypeDef *)message_event.value.v;
      
      command_veh_direction = ptcommand_message->command_direction;
      
      // speed and distance commands only affect forward/reverse motion
      if((command_veh_direction == MOVE_FORWARD) || \
        (command_veh_direction == MOVE_REVERSE))
      {
        // set speed to zero so that PID control starts from zero 
        set_vehicle_speed(&leftmotors,&rightmotors,0);
        
        // send speed/distance commands to task
        osMessagePut(myCommandQueue02Handle,message_event.value.v,10);
        message_sent = true;
      }
      
      set_vehicle_motion(&leftmotors,&rightmotors, command_veh_direction);
      
    }
    
    
    osDelay(1);
  }
  /* USER CODE END StartVehicleMotionTask */
}

/* USER CODE BEGIN Header_StartVoiceCommandTask */
/**
* @brief Function implementing the voiceCommand thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartVoiceCommandTask */
void StartVoiceCommandTask(void const * argument)
{
  /* USER CODE BEGIN StartVoiceCommandTask */
  /* Infinite loop */

  
  // receive first VC message. the rest are received from UART callback
  if(HAL_UART_Receive_IT(&huart2,data_receive,sizeof(data_receive)/sizeof(data_receive[0])) == HAL_OK){
    VCmessage_received = true;
  }
  
  for(;;)
  {

    if(VCmessage_received)
    {

      switch(system_status){
      case NORMAL:
        // decode bluetooth/UART message into command type struct 
        ptcommand = (command_HandleTypeDef *)data_receive;

        command.command_direction       = ptcommand->command_direction;
        command.command_distance        = ptcommand->command_distance;
        command.command_speed           = ptcommand->command_speed;        
        
        // send pointer to struct not struct itself
        osMessagePut(myCommandQueue01Handle, (uint32_t)ptcommand, 10); 
        
        // indicate VC successfully received
        leds_flash(GREEN,2,100);

        break;
      case AI:
        // decode message into struct 
        ptcommand = (command_HandleTypeDef *)data_receive;
        command.command_direction       = ptcommand->command_direction;
        command.command_distance        = ptcommand->command_distance;
        command.command_speed           = ptcommand->command_speed;        
          
        // send pointer to struct not struct itself
        if(command.command_direction == HALT && AI_mutex== false)
        {
          AI_mutex = true;
          osMessagePut(myCommandQueue01Handle, (uint32_t)ptcommand, 10);
          system_status = NORMAL;
          obj_detect_state = IDLE;
          veh_direction = 0;
          AI_mutex = false;
        }
        break;
      case FAULT:
        leds_flash(RED,3,100);
        break;
      default:
        leds_flash(RED,3,100);
        break;
      }
      
      VCmessage_received = false;
    }
    else {
      // do nothing. no message received
    }
    
    osDelay(10);
  }
  /* USER CODE END StartVoiceCommandTask */
}

/* USER CODE BEGIN Header_StartvehicleMotion_PIDControl */
/**
* @brief Function implementing the vehicleMotion_P thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartvehicleMotion_PIDControl */
void StartvehicleMotion_PIDControl(void const * argument)
{
  /* USER CODE BEGIN StartvehicleMotion_PIDControl */
  /* Infinite loop */
  
  // Queue message and command stuff
  osEvent message_event;
  command_HandleTypeDef * ptcommand_message;
  float loc_command_distance;
  vehicle_speed loc_command_speed;

  // PID Control and distance tracking stuff
  bool track_distance = false;
  float linear_velocity = 0.0;
  float REF_VAL = (float)SLOW_VEHICLE_SPEED;    // desired vehicle speed
  float PREVIOUS_REF_VAL = REF_VAL;  
  float distance_travelled = 0.0;
  float gain_array[] = {1.0,10.0,0.0,2.5};       // PID gains [Kff, Kp, Ki, Kd]
  float delta_t = (float)SENSOR_SAMPLING_TIME;  // sensor sampling rate in ms
  float sat_limits[] = {0.0,12.0};              // min and max voltages to motors
  float I_error_leftmotors = 0.0;               // integral error
  float I_error_rightmotors = 0.0;
  float MEA_VAL_previous_leftmotors = 0.0;      // previous measured rotation speed
  float MEA_VAL_previous_rightmotors = 0.0;
  float pid_val_leftmotors;                     // total PID value
  float pid_val_rightmotors;
  float control_input_leftmotors = 0.0;         // signal to motor driver board
  float control_input_rightmotors = 0.0;
  
  for(;;)
  {
  
    if(message_sent){
      message_event = osMessageGet(myCommandQueue02Handle,osWaitForever);
      if(message_event.status == osEventMessage){
        // everytime we get a new command we re-iniatilize everything. 
        // the system is currently set up to receive a new command at any point
        // in time even when it hasnt finished executing the previous command
        
        // get pointer message of command struct
        ptcommand_message = (command_HandleTypeDef *)message_event.value.v;
        
        loc_command_distance = ptcommand_message->command_distance;
        loc_command_speed = ptcommand_message->command_speed;
        //convert command distance to units of meters
        loc_command_distance = loc_command_distance / 3.281;
        
        distance_travelled = 0.0;
        // command distance default is 0.0 if not specified by VC
        if(loc_command_distance != 0.0){
          track_distance = true;
        }
        else {
          track_distance = false;
        }

        I_error_leftmotors = 0.0;
        I_error_rightmotors = 0.0;
        MEA_VAL_leftmotors = 0.0;
        MEA_VAL_rightmotors = 0.0;
        MEA_VAL_previous_leftmotors = 0.0;
        MEA_VAL_previous_rightmotors = 0.0;
        
        switch(loc_command_speed){
        case INVALID_SPEED:
          REF_VAL = PREVIOUS_REF_VAL;
          break;
        case SLOW:
          REF_VAL = SLOW_VEHICLE_SPEED;
          PREVIOUS_REF_VAL = REF_VAL;
          break;
        case MEDIUM:
          REF_VAL = MEDIUM_VEHICLE_SPEED;
          PREVIOUS_REF_VAL = REF_VAL;
          break;
        case FAST:
          REF_VAL = FAST_VEHICLE_SPEED;
          PREVIOUS_REF_VAL = REF_VAL;
          break;
        case NO_UPDATE:
          REF_VAL = PREVIOUS_REF_VAL;
        default:
          REF_VAL = PREVIOUS_REF_VAL;
        } 
      }
      else if((message_event.status == osErrorValue) || (message_event.status == osErrorOS))
      {
        // some sort of error handling
        set_vehicle_motion(&leftmotors,&rightmotors,HALT);
        system_status = FAULT;
        leds_flash(RED,3,50);
      }

      message_sent = false;
    }
    
    if(((command.command_direction == MOVE_FORWARD) || (command.command_direction == \
      MOVE_REVERSE)) && (system_status != FAULT)){

      // do PID control stuff here where command vehicle speed is input
      // ** left encoder sensor currently not working. **
      
//      pid_val_leftmotors = PID_controller(REF_VAL,MEA_VAL_leftmotors,gain_array,\
//        &I_error_leftmotors,MEA_VAL_previous_leftmotors,delta_t);
      pid_val_rightmotors = PID_controller(REF_VAL,MEA_VAL_rightmotors,gain_array,\
        &I_error_rightmotors,MEA_VAL_previous_rightmotors,delta_t);
      
      MEA_VAL_previous_rightmotors = MEA_VAL_rightmotors;
//      MEA_VAL_previous_leftmotors = MEA_VAL_leftmotors;
      
//      control_input_leftmotors = REF_VAL * gain_array[0] + pid_val_leftmotors;
      control_input_rightmotors = REF_VAL * gain_array[0] + pid_val_rightmotors;
      
//      control_input_leftmotors = controller_Saturation(control_input_leftmotors,sat_limits);
      control_input_rightmotors = controller_Saturation(control_input_rightmotors,sat_limits);
      
//      set_motorDuty(control_input_leftmotors,sat_limits,&leftmotors);
      set_motorDuty(control_input_rightmotors,sat_limits,&leftmotors);
      set_motorDuty(control_input_rightmotors,sat_limits,&rightmotors);
    }


    if(track_distance){
      // just go off one RPM sensor reading
      linear_velocity = MEA_VAL_rightmotors * wheel_radius;
      distance_travelled = distance_travelled + (linear_velocity * (delta_t/1000));
      
      if(distance_travelled >= loc_command_distance){
        set_vehicle_motion(&leftmotors,&rightmotors,HALT);
        track_distance = false;
      }

    }
    
    osDelay(SENSOR_SAMPLING_TIME);
  }
  /* USER CODE END StartvehicleMotion_PIDControl */
}

/* USER CODE BEGIN Header_StartstatusLEDSTask */
/**
* @brief Function implementing the statusLEDS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartstatusLEDSTask */
void StartstatusLEDSTask(void const * argument)
{
  /* USER CODE BEGIN StartstatusLEDSTask */
  /* Infinite loop */

  for(;;)
  {
    
    switch(system_status){
    case FAULT:
      leds_flash(RED,1,FAULT_LED_DELAY_TIME);
      break;
    case NORMAL:
      leds_flash(GREEN,1,NORMAL_LED_DELAY_TIME);
      break;
    case AI:
      leds_flash(ORANGE,1,AI_LED_DELAY_TIME);
      break;
    default:
      leds_flash(RED,1,FAULT_LED_DELAY_TIME);
      break;
    }
    
    // Actual osDelay specified by LED delay times
    osDelay(1);
  }
  /* USER CODE END StartstatusLEDSTask */
}

/* USER CODE BEGIN Header_StartpidEncoderTask */
/**
* @brief Function implementing the pidEncoders thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartpidEncoderTask */
void StartpidEncoderTask(void const * argument)
{
  /* USER CODE BEGIN StartpidEncoderTask */
  /* Infinite loop */
  
  for(;;)
  {
    if (isMeasured)
    {
      TIM2->CNT = 0;

      HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_2, riseData, numval);

      HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_4, left_riseData, numval);

      isMeasured = 0;

      read_EncoderSensors(&MEA_VAL_leftmotors,&MEA_VAL_rightmotors);
    }
    
    osDelay(SENSOR_SAMPLING_TIME);
  }
  /* USER CODE END StartpidEncoderTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void ir_sensor_detection(void const * argument)
{
  for(;;)
  {
    fall_sensor_triggered = HAL_GPIO_ReadPin(IR_DOWN_GPIO_Port, IR_DOWN_Pin);
    front_sensor_triggered = fall_sensor_triggered
                            | (HAL_GPIO_ReadPin(IR_FRONT_GPIO_Port, IR_FRONT_Pin) == false)
                            | (HAL_GPIO_ReadPin(IR_FRONT_L_GPIO_Port, IR_FRONT_L_Pin) == false)   
                            | (HAL_GPIO_ReadPin(IR_FRONT_R_GPIO_Port, IR_FRONT_R_Pin) == false);
    left_sensor_triggered = (HAL_GPIO_ReadPin(IR_LEFT_F_GPIO_Port, IR_LEFT_F_Pin) == false) 
                            | (HAL_GPIO_ReadPin(IR_LEFT_B_GPIO_Port, IR_LEFT_B_Pin) == false);
    right_sensor_triggered = (HAL_GPIO_ReadPin(IR_RIGHT_F_GPIO_Port, IR_RIGHT_F_Pin) == false) 
                            | (HAL_GPIO_ReadPin(IR_RIGHT_B_GPIO_Port, IR_RIGHT_B_Pin) == false);
    
    osDelay(10);
  }
}

void ir_sensor_state_machine(void const * argument)
{
  for(;;)
  {
    // if we are rotating then skip the object detection state until we aren't rotating
    if(AI_mutex == false)
    {
      AI_mutex = true;
      switch(obj_detect_state)
      {
        case IDLE:
          veh_direction = 0;
          if(front_sensor_triggered && (leftmotors.motors_dir != HALTED && rightmotors.motors_dir != HALTED))
          {
            system_status = AI;
            if(fall_sensor_triggered)
            {
              set_vehicle_motion(&leftmotors,&rightmotors,HALT);
            } else {
              obj_detect_state = OBJECT_AHEAD;
            }
          } else 
          {
            system_status = NORMAL;
          }
          AI_mutex = false;
          osDelay(10);
          break;
        case OBJECT_AHEAD:
          // HALT
          set_vehicle_motion(&leftmotors,&rightmotors,HALT);
          // Currently just move back to IDLE state
          if(front_sensor_triggered == false)
          {
            obj_detect_state = IDLE;
          } else 
          {
            //keep previous rotation time for 90 to move back to
            uint16_t rot_time_prev = rot_time; 
            rot_time = 200; //set small rotation time
            if(right_sensor_triggered)
            {
              // ROTATE LEFT
              veh_direction -= 1;
              set_vehicle_motion(&leftmotors,&rightmotors,ROTATE_LEFT);
            } else
            {
              // ROTATE RIGHT
              veh_direction += 1;
              set_vehicle_motion(&leftmotors,&rightmotors,ROTATE_RIGHT);
            }
            rot_time = rot_time_prev;
            obj_detect_state = OBJECT_SIDE;
          }
          AI_mutex = false;
          osDelay(300);
          break;
        case OBJECT_SIDE:
          if(front_sensor_triggered)
          {
            obj_detect_state = OBJECT_AHEAD;
            AI_mutex = false;
            osDelay(10);
          } else {
            set_vehicle_motion(&leftmotors,&rightmotors,MOVE_FORWARD);
            AI_mutex = false;
            osDelay(1000); //go forward for 1 seconds
            obj_detect_state = ORIENT_VEH;
          }
          break;
        case ORIENT_VEH:
          uint16_t rot_time_prev = rot_time; 
          rot_time = 200; //set small rotation time
          // if we are pretty close to straight call it good
          if(veh_direction == 0)
          {
            obj_detect_state = IDLE;
          } else if(veh_direction < 0)
          {
            veh_direction += 1;
            set_vehicle_motion(&leftmotors,&rightmotors,ROTATE_RIGHT);
            obj_detect_state = OBJECT_SIDE;
          } else
          {
            veh_direction -= 1;
            set_vehicle_motion(&leftmotors,&rightmotors,ROTATE_LEFT);
            obj_detect_state = OBJECT_SIDE;
          }
          rot_time = rot_time_prev;
          AI_mutex = false;
          osDelay(300);
          break;
        default:
          AI_mutex = false;
          break;


      }
    }
  }
}



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
  // Set both sets of motors to halt
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
  // Save previous motor speed first to restore after rotating
  uint16_t prev_motor_speed1 = motors_set1->pulse;
  uint16_t prev_motor_speed2 = motors_set2->pulse;
  
  set_motor_speed(motors_set1,default_rot_pulse);
  set_motor_speed(motors_set2,default_rot_pulse);
  
  // Set sets of motors to opposite directions
  set_motors_direction(motors_set1,REVERSE);
  set_motors_direction(motors_set2,FORWARD);  
  
  // Run for set time resulting in 90 deg turn
  osDelay(rot_time);
  
  // Halt motors
  set_motors_direction(motors_set1,HALTED);
  set_motors_direction(motors_set2,HALTED);
  
  // Restore initial motor speed
  set_motor_speed(motors_set1,prev_motor_speed1);
  set_motor_speed(motors_set2,prev_motor_speed2);
}

/** @brief set the vehicles motion to rotate ("steering")
  * @param motors_set1: motors handle
  * @param motors_set2: motors handle
  * @retval none
 */
void rotate_left_vehicle(motors * motors_set1, motors * motors_set2)
{
  // Save previous motor speed first to restore after rotating
  uint16_t prev_motor_speed1 = motors_set1->pulse;
  uint16_t prev_motor_speed2 = motors_set2->pulse;
  
  set_motor_speed(motors_set1,default_rot_pulse);
  set_motor_speed(motors_set2,default_rot_pulse);
  
  // Set sets of motors to opposite directions
  set_motors_direction(motors_set1,FORWARD);
  set_motors_direction(motors_set2,REVERSE);  
  
  // Run for set time resulting in 90 deg turn
  osDelay(rot_time);
  
  // Halt motors
  set_motors_direction(motors_set1,HALTED);
  set_motors_direction(motors_set2,HALTED);
  
  // Restore initial motor speed
  set_motor_speed(motors_set1,prev_motor_speed1);
  set_motor_speed(motors_set2,prev_motor_speed2);
}

/** @brief set the vehicles motion wrapper function
  * @param motors_set1: motors handle
  * @param motors_set2: motors handle
  * @param dir: The desired motion
  * @retval none
  * @note rotate commands have a default speed already set. However, if using 
  *     other commands like forward or reverse then its necessary to assign 
  *     the speed either directly with set_vehicle_speed() or with the enums 
  *     type 'vehicle_speed' which will go through a PID control algorithm
 */
void set_vehicle_motion(motors * motors_set1, motors * motors_set2, vehicle_direction dir)
{

  switch(dir)
  {
    case INVALID_COMMAND:
      // do nothing
      break;
    case MOVE_FORWARD:
      forward_vehicle(motors_set1,motors_set2);
      break;
    case MOVE_REVERSE:
      reverse_vehicle(motors_set1,motors_set2);
      break;
    case ROTATE_RIGHT:
      rotate_right_vehicle(motors_set1,motors_set2);
      break;
    case ROTATE_LEFT:
      rotate_left_vehicle(motors_set1,motors_set2);
      break;
    case U_TURN:
      rotate_right_vehicle(motors_set1,motors_set2);
      rotate_right_vehicle(motors_set1,motors_set2);
      break;
    case HALT:
      halt_vehicle(motors_set1,motors_set2);
      break;    
    default:
      halt_vehicle(motors_set1, motors_set2); 
  }
}


/** 
  * @brief Sets the vehicle speed with the PWM pulse value for use with PID control
  * @param motors_set1: handle of left motor set struct
  * @param motors_set2: handle of right motor set struct
  * @param pulse: new pulse value to assign to pwm timer compare counter register
  *             which must be <= timer period
 */
void set_vehicle_speed(motors * motors_set1, motors * motors_set2, uint16_t pulse)
{
  set_motor_speed(motors_set1, pulse);
  set_motor_speed(motors_set2, pulse);
}


/************* CONTROL UTILITIES ***********************************************/

/** Computes the PID values with the supplied gain values
  * @param REF_Val; the setpoint aka desired value of the system in rad/sec
  * @param MEA_Val; the sample (measured) value from a sensor in rad/sec
  * @param gain_arr[]; an array that hold the gain values in order Kff,P,I,D
  * @param sample_delta; the sampling rate of the sensor
  * @retval a PID value to adjust the control input
 */
float PID_controller(float REF_Val, float MEA_Val, float gain_arr[],
                     float * I_error, float MEA_Val_previous, float sample_delta)
{
  float error = REF_Val - MEA_Val;
  
  float P = gain_arr[1] * error;
  float I = gain_arr[2] * ((*I_error) + (error * sample_delta));
  float D = gain_arr[3] * (MEA_Val - (MEA_Val_previous)) / sample_delta;
  
  *I_error = I;
  
  return P + I - D;
}

/** A saturation filter for the controller. This is meant to ensure that 
  * the control input value does not surpass some sort of user defined limit. In this case,
  * the limit is a physical limit on the voltage output of the motor driver board.
  * @param u; control input value to be filtered
  * @param u_sat[]; an array of 1x2 holding the upper/lower limits
  * @retval the filtered control input value
*/
float controller_Saturation(float u, float u_sat[])
{
  (u < u_sat[0]) ? (u = 0.0) : \
    ((u > u_sat[1]) ? (u = u_sat[1]) : (u = u));
  return u;
}

/** Compute the duty for the PWM signal to the motor driver board
  * @param u; control input
  * @param voltage_range[]; 1x2 array with [MIN,MAX] voltage output of motor driver board
  *     -Note- MAX val must be >= u_sat(max)
  * @param motors_set: pointer to struct of type motors
  * @retval none
*/
void set_motorDuty(float u, float voltage_range[], motors * motors_set)
{
  /* Compute duty cycle using control input signal and 
      linear relationship of driver board voltage range output */
  uint16_t duty = (uint16_t)((u * 100) / (voltage_range[1] - voltage_range[0]));
  
  /* Compute pulse value */
  uint32_t tim_period = motors_set->htim->Init.Period;
  uint32_t pulse = (((tim_period + 1) * duty) / 100) - 1;
  
  set_motor_speed(motors_set,pulse);
}



/** Sets up the post processing of data obtained from Input Capture and DMA
  * @param htim: timer handle which is set up for input capture
  * @note this function is not explicitly called by user. its called automatically
  *     by the DMA after it has stored the data in memory
  * @note reference:
  * https://github.com/controllerstech/STM32/blob/master/INPUT%20CAPTURE%20DMA/F446/main.c
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

  // If the Interrupt is triggered by 2nd Channel
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
    rightriseCaptured = 1;
  }

  // If the Interrupt is triggered by 4th Channel
//  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4){
//     leftriseCaptured = 1;
//  }


  /* Rest of the calculations will be done,
   * once both the DMAs have finished capturing enough data */
  if(rightriseCaptured)
  {
    // calculate the reference clock
    float refClock = TIMCLOCK/(PSCALAR+1);

    uint16_t indxr = 0;

    uint16_t countr = 0;

    float rightriseavg = 0.0;
    

    /* start adding the values to the riseavg */
    while (indxr < (numval-1))
    {
      
      if( (riseData[indxr] > riseData[indxr+1]) && (indxr <= (numval-3))){
        indxr += 2;
      }
      
      rightriseavg += (riseData[indxr+1] - riseData[indxr]);
      countr++;
      indxr += 2;
    }
    
    /* Find the average riseavg, the average time between the rising edges */
    rightriseavg = rightriseavg/countr;
    right_encoder_risetime = rightriseavg/refClock;

    /* Calculate Frequency
     * Freq = Clock/(time taken between 2 Rise)
     */
    right_frequency = (refClock/(float)rightriseavg);

    rightriseCaptured = 0;

    isMeasured = 1;
  }
  

  // Left encoder sensor currently not working. Uncomment once fixed.
  
//  if(leftriseCaptured)
//  {
//
//    // calculate the reference clock
//    float refClock = TIMCLOCK/(PSCALAR+1);
//
//    uint16_t indxl = 0;
//
//    uint16_t countl = 0;
//
//    float leftriseavg = 0.0;
//    
//    /* start adding the values to the riseavg */
//    while (indxl < (numval-1))
//    {
//      
//      if( (left_riseData[indxl] > left_riseData[indxl+1]) && (indxl <= (numval-3))){
//        indxl += 2;
//      }
//      
//      leftriseavg += (left_riseData[indxl+1] - left_riseData[indxl]);
//      countl++;
//      indxl += 2;
//    }
//    
//    /* Find the average riseavg, the average time between the rising edges */
//    leftriseavg = leftriseavg/countl;
//    left_encoder_risetime = leftriseavg/refClock;
//
//    /* Calculate Frequency
//     * Freq = Clock/(time taken between 2 Rise)
//     */
//    left_frequency = (refClock/(float)leftriseavg);
//
//    leftriseCaptured = 0;
//
//    isMeasured = 1;
//  }    
}


/** 
  * "Reads" the photoelectric encoder sensors. The actual reading is done 
  * by the input capture timers, DMA transfers, and callback functions. 
  * @param left: pointer to float variable to store angular velocity (rad/sec) of left encoder
  * @param right: pointer to float variable to store angular velocity (rad/sec) of right encoder
  * @retval none
 */
void read_EncoderSensors(float * left, float * right)
{
  uint8_t encoder_windows = 20;
  float twopi_rad = 2 * 3.1416;
  float edge_edge_rad = twopi_rad / encoder_windows;
  
  if((rightmotors.motors_dir != HALTED) && (leftmotors.motors_dir != HALTED) \
    && (right_encoder_risetime < 1000) && (left_encoder_risetime < 1000)){
  //*left = edge_edge_rad / left_encoder_risetime;
  *right = edge_edge_rad / right_encoder_risetime;
  //*right = 0.314159 / (right_encoder_risetime);
  }
  else {
    *left = 0.0;
    *right = 0.0;
  }
}

/************* MISCEL. UTILITIES ***********************************************/



/** 
  * Creates a flashing LED sequence
  * @param color:  type enum
  *     RED
  *     GREEN
  *     ORANGE
  * @param pulses: number of times you want the led to turn on 
  * @param delay: how long to wait before led turns back on in ms
 */
void leds_flash(led_color color, uint32_t pulses, uint32_t delay){
  
  // ON duration of led
  uint16_t led_delay = 100;
  
  switch(color){
  case RED:
    for(uint16_t i = 0;i < pulses; i++){
      HAL_GPIO_WritePin(GPIOD,RED_LED_Pin,GPIO_PIN_SET);
      osDelay(led_delay);
      HAL_GPIO_WritePin(GPIOD,RED_LED_Pin,GPIO_PIN_RESET);
      osDelay(delay);
    }
    break;
  case GREEN:
    for(uint16_t i = 0;i < pulses; i++){
      HAL_GPIO_WritePin(GPIOD,GREEN_LED_Pin,GPIO_PIN_SET);
      osDelay(led_delay);
      HAL_GPIO_WritePin(GPIOD,GREEN_LED_Pin,GPIO_PIN_RESET);
      osDelay(delay);
    }
    break;  
  case ORANGE:
    for(uint16_t i = 0;i < pulses; i++){
      HAL_GPIO_WritePin(GPIOD,ORANGE_LED_Pin,GPIO_PIN_SET);
      osDelay(led_delay);
      HAL_GPIO_WritePin(GPIOD,ORANGE_LED_Pin,GPIO_PIN_RESET);
      osDelay(delay);
    }
    break;
      
  }
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
  if(HAL_UART_Receive_IT(&huart2, data_receive, 12) == HAL_OK){
    VCmessage_received = true;
  }
}

/* USER CODE END Application */
