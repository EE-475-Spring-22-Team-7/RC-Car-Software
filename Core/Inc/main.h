/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum direction{
  FORWARD,
  REVERSE,
  HALTED
}direction;

typedef enum veh_object_detect_state{
  VEH_FAULT,
  IDLE,
  OBJECT_AHEAD,
  OBJECT_SIDE,
  ORIENT_VEH
} obj_detection_state;

typedef enum object_side{
  NOT_DETECTED,
  FRONT_IR_DETECTED,
  BOTH_IR_DETECTED,
  BACK_IR_DETECTED,
  OBJECT_PASSED
} object_side;

typedef enum veh_direction{
  INVALID_COMMAND,
  MOVE_FORWARD,
  MOVE_REVERSE,
  ROTATE_RIGHT,
  ROTATE_LEFT,
  U_TURN,
  HALT
} vehicle_direction;
  
typedef enum led_color {
  RED,
  GREEN,
  ORANGE
} led_color;

typedef enum vehicle_speed {
  INVALID_SPEED,
  SLOW,
  MEDIUM,
  FAST,
  NO_UPDATE
} vehicle_speed;

typedef struct {
  TIM_HandleTypeDef *htim;
  uint32_t motor_Channel;
  uint16_t pulse;
  direction motors_dir;
  GPIO_TypeDef * motors_GPIOx;
  uint16_t H1_GPIO_Pin;
  uint16_t H2_GPIO_Pin;
} motors;

typedef struct {
  vehicle_direction command_direction;
  float command_distance;
  vehicle_speed command_speed;
} command_HandleTypeDef;

typedef enum sys_state {
  FAULT,
  NORMAL,
  AI
} sys_state;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CS_I2C_SPI_Pin GPIO_PIN_3
#define CS_I2C_SPI_GPIO_Port GPIOE
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define OTG_FS_PowerSwitchOn_Pin GPIO_PIN_0
#define OTG_FS_PowerSwitchOn_GPIO_Port GPIOC
#define PDM_OUT_Pin GPIO_PIN_3
#define PDM_OUT_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define ENCODER_RIGHT_Pin GPIO_PIN_1
#define ENCODER_RIGHT_GPIO_Port GPIOA
#define ZS_040_BLUETOOTH_TX_Pin GPIO_PIN_2
#define ZS_040_BLUETOOTH_TX_GPIO_Port GPIOA
#define ZS_040_BLUETOOTH_RX_Pin GPIO_PIN_3
#define ZS_040_BLUETOOTH_RX_GPIO_Port GPIOA
#define I2S3_WS_Pin GPIO_PIN_4
#define I2S3_WS_GPIO_Port GPIOA
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_MOSI_Pin GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port GPIOA
#define MOTORS_PWM_LEFT_Pin GPIO_PIN_0
#define MOTORS_PWM_LEFT_GPIO_Port GPIOB
#define MOTORS_PWM_RIGHT_Pin GPIO_PIN_1
#define MOTORS_PWM_RIGHT_GPIO_Port GPIOB
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define MOTORS_LEFT_H1_Pin GPIO_PIN_7
#define MOTORS_LEFT_H1_GPIO_Port GPIOE
#define MOTORS_LEFT_H2_Pin GPIO_PIN_8
#define MOTORS_LEFT_H2_GPIO_Port GPIOE
#define MOTORS_RIGHT_H1_Pin GPIO_PIN_12
#define MOTORS_RIGHT_H1_GPIO_Port GPIOE
#define MOTORS_RIGHT_H2_Pin GPIO_PIN_13
#define MOTORS_RIGHT_H2_GPIO_Port GPIOE
#define CLK_IN_Pin GPIO_PIN_10
#define CLK_IN_GPIO_Port GPIOB
#define ENCODER_LEFT_Pin GPIO_PIN_11
#define ENCODER_LEFT_GPIO_Port GPIOB
#define IR_RIGHT_B_Pin GPIO_PIN_12
#define IR_RIGHT_B_GPIO_Port GPIOB
#define IR_RIGHT_F_Pin GPIO_PIN_13
#define IR_RIGHT_F_GPIO_Port GPIOB
#define IR_LEFT_B_Pin GPIO_PIN_14
#define IR_LEFT_B_GPIO_Port GPIOB
#define IR_LEFT_F_Pin GPIO_PIN_15
#define IR_LEFT_F_GPIO_Port GPIOB
#define IR_FRONT_R_Pin GPIO_PIN_8
#define IR_FRONT_R_GPIO_Port GPIOD
#define IR_FRONT_L_Pin GPIO_PIN_9
#define IR_FRONT_L_GPIO_Port GPIOD
#define IR_FRONT_Pin GPIO_PIN_10
#define IR_FRONT_GPIO_Port GPIOD
#define IR_DOWN_Pin GPIO_PIN_11
#define IR_DOWN_GPIO_Port GPIOD
#define LD4_Pin GPIO_PIN_12
#define LD4_GPIO_Port GPIOD
#define LD3_Pin GPIO_PIN_13
#define LD3_GPIO_Port GPIOD
#define LD5_Pin GPIO_PIN_14
#define LD5_GPIO_Port GPIOD
#define LD6_Pin GPIO_PIN_15
#define LD6_GPIO_Port GPIOD
#define I2S3_MCK_Pin GPIO_PIN_7
#define I2S3_MCK_GPIO_Port GPIOC
#define VBUS_FS_Pin GPIO_PIN_9
#define VBUS_FS_GPIO_Port GPIOA
#define OTG_FS_DM_Pin GPIO_PIN_11
#define OTG_FS_DM_GPIO_Port GPIOA
#define OTG_FS_DP_Pin GPIO_PIN_12
#define OTG_FS_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define I2S3_SCK_Pin GPIO_PIN_10
#define I2S3_SCK_GPIO_Port GPIOC
#define I2S3_SD_Pin GPIO_PIN_12
#define I2S3_SD_GPIO_Port GPIOC
#define GREEN_LED_Pin GPIO_PIN_0
#define GREEN_LED_GPIO_Port GPIOD
#define ORANGE_LED_Pin GPIO_PIN_1
#define ORANGE_LED_GPIO_Port GPIOD
#define RED_LED_Pin GPIO_PIN_2
#define RED_LED_GPIO_Port GPIOD
#define Audio_RST_Pin GPIO_PIN_4
#define Audio_RST_GPIO_Port GPIOD
#define OTG_FS_OverCurrent_Pin GPIO_PIN_5
#define OTG_FS_OverCurrent_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Audio_SCL_Pin GPIO_PIN_6
#define Audio_SCL_GPIO_Port GPIOB
#define WhatupsFucker_Pin GPIO_PIN_7
#define WhatupsFucker_GPIO_Port GPIOB
#define Audio_SDA_Pin GPIO_PIN_9
#define Audio_SDA_GPIO_Port GPIOB
#define MEMS_INT2_Pin GPIO_PIN_1
#define MEMS_INT2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
