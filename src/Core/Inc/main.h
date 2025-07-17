/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IMU_DRDY_EXTI13_Pin GPIO_PIN_13
#define IMU_DRDY_EXTI13_GPIO_Port GPIOC
#define IMU_DRDY_EXTI13_EXTI_IRQn EXTI15_10_IRQn
#define Osc_32k768Hz_In_Pin GPIO_PIN_14
#define Osc_32k768Hz_In_GPIO_Port GPIOC
#define Osc_32k768Hz_Out_Pin GPIO_PIN_15
#define Osc_32k768Hz_Out_GPIO_Port GPIOC
#define Osc_25MHz_In_Pin GPIO_PIN_0
#define Osc_25MHz_In_GPIO_Port GPIOH
#define Osc_25MHz_Out_Pin GPIO_PIN_1
#define Osc_25MHz_Out_GPIO_Port GPIOH
#define Encoder_A_Motor1_T5_CH1_Pin GPIO_PIN_0
#define Encoder_A_Motor1_T5_CH1_GPIO_Port GPIOA
#define Encoder_B_Motor1_T5_CH2_Pin GPIO_PIN_1
#define Encoder_B_Motor1_T5_CH2_GPIO_Port GPIOA
#define GPS_UART2_TX_Pin GPIO_PIN_2
#define GPS_UART2_TX_GPIO_Port GPIOA
#define GPS_UART2_RX_Pin GPIO_PIN_3
#define GPS_UART2_RX_GPIO_Port GPIOA
#define Corrente_Motor1_ADC1_IN4_Pin GPIO_PIN_4
#define Corrente_Motor1_ADC1_IN4_GPIO_Port GPIOA
#define Corrente_Motor2_ADC1_IN5_Pin GPIO_PIN_5
#define Corrente_Motor2_ADC1_IN5_GPIO_Port GPIOA
#define ETH_SPI1_MISO_Pin GPIO_PIN_6
#define ETH_SPI1_MISO_GPIO_Port GPIOA
#define ETH_SPI1_MOSI_Pin GPIO_PIN_7
#define ETH_SPI1_MOSI_GPIO_Port GPIOA
#define Corrente_Motor3_ADC1_IN8_Pin GPIO_PIN_0
#define Corrente_Motor3_ADC1_IN8_GPIO_Port GPIOB
#define Monitor_Voltagem_Bateria_Pin GPIO_PIN_1
#define Monitor_Voltagem_Bateria_GPIO_Port GPIOB
#define Relays_Motores_Pin GPIO_PIN_10
#define Relays_Motores_GPIO_Port GPIOB
#define NRF24_CE_Pin GPIO_PIN_12
#define NRF24_CE_GPIO_Port GPIOB
#define PWM_Motor1_T1_CH1N_Pin GPIO_PIN_13
#define PWM_Motor1_T1_CH1N_GPIO_Port GPIOB
#define PWM_Motor2_T1_CH2N_Pin GPIO_PIN_14
#define PWM_Motor2_T1_CH2N_GPIO_Port GPIOB
#define PWM_Motor3_T1_CH2N_Pin GPIO_PIN_15
#define PWM_Motor3_T1_CH2N_GPIO_Port GPIOB
#define PWM_Motor1_T1_CH1_Pin GPIO_PIN_8
#define PWM_Motor1_T1_CH1_GPIO_Port GPIOA
#define PWM_Motor2_T1_CH2_Pin GPIO_PIN_9
#define PWM_Motor2_T1_CH2_GPIO_Port GPIOA
#define PWM_Motor3_T1_CH3_Pin GPIO_PIN_10
#define PWM_Motor3_T1_CH3_GPIO_Port GPIOA
#define IHM_UART6_TX_Pin GPIO_PIN_11
#define IHM_UART6_TX_GPIO_Port GPIOA
#define IHM_UART6_RX_Pin GPIO_PIN_12
#define IHM_UART6_RX_GPIO_Port GPIOA
#define ETH_SPI1_NSS_Pin GPIO_PIN_15
#define ETH_SPI1_NSS_GPIO_Port GPIOA
#define ETH_SPI1_SCK_Pin GPIO_PIN_3
#define ETH_SPI1_SCK_GPIO_Port GPIOB
#define Encoder_A_Motor2_T3_CH1_Pin GPIO_PIN_4
#define Encoder_A_Motor2_T3_CH1_GPIO_Port GPIOB
#define Encoder_B_Motor2_T3_CH2_Pin GPIO_PIN_5
#define Encoder_B_Motor2_T3_CH2_GPIO_Port GPIOB
#define Encoder_A_Motor3_T4_CH1_Pin GPIO_PIN_6
#define Encoder_A_Motor3_T4_CH1_GPIO_Port GPIOB
#define Encoder_B_Motor3_T4_CH2_Pin GPIO_PIN_7
#define Encoder_B_Motor3_T4_CH2_GPIO_Port GPIOB
#define IMU_I2C1_SCL_Pin GPIO_PIN_8
#define IMU_I2C1_SCL_GPIO_Port GPIOB
#define IMU_I2C1_SDA_Pin GPIO_PIN_9
#define IMU_I2C1_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
