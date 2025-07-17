#pragma once
#ifndef SPEEDCONTROLLER_H
#define SPEEDCONTROLLER_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define WHEELS_COUNT 3
#define WHEEL_A_INDEX 0
#define WHEEL_B_INDEX 1
#define WHEEL_C_INDEX 2

// Canais do ADC e PWM para as rodas
//Roda 1:
#define WHEEL_A_ADC_CHANNEL ADC_1_IN_4
#define WHEEL_A_PWM_CHANNEL_A TIM_1_CHANNEL_1
#define WHEEL_A_PWM_CHANNEL_B TIM_1_CHANNEL_1N
#define WHEEL_A_ENCODER_CHANNEL_A TIM_5_CHANNEL_1
#define WHEEL_A_ENCODER_CHANNEL_B TIM_5_CHANNEL_2
#define WHEEL_C_ENCODER_TIM htim1

//Roda 2:
#define WHEEL_B_ADC_CHANNEL ADC_1_IN_5
#define WHEEL_B_PWM_CHANNEL_A TIM_1_CHANNEL_2
#define WHEEL_B_PWM_CHANNEL_B TIM_1_CHANNEL_2N
#define WHEEL_B_ENCODER_CHANNEL_A TIM_3_CHANNEL_1
#define WHEEL_B_ENCODER_CHANNEL_B TIM_3_CHANNEL_2
#define WHEEL_C_ENCODER_TIM htim2

//Roda 3:
#define WHEEL_C_ADC_CHANNEL ADC_1_IN_8
#define WHEEL_C_PWM_CHANNEL_A TIM_1_CHANNEL_3
#define WHEEL_C_PWM_CHANNEL_B TIM_1_CHANNEL_3N
#define WHEEL_C_ENCODER_CHANNEL_A TIM_4_CHANNEL_1
#define WHEEL_C_ENCODER_CHANNEL_B TIM_4_CHANNEL_2
#define WHEEL_C_ENCODER_TIM htim4 
// Definições de constantes para as Tasks
#define TORQUE_CONTROLLER_TASK_PRIORITY 5
#define TORQUE_CONTROLLER_TASK_STACK_SIZE 2048
#define TORQUE_CONTROLLER_TASK_MS 20
#define SPEED_CONTROLLER_TASK_PRIORITY TORQUE_CONTROLLER_TASK_PRIORITY - 1
#define SPEED_CONTROLLER_TASK_STACK_SIZE 2048
#define SPEED_CONTROLLER_TASK_MS 5

// Definições de constantes
#define QUEUE_LENGTH 4

// --- Parâmetros físicos ---
#define WHEEL_RADIUS 0.05f
#define ROBOT_RADIUS 0.15f




typedef struct {
    float x;
    float y;
    float phi;
} Vect3;

typedef struct {
    uint8_t pwm_channel_a;
    uint8_t pwm_channel_b;
    uint8_t adc_channel;
    TIM_HandleTypeDef *encoder_tim;
    uint32_t encoder_channel_a;
    uint32_t encoder_channel_b;

} WheelInfo;

typedef struct {
    float w1;
    float w2;
    float w3;
} WheelSpeeds;


#define MAX_RPM 360
#define MAX_PWM 255
#define abs(x) ((x) < 0 ? -(x) : (x))
#define SpeedType_t Vect3
#define WheelSpeeds_t WheelSpeeds

void SpeedController_Init(void);

/*
    Expõe os Handles das Tasks de controle de velocidade e corrente
    para que possam ser acessados por outras partes do código. (Caso precise)
*/

/// @brief Handle da tarefa de controle de velocidade
extern xTaskHandle_t speedControlTaskHandle;
/// @brief Handle da tarefa de controle de corrente
extern xTaskHandle_t currentControlTaskHandle;
/* 
    Fn para obter velocidades e correntes das rodas
    caso Outras Tasks precisem acessar as velocidades e correntes das rodas.
    IE. DataLogger, Telemetria, etc.
*/
SpeedType_t GetWheelSpeeds(uint8_t wheelIndex);
WheelSpeeds_t GetWheelCurrents(uint8_t wheelIndex);

#endif // SPEEDCONTROLLER_H
