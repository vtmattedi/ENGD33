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
// Roda 1:
#define WHEEL_A_ADC_CHANNEL ADC_1_IN_4
#define WHEEL_A_PWM_CHANNEL_A TIM_1_CHANNEL_1
#define WHEEL_A_PWM_CHANNEL_B TIM_1_CHANNEL_1N
#define WHEEL_A_ENCODER_CHANNEL_A TIM_5_CHANNEL_1
#define WHEEL_A_ENCODER_CHANNEL_B TIM_5_CHANNEL_2
#define WHEEL_C_ENCODER_TIM htim1

// Roda 2:
#define WHEEL_B_ADC_CHANNEL ADC_1_IN_5
#define WHEEL_B_PWM_CHANNEL_A TIM_1_CHANNEL_2
#define WHEEL_B_PWM_CHANNEL_B TIM_1_CHANNEL_2N
#define WHEEL_B_ENCODER_CHANNEL_A TIM_3_CHANNEL_1
#define WHEEL_B_ENCODER_CHANNEL_B TIM_3_CHANNEL_2
#define WHEEL_C_ENCODER_TIM htim2

// Roda 3:
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

// -- Definições para os PIDS
#define Kp 0
#define Ki 1
#define Kd 2
#define PID_CONSTANTS_SPEED {0.1f, 0.01f, 0.001f} // Kp, Ki, Kd para velocidade
#define PID_CONSTANTS_CURRENT {0.1f, 0.01f, 0.001f} // Kp, Ki, Kd para corrente
// Uso: PID_CONSTANTS_SPEED[Kp] = Kp para velocidade
// Uso: PID_CONSTANTS_CURRENT[Kd] = Kd para corrente
/*
    PID_CONSTANTS_SPEED_A = {0.1f, 0.01f, 0.001f} // Kp, Ki, Kd para velocidade da roda A
    PID_CONSTANTS_SPEED_B = {0.1f, 0.01f, 0.001f} // Kp, Ki, Kd para velocidade da roda B
    PID_CONSTANTS_SPEED_C = {0.1f, 0.01f, 0.001f} // Kp, Ki, Kd para velocidade da roda C

    PID_CONSTANTS_SPEED = {PID_CONSTANTS_SPEED_A, PID_CONSTANTS_SPEED_B, PID_CONSTANTS_SPEED_C}

    PID_CONSTANTS_CURRENT_A = {0.1f, 0.01f, 0.001f} // Kp, Ki, Kd para corrente da roda A
    PID_CONSTANTS_CURRENT_B = {0.1f, 0.01f, 0.001f} // Kp, Ki, Kd para corrente da roda B
    PID_CONSTANTS_CURRENT_C = {0.1f, 0.01f, 0.001f} // Kp, Ki, Kd para corrente da roda C
    
    PID_CONSTANTS_CURRENT = {PID_CONSTANTS_CURRENT_A, PID_CONSTANTS_CURRENT_B, PID_CONSTANTS_CURRENT_C}

    //Uso: PID_CONSTANTS_CURRENT[WHEEL_A_INDEX][Kp] = Kp para corrente da roda A
    //Uso: PID_CONSTANTS_SPEED[WHEEL_B_INDEX][Kd] = Kd para velocidade da roda B
    //Uso: PID_CONSTANTS_CURRENT[WHEEL_C_INDEX][Ki] = Ki para corrente da roda C
*/

// Definições de constantes
#define QUEUE_LENGTH 4

// --- Parâmetros físicos ---
#define WHEEL_RADIUS 0.05f
#define ROBOT_RADIUS 0.15f

/// @brief Vector 3D para representar a localização e direção do robô.
/// @note x: posição X, y: posição Y, phi: ângulo de orientação
/// Pode ser usada como um vetor float [3].
typedef struct
{
    float x;
    float y;
    float phi;
    operator[uint8_t index]
    {
        if (index < 0 || index > 2)
        {
            __THROW std::out_of_range("Index out of range");
        }
        return (index == 0) ? x : (index == 1) ? y
                                               : phi;
    }
} Vect3;

/// @brief Estrutura para armazenar informações de cada roda.
typedef struct
{
    uint8_t pwm_channel_a;
    uint8_t pwm_channel_b;
    uint8_t adc_channel;
    TIM_HandleTypeDef *encoder_tim;
    uint32_t encoder_channel_a;
    uint32_t encoder_channel_b;

} WheelInfo;

/// @brief Estrutura para armazenar as velocidades das rodas.
typedef struct
{
    float w1;
    float w2;
    float w3;
} WheelSpeeds;

#define MAX_RPM 360
#define MAX_PWM 255
#define abs(x) ((x) < 0 ? -(x) : (x))
/* 
    Define o tipo de dados para as velocidades das rodas e correntes
    cada tipo deve conter os 3 valores de velocidade ou corrente
    e deve poder ser indexado como um vetor de 3 elementos
    Ver os indexes WHEEL_X_INDEX.
*/

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

SpeedType_t GetWheelSpeeds();
WheelSpeeds_t GetWheelCurrents();

#endif // SPEEDCONTROLLER_H
