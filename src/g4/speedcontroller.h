#pragma once

#ifndef SPEEDCONTROLLER_H
#define SPEEDCONTROLLER_H

// TODO: Verificar os includes no projeto CUBE IDE (Final do projeto)
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "stm32f4xx_hal.h"

// Returns -X if X is negative, otherwise returns X
#define abs(x) ((x) < 0 ? -(x) : (x))

// Definições das constantes de Hardware
#pragma region Hardware

#define WHEELS_COUNT 3  // Número de rodas
#define WHEEL_A_INDEX 0 // Índice da roda A ou Motor 1
#define WHEEL_B_INDEX 1 // Índice da roda B ou Motor 2
#define WHEEL_C_INDEX 2 // Índice da roda C ou Motor 3

// Canais do ADC e PWM para as rodas
/*
    Para cada roda precisamos:
    - Canal ADC para leitura de corrente
    - Canal PWM A (forward)
    - Canal PWM B (reverse)
    - Canal do encoder (TIM) para leitura de velocidade

*/

// Roda 1:
#define WHEEL_A_ADC_CHANNEL ADC_1_IN_4
#define WHEEL_A_PWM_CHANNEL_A TIM_1_CHANNEL_1
#define WHEEL_A_PWM_CHANNEL_B TIM_1_CHANNEL_1N
#define WHEEL_A_ENCODER_TIM htim5

// Roda 2:
#define WHEEL_B_ADC_CHANNEL ADC_1_IN_5
#define WHEEL_B_PWM_CHANNEL_A TIM_1_CHANNEL_2
#define WHEEL_B_PWM_CHANNEL_B TIM_1_CHANNEL_2N
#define WHEEL_B_ENCODER_TIM htim3

// Roda 3:
#define WHEEL_C_ADC_CHANNEL ADC_1_IN_8
#define WHEEL_C_PWM_CHANNEL_A TIM_1_CHANNEL_3
#define WHEEL_C_PWM_CHANNEL_B TIM_1_CHANNEL_3N
#define WHEEL_C_ENCODER_TIM htim4

/// @brief Estrutura para armazenar informações de cada roda.
/// vamos utilizar essa estrutura para deixar o código mais legível e fácil de manter.
typedef struct
{
    uint8_t pwm_channel_a;          // Canal PWM A (foward?)
    uint8_t pwm_channel_b;          // Canal PWM B (reverse?)
    uint8_t adc_channel;            // Canal ADC para leitura corrente
    TIM_HandleTypeDef *encoder_tim; // Timer associado ao encoder para a leitura de velocidade

} WheelInfo;

#pragma endregion

// Definições de constantes para as Tasks
#pragma region Tasks

// Speed Controller Task
#define SPEED_CONTROLLER_TASK_PRIORITY 5
#define SPEED_CONTROLLER_TASK_STACK_SIZE 2048
#define SPEED_CONTROLLER_TASK_MS 20
// Torque Controller Task
// Pelo que eu vi, numero maior = prioridade maior
// https://www.youtube.com/watch?v=QN1-QE6z3No
#define TORQUE_CONTROLLER_TASK_PRIORITY TORQUE_CONTROLLER_TASK_PRIORITY + 1
#define TORQUE_CONTROLLER_TASK_STACK_SIZE 2048
#define TORQUE_CONTROLLER_TASK_MS 2
// Tamanho da fila para as velocidades e correntes
// TODO: verificar tamanho da fila e ticks de espera para obter 
#define QUEUE_LENGTH 4
#define QUEUE_MAX_WAIT_TICKS 100 // Tempo máximo de espera para a fila (em ticks)
#define MUTEX_TIMEOUT_TICKS 100  // Tempo máximo de espera para o mutex (em ticks)
#define MUTEX_DELAY_TICKS 1      // Tempo de delay para esperar o mutex (em ticks)
// Macro para facilitar a espera por semáforos
#define xTakeSemaphore(xSemaphore)                                    \
    while (xSemaphoreTake(xSemaphore, MUTEX_TIMEOUT_TICKS) != pdTRUE) \
    {                                                                 \
        vTaskDelay(MUTEX_DELAY_TICKS);                                \
    }

/*
     xTakeSemaphore é uma macro que:
            - Espera até que o semáforo esteja disponível por no maximo (MUTEX_TIMEOUT_TICKS) ticks
            - Dorme por (MUTEX_DELAY_TICKS) ticks para evitar busy waiting
            - repete até que o semáforo esteja disponível
    O nome é proposital pois xSemaphoreTake é a função do freeRTOS.
    Então o nome ainda descreve a ação que a macro faz
    sem ter problema de compatibilidade com o nome da função do freeRTOS.
*/

#pragma endregion

// Definições de constantes para as constantes dos PIDs
#pragma region PID
// Indeces
#define Kp 0
#define Ki 1
#define Kd 2

// Uso com PID onde as constantes são iguas para todas as rodas
/*
#define PID_CONSTANTS_SPEED {0.1f, 0.01f, 0.001f}   // Kp, Ki, Kd para velocidade
#define PID_CONSTANTS_CURRENT {0.1f, 0.01f, 0.001f} // Kp, Ki, Kd para corrente
// Uso: PID_CONSTANTS_SPEED[Kp] = Kp para velocidade
// Uso: PID_CONSTANTS_CURRENT[Kd] = Kd para corrente
*/

// Uso com PID onde as constantes são diferentes para cada roda

/*
    Vamos utilizar Kd = 0 pois não teeremos o termo derivativo no PID
    As outras constantes serão definidas Apenas Teoricamente
    Proxima fase seria o PID Tunning para acharmos os valores ideais de
    Kp, Ki e Kd para cada controle para cada roda.
*/

#define PID_CONSTANTS_SPEED_A {0.1f, 0.01f, 0.0f} // Kp, Ki, Kd para velocidade da roda A
#define PID_CONSTANTS_SPEED_B {0.1f, 0.01f, 0.0f} // Kp, Ki, Kd para velocidade da roda B
#define PID_CONSTANTS_SPEED_C {0.1f, 0.01f, 0.0f} // Kp, Ki, Kd para velocidade da roda C

#define PID_CONSTANTS_CURRENT_A {0.1f, 0.01f, 0.0f} // Kp, Ki, Kd para corrente da roda A
#define PID_CONSTANTS_CURRENT_B {0.1f, 0.01f, 0.0f} // Kp, Ki, Kd para corrente da roda B
#define PID_CONSTANTS_CURRENT_C {0.1f, 0.01f, 0.0f} // Kp, Ki, Kd para corrente da roda C

// exemplo:
//  float saida_A = PID_CONSTANTS_SPEED[WHEEL_A_INDEX][Kp] * erro + PID_CONSTANTS_SPEED[WHEEL_A_INDEX][Ki] * integral;
// Uso: PID_CONSTANTS_CURRENT[WHEEL_A_INDEX][Kp] = Kp para corrente da roda A
// Uso: PID_CONSTANTS_SPEED[WHEEL_B_INDEX][Kd] = Kd para velocidade da roda B
// Uso: PID_CONSTANTS_CURRENT[WHEEL_C_INDEX][Ki] = Ki para corrente da roda C

// Definições de onde as constantes do PID serão armazenadas
// Podemos armazenar as constantes do PID em RAM ou em Flash
// Se os dois forem definidos, o compilador irá usar o que estiver definido por último

#define PID_IN_RAM 0
#define PID_IN_FLASH 1

#ifdef PID_IN_RAM
#define PID_LOCATION static
#endif
#ifdef PID_IN_FLASH
#define PID_LOCATION const
#endif
// Se nenhum for definido, o compilador irá usar a RAM
#ifndef PID_LOCATION
#define PID_LOCATION static
#endif // PID_LOCATION

#pragma endregion

// Definições de parametros físicos como Resolução do ADC, Tensão de referência, etc.
#pragma region Parametros Físicos

#define ADC_BITS 10                // Resolução do ADC (10 bits)
#define Voltage_REF 3.3f           // Tensão de referência do ADC
#define PULSES_PER_REVOLUTION 1000 // Pulsos por revolução do encoder

#pragma endregion

// Definições dos tipos de dados para as velocidades e correntes das rodas
#pragma region Tipos de Dados
/*
    Define o tipo de dados para as velocidades das rodas e correntes
    cada tipo deve conter os 3 valores de velocidade ou corrente
    e deve poder ser indexado como um vetor de 3 elementos
    Ver os indexes WHEEL_X_INDEX.
*/

#define baseSpeedType_t float // Tipo base para as velocidades das rodas
#define baseCurrentType_t float // Tipo base para as correntes das rodas
#define SpeedType_t baseSpeedType_t[WHEELS_COUNT]   // Tipo de dados utilizado para as velocidades das rodas
#define CurrentType_t baseCurrentType_t[WHEELS_COUNT] // Tipo de dados utilizado para as correntes das rodas

#pragma endregion

// Assinaturas das funções e variáveis expostas para outras partes do código
#pragma region Funções e Variaveis Expostas
/*
    Expõe os Handles das Tasks de controle de velocidade e corrente
    para que possam ser acessados por outras partes do código. (Caso precise)
*/

void SpeedController_Init(void);

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
CurrentType_t GetWheelCurrents();

#pragma endregion

#endif // SPEEDCONTROLLER_H
