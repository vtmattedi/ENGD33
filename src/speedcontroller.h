#pragma once
#include "FreeRTOS.h" // FreeRTOS header
#include "task.h" // Task management
#include "semphr.h" // Semaphore/Mutex
#include "queue.h" // Queue for direction vector
#define TORQUE_CONTROLLER_TASK_PRIORITY 5
#define TORQUE_CONTROLLER_TASK_STACK_SIZE 2048
#define SPEED_CONTROLLER_TASK_PRIORITY 5
#define SPEED_CONTROLLER_TASK_STACK_SIZE 2048

// HAL pin inputs
#define SPEED_WHEEL_A_PIN_A 32
#define SPEED_WHEEL_A_PIN_B 25
#define SPEED_WHEEL_B_PIN_A 33
#define SPEED_WHEEL_B_PIN_B 27
#define SPEED_WHEEL_C_PIN_A 35
#define SPEED_WHEEL_C_PIN_B 26

// AC 712 current sensor pins
#define WHEEL_A_CURRENT_PIN 34
#define WHEEL_B_CURRENT_PIN 39
#define WHEEL_C_CURRENT_PIN 36
#define WHEEL_A_CURRENT_ADC_CHANNEL 0
#define WHEEL_B_CURRENT_ADC_CHANNEL 1
#define WHEEL_C_CURRENT_ADC_CHANNEL 2

#define WHEEL_A_PIN 12
#define WHEEL_B_PIN 13
#define WHEEL_C_PIN 14
#define WHEEL_A_PWM_CHANNEL 0
#define WHEEL_B_PWM_CHANNEL 1
#define WHEEL_C_PWM_CHANNEL 2
// C Shenanigans
#define true 1
#define false 0
#define bool int
#define NULL 0
#define INPUT 1
#define OUTPUT 0
 
struct Vect2{
    float x;
    float y;
};
struct PID_DATA
{
    float last_error;
    float integral;
    float derivative;
    float setpoint;
    float output;
};


struct Vect3{
    float theta;
    float radius;
    float phi;
};

// Initialize ADC and hal inputs and PWM outputs
void setupSpeedControllers();

void xTaskTorqueController(void *pvParameters);
void xTaskSpeedController(void *pvParameters);
// Option 1
void setDirectionVector(struct Vect2 dir);
struct Vect2 getDirectionVector();
//Option 2
extern QueueHandle_t xDirectionVectorQueue;

// Expose Tasks handles???
extern TaskHandle_t xSpeedControllerTaskHandle;
extern TaskHandle_t xTorqueControllerTaskHandle;