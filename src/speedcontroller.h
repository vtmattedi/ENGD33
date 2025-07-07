#pragma once
#include "freertos/FreeRTOS.h"
// #include <freertos/FreeRTOS.h>
// #include <freertos/task.h>

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
 
struct Vect2{
    float x;
    float y;
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

// extern QueueHandle_t xDirectionVectorQueue;