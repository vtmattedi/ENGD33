#include "speedcontroller.h"
TaskHandle_t xSpeedControllerTaskHandle = NULL;
TaskHandle_t xTorqueControllerTaskHandle = NULL;
SemaphoreHandle_t xDirectionVectorSemaphore = NULL;
struct Vect2 Dir = {0.0f, 0.0f};
/*
 Should not be our responsibility to manage these variables.
 Vect2 target = {0.0f, 0.0f};
 Vect2 current = {0.0f, 0.0f};
*/
struct PID_DATA PIDs[3];

float Motors_PID_Constants[3][3] = {
    {0.0f, 0.0f, 0.0f}, // PID for Wheel A: {Kp, Ki, Kd}
    {0.0f, 0.0f, 0.0f}, // PID for Wheel B: {Kp, Ki, Kd}
    {0.0f, 0.0f, 0.0f}  // PID for Wheel C: {Kp, Ki, Kd}
};

int MotorsPWM[3] = {WHEEL_A_PWM_CHANNEL, WHEEL_B_PWM_CHANNEL, WHEEL_C_PWM_CHANNEL};

void setPWM(int channel, int value)
{
    // This function would typically set the PWM value for a specific channel.
    // In a real implementation, you would use the appropriate HAL functions.

    printf("Setting PWM channel %d to value %d\n", channel, value);
}

void xTaskSpeedController(void *pvParameters)
{
    // Maybe do Setup here?
    while (true)
    {
        for (int i = 0; i < 3; i++)
        {
            float pid_value = 0;
            for (int j = 0; j < 3; j++)
            {
                pid_value += Motors_PID_Constants[i][j] * PIDs[i].output;
            }
            setPWM(MotorsPWM[i], (int)pid_value);
        }
        // Simulate some processing
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
        printf("Speed Controller Task Running\n");

        // Here you would typically read sensors, compute speed, etc.
        // For now, we just print the direction vector
        printf("Direction Vector: x = %.2f, y = %.2f\n", Dir.x, Dir.y);
    }
}

void xTaskTorqueController(void *pvParameters)
{
    // Task implementation
    while (true)
    {
        // Simulate some processing
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
        printf("Torque Controller Task Running\n");

        // Here you would typically read sensors, compute torque, etc.
        // For now, we just print the direction vector
        printf("Direction Vector: x = %.2f, y = %.2f\n", Dir.x, Dir.y);
    }
}

void pinMode(int pin, int mode)
{
    // This function would typically set the pin mode for GPIO pins.
    // In a real implementation, you would use the appropriate HAL functions.

    if (mode == INPUT)
    {
        printf("Setting pin %d to INPUT mode\n", pin);
    }
    else if (mode == OUTPUT)
    {
        printf("Setting pin %d to OUTPUT mode\n", pin);
    }
    else
    {
        printf("Invalid mode for pin %d\n", pin);
    }
}
void configurePwmChannel(int channel, int pin)
{

    printf("Configuring PWM channel %d for pin %d\n", channel, pin);
}

void configureADC(int channel, int pin)
{
    printf("Configuring ADC channel %d for pin %d\n", channel, pin);
}

void setDirectionVector(struct Vect2 dir)
{
    if (xSemaphoreTake(xDirectionVectorSemaphore, portMAX_DELAY) == pdTRUE)
    {
        Dir.x = dir.x;
        Dir.y = dir.y;
        xSemaphoreGive(xDirectionVectorSemaphore);
    }
}

struct Vect2 getDirectionVector()
{
    struct Vect2 copy;
    if (xSemaphoreTake(xDirectionVectorSemaphore, portMAX_DELAY) == pdTRUE)
    {
        copy.x = Dir.x;
        copy.y = Dir.y;
        xSemaphoreGive(xDirectionVectorSemaphore);
    }
    return copy;
}

void setupSpeedControllers()
{
    // Initialize ADC and HAL inputs
    // This function would typically set up the ADC for reading current sensors
    // and configure the GPIO pins for the speed wheel encoders.
    if (!vCreateMutex(&xDirectionVectorSemaphore))
    {
        printf("Failed to create Direction Vector Semaphore\n");
        return;
    }
    // Initialize ADC and HAL inputs
    // Config Outputs
    pinMode(WHEEL_A_PIN, OUTPUT);
    configurePwmChannel(WHEEL_A_PWM_CHANNEL, WHEEL_A_PIN);
    pinMode(WHEEL_B_PIN, OUTPUT);
    configurePwmChannel(WHEEL_B_PWM_CHANNEL, WHEEL_B_PIN);
    pinMode(WHEEL_C_PIN, OUTPUT);
    configurePwmChannel(WHEEL_C_PWM_CHANNEL, WHEEL_C_PIN);

    // Config Inputs
    //  Encoder pins for speed wheels
    pinMode(SPEED_WHEEL_A_PIN_A, INPUT);
    pinMode(SPEED_WHEEL_A_PIN_B, INPUT);
    pinMode(SPEED_WHEEL_B_PIN_A, INPUT);
    pinMode(SPEED_WHEEL_B_PIN_B, INPUT);
    pinMode(SPEED_WHEEL_C_PIN_A, INPUT);
    pinMode(SPEED_WHEEL_C_PIN_B, INPUT);
    // Current sensor pins for wheels
    pinMode(WHEEL_A_CURRENT_PIN, INPUT);
    pinMode(WHEEL_B_CURRENT_PIN, INPUT);
    pinMode(WHEEL_C_CURRENT_PIN, INPUT);
    configureADC(WHEEL_A_CURRENT_ADC_CHANNEL, WHEEL_A_CURRENT_PIN);
    configureADC(WHEEL_B_CURRENT_ADC_CHANNEL, WHEEL_B_CURRENT_PIN);
    configureADC(WHEEL_C_CURRENT_ADC_CHANNEL, WHEEL_C_CURRENT_PIN);

    printf("Speed Controllers Setup Complete\n");

    xTaskCreate(xTaskSpeedController,
                "SpeedController",
                SPEED_CONTROLLER_TASK_STACK_SIZE,
                NULL, SPEED_CONTROLLER_TASK_PRIORITY,
                &xSpeedControllerTaskHandle);
    xTaskCreate(xTaskTorqueController,
                "TorqueController",
                TORQUE_CONTROLLER_TASK_STACK_SIZE,
                NULL,
                TORQUE_CONTROLLER_TASK_PRIORITY,
                &xTorqueControllerTaskHandle);

    // In a real implementation, you would initialize hardware here.
}