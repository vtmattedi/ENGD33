// File: src/speedcontroller.h
#define TORQUE_CONTROLLER_TASK_PRIORITY 5
#define TORQUE_CONTROLLER_TASK_STACK_SIZE 2048
#define SPEED_CONTROLLER_TASK_PRIORITY TORQUE_CONTROLLER_TASK_PRIORITY - 1
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

struct Vect3{
    float x;
    float y;
    float phi;
};
// Controle de Corrente
// extern mutex_t mutex_ADC; #include "common.h";
mutex_t  mutex_target_current;
float target_current[3]; // xQueue 
float PID_current[3][3] = {
    {0.0f, 0.0f, 0.0f}, // PID for Wheel A: {Kp, Ki, Kd}
    {0.0f, 0.0f, 0.0f}, // PID for Wheel B: {Kp, Ki, Kd}
    {0.0f, 0.0f, 0.0f}  // PID for Wheel C: {Kp, Ki, Kd}
};
// Controle de velocidade
float PID_speed[3][3] = {
    {0.0f, 0.0f, 0.0f}, // PID for Wheel A: {Kp, Ki, Kd}
    {0.0f, 0.0f, 0.0f}, // PID for Wheel B: {Kp, Ki, Kd}
    {0.0f, 0.0f, 0.0f}  // PID for Wheel C: {Kp, Ki, Kd}
};
float target_speed[3] = {0.0f, 0.0f, 0.0f}; // Target speed for each wheel 
// Ler a corrente de cada motor
// Aplicar o PID de corrente
// Enviar o PWM para cada motor
void currentTask(void *pvParameters);
// Controle de velocidade
// Ler a Velocidade de cada roda
// Setar o target de velocidade de cada roda (target_current)
// configurar a velocidade atual
void speedTask(void *pvParameters);
// Inicia tudo que precisar do PWM e ADC
void setupPins();
// Ler ADC de um canal específico (corrente dos motores)
float readCurrent(int channel);
// Ler velocidade de uma roda (lendo os encoder)
float readWheelSpeed(int channelA, int channelB);
// Seta O PWM de uma RODA
void setPWM(int channel, int value);
// Seta o target da direção (x, y, phi)
void setDirectionVector(struct Vect3 dir);
//Utilidade -> nao nescessário
struct Vect3 currentSpeedVector = {0.0f, 0.0f, 0.0f};
struct Vect3 getSpeedVector();
// Opção para Q externo
extern xQueueHandle_t xQueueDir;