#include "speedcontroller.h"
#include "main.h"
#include "math.h"

// --- Handles externos ---
extern TIM_HandleTypeDef htim3; //
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;

// --- Filas e mutexes ---
// Fila para receber o target de velocidade (Speed)
QueueHandle_t speedQueueHandle;
// Fila para enviar/receber as correntes das rodas (Torque)
QueueHandle_t wheelCurrentQueueHandle;
// Semaphore para proteger o acesso aos vetores (Speed)
SemaphoreHandle_t speedMutexHandle;
// Semaphore para proteger o acesso às correntes (Torque)
SemaphoreHandle_t currentMutexHandle;
// Semaphore para proteger o acesso às velocidades das rodas (Libera a Velocidade Externamente)
SemaphoreHandle_t ext_wheelSpeedMutexHandle;
// Semaphore para proteger o acesso às correntes das rodas (Libera a Corrente Externamente)
SemaphoreHandle_t ext_wheelCurrentMutexHandle;

/* Essas variáveis são usadas para armazenar as velocidades e correntes atuais das rodas
   Elas são atualizadas pelas tarefas de controle de velocidade e corrente para saber o delta entre o
   target e o ultimo valor lido.
   Elas estão no heap pois podem precisar ser acessadas por outras tarefas, como a de telemetria ou data logger.
   Elas estão protegidas por mutexes para evitar condições de corrida.
    No Heap pois precisam ser lidas por multiplas tarefas e não entre duas tarefas.
*/
// Velocidade Atual das rodas
static SpeedType_t _wheelSpeeds = {0}; // Velocidades das rodas
// Corrente Atual das rodas
static WheelSpeeds_t _wheelCurrents = {0}; // Correntes das rodas

// Configurações das rodas
#define WHEEL_A_CONFIG                                  \
    WheelInfo                                           \
    {                                                   \
        .pwm_channel_a = WHEEL_A_PWM_CHANNEL_A,         \
        .pwm_channel_b = WHEEL_A_PWM_CHANNEL_B,         \
        .adc_channel = WHEEL_A_ADC_CHANNEL,             \
        .encoder_tim = &htim5,                          \
        .encoder_channel_a = WHEEL_A_ENCODER_CHANNEL_A, \
        .encoder_channel_b = WHEEL_A_ENCODER_CHANNEL_B  \
    }

#define WHEEL_B_CONFIG                                  \
    WheelInfo                                           \
    {                                                   \
        .pwm_channel_a = WHEEL_B_PWM_CHANNEL_A,         \
        .pwm_channel_b = WHEEL_B_PWM_CHANNEL_B,         \
        .adc_channel = WHEEL_B_ADC_CHANNEL,             \
        .encoder_tim = &htim3,                          \
        .encoder_channel_a = WHEEL_B_ENCODER_CHANNEL_A, \
        .encoder_channel_b = WHEEL_B_ENCODER_CHANNEL_B  \
    }

#define WHEEL_C_CONFIG                                  \
    WheelInfo                                           \
    {                                                   \
        .pwm_channel_a = WHEEL_C_PWM_CHANNEL_A,         \
        .pwm_channel_b = WHEEL_C_PWM_CHANNEL_B,         \
        .adc_channel = WHEEL_C_ADC_CHANNEL,             \
        .encoder_tim = &htim4,                          \
        .encoder_channel_a = WHEEL_C_ENCODER_CHANNEL_A, \
        .encoder_channel_b = WHEEL_C_ENCODER_CHANNEL_B  \
    }

static WheelInfo wheelInfo[3] = {
    WHEEL_A_CONFIG,
    WHEEL_B_CONFIG,
    WHEEL_C_CONFIG};



// --- Prototypes internos ---
// Foward declarations
static float readEncoderSpeed(TIM_HandleTypeDef *htim);
static float readMotorCurrent(uint32_t adc_channel);
static void setMotorPWM(uint8_t channel, float pwm);
static void SpeedControlTask(void *argument);
static void CurrentControlTask(void *argument);

/// @brief Tarefa de inicialização do controlador de velocidade e corrente.
/// Inicializa as filas, mutexes e tarefas do controlador de velocidade e corrente.
/// @note Essa Função deve ser ANTES de acionar o escalonador e APÓS a inicialização do sistema,
/// ela, e as tasks inicializadas por ela assumem que os pinos, timers, PWM e ADC estão corretamente configurados e prontos para uso.
void SpeedController_Init(void)
{
    // Inicialização das filas e mutexes
    vSemaphoreCreateBinary(speedMutexHandle);
    vSemaphoreCreateBinary(currentMutexHandle);
    vSemaphoreCreateBinary(ext_wheelSpeedMutexHandle);
    vSemaphoreCreateBinary(ext_wheelCurrentMutexHandle);
    speedQueueHandle = xQueueCreate(QUEUE_LENGTH, sizeof(SpeedType_t));
    wheelCurrentQueueHandle = xQueueCreate(QUEUE_LENGTH, sizeof(WheelSpeeds_t));
    // Inicialização das tarefas
    xTaskCreate(SpeedControlTask,                 /* Pointer para a task */
                "SpeedControl",                   /* Nome da task */
                SPEED_CONTROLLER_TASK_STACK_SIZE, /* Tamanho da pilha da task */
                NULL,                             /* Argumento da task */
                SPEED_CONTROLLER_TASK_PRIORITY,   /* Prioridade da task */
                &speedControlTaskHandle           /* Handle da task */
    );
    xTaskCreate(CurrentControlTask,                /* Pointer para a task */
                "CurrentControl",                  /* Nome da task */
                TORQUE_CONTROLLER_TASK_STACK_SIZE, /* Tamanho da pilha da task */
                NULL,                              /* Argumento da task */
                TORQUE_CONTROLLER_TASK_PRIORITY,   /* Prioridade da task */
                &currentControlTaskHandle          /* Handle da task */
    );
}

// --- Tarefa de cálculo de velocidades das rodas ---
/// @brief Tarefa que calcula as velocidades das rodas a partir de um vetor alvo.
/// Recebe o vetor alvo da fila, calcula as velocidades das rodas e envia para a fila
/// de corrente.
/// @param argument Argumento da tarefa (não utilizado).
void SpeedControlTask(void *argument)
{
    SpeedType_t targetVect;
    WheelSpeeds_t wheelSpeeds;

    for (;;)
    {
        // Espera até que a fila de vetores esteja disponível
        if (xSemaphoreTake(speedMutexHandle, portMAX_DELAY) == pdTRUE)
        {
            // Recebe o vetor alvo
            if (xQueueReceive(speedQueueHandle, &targetVect, 0) == pdTRUE)
                ;

            xSemaphoreGive(speedMutexHandle);
        }

        while (xSemaphoreTake(ext_wheelSpeedMutexHandle, portMAX_DELAY) != pdTRUE)
        {
            vTaskDelay(pdMS_TO_TICKS(1)); // Espera até que o mutex esteja disponível
            yield();                      // Permite que outras tarefas rodem
            // watchdog();
        }

        // Calcula as velocidades das rodas a partir do vetor alvo
        for (size_t i = 0; i < WHEELS_COUNT; i++)
        {
            // Target: targetVect
            // Input = readEncoderSpeed();
            // Output = wheelSpeeds
            // last = _wheelSpeeds[i]; // Último valor lido
            float targetSpeed = wheelSpeeds[i]; // Ajustar para w2 e w3 conforme necessário
            float real = readEncoderSpeed(wheelInfo[i].encoder_tim);
            float last = _wheelSpeeds[i]; // Último valor lido
            _wheelSpeeds[i] = real;       // Atualiza a ultima velocidade lida
            // Aplicar PID aqui
            // TODO: Implementar o PID de velocidade
            float output = targetSpeed - real; // Simples controle proporcional
            wheelSpeeds[i] = output;           // Armazena a velocidade calculada
        }

        // Atualiza as velocidades atuais das rodas
        _wheelSpeeds = wheelSpeeds;                // Atualiza as velocidades atuais das rodas
        xSemaphoreGive(ext_wheelSpeedMutexHandle); // Libera o mutex de velocidade para outras tarefas

        // Envia as velocidades calculadas para a fila de corrente
        while (xSemaphoreTake(speedMutexHandle, 0) != pdTRUE)
        {
            vTaskDelay(pdMS_TO_TICKS(1)); // Espera até que o mutex esteja disponível
            yield();                      // Permite que outras tarefas rodem
            // watchdog();
        }

        // Espera até que o mutex de velocidade esteja disponível
        if (xSemaphoreTake(currentMutexHandle, portMAX_DELAY) == pdTRUE)
        {
            xQueueSend(currentQueueHandle, &wheelSpeeds, 0);
            xSemaphoreGive(currentMutexHandle);
        }

        vTaskDelay(pdMS_TO_TICKS(SPEED_CONTROLLER_TASK_MS));
    }
}

// --- Tarefa de controle de corrente e envio de PWM ---
/// @brief Tarefa que controla a corrente dos motores e envia o PWM para os motores.
/// Recebe as velocidades das rodas da fila de corrente, calcula a corrente e envia o PWM
/// para os motores.
/// @param argument Argumento da tarefa (não utilizado).
void CurrentControlTask(void *argument)
{
    WheelSpeeds_t wheelSpeeds;

    for (;;)
    {
        // Espera até que a fila de velocidades esteja disponível
        if (xSemaphoreTake(currentMutexHandle, portMAX_DELAY) == pdTRUE)
        {
            while (xQueueReceive(currentQueueHandle, &wheelSpeeds, 0) != pdTRUE)
            {
                vTaskDelay(pdMS_TO_TICKS(1)); // Espera até que a fila esteja disponível
                yield();                      // Permite que outras tarefas rodem
                // watchdog();
            }
            xSemaphoreGive(currentMutexHandle);
            // Espera até que o mutex de ler a ultima corrente esteja disponível
            while (xSemaphoreTake(ext_wheelCurrentMutexHandle, portMAX_DELAY) != pdTRUE)
                ;
            // Aplica o PID de corrente para cada roda
            for (size_t i = 0; i < WHEELS_COUNT; i++)
            {
                // APlicar PID de corrente aqui
                // target = wheelSpeeds[i]; Converter para corrente?
                // real = readMotorCurrent(wheelInfo[i].adc_channel);
                // last = _wheelCurrents[i]; // Último valor lido
                // output = pwm
                float target = wheelSpeeds[i]; // Ajustar para w2 e w3 conforme necessário
                float real = readMotorCurrent(wheelInfo[i].adc_channel);
                float last = _wheelCurrents[i]; // Último valor lido
                _wheelCurrents[i] = real;       // Atualiza a ultima corrente lida

                // Aplicar PID aqui
                // TODO: Implementar o PID de corrente
                // Fake output
                float output = target - real; // Simples controle proporcional

                // Envia o sinal PWM para o motor
                setMotorPWM(output >= 0 ? wheelInfo[i].pwm_channel_a : wheelInfo[i].pwm_channel_b, abs(output));
            }
            xSemaphoreGive(ext_wheelCurrentMutexHandle); // Libera o mutex de corrente para outras tarefas
        }

        vTaskDelay(pdMS_TO_TICKS(TORQUE_CONTROLLER_TASK_MS));
    }
}

// --- Leitura de encoder simples ---
/// @brief Lê a velocidade do encoder associado ao timer.
/// @param htim Ponteiro para o handle do timer associado ao encoder.
/// @return Velocidade do encoder em RPM.
static float readEncoderSpeed(TIM_HandleTypeDef *htim)
{
    // TODO: Implementar A Leitura do encoder correntamente
    // Dummy, Rever o encoder
    static uint32_t last[3] = {0};
    uint8_t index = (htim == &htim3) ? 0 : (htim == &htim4) ? 1
                                                            : 2;
    uint32_t now = __HAL_TIM_GET_COUNTER(htim);
    float delta = (int32_t)(now - last[index]);
    last[index] = now;

    float rpm = (delta * 6000.0f / 65536.0f); // ajuste conforme encoder
    return rpm;
}

// --- Leitura da corrente dos motores ---
/// @brief Lê a corrente do motor a partir do canal ADC especificado.
/// @param channel Canal ADC a ser lido.
/// @return Corrente do motor em Amperes.
static float readMotorCurrent(uint32_t channel)
{
    // TODO: Confirmar a implementação do ADC
    // O ADC pelo que eu vi ta certo.
    ADC_ChannelConfTypeDef config = {0};
    config.Channel = channel;
    config.Rank = 1;
    config.SamplingTime = ADC_SAMPLETIME_112CYCLES;

    HAL_ADC_ConfigChannel(&hadc1, &config);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint32_t raw = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    float voltage = raw * 3.3f / 1023.0f;
    return (voltage - 2.5f) / 0.1f;
}

// --- PWM por motor ---
/// @brief Configura o PWM para o motor especificado.
/// @param channel Canal PWM a ser configurado.
/// @param pwm Valor do PWM a ser configurado (0-100%).
/// @note O PWM é invertido, pois o canal A é o normal e o canal B é o invertido.
static void setMotorPWM(uint8_t channel, float pwm)
{
    // TODO: Implementar o PWM corretamente
    // Dummy, rever O PWM

    uint32_t pulse = (uint32_t)(fminf(fabsf(pwm), 100.0f) * htim1.Init.Period / 100.0f);
    __HAL_TIM_SET_COMPARE(&htim1, channel, pulse);
    break;
}

/// @brief Obtém as velocidades das rodas.
/// @return um SpeedType_t com a velocidade de cada roda.
/// @note Ver os indexes WHEEL_X_INDEX.
SpeedType_t GetWheelSpeeds()
{
    SpeedType_t speeds = {0};

    if (xSemaphoreTake(ext_wheelSpeedMutexHandle, portMAX_DELAY) != pdTRUE)
    {
        return speeds; // Retorna vazio se não conseguir pegar o mutex
    }

    speeds = _wheelSpeeds;

    xSemaphoreGive(ext_wheelSpeedMutexHandle);
    return speeds;
}

/// @brief  Obtém as correntes das rodas.
/// @return um SpeedType_t com a corrente de cada roda.
/// @note Ver os indexes WHEEL_X_INDEX.
WheelSpeeds_t GetWheelCurrents()
{
    WheelSpeeds_t currents = {0};
    // Espera até que o mutex de corrente esteja disponível
    if (xSemaphoreTake(ext_wheelCurrentMutexHandle, portMAX_DELAY) != pdTRUE)
    {
        return currents; // Retorna vazio se não conseguir pegar o mutex
    }

    currents = _wheelCurrents;

    // Libera o mutex de corrente para outras tarefas
    xSemaphoreGive(ext_wheelCurrentMutexHandle);
    return currents;
}