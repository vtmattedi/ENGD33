#include "speedcontroller.h"
#include "main.h"
#include "math.h"

// --- Handles externos ---
extern TIM_HandleTypeDef htim3;
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

// Configurações de cada roda
// TODO: Ver quais configurações seram utilizadas e remover as não utilizadas <-ultima coisa
// Aplicar também a estrutura WheelInfo
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
    vSemaphoreCreateBinary(speedMutexHandle);            // Semaphore para entrada de velocidade
    vSemaphoreCreateBinary(currentMutexHandle);          // semaphore para entrada de corrente
    vSemaphoreCreateBinary(ext_wheelSpeedMutexHandle);   // semaphore para velocidade atual
    vSemaphoreCreateBinary(ext_wheelCurrentMutexHandle); // semaphore para corrente atual

    // TODO: Verificar Tamanho das filas
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
    SpeedType_t input;
    WheelSpeeds_t output;
    static float integral[WHEELS_COUNT] = {0};

    for (;;)
    {
        // Espera até que a fila de vetores esteja disponível
        if (xSemaphoreTake(speedMutexHandle, portMAX_DELAY) == pdTRUE)
        {
            // Recebe o vetor alvo
            if (xQueueReceive(speedQueueHandle, &input, 0) == pdTRUE)
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
            // TODO: Checar PID principalmente Integral
            // Target: input (RPS) 
            // Input = readEncoderSpeed(RPS);
            // Output = wheelSpeeds -> (RPS);
            // last = _wheelSpeeds[i]; // Último valor lido
            float target = input[i]; // Ajustar para w2 e w3 conforme necessário
            float real = readEncoderSpeed(wheelInfo[i].encoder_tim);
            float last = _wheelSpeeds[i]; // Último valor lido
            _wheelSpeeds[i] = real;       // Atualiza a ultima velocidade lida
            // Aplicar PID aqui
            integral[i] += (target - real); // Integral do erro (acumulada)
            output[i] = (target - real) * PID_CONSTANTS_SPEED[i][Kp] +
                        integral[i] / SPEED_CONTROLLER_TASK_MS * PID_CONSTANTS_SPEED[i][Ki]; // Simples controle proporcional
        }

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
    WheelSpeeds_t input;
    WheelSpeeds_t lastCurrents = {0};
    static float integral[WHEELS_COUNT] = {0};

    for (;;)
    {
        // Espera até que a fila de velocidades esteja disponível
        if (xSemaphoreTake(currentMutexHandle, portMAX_DELAY) == pdTRUE)
        {
            xSemaphoreGive(currentMutexHandle);
            // Espera até que o mutex de ler a ultima corrente esteja disponível

            // Aplica o PID de corrente para cada roda
            for (size_t i = 0; i < WHEELS_COUNT; i++)
            {
                // APlicar PID de corrente aqui
                // target = Saida da SpeedTask (RPS) -> precisa converter para corrente
                // input = Corrente lida do ADC (Amperes)
                // output = (Amperes) -> PWM
                float target = input[i]; // Corrente alvo (Amperes)
                // float target = input * alguma coisa RPM -> corrente alvo (Amperes)
                float real = readMotorCurrent(wheelInfo[i].adc_channel); // Lê a corrente do motor
                lastCurrents[i] = real;                                  // Atualiza a ultima corrente lida
                integral[i] += real;

                // Aplicar PID aqui
                // TODO: Implementar o PID de corrente
                // Fake output
                float output = (target - real) * PID_CONSTANTS_CURRENT[i][Kp] +
                               (lastCurrents[i] - real) * PID_CONSTANTS_CURRENT[i][Kd];
                // Envia o sinal PWM para o motor
                setMotorPWM(&wheelInfo[i], output);
            }
            // Mantem a ultima corrente lida
            while (xSemaphoreTake(ext_wheelCurrentMutexHandle, portMAX_DELAY) != pdTRUE)
                ;
            _wheelCurrents = lastCurrents;               // Atualiza as correntes atuais
            xSemaphoreGive(ext_wheelCurrentMutexHandle); // Libera o mutex de corrente para outras tarefas
        }

        vTaskDelay(pdMS_TO_TICKS(TORQUE_CONTROLLER_TASK_MS));
    }
}

// --- Leitura de encoder simples ---
/// @brief Lê a velocidade do encoder associado ao timer.
/// @param htim Ponteiro para o handle do timer associado ao encoder.
/// @return Velocidade do encoder em RPS.
static float readEncoderSpeed(TIM_HandleTypeDef *htim)
{
    // Medida anterior é sempre 0
    uint32_t count = __HAL_TIM_GET_COUNTER(htim); // Lê o contador do timer
    __HAL_TIM_SET_COUNTER(htim, 0);               // Reseta o contador do timer para manter ele sempre 0
    // Pulsos por revolução / Delta T
    float speed = (count / PULSES_PER_REVOLUTION) / (SPEED_CONTROLLER_TASK_MS / 1000.0f); // Calcula a velocidade em RPS
    // Checa se o timer está contando para baixo (velocidade negativa)
    speed = __HAL_TIM_IS_TIM_COUNTING_DOWN(htim) ? -speed : speed; // Inverte a velocidade se o timer estiver contando para baixo
    return speed;
}

// --- Leitura da corrente dos motores ---
/// @brief Lê a corrente do motor a partir do canal ADC especificado.
/// @param channel Canal ADC a ser lido.
/// @return Corrente do motor em Amperes.
static float readMotorCurrent(uint8_t channel)
{
    // O ADC pelo que eu vi ta certo.
    // Configura o canal ADC

    ADC_ChannelConfTypeDef config = {0};
    config.Channel = channel;                       // Canal ADC a ser lido
    config.Rank = 1;                                // Rank do canal ADC
    config.SamplingTime = ADC_SAMPLETIME_112CYCLES; // Tempo de amostragem do ADC
    HAL_ADC_ConfigChannel(&hadc1, &config);         // Configura o canal ADC
    HAL_ADC_Start(&hadc1);                          // Inicia a conversão do ADC
    HAL_ADC_PollForConversion(&hadc1, ADC_BITS);    // Polling para esperar a conversão do ADC
    uint16_t raw = HAL_ADC_GetValue(&hadc1);        // Lê o valor do ADC
    HAL_ADC_Stop(&hadc1);                           // Para a conversão do ADC
    // Tensão : raw * 3.3V / 1023.0f
    float voltage = raw * Voltage_REF / (float)((1 << ADC_BITS) - 1);
    // Converter Tensão em Corrente (ACS_712) Tem um divisor de tensão 1:1
    float amp = voltage * 2 /*Tensao No ACS_712*/ - 2.5f /*Offset*/ / 0.1f /*Sensibilidade do ACS_712 em V/A*/; // Corrente em Amperes
    return amp;                                                                                                 // Retorna a corrente em Amperes
}

// --- PWM por motor ---
/// @brief Configura o PWM para o motor especificado.
/// @param channel Canal PWM a ser configurado.
/// @param pwm Valor do PWM a ser configurado (0-100%).
/// @note O PWM é invertido, pois o canal A é o normal e o canal B é o invertido.
static void setMotorPWM(WheelInfo *wheel, float value)
{
    // trasformar o valor recebido em Duty Cycle
    // Enviar 0 para o canal A/B (dependendo do sinal)
    // Enviar o Duty Cycle para o canal B/A (dependendo do sinal)
    //
    // TODO: Implementar o PWM corretamente
    // Dummy, rever O PWM
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
