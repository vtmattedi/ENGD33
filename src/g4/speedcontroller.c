#include "speedcontroller.h"

// --- Handles externos ---
// Handles do Hardware configurados no CubeIDE (Não devem ser alterados!)
extern TIM_HandleTypeDef htim3; // Handle do Timer 3 (Encoder  Motor 1)
extern TIM_HandleTypeDef htim4; // Handle do Timer 4 (Encoder  Motor 2)
extern TIM_HandleTypeDef htim5; // Handle do Timer 5 (Encoder  Motor 3)
extern ADC_HandleTypeDef hadc1; // Handle do ADC 1
extern TIM_HandleTypeDef htim1; // Handle do Timer 1 (PWM)

// --- Filas e mutexes ---
// Fila para receber o target de velocidade (Speed)
QueueHandle_t speedQueueHandle;
// Fila para enviar/receber as correntes das rodas (Torque)
QueueHandle_t currentQueueHandle;
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
static SpeedType_t currentWheelSpeeds = {0}; // Velocidades das rodas atual (ultima leitura)
// Corrente Atual das rodas
static CurrentType_t currentWheelCurrents = {0}; // Correntes das rodas atual (ultima leitura)

// Configurações de cada roda
// Aplicar também a estrutura WheelInfo
#define WHEEL_A_CONFIG                          \
    WheelInfo                                   \
    {                                           \
        .pwm_channel_a = WHEEL_A_PWM_CHANNEL_A, \
        .pwm_channel_b = WHEEL_A_PWM_CHANNEL_B, \
        .adc_channel = WHEEL_A_ADC_CHANNEL,     \
        .encoder_tim = &htim5,                  \
    }

#define WHEEL_B_CONFIG                          \
    WheelInfo                                   \
    {                                           \
        .pwm_channel_a = WHEEL_B_PWM_CHANNEL_A, \
        .pwm_channel_b = WHEEL_B_PWM_CHANNEL_B, \
        .adc_channel = WHEEL_B_ADC_CHANNEL,     \
        .encoder_tim = &htim3,                  \
    }

#define WHEEL_C_CONFIG                          \
    WheelInfo                                   \
    {                                           \
        .pwm_channel_a = WHEEL_C_PWM_CHANNEL_A, \
        .pwm_channel_b = WHEEL_C_PWM_CHANNEL_B, \
        .adc_channel = WHEEL_C_ADC_CHANNEL,     \
        .encoder_tim = &htim4,                  \
    }

static WheelInfo wheelInfo[3] = {
    WHEEL_A_CONFIG,
    WHEEL_B_CONFIG,
    WHEEL_C_CONFIG};

// Foward declarations
static float readEncoderSpeed(TIM_HandleTypeDef *htim);
static float readMotorCurrent(uint8_t adc_channel);
static void setMotorPWM(WheelInfo *wheel, float value) static void SpeedControlTask(void *argument);
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
    currentQueueHandle = xQueueCreate(QUEUE_LENGTH, sizeof(CurrentType_t));

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
    SpeedType_t target_rpm;                    // target de velocidade (RPS) vindo externamente da fila
    // Tem que ser mesmo tipo da fila de corrente.
    SpeedType_t output_rpm;                  // Saída de velocidade (RPS) para a fila de corrente
    static float integral[WHEELS_COUNT] = {0}; // Integral do erro para cada roda.
    // Não precisa ser estático (heap) -> verificar.

    for (;;)
    {
        // TODO: Rever a logica dos dos mutexes e filas (Vitor)
        // tem que rever a logica também na outra task
        // (Vitor) -> Feito falta descrever a mudança:
        /*
            xTakeSemaphore é uma macro que:
            - Espera até que o semáforo esteja disponível por no maximo (MUTEX_TIMEOUT_TICKS) ticks
            - Dorme por (MUTEX_DELAY_TICKS) ticks para evitar busy waiting
            - repete até que o semáforo esteja disponível

        */

        // Espera até que a fila do input de velocidade esteja disponível
        xTakeSemaphore(speedMutexHandle);
        xQueueReceive(speedQueueHandle, &target_rpm, QUEUE_MAX_WAIT_TICKS);
        xSemaphoreGive(speedMutexHandle); // Libera o mutex da fila do input de velocidade

        // Espera até que o mutex de velocidade atual esteja disponível
        xTakeSemaphore(ext_wheelSpeedMutexHandle);

        // Aqui ja temos os dados da fila se precisar e
        // Calcula as velocidades das rodas a partir do vetor alvo
        for (size_t i = 0; i < WHEELS_COUNT; i++)
        {
            // TODO: Checar PID (Velocidade) principalmente Integral mas a principio esta pronto
            // Target (ou set point): input recebido externamente (RPS)
            // Input (ou real) = velocidade lida com readEncoderSpeed(RPS);
            // Output = Saida em RPS para a fila de corrente;
            // erro = set point - real;
            // delta = real - ultimo valor real lido;
            float set_point = target_rpm[i]; 
            float real = readEncoderSpeed(wheelInfo[i].encoder_tim); // Lê a velocidade usando o encoder
            float delta = real - currentWheelSpeeds[i]; // Delta entre a velocidade lida e a ultima velocidade lida
            currentWheelSpeeds[i] = real;            // Atualiza a ultima velocidade lida
            // Aplicar PID aqui
            // NOTA: Tirei o / SPEED_CONTROLLER_TASK_MS  pois é constante, portanto sera incorporado no PID
            // NOTA: Inclui o erro derivativo (Kd) para ficar completo. se Não formos usar,
            // podemos seta Kd = 0
            integral[i] += (set_point - real); // Integral do erro (acumulada)
            output_rpm[i] = (set_point - real) * PID_CONSTANTS_SPEED[i][Kp] +
                        integral[i] * PID_CONSTANTS_SPEED[i][Ki] +
                        delta * PID_CONSTANTS_SPEED[i][Kd];
        }

        xSemaphoreGive(ext_wheelSpeedMutexHandle); // Libera o mutex de velocidade para outras tarefas

        // Envia as velocidades calculadas para a fila de corrente
        xTakeSemaphore(currentMutexHandle);
        // Aqui a QUEUE_MAX_WAIT_TICKS é multiplicado por 2 pois estamos enviando para uma tarefa com mais prioridade.
        xQueueSend(currentQueueHandle, &output_rpm, QUEUE_MAX_WAIT_TICKS * 2);
        xSemaphoreGive(currentMutexHandle);

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
    SpeedType_t input_rpm;                    // target de corrente (Amperes) vindo externamente da fila
    static float integral[WHEELS_COUNT] = {0}; // Integral do erro para cada roda.

    for (;;)
    {
        // Espera até que a fila de corrente esteja disponível
        xTakeSemaphore(currentMutexHandle);
        xQueueReceive(currentQueueHandle, &input_rpm, QUEUE_MAX_WAIT_TICKS);
        xSemaphoreGive(currentMutexHandle); // Libera o mutex da fila de corrente
        // Espera até que o mutex de corrente atual esteja disponível
        xTakeSemaphore(ext_wheelCurrentMutexHandle);
        // Aqui ja temos os dados da fila se houver e temos o controle sobre o mutex da variavel corrente atual

        // Aplica o PID de corrente para cada roda
        for (size_t i = 0; i < WHEELS_COUNT; i++)
        {
            // PID de corrente:
            // target (ou set point) = Saida da SpeedTask (RPS) -> precisa converter para corrente
            // input (ou valor real) = Corrente lida do ADC (Amperes)
            // output = (Amperes) -> PWM
            float set_point = target_rpm[i]; // Corrente alvo (Amperes)
            // ! CONVERTER A VELOCIDADE PARA CORRENTE !
            // float target = input * alguma coisa RPM -> corrente alvo (Amperes)
            float real = readMotorCurrent(wheelInfo[i].adc_channel); // Lê a corrente do motor
            float delta = real - currentWheelCurrents[i];            // Delta entre a corrente lida e a ultima corrente lida
            currentWheelCurrents[i] = real;                          // Atualiza a ultima corrente lida

            integral[i] += real;
            // TODO: Implementar o PID de corrente
            // Precisamos converter a saida do PID de VELOCIDADE (RPS) para CORRENTE (Amperes)
            // Acho que o PID em sí eh só isso: erro * Kp + integral * Ki;
            // Além disso, o output desse PWM precisa ser enviado para o motor
            // portanto o output precisa ser convertido para PWM
            // pode ser em duty cycle (0-100%) ou em valor de PWM (0-255)
            float output = (target - real) * PID_CONSTANTS_CURRENT[i][Kp] +
                           (lastCurrents[i] - real) * PID_CONSTANTS_CURRENT[i][Kd] +
                           delta * PID_CONSTANTS_CURRENT[i][Ki];
            // Envia o sinal PWM para o motor

            setMotorPWM(&wheelInfo[i], output);
        }

        xSemaphoreGive(ext_wheelCurrentMutexHandle); // Libera o mutex de corrente para outras tarefas
        vTaskDelay(pdMS_TO_TICKS(TORQUE_CONTROLLER_TASK_MS));
    }
}

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
    // TODO: Implementar o PWM
}

/// @brief Obtém as velocidades das rodas.
/// @return um SpeedType_t com a velocidade de cada roda.
/// @note Ver os indexes WHEEL_X_INDEX.
SpeedType_t GetWheelSpeeds()
{
    SpeedType_t speeds = {0};

    xTakeSemaphore(ext_wheelSpeedMutexHandle);
    speeds = currentWheelSpeeds;
    xSemaphoreGive(ext_wheelSpeedMutexHandle);
    return speeds;
}

/// @brief  Obtém as correntes das rodas.
/// @return um SpeedType_t com a corrente de cada roda.
/// @note Ver os indexes WHEEL_X_INDEX.
CurrentType_t GetWheelCurrents()
{
    CurrentType_t currents = {0};
    // Espera até que o mutex de corrente esteja disponível
    xTakeSemaphore(ext_wheelCurrentMutexHandle);
    currents = currentWheelCurrents;
    xSemaphoreGive(ext_wheelCurrentMutexHandle);
    return currents;    
}
