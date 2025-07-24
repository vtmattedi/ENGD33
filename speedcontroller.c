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
/*
    Constantes de PID para velocidade e corrente
    Modificar no .h para alterar os valores das constantes
*/
PID_LOCATION baseCurrentType_t PID_CONSTANTS_SPEED[WHEELS_COUNT][3] = {
    PID_CONSTANTS_SPEED_A,
    PID_CONSTANTS_SPEED_B,
    PID_CONSTANTS_SPEED_C};

PID_LOCATION baseCurrentType_t PID_CONSTANTS_CURRENT[WHEELS_COUNT][3] = {
    PID_CONSTANTS_CURRENT_A,
    PID_CONSTANTS_CURRENT_B,
    PID_CONSTANTS_CURRENT_C};

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
static baseSpeedType_t readEncoderSpeed(TIM_HandleTypeDef *htim);
static baseCurrentType_t readMotorCurrent(uint8_t adc_channel);
static void setMotorPWM(WheelInfo *wheel, baseCurrentType_t value);
static void SpeedControlTask(void *argument);
static void CurrentControlTask(void *argument);
// Função de erro definida no main.c
extern void Error_Handler(void);

/// @brief Tarefa de inicialização do controlador de velocidade e corrente.
/// Inicializa as filas, mutexes e tarefas do controlador de velocidade e corrente.
/// @note Essa Função deve ser ANTES de acionar o escalonador e APÓS a inicialização do sistema,
/// ela, e as tasks inicializadas por ela assumem que os pinos, timers, PWM e ADC estão corretamente configurados e prontos para uso.
/// Além disso caso ela falhe em qualquer ponto, ela chama o Error_Handler() para tratar o erro.
void SpeedController_Init(void)
{
    // Inicialização das filas e mutexes
    vSemaphoreCreateBinary(speedMutexHandle);            // Semaphore para entrada de velocidade
    vSemaphoreCreateBinary(currentMutexHandle);          // semaphore para entrada de corrente
    vSemaphoreCreateBinary(ext_wheelSpeedMutexHandle);   // semaphore para velocidade atual
    vSemaphoreCreateBinary(ext_wheelCurrentMutexHandle); // semaphore para corrente atual

    speedQueueHandle = xQueueCreate(QUEUE_LENGTH, sizeof(SpeedType_t));
    currentQueueHandle = xQueueCreate(QUEUE_LENGTH, sizeof(CurrentType_t));
    // Se houver erro na criação das filas ou mutexes, chama o Error_Handler
    if (speedQueueHandle == NULL || currentQueueHandle == NULL ||
        speedMutexHandle == NULL || currentMutexHandle == NULL ||
        ext_wheelSpeedMutexHandle == NULL || ext_wheelCurrentMutexHandle == NULL)
    {
        /*
            Ver a criação das tasks abaixo.
            set_error(QUEUE_ERROR); // Define um erro de fila
        */

        Error_Handler();
    }

    /* xTaskCreate retorna pdPASS em caso de sucesso e errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY em caso de falha
     pdPASS é definido como 1 e errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY é definido como -1
     definida:   src\Middlewares\Third_Party\FreeRTOS\Source\include\projdefs.h
     source: https://www.freertos.org/Documentation/02-Kernel/04-API-references/01-Task-creation/01-xTaskCreate
    */
    int8_t success = 1;
    // Inicialização das tarefas
    success *= xTaskCreate(SpeedControlTask,                 /* Pointer para a task */
                           "SpeedControl",                   /* Nome da task */
                           SPEED_CONTROLLER_TASK_STACK_SIZE, /* Tamanho da pilha da task */
                           NULL,                             /* Argumento da task */
                           SPEED_CONTROLLER_TASK_PRIORITY,   /* Prioridade da task */
                           &speedControlTaskHandle           /* Handle da task */
    );
    success *= xTaskCreate(CurrentControlTask,                /* Pointer para a task */
                           "CurrentControl",                  /* Nome da task */
                           TORQUE_CONTROLLER_TASK_STACK_SIZE, /* Tamanho da pilha da task */
                           NULL,                              /* Argumento da task */
                           TORQUE_CONTROLLER_TASK_PRIORITY,   /* Prioridade da task */
                           &currentControlTaskHandle          /* Handle da task */
    );

    if (success != pdPASS)
    {
        // Se houver erro na criação das tarefas, chama o Error_Handler
        /*
            Seria interessante verificar onde ocorreu o erro, mas não tem um sistema definido no projeto.
            E criar tal sistema foge do escopo do projeto.
            if (!speedControlTaskHandle)
            {
                set_error(SPEED_CONTROL_TASK_ERROR);
            }
            if (!currentControlTaskHandle)
            {
                set_error(CURRENT_CONTROL_TASK_ERROR);
            }
            void set_error(int error_code)
            {
                error |= error_code; // adiciona o erro ao bitmask
            }
            void clear_error(int error_code)
            {
                error &= ~error_code; // remove o erro do bitmask
            }
        */
        Error_Handler();
    }
}

// --- Tarefa de cálculo de velocidades das rodas ---
/// @brief Tarefa que calcula as velocidades das rodas a partir de um vetor alvo.
/// Recebe o vetor alvo da fila, calcula as velocidades das rodas e envia para a fila
/// de corrente.
/// @param argument Argumento da tarefa (não utilizado).
void SpeedControlTask(void *argument)
{
    static SpeedType_t target_rps; // target de velocidade (RPS) vindo externamente da fila
    // Tem que ser mesmo tipo da fila de corrente.
    static SpeedType_t output_rps;             // Saída de velocidade (RPS) para a fila de corrente
    static baseSpeedType_t integral[WHEELS_COUNT] = {0}; // Integral do erro para cada roda.
    static baseSpeedType_t _currentWheelSpeeds[WHEELS_COUNT] = {0}; // Velocidades das rodas atual (ultima leitura)
    // tempo em que a tarefa foi chamada pela ultima vez
    for (;;)
    {
        TickType_t xLastWakeTime = xTaskGetTickCount();
        // Espera até que a fila do input de velocidade esteja disponível
        xTakeSemaphore(speedMutexHandle);
        if (!xQueueIsQueueEmpty(speedQueueHandle))
        {
            // Recebe o vetor de velocidade (RPS) da fila de velocidade
            // Se a fila estiver vazia continua com o valor anterior
            xQueueReceive(speedQueueHandle, &target_rps, QUEUE_MAX_WAIT_TICKS);
        }
        xSemaphoreGive(speedMutexHandle); // Libera o mutex da fila do input de velocidade

      
        // Aqui ja temos os dados da fila se precisar e
        // Calcula as velocidades das rodas a partir do vetor alvo
        for (size_t i = 0; i < WHEELS_COUNT; i++)
        {
            // Target (ou set point): input recebido externamente (RPS)
            // Input (ou real) = velocidade lida com readEncoderSpeed(RPS);
            // Output = Saida em RPS para a fila de corrente;
            // erro = set point - real;
            // delta = real - ultimo valor real lido;
            baseSpeedType_t set_point = target_rps.data[i];
            baseSpeedType_t real = (baseSpeedType_t)readEncoderSpeed(wheelInfo[i].encoder_tim); // Lê a velocidade usando o encoder
            _currentWheelSpeeds[i] = real;                                                       // Atualiza a ultima velocidade lida
            // Aplicar PID aqui
            // NOTA: Tirei o / SPEED_CONTROLLER_TASK_MS  pois é constante, portanto sera incorporado no PID
            // NOTA: Inclui o erro derivativo (Kd) para ficar completo. se Não formos usar,
            // podemos seta Kd = 0
            integral[i] += (set_point - real); // Integral do erro (acumulada)
            output_rps.data[i] = (set_point - real) * PID_CONSTANTS_SPEED[i][Kp] +
                            integral[i] * PID_CONSTANTS_SPEED[i][Ki];
        }
        
        // Espera até que o mutex de velocidade atual esteja disponível
        xTakeSemaphore(ext_wheelSpeedMutexHandle);
        {
            // Copia as velocidades atuais para a variável externa
            for (size_t i = 0; i < WHEELS_COUNT; i++)
            {
                currentWheelSpeeds.data[i] = _currentWheelSpeeds[i];
            }
        }
        xSemaphoreGive(ext_wheelSpeedMutexHandle); // Libera o mutex de velocidade para outras tarefas
        
        xTakeSemaphore(currentMutexHandle); // Espera até que o mutex de corrente esteja disponível
        if (xQueueSend(currentQueueHandle, &output_rps, QUEUE_MAX_WAIT_TICKS ) != pdTRUE)
        {
            // Se a fila estiver cheia, podemos lidar com isso aqui
            // Por exemplo, podemos descartar a mensagem ou tentar novamente mais tarde
        }
        xSemaphoreGive(currentMutexHandle);

        // Espera o tempo da tarefa de controle de velocidade
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SPEED_CONTROLLER_TASK_MS));
    }
}

// --- Tarefa de controle de corrente e envio de PWM ---
/// @brief Tarefa que controla a corrente dos motores e envia o PWM para os motores.
/// Recebe as velocidades das rodas da fila de corrente, calcula a corrente e envia o PWM
/// para os motores.
/// @param argument Argumento da tarefa (não utilizado).
void CurrentControlTask(void *argument)
{
    /*
        static vs não:
        static: Heap
        não static: Stack
        As duas podem ser usadas pois a tarefa é executada em loop infinito
        A diferença é que no heap, talvez seja mais eficiente pois diminui o tamanho do stack
    */
    static SpeedType_t input_rps;                          // target de corrente (Amperes) vindo externamente da fila
    static baseCurrentType_t integral[WHEELS_COUNT] = {0}; // Integral do erro para cada roda.
    static baseCurrentType_t _currentWheelCurrents[WHEELS_COUNT] = {0}; // Correntes das rodas atual (ultima leitura)
    for (;;)
    {   
        TickType_t xLastWakeTime = xTaskGetTickCount();
        xTakeSemaphore(currentMutexHandle);
        // Recebe o vetor de velocidade (RPS) da fila de corrente
        // Se a fila estiver vazia continua com o valor anterior
        if (!xQueueIsQueueEmpty(currentQueueHandle))
        {
            xQueueReceive(currentQueueHandle, &input_rps, QUEUE_MAX_WAIT_TICKS);
        }
        xSemaphoreGive(currentMutexHandle); // Libera o mutex da fila de corrente
        // Espera até que o mutex de corrente atual esteja disponível
        
        // Aplica o PID de corrente para cada roda
        for (size_t i = 0; i < WHEELS_COUNT; i++)
        {
            // PID de corrente:
            // target (ou set point) = Saida da SpeedTask (RPS) -> precisa converter para corrente
            // input (ou valor real) = Corrente lida do ADC (Amperes)
            // output = (Amperes) -> PWM
            baseCurrentType_t set_point = input_rps[i] * RPS_TO_CURRENT; // Corrente alvo (Amperes)
            // Convert RPS para corrente utilizando a constante de conversão
            baseCurrentType_t real = (baseCurrentType_t)readMotorCurrent(wheelInfo[i].adc_channel); // Lê a corrente do motor
            _currentWheelCurrents[i] = real;                                                         // Atualiza a ultima corrente lida
            integral[i] += set_point - real; // Integral do erro (acumulada)
            // Controle PI da corrente (Torque) do motor
            baseCurrentType_t output = (set_point - real) * PID_CONSTANTS_CURRENT[i][Kp] +
                                        integral[i] * PID_CONSTANTS_CURRENT[i][Ki];
            
            // Aqui é especificamente float e não baseCurrentType_t pois o valor do motor é um percentual
            float motor_value = output * CURRENT_TO_MOTOR_PERCENTAGE; // Valor do motor (PWM) a ser enviado
            // Envia o sinal PWM para o motor
            // motor_value deve ser entre -100 e 100
            // com o sinal indicando a direção do motor
            // e o valor indicando a intensidade em %
            setMotorPWM(&wheelInfo[i], motor_value);
        }
        xTakeSemaphore(ext_wheelCurrentMutexHandle);
        for (size_t i = 0; i < WHEELS_COUNT; i++)
        {
            // Copia as correntes atuais para a variável externa
           currentWheelCurrents.data[i] = _currentWheelCurrents[i]; // Atualiza a corrente atual
        }
        xSemaphoreGive(ext_wheelCurrentMutexHandle); // Libera o mutex de corrente para outras tarefas
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TORQUE_CONTROLLER_TASK_MS));
    }
}

/// @brief Lê a velocidade do encoder associado ao timer.
/// @param htim Ponteiro para o handle do timer associado ao encoder.
/// @return Velocidade do encoder em RPS.
static baseSpeedType_t readEncoderSpeed(TIM_HandleTypeDef *htim)
{
    // Medida anterior é sempre 0
    uint32_t count = __HAL_TIM_GET_COUNTER(htim); // Lê o contador do timer
    __HAL_TIM_SET_COUNTER(htim, 0);               // Reseta o contador do timer para manter ele sempre 0
    // Pulsos por revolução / Delta T
    baseSpeedType_t speed = (count / PULSES_PER_REVOLUTION) / (SPEED_CONTROLLER_TASK_MS / 1000.0f); // Calcula a velocidade em RPS
    // Checa se o timer está contando para baixo (velocidade negativa)
    speed = __HAL_TIM_IS_TIM_COUNTING_DOWN(htim) ? -speed : speed; // Inverte a velocidade se o timer estiver contando para baixo
    return speed;
}

/// @brief Lê a corrente do motor a partir do canal ADC especificado.
/// @param channel Canal ADC a ser lido.
/// @return Corrente do motor em Amperes.
static baseCurrentType_t readMotorCurrent(uint8_t channel)
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
    baseCurrentType_t voltage = raw * Voltage_REF / (baseCurrentType_t)((1 << ADC_BITS) - 1);
    // Converter Tensão em Corrente (ACS_712) Tem um divisor de tensão 1:1
    // corrente = (tensão lida * divisor de tensão - offset) / sensibilidade
    baseCurrentType_t amp = (voltage * ACS_VOLTAGE_DIVISOR - ACS_712_OFFSET) / ACS_712_SENSITIVITY; // Corrente em Amperes
    return amp;                                                                                                 // Retorna a corrente em Amperes
}

/// @brief Configura o PWM para o motor especificado.
/// @param channel Canal PWM a ser configurado.
/// @param pwm Valor do PWM a ser configurado (0-100%).
/// @note O PWM é invertido, pois o canal A é o normal e o canal B é o invertido.
static void setMotorPWM(WheelInfo *wheel, baseCurrentType_t value)
{
    // trasformar o valor recebido em Duty Cycle
    // Enviar 0 para o canal A/B (dependendo do sinal)
    // Enviar o Duty Cycle para o canal B/A (dependendo do sinal)
    //
    int positive = value >= 0; // Verifica se o valor é positivo ou negativo
    uint16_t duty_cycle = (uint16_t)(abs(value) / 100.0f * __HAL_TIM_GET_AUTORELOAD(&WHEEL_PWM_TIM)); // Converte o valor para Duty Cycle (0-100%)
    // Primeiro desabilita o PWM do canal contrario
    __HAL_TIM_SET_COMPARE(&WHEEL_PWM_TIM, positive ? wheel->pwm_channel_b : wheel->pwm_channel_a, 0);
    // Depois habilita o PWM do canal correto 
    __HAL_TIM_SET_COMPARE(&WHEEL_PWM_TIM, positive ? wheel->pwm_channel_a : wheel->pwm_channel_b, duty_cycle);
}


void GetWheelSpeeds(SpeedType_t *output)
{
    if (output == NULL)
    {
        return; // Verifica se o ponteiro é nulo
    }

    xTakeSemaphore(ext_wheelSpeedMutexHandle);
    memcpy(output, &currentWheelSpeeds, sizeof(SpeedType_t)); // Copia as velocidades atuais para o ponteiro de saída
    xSemaphoreGive(ext_wheelSpeedMutexHandle);
}

/// @brief Obtém as correntes atuais das rodas.
/// @param output um ponteiro para um CurrentType_t onde as correntes serão armazenadas.
/// @note Se o pointer for nulo, a função não faz nada. Reponsabilidade do usuario garantir o tipo do ponteiro.
void GetWheelCurrents(CurrentType_t *output)
{
    if (output == NULL)
    {
        return; // Verifica se o ponteiro é nulo
    }
    // Espera até que o mutex de corrente esteja disponível
    xTakeSemaphore(ext_wheelCurrentMutexHandle);
    memcpy(output, &currentWheelCurrents, sizeof(CurrentType_t)); // Copia as correntes atuais para o ponteiro de saída
    xSemaphoreGive(ext_wheelCurrentMutexHandle);
}

/// @brief  Configura as velocidades das rodas.
/// @param speeds Um array com as velocidades a serem configuradas.
/// @note As velocidades devem estar em RPS e devem ser passadas como um array de SpeedType_t.
/// @note É responsabilidade do chamador garantir que o vetor de velocidades tenha o tamanho correto (WHEELS_COUNT).
void setWheelSpeeds(baseSpeedType_t speeds[])
{
    // Espera até que o mutex de velocidade esteja disponível
    xTakeSemaphore(speedMutexHandle);
    SpeedType_t target_speeds = {0};
    for (size_t i = 0; i < WHEELS_COUNT; i++)
    {
        target_speeds[i] = speeds[i]; // Copia as velocidades para o vetor de destino
    }
    int res  = xQueueSend(speedQueueHandle, &target_speeds, QUEUE_MAX_WAIT_TICKS); // Envia o vetor de velocidades para a fila
    if (res != pdTRUE)
    {
        // Se a fila estiver cheia, podemos lidar com isso aqui
        // Por exemplo, podemos descartar a mensagem ou tentar novamente mais tarde
        // Aqui não vamos fazer nada, mas é bom saber que pode acontecer.
    }
    xSemaphoreGive(speedMutexHandle); // Libera o mutex de velocidade para outras tarefas
}