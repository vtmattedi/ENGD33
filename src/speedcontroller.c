void currentTask(void *pvParameters)
{
    // Ler ADC de cada motor
    //  Aplicar o PID DE Corrent de cada motor
    //  init PWM
    while (true)
    {
        for (int i = 0; i < 3; i++)
        {
            float target = target_speed[i];
            uint16_t current = readADC(MotorsCurrentADC[i]);
        }
        vTaskDelay(pdMS_TO_TICKS(CURRENT_TASK_MS));
    }
}

void speedTask(void *pvParameters)
{
    // Ler a velocidade de cada roda
    // Setar o target de velocidade de cada roda
    while (true)
    {
        for (int i = 0; i < 3; i++)
        {
        }
        xSemaphoreTake(mutex_speed, portMAX_DELAY);
        currentSpeedVector = {0.0f, 0.0f, 0.0f}; // Reset speed vector
        xSemaphoreGive(mutex_speed);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

float readCurrent(int channel)
{
    // This function would typically read the ADC value for a specific channel.
    // In a real implementation, you would use the appropriate HAL functions.
    uint16_t adc_value = readADC(channel);
    float current = (adc_value / 4096.0f) * 3.3f;
    return current;
}

int getLastCount()
{
    disableISR();
    int last_count = count;
    count = 0; // Reset the count after reading
    enableISR();
    return last_count;
}

float readWheelSpeed(int channelA, int channelB)
{
    // Ver maneiras de ler a velocidade da roda usando os encoders
    //  This function would typically read the speed of a wheel based on encoder values.
    //  In a real implementation, you would use the appropriate HAL functions.
}
struct Vect3 getSpeedVector()
{
    xSemaphoreTake(mutex_speed, portMAX_DELAY);
    struct Vect3 speed = currentSpeedVector;
    xSemaphoreGive(mutex_speed);
    return speed;
}