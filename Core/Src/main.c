#include "main.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h> // strtof için

// Periferik tanımlar
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim1; // TIM1 kullanılıyor
UART_HandleTypeDef huart1;

// PID değişkenleri
volatile float Kp = 2.5f, Ki = 0.2f, Kd = 0.5f;
volatile float setpoint = 13.0f; // Hedef mesafe (cm)
float error = 0.0f, previous_error = 0.0f, integral = 0.0f, derivative = 0.0f, output = 0.0f;

// Diğer değişkenler
uint16_t adc_value = 0;
float distance_cm = 0.0f;
float current_servo_angle = 90.0f;

// UART RX (alıcı) değişkenleri
#define UART_RX_BUFFER_SIZE 64
uint8_t uart_rx_buffer_byte;
char received_command[UART_RX_BUFFER_SIZE];
volatile uint8_t uart_data_received_flag = 0;
volatile uint8_t rx_buffer_index = 0;

char uart_tx_buffer[128];

// Kontrol bayrakları
volatile uint8_t pid_control_enabled = 1;

// Fonksiyon prototipleri
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
void Error_Handler(void);

// ADC değeri → cm dönüşümü
float adc_to_cm(uint16_t adc)
{
    float voltage = (adc * 3.3f) / 4096.0f;

    float calculated_distance = 12.08f * powf(voltage, -1.058f);

    if (calculated_distance < 4.0f || calculated_distance > 30.0f)
        return -1.0f;

    return calculated_distance;
}

// Servo motor açısını ayarla
void set_servo_angle(float angle)
{
    if (angle < 30.0f) angle = 30.0f;
    if (angle > 150.0f) angle = 150.0f;

    current_servo_angle = angle;

    uint32_t pulse = 500 + (angle / 180.0f) * 2000.0f;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
}

// printf için UART yönlendirmesi
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

// PID kontrol fonksiyonu
float pid_control(float setpoint_val, float measured_value)
{
    error = setpoint_val - measured_value;
    integral += error;

    if (integral > 100.0f) integral = 100.0f;
    if (integral < -100.0f) integral = -100.0f;

    derivative = error - previous_error;
    previous_error = error;

    float pid_out = (Kp * error) + (Ki * integral) + (Kd * derivative);

    if (pid_out > 45.0f) pid_out = 45.0f;
    if (pid_out < -45.0f) pid_out = -45.0f;

    return pid_out;
}

// UART RX Tamamlandı Kesmesi Callback'i
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (uart_rx_buffer_byte != '\n' && rx_buffer_index < (UART_RX_BUFFER_SIZE - 1))
        {
            received_command[rx_buffer_index++] = uart_rx_buffer_byte;
        }
        else
        {
            received_command[rx_buffer_index] = '\0';
            uart_data_received_flag = 1;
            rx_buffer_index = 0;
        }
        HAL_UART_Receive_IT(&huart1, &uart_rx_buffer_byte, 1);
    }
}

// Ana program
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_TIM1_Init();
    MX_USART1_UART_Init();

    HAL_ADC_Start(&hadc1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    HAL_UART_Receive_IT(&huart1, &uart_rx_buffer_byte, 1);

    set_servo_angle(90.0f);

    while (1)
    {
        // ***** UART Komut İşleme *****
        if (uart_data_received_flag)
        {
            uart_data_received_flag = 0;

            if (strncmp(received_command, "SET_PID:", 8) == 0)
            {
                float p_new, i_new, d_new;
                char *token;

                token = strtok(received_command + 8, ",");
                if (token && sscanf(token, "P%f", &p_new) == 1)
                {
                    token = strtok(NULL, ",");
                    if (token && sscanf(token, "I%f", &i_new) == 1)
                    {
                        token = strtok(NULL, "\n");
                        if (token && sscanf(token, "D%f", &d_new) == 1)
                        {
                            Kp = p_new;
                            Ki = i_new;
                            Kd = d_new;
                            printf("PID Ayarlari: P=%.2f, I=%.2f, D=%.2f\r\n", Kp, Ki, Kd);
                        }
                    }
                }
            }
            else if (strncmp(received_command, "SET_SETPOINT:", 13) == 0)
            {
                float new_setpoint;
                if (sscanf(received_command + 13, "%f", &new_setpoint) == 1)
                {
                    setpoint = new_setpoint;
                    printf("Yeni Setpoint: %.2f cm\r\n", setpoint);
                }
            }
            else if (strncmp(received_command, "START_CONTROL", 13) == 0)
            {
                pid_control_enabled = 1;
                integral = 0.0f;
                previous_error = 0.0f;
                printf("Kontrol BASLATILDI\r\n");
            }
            else if (strncmp(received_command, "STOP_CONTROL", 12) == 0)
            {
                pid_control_enabled = 0;
                set_servo_angle(90.0f);
                printf("Kontrol DURDURULDU\r\n");
            }
        }

        // ***** Sensör Okuma ve PID Kontrolü *****
        if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK)
        {
            adc_value = HAL_ADC_GetValue(&hadc1);
            distance_cm = adc_to_cm(adc_value);

            if (distance_cm == -1.0f)
            {
                // Geçersiz mesafe durumunda PID'yi etkilemeyebilir veya durdurabilirsiniz.
            }
            else
            {
                if (pid_control_enabled)
                {
                    output = pid_control(setpoint, distance_cm);
                    float servo_angle = 90.0f + output;
                    set_servo_angle(servo_angle);
                }
            }

            // LED kontrolü: Top 10 cm ile 20 cm arasındaysa LED ON, değilse OFF
            if (distance_cm > 10.0f && distance_cm < 20.0f)
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
            else
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        }

        // ***** Arayüze Veri Gönderme (Periyodik) *****
        snprintf(uart_tx_buffer, sizeof(uart_tx_buffer), "POS:%.2f,SET:%.2f,SERVO:%.2f\r\n",
                 distance_cm, setpoint, current_servo_angle);
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), HAL_MAX_DELAY);

        HAL_Delay(50);
    }
}

// Sistem Saat Konfigürasyonu (72MHz HSE ile)
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

// GPIO Başlatma
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE(); // Gerekli değilse kaldırılabilir (PB0 kaldırıldığı için)

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // PC13 LED Pin
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // PA9 (USART1_TX) ve PA10 (USART1_RX) için UART GPIO Ayarları
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PA8 (TIM1_CH1) PWM çıkışı için GPIO ayarları
    GPIO_InitStruct.Pin = GPIO_PIN_8; // PA8
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PA1 (ADC1_IN1) Analog Giriş için GPIO Ayarı
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

// ADC1 Başlatma (PA1, Channel 1)
static void MX_ADC1_Init(void)
{
    __HAL_RCC_ADC1_CLK_ENABLE();

    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

// TIM1 (PWM) Başlatma (PA8, TIM1_CH1) - Gelişmiş Ayarlar Kaldırıldı
static void MX_TIM1_Init(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 72 - 1; // 72MHz / 72 = 1MHz (1 us tick)
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 20000 - 1; // 20ms periyot (yaklaşık 50Hz)
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0; // Eğer bu satır hata verirse kaldırılabilir (basit PWM için gereksiz olabilir)
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; // Bu satır da hata verirse kaldırılabilir
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 1500;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    // TIM1, Gelişmiş Kontrol Timer'ı olduğu için OCNPolarity ve OCNIdleState alanları olabilir.
    // Eğer bu satırlar da hata verirse veya kullanılmıyorsa kaldırılabilir:
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    // HAL_TIMEx_BreakDeadTimeConfig çağrısı tamamen kaldırıldı.
}

// USART1 (UART) Başlatma (PA9 TX, PA10 RX)
static void MX_USART1_UART_Init(void)
{
    __HAL_RCC_USART1_CLK_ENABLE();

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

// Hata İşleyici Fonksiyon
void Error_Handler(void)
{
  while(1)
  {
    // Hata durumunda LED yakıp söndürme eklenebilir
  }
}
