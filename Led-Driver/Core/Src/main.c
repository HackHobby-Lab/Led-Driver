/* Simple LED Controller with Smooth Fade */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Simple click ON/OFF with fade, hold to ramp up/down
  ******************************************************************************
  */
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Private variables */
ADC_HandleTypeDef hadc1;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;

/* Private function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
void delay(int x);
void uart_print(const char* str);
void smooth_fade_to(uint32_t target_brightness);

/* Global variables */
uint8_t is_on = 0;                   // LED state: 0=OFF, 1=ON
uint32_t current_brightness = 0;     // Current brightness level
uint32_t target_brightness = 2999;  // Target brightness when ON (30%)
uint32_t max_brightness = 9999;     // Maximum brightness (100%)

uint8_t button_pressed = 0;
uint32_t button_press_time = 0;
uint8_t ramp_direction = 1;          // 1=ramp up, 0=ramp down
uint8_t at_extreme = 0;   // 1 = reached MAX or MIN and must stop until released

uint32_t last_brightness = 0; // Stores brightness to restore on next click


#define CLICK_TIME_MS 300            // Max time for a click (not hold)
#define FADE_TIME_MS 500             // Fade in/out duration
#define FADE_STEPS 50                // Number of steps in fade

int main(void)
{
    char msg[100];

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_SPI1_Init();
    MX_USART1_UART_Init();
    MX_TIM1_Init();
    MX_TIM3_Init();

    // Configure PWM
    __HAL_TIM_SET_AUTORELOAD(&htim3, 9999);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

    HAL_Delay(500);
    uart_print("\r\n=== Simple LED Controller ===\r\n");
    uart_print("Click: Smooth ON/OFF\r\n");
    uart_print("Hold: Ramp UP/DOWN\r\n\r\n");

    uint32_t current_time = 0;
    last_brightness = 2999;
    while (1)
    {
        current_time = HAL_GetTick();

        GPIO_PinState button_state = HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin);

        /* Button pressed */
        if (button_state == GPIO_PIN_RESET && !button_pressed)
        {
            button_pressed = 1;
            button_press_time = current_time;

            // Do NOT reset extreme here; extreme should only reset on release
            uart_print("Button PRESSED\r\n");
        }

        /* Button released */
        else if (button_state == GPIO_PIN_SET && button_pressed)
        {
            button_pressed = 0;
            uint32_t press_duration = current_time - button_press_time;

            // Release --> allow ramp again
            at_extreme = 0;

            sprintf(msg, "Button RELEASED (%lums)\r\n", press_duration);
            uart_print(msg);

            if (press_duration < CLICK_TIME_MS)
            {
                // Toggle ON/OFF with fade
                if (is_on)
                {
                    uart_print(">> CLICK: Fading OFF\r\n");
                    last_brightness = current_brightness;  // Save current level
                    smooth_fade_to(0);
                    current_brightness = 0;
                    is_on = 0;
                }
                else
                {
                    uart_print(">> CLICK: Fading ON\r\n");
                    // Fade back to the last saved brightness
                    smooth_fade_to(last_brightness);
                    current_brightness = last_brightness;
                    is_on = 1;
                }
            }

            else
            {
                // HOLD → set next direction
                if (current_brightness >= max_brightness)
                    ramp_direction = 0;
                else if (current_brightness == 0)
                    ramp_direction = 1;
            }
        }

        /* Button held */
        else if (button_pressed)
        {
            uint32_t hold_duration = current_time - button_press_time;

            if (hold_duration > CLICK_TIME_MS)
            {
                // If reached min/max, ignore holds until release
                if (at_extreme)
                    continue;

                // RAMP UP
                if (ramp_direction == 1)
                {
                    current_brightness += 100;

                    if (current_brightness >= max_brightness)
                    {
                        current_brightness = max_brightness;
                        at_extreme = 1;     // STOP ramping
                        ramp_direction = 0; // Next time ramp down
                        uart_print(">> HOLD: MAX reached, stop until release\r\n");
                    }
                }
                // RAMP DOWN
                else
                {
                    if (current_brightness > 100)
                        current_brightness -= 100;
                    else
                        current_brightness = 0;

                    if (current_brightness == 0)
                    {
                        at_extreme = 1;    // STOP ramping
                        ramp_direction = 1; // Next time ramp up
                        uart_print(">> HOLD: MIN reached, stop until release\r\n");
                    }
                }

                target_brightness = current_brightness;
                last_brightness = target_brightness;
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, current_brightness);
                HAL_Delay(10);
            }
        }


        HAL_Delay(5);
    }
}

void smooth_fade_to(uint32_t target)
{
    char msg[80];
    uint32_t start_brightness = current_brightness;
    int32_t brightness_diff = (int32_t)target - (int32_t)start_brightness;
    uint32_t step_delay = FADE_TIME_MS / FADE_STEPS;

    sprintf(msg, "Fading from %lu to %lu\r\n", start_brightness, target);
    uart_print(msg);

    for (int i = 1; i <= FADE_STEPS; i++)
    {
        current_brightness = start_brightness + (brightness_diff * i / FADE_STEPS);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, current_brightness);
        HAL_Delay(step_delay);
    }

    current_brightness = target;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, current_brightness);

    sprintf(msg, "Fade complete at %lu (%.1f%%)\r\n",
            current_brightness, (current_brightness * 100.0) / 48000.0);
    uart_print(msg);
}

void delay(int x)
{
    volatile int i, j;
    for (i = 0; i < x; i++)
    {
        j++;
    }
}

void uart_print(const char* str)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode = ADC_SCAN_SEQ_FIXED;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.LowPowerAutoPowerOff = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
    hadc1.Init.OversamplingMode = DISABLE;
    hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) { Error_Handler(); }
    sConfig.Channel = ADC_CHANNEL_8;
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
}

static void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) { Error_Handler(); }
}

static void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 65535;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) { Error_Handler(); }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

static void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 11999;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) { Error_Handler(); }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) { Error_Handler(); }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) { Error_Handler(); }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
    HAL_TIM_MspPostInit(&htim3);
}

static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK) { Error_Handler(); }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) { Error_Handler(); }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) { Error_Handler(); }
    if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) { Error_Handler(); }
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = Button_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = Led_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Led_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { }
#endif
