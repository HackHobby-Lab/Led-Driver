/* Simple LED Controller with Smooth Fade and Lockout Mode - FIXED RAMP DIP */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Instant single click ON/OFF, hold to ramp, 4C lockout with momentary
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
void handle_click_sequence(uint8_t click_count);

/* Global variables */
uint8_t is_on = 0;                   // LED state: 0=OFF, 1=ON
uint8_t is_locked = 0;               // Lockout mode: 0=normal, 1=locked
uint32_t current_brightness = 0;     // Current brightness level
uint32_t target_brightness = 2999;   // Target brightness when ON (30%)
uint32_t max_brightness = 9999;      // Maximum brightness (100%)

uint8_t button_pressed = 0;
uint32_t button_press_time = 0;
uint8_t ramp_direction = 1;          // 1=ramp up, 0=ramp down
uint8_t at_extreme = 0;              // 1 = reached MAX or MIN

uint8_t hold_started = 0;            // becomes 1 once press is recognized as hold
uint8_t press_handled = 0;           // becomes 1 after instant action on press
uint8_t instant_action_done = 0;     // tracks if instant action was taken
uint32_t brightness_at_press = 0;    // brightness when button was first pressed

/* Click detection */
#define MAX_CLICKS 4
//#define CLICK_TIMEOUT_MS 500         // Increased from 500ms for easier 4-click
uint8_t click_count = 0;
uint32_t last_click_time = 0;

uint32_t last_brightness = 0;        // Stores brightness to restore
uint32_t min_ramp_brightness = 0;    // 5% duty calculation
uint32_t momentary_brightness = 0;   // 5% for momentary mode
uint32_t ARR = 0;

//#define CLICK_TIME_MS 150            // Max time for a click (not hold)
//#define INSTANT_CLICK_MS 80         // INCREASED: Delay before instant action (was 50ms)
//#define FADE_TIME_MS 200             // Fade in/out duration
#define FADE_STEPS 10                // Number of steps in fade

#define CLICK_TIME_MS 200            // Max time for a click (not hold)
#define INSTANT_CLICK_MS 20          // Much faster response!
#define CLICK_TIMEOUT_MS 450         // Time between clicks
#define FADE_TIME_MS 150             // Faster fade

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
    uart_print("\r\n=== LED Controller with Lockout (FIXED) ===\r\n");
    uart_print("Click: ON/OFF | Hold: Ramp\r\n");
    uart_print("4 Clicks: Lockout Mode\r\n\r\n");

    last_brightness = 2999;
    ARR = __HAL_TIM_GET_AUTORELOAD(&htim3);          // should be 9999
    min_ramp_brightness = (ARR + 1) * 50 / 1000;     // 5% of ARR
    momentary_brightness = min_ramp_brightness;       // 5% for momentary

    while (1)
    {
        uint32_t now = HAL_GetTick();
        GPIO_PinState bs = HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin);

        /* Check for click timeout */
        if (click_count > 0 && (now - last_click_time) > CLICK_TIMEOUT_MS)
        {
            handle_click_sequence(click_count);
            click_count = 0;
        }

        /* --- BUTTON PRESSED (edge) --- */
        if (bs == GPIO_PIN_RESET && !button_pressed)
        {
            button_pressed = 1;
            button_press_time = now;
            hold_started = 0;
            press_handled = 0;
            instant_action_done = 0;  // Reset instant action flag
            brightness_at_press = current_brightness;  // Store brightness at press time
            uart_print("Button PRESSED\r\n");
        }

        /* --- BUTTON RELEASED (edge) --- */
        if (bs == GPIO_PIN_SET && button_pressed)
        {
            uint32_t press_duration = now - button_press_time;
            button_pressed = 0;

            at_extreme = 0;
            hold_started = 0;

//            sprintf(msg, "Button RELEASED (%lums)\r\n", press_duration);
//            uart_print(msg);

            if (press_duration < CLICK_TIME_MS)
            {
                // Count as click for multi-click detection FIRST
                click_count++;
                last_click_time = now;

                // SHORT PRESS - if instant action wasn't done, do it now
                if (!instant_action_done && !is_locked)
                {
                    // Perform the toggle now on release
                    if (is_on)
                    {
                        last_brightness = current_brightness;

                        // Skip fade if we're in a multi-click sequence
                        if (click_count == 1) {
                            smooth_fade_to(0);
                        } else {
                            current_brightness = 0;
                            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                        }
                        is_on = 0;
                    }
                    else
                    {
                        // Skip fade if we're in a multi-click sequence
                        if (click_count == 1) {
                            smooth_fade_to(last_brightness);
                        } else {
                            current_brightness = last_brightness;
                            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, last_brightness);
                        }
                        is_on = 1;
                    }
                }
            }
            else
            {
                // LONG PRESS released
                if (is_locked)
                {
                    // Turn off momentary light
                    smooth_fade_to(0);
                    current_brightness = 0;
                    uart_print(">> Momentary OFF\r\n");
                }
                else
                {
                    // Update is_on state based on current brightness
                    if (current_brightness > 0)
                    {
                        is_on = 1;
                        last_brightness = current_brightness;
                    }
                    else
                    {
                        is_on = 0;
                    }
                    sprintf(msg, ">> Hold released: is_on=%d, brightness=%lu\r\n",
                            is_on, current_brightness);
                    uart_print(msg);
                }
                // Reset click counter after hold
                click_count = 0;
            }

            press_handled = 0;
            instant_action_done = 0;
        }

        /* --- BUTTON HELD (after crossing CLICK_TIME_MS) --- */
        if (button_pressed && !hold_started && (now - button_press_time) > CLICK_TIME_MS)
        {
            hold_started = 1;
            click_count = 0; // Clear clicks on hold

            if (is_locked)
            {
                // Lockout: turn on momentary light at 5%
                uart_print(">> LOCKOUT: Momentary ON (5%)\r\n");
                smooth_fade_to(momentary_brightness);
                current_brightness = momentary_brightness;
            }
            else
            {
                // Normal ramping mode - NO TOGGLE, just start ramping from current state
                uart_print(">> HOLD mode activated - starting ramp\r\n");

                // Determine ramp direction based on brightness at press time
                if (brightness_at_press <= min_ramp_brightness)
                {
                    ramp_direction = 1;  // force ramp UP
                    uart_print(">> At MIN - forcing ramp UP\r\n");
                }
                else if (brightness_at_press >= max_brightness)
                {
                    ramp_direction = 0;  // force ramp DOWN
                    uart_print(">> At MAX - forcing ramp DOWN\r\n");
                }
                else
                {
                    // Toggle direction for middle brightness values
                    ramp_direction ^= 1;
                    sprintf(msg, ">> Toggling ramp direction to %s\r\n",
                            ramp_direction ? "UP" : "DOWN");
                    uart_print(msg);
                }

                sprintf(msg, ">> HOLD started from %lu, direction: %s\r\n",
                        current_brightness, ramp_direction ? "UP" : "DOWN");
                uart_print(msg);
            }
        }

        /* --- Perform ramping if hold is active (not in lockout) --- */
        if (hold_started && !at_extreme && !is_locked)
        {
            if (ramp_direction == 1) // ramp UP
            {
                uint32_t step = 50;  // Reduced step size for smoother ramping
                if (current_brightness + step >= max_brightness)
                {
                    current_brightness = max_brightness;
                    at_extreme = 1;
                    uart_print(">> HOLD: MAX reached\r\n");
                }
                else
                {
                    current_brightness += step;
                }
            }
            else // ramp DOWN
            {
                uint32_t step = 50;  // Reduced step size for smoother ramping
                if (current_brightness <= min_ramp_brightness + step)
                {
                    current_brightness = min_ramp_brightness;
                    at_extreme = 1;
                    uart_print(">> HOLD: MIN reached (5%)\r\n");
                }
                else
                {
                    current_brightness -= step;
                }
            }

            target_brightness = current_brightness;
            last_brightness = current_brightness;
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, current_brightness);

            // Slower ramp - 2 seconds from 5% to 100%
            HAL_Delay(10);
        }

        HAL_Delay(1); // small loop delay
    }
}

void handle_click_sequence(uint8_t clicks)
{
    char msg[100];
    sprintf(msg, ">> Processing %d click(s)\r\n", clicks);
    uart_print(msg);

    if (is_locked)
    {
        /* LOCKOUT MODE BEHAVIOR */
        if (clicks == 4)
        {
            // 4 clicks: Exit lockout
            is_locked = 0;
            is_on = 0;  // Start in OFF state when exiting lockout
            current_brightness = 0;
            smooth_fade_to(0);
            uart_print(">> UNLOCKED - Back to OFF\r\n");
        }
        else if (clicks == 3)
        {
            // 3 clicks: Battery check
            uart_print(">> LOCKOUT: Battery Check\r\n");
            // TODO: Add actual battery voltage reading here
            uart_print("Battery: OK (placeholder)\r\n");
        }
        else
        {
            sprintf(msg, ">> LOCKOUT: %d click(s) - no action\r\n", clicks);
            uart_print(msg);
        }
    }
    else
    {
        /* NORMAL MODE BEHAVIOR */
        if (clicks == 1)
        {
            // Single click was already handled instantly on press
            sprintf(msg, ">> Single click already handled (is_on=%d)\r\n", is_on);
            uart_print(msg);
        }
        else if (clicks == 4)
        {
            uart_print(">> ENTERING LOCKOUT MODE\r\n");

            is_locked = 1;
            is_on = 0;

            // Fade to 0 when entering lockout
            smooth_fade_to(0);
            current_brightness = 0;

            uart_print(">> Lockout active - brightness at 0\r\n");
        }
        else
        {
            sprintf(msg, ">> %d click(s) - no action assigned\r\n", clicks);
            uart_print(msg);
        }
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
            current_brightness, (current_brightness * 100.0) / 9999.0);
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
