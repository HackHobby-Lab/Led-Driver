#include "debug.h"
// #include "ch32v203.h"

void TIM2_PWM_Init(uint32_t freq)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = {0};
    TIM_OCInitTypeDef TIM_OCInitStructure = {0};

    // Enable clocks for GPIOA and TIM2
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // Configure PA1 (TIM2_CH2) as alternate function push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Timer base configuration
    uint32_t timer_clock = 96000000;  // SYSCLK = 96 MHz
    uint32_t prescaler = 1 - 1;       // Prescaler = 1 (no scaling)
    uint32_t period = (timer_clock / (prescaler + 1)) / freq - 1;

    TIM_TimeBaseStructure.TIM_Period = period;
    TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // PWM mode configuration (toggle mode for square wave)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;  // Toggle output on compare match
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

int main(void)
{
    SystemInit();
    Delay_Init();

    // Example: Generate 48 MHz square wave on PA1
    TIM2_PWM_Init(2200000); // 48 MHz PWM frequency

    while(1)
    {
        // Main loop does nothing since the PWM is generated in the background
    }
}