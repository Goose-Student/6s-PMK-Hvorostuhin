#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

//
// Константы для настройки таймеров
#define TIMER_PRESCALER 539
#define PWM_TIMER_PRESCALER 10799
#define PWM_TIMER_PERIOD 32000;

uint16_t TIM_VALUE = 0;

// Инициализация порта
void initPorts() {
    GPIO_InitTypeDef port;
    GPIO_StructInit(&port);

    // Включение тактирования порта B
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    port.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    port.GPIO_Pin = GPIO_Pin_8;
    port.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &port);

    // Включение тактирования порта A
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    port.GPIO_Mode = GPIO_Mode_AF_PP;
    port.GPIO_Pin = GPIO_Pin_6;
    port.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &port);
}

// Инициализация таймера захвата
void initCaptureTimer() {
    // TIM4, CHANNEL 3 - PB8, TIM3 CH1 - PA6
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    TIM_TimeBaseInitTypeDef timer;
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = TIMER_PRESCALER;
    // timer.TIM_Period = TIMER_PERIOD;
    TIM_TimeBaseInit(TIM4, &timer);

    TIM_ICInitTypeDef ic;
    ic.TIM_Channel = TIM_Channel_3;
    ic.TIM_ICPolarity = TIM_ICPolarity_Rising;
    ic.TIM_ICSelection = TIM_ICSelection_DirectTI;
    ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    ic.TIM_ICFilter = 0;
    TIM_ICInit(TIM4, &ic);

    TIM_ITConfig(TIM4, TIM_IT_CC3, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
    NVIC_EnableIRQ(TIM4_IRQn);
}

// Обработчик прерывания таймера 4
void TIM4_IRQHandler() {
    if(TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET) {
        TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);
        TIM_VALUE = TIM_GetCapture3(TIM4);
    }
}

// Расчет разницы времени
uint16_t diffTime(uint16_t a, uint16_t b) {
    return ( a >> b ) ? ( a - b ) : (UINT16_MAX - b + a);
}

int main() {
    initPorts();
    initCaptureTimer();
    while(1) {
    }
    return 0;
}
