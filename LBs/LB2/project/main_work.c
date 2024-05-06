#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

//
// Константы для настройки таймеров
#define TIMER_PRESCALER 540 - 1
#define TIMER_PERIOD 0xFF4F
#define PWM_TIMER_PRESCALER 21600 - 1
#define PWM_TIMER_PERIOD 200

uint16_t capture1 = 0, capture2 = 0;
uint16_t first_capture = 1, ready_capture = 0;
uint16_t period_capture = 0;

// Инициализация порта
void initPort() {
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
    timer.TIM_Period = TIMER_PERIOD;
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
        capture1 = capture2;
        capture2 = TIM_GetCapture3(TIM4);
        if(!first_capture)
            ready_capture = 1;
        first_capture = 0;
    }
}

// Расчет разницы времени
uint16_t diffTime(uint16_t a, uint16_t b) {
    return ( a >> b ) ? ( a - b ) : (UINT16_MAX - b + a);
}

// Инициализация таймера ШИМ
void initPWMTimer() {
    // TIM3, CHANNEL 1 - PA6, TIM2 CH4 - PA3
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    TIM_TimeBaseInitTypeDef PWM_timer;
    TIM_TimeBaseStructInit(&PWM_timer);
    // 21600 = 108000000 * 0.00020
    PWM_timer.TIM_Prescaler = PWM_TIMER_PRESCALER;
    PWM_timer.TIM_Period = PWM_TIMER_PERIOD;
    TIM_TimeBaseInit(TIM3, &PWM_timer);

    TIM_OCInitTypeDef oc;
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState= TIM_OutputState_Enable;
    oc.TIM_Pulse = 0xFF;
    TIM_OC1Init(TIM3, &oc);

    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
    NVIC_EnableIRQ(TIM3_IRQn);
}

// Обработчик прерывания таймера 3
void TIM3_IRQHandler() {
    if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        TIM_SetCompare1(TIM3, period_capture);
    }
}

int main() {
    initPort();
    initCaptureTimer();
    initPWMTimer();
    while(1) {
        if(ready_capture) {
            NVIC_DisableIRQ(TIM4_IRQn);
            ready_capture = 0;
            period_capture = diffTime(capture2, capture1);
            NVIC_EnableIRQ(TIM4_IRQn);
        }
    }
    return 0;
}
