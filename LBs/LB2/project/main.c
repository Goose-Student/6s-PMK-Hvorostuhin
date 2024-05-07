#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

// Константы для настройки таймеров
uint16_t CAPTURE_PRESCALER = 539;
uint16_t СAPTURE_PERIOD = 32000;

uint16_t PWM_TIMER_PRESCALER = 10799;
uint16_t PWM_TIMER_PERIOD = 32000;

uint32_t TimerValue = 0;  // Объявление глобальной переменной для хранения значения таймера

void Port_Init(void) {  // Функция инициализации порта
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  // Включение тактирования порта A
  GPIO_InitTypeDef GPIO_InitStructure;  // Объявление структуры для инициализации порта
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;  // Настройка пина 6
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  // Режим входа, подтяжка отключена
  GPIO_Init(GPIOA, &GPIO_InitStructure);  // Применение настроек к порту A
}

void TIM3_CH1_Capture_Init(void) {  // Функция инициализации таймера TIM3 CH1 в режиме захвата
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  // Включение тактирования таймера TIM3
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  // Объявление структуры для инициализации таймера
  TIM_TimeBaseStructure.TIM_Period = СAPTURE_PERIOD;    // Период счета таймера
  TIM_TimeBaseStructure.TIM_Prescaler = CAPTURE_PRESCALER;     // Предделитель таймера
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;  // Делитель частоты таймера
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  // Режим счета вверх
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);  // Применение настроек к таймеру TIM3

  TIM_ICInitTypeDef TIM_ICInitStructure;  // Объявление структуры для инициализации канала таймера
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;  // Настройка канала 1
  TIM_ICInitStructure.TIM_ICPolarity =
      TIM_ICPolarity_Rising;  // Полярность сигнала - передний фронт
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;  // Выбор входа - прямой вход
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;  // Предделитель входного сигнала
  TIM_ICInitStructure.TIM_ICFilter = 0x0;  // Фильтр входного сигнала
  TIM_ICInit(TIM3, &TIM_ICInitStructure);  // Применение настроек к каналу 1 таймера TIM3

  TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);  // Включение прерывания по событию захвата на канале 1
  TIM_Cmd(TIM3, ENABLE);  // Включение таймера TIM3
}

void TIM3_IRQHandler(void) {  // Обработчик прерывания таймера TIM3
  if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET) {  // Если произошло прерывание по событию захвата на канале 1
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);  // Сброс флага прерывания
    TimerValue = TIM_GetCapture1(TIM3);  // Запись значения таймера в переменную
 
  }
}

int main(void) {  // Главная функция
  __enable_irq();
	NVIC_EnableIRQ(TIM3_IRQn);
  Port_Init();              // Вызов функции инициализации порта
  TIM3_CH1_Capture_Init();  // Вызов функции инициализации таймера
  while (1) {               // Бесконечный цикл
  
	}
}
