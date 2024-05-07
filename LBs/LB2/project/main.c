#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

/* Константы для настройки таймеров */
static uint16_t CAPTURE_PRESCALER = 539;
static uint16_t СAPTURE_PERIOD = 32000;
static uint16_t PWM_PRESCALER = 10799;
static uint16_t PWM_PERIOD = 32000;

/* Хранение значения таймера */
static uint32_t TIM3_VALUE = 0;


/* Функция инициализации портов */
static void initGPIO(void)
{
	/* Объявление структуры для инициализации порта */
  GPIO_InitTypeDef GPIO_InitStructure;
  /* Включение тактирования порта A */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  
  /* Настройка порта PA6 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;             /* Настройка пина 6 */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; /* Режим входа, подтяжка отключена */
  GPIO_Init(GPIOA, &GPIO_InitStructure);                /* Применение настроек к порту A */
}

static void initTIM3(void)
{
	/* Объявление структуры для инициализации канала таймера */
  TIM_ICInitTypeDef TIM_ICInitStructure;
	/* Объявление структуры для инициализации таймера */
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
  /* Включение тактирования таймера TIM3 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  
  /* Настройка периода и прескеллера TIM3 */
  TIM_TimeBaseStructure.TIM_Period = СAPTURE_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = CAPTURE_PRESCALER;

  /* Настройка делителя частоты, режим счета таймера TIM3 */
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;                /* Делитель частоты таймера */
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; /* Режим счета вверх */
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);             /* Применение настроек */

  /* Настройка первого канала */
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;     /* Передний фронт */
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; /* Выбор входа - прямой вход */
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;           /* Предделитель входного сигнала */
  TIM_ICInitStructure.TIM_ICFilter = 0x0;                         /* Фильтр входного сигнала */
  TIM_ICInit(TIM3, &TIM_ICInitStructure);                         /* Применение настроек к каналу 1 таймера TIM3 */
  TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);                         /* Включение прерывания по событию захвата на канале 1 */

  TIM_Cmd(TIM3, ENABLE);     /* Включение таймера TIM3 */
  NVIC_EnableIRQ(TIM3_IRQn); /* Разрешить прерывания от таймера 3 */
}

/* Обработчик прерывания таймера TIM3 */
void TIM3_IRQHandler(void)
{
  /* Если произошло прерывание по событию захвата на канале 1 */
  if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1); /* Сброс флага прерывания */
    TIM3_VALUE = TIM_GetCapture1(TIM3);      /* Запись значения таймера в переменную */
  }
}

int main(void)
{
  __enable_irq(); /* Разрешить прерывания */
  initGPIO();     /* Вызов функции инициализации порта */
  initTIM3();     /* Вызов функции инициализации таймера */
  while (1)
  { 
  }
}
