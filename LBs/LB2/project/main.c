#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

/* Константы для настройки таймеров */
static uint16_t CAPTURE_PRESCALER = 539;
static uint16_t СAPTURE_PERIOD = 32000;
static uint16_t PWM_PRESCALER = 10799;
static uint16_t PWM_PERIOD = 200; /* 200 (max input period) * 2  */

/* Хранение значения таймера */
uint16_t tim3_value = 0;
uint16_t current_value = 0;
uint16_t tim2_value = 0;
uint16_t pwm_width = 0;


/* Функция инициализации портов */
static void initGPIO(void)
{
  /* Объявление структуры для инициализации порта */
  GPIO_InitTypeDef PORT;
  /* Включение тактирования порта A */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  /* Настройка порта PA6 */
  PORT.GPIO_Pin = GPIO_Pin_6;             /* Настройка пина 6 */
  PORT.GPIO_Mode = GPIO_Mode_IN_FLOATING; /* Режим входа, подтяжка отключена */
	PORT.GPIO_Speed = GPIO_Speed_2MHz;     /* Скорость порта 2 МГц */
  GPIO_Init(GPIOA, &PORT);                /* Применение настроек к порту A */

  /* Настройка порта PA3 */
  PORT.GPIO_Pin = GPIO_Pin_3;             /* Настройка пина 3 */
  PORT.GPIO_Mode = GPIO_Mode_AF_PP;       /* Режим альтернативной функции, push-pull */
  PORT.GPIO_Speed = GPIO_Speed_2MHz;     /* Скорость порта 2 МГц */
  GPIO_Init(GPIOA, &PORT);                /* Применение настроек к порту A */
}


static void initTIM3(void)
{
  /* Объявление структуры для инициализации канала таймера */
  TIM_ICInitTypeDef CHANEL;
  /* Объявление структуры для инициализации таймера */
  TIM_TimeBaseInitTypeDef TIMER;

  /* Включение тактирования таймера TIM3 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* Настройка периода и прескеллера TIM3 */
  TIMER.TIM_Period = СAPTURE_PERIOD;
  TIMER.TIM_Prescaler = CAPTURE_PRESCALER;

  /* Настройка делителя частоты, режим счета таймера TIM3 */
  TIMER.TIM_ClockDivision = 0;                /* Делитель частоты таймера */
  TIMER.TIM_CounterMode = TIM_CounterMode_Up; /* Режим счета вверх */
  TIM_TimeBaseInit(TIM3, &TIMER);             /* Применение настроек */

  /* Настройка первого канала */
  CHANEL.TIM_Channel = TIM_Channel_1;
  CHANEL.TIM_ICPolarity = TIM_ICPolarity_Rising;     /* Передний фронт */
  CHANEL.TIM_ICSelection = TIM_ICSelection_DirectTI; /* Выбор входа - прямой вход */
  CHANEL.TIM_ICPrescaler = TIM_ICPSC_DIV1;           /* Предделитель входного сигнала */
  CHANEL.TIM_ICFilter = 0x0;                         /* Фильтр входного сигнала */
  TIM_ICInit(TIM3, &CHANEL);                         /* Применение настроек к каналу 1 таймера TIM3 */
  TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);            /* Включение прерывания по событию захвата на канале 1 */

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
		current_value = TIM_GetCapture1(TIM3);
		tim3_value = current_value; /* Запись значения таймера в переменную */
		
		/* определение переода */
		if (pwm_width == current_value - tim3_value)
			return;
		pwm_width = 0;
		if (current_value >= tim3_value)
			pwm_width = (current_value - tim3_value);
  }
}

/* Функция инициализации таймера TIM2 в режиме PWM */
static void initTIM2(void)
{
  /* Объявление структуры для инициализации канала таймера */
  TIM_OCInitTypeDef OUTPUT_CHANNEL;
  /* Объявление структуры для инициализации таймера */
  TIM_TimeBaseInitTypeDef TIMER;

  /* Включение тактирования таймера TIM2 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  /* Настройка периода и прескеллера TIM2 */
  TIMER.TIM_Period = PWM_PERIOD;
  TIMER.TIM_Prescaler = PWM_PRESCALER;

  /* Настройка делителя частоты, режим счета таймера TIM2 */
  TIMER.TIM_ClockDivision = 0;                /* Делитель частоты таймера */
  TIMER.TIM_CounterMode = TIM_CounterMode_Up; /* Режим счета вверх */
  TIM_TimeBaseInit(TIM2, &TIMER);             /* Применение настроек */

  /* Настройка канала 4 в режиме PWM */
  OUTPUT_CHANNEL.TIM_OCMode = TIM_OCMode_PWM1;       /* Режим работы канала - PWM1 */
  OUTPUT_CHANNEL.TIM_OutputState = TIM_OutputState_Enable; /* Включение выхода канала */
  // OUTPUT_CHANNEL.TIM_Pulse = PWM_PERIOD / 2;         /* Установка коэффициента заполнения в 50% */
  OUTPUT_CHANNEL.TIM_OCPolarity = TIM_OCPolarity_High; /* Полярность выходного сигнала - прямая */
  TIM_OC4Init(TIM2, &OUTPUT_CHANNEL);                 /* Применение настроек к каналу 4 таймера TIM2 */
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);            /* Включение прерывания на канале 4 */
	TIM_SetCompare4(TIM2, 0); /* установки нового значения сравнения */
	
  TIM_Cmd(TIM2, ENABLE);     /* Включение таймера TIM2 */
	NVIC_EnableIRQ(TIM2_IRQn); /* Разрешить прерывания от таймера 2 */
	
}

/* Обработчик прерывания таймера TIM2 */
void TIM2_IRQHandler(void)
{
  /* Если произошло прерывание по событию захвата на канале 4 */
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update); /* Сброс флага прерывания */
		TIM_SetCompare4(TIM2, pwm_width / 10); /* установки нового значения сравнения */
		tim2_value = TIM_GetCapture4(TIM2);  /* Запись значения таймера в переменную */		
		
  }
}


int main(void)
{
  __enable_irq(); /* Разрешить прерывания */
  initGPIO();     /* Вызов функции инициализации порта */
  initTIM3();     /* Вызов функции инициализации таймера */
	initTIM2();     /* Вызов функции инициализации таймера */
  while (1)
  {
  }
}
