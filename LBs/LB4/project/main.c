#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"

static float S, V, T;
static float temperature_delta = 0.0;
static char buffer;      // Буфер для хранения принятых данных
static char xbuffer[64]; // Буфер для хранения массива данных
static uint8_t xlen = 0; // Длина буфера

static uint32_t readADC()
{
    ADC1->SQR3 = 8; // Номер канала АЦП
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC))
    {
    }
    return ADC1->DR;
}

static void USART_SendArray(const char *data)
{
    uint64_t i = 0;
    while (data[i])
    {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
            ; // Ожидание завершения передачи
        USART_SendData(USART2, data[i]);
        i++;
    }
}

static void initGPIO(void) // Функция инициализации GPIO
{
    GPIO_InitTypeDef PortObj;                             // Объявление структуры для инициализации GPIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // Включение тактирования порта A

    /* Настройка порта PA2 */
    PortObj.GPIO_Pin = GPIO_Pin_2;         // Настройка пина 2
    PortObj.GPIO_Speed = GPIO_Speed_50MHz; // Установка скорости GPIO
    PortObj.GPIO_Mode = GPIO_Mode_AF_PP;   // Установка режима альтернативной функции push-pull
    GPIO_Init(GPIOA, &PortObj);            // Инициализация GPIOA с использованием структуры Obj

    /* Настройка порта PA3 */
    PortObj.GPIO_Pin = GPIO_Pin_3;             // Настройка пина 3
    PortObj.GPIO_Mode = GPIO_Mode_IN_FLOATING; // Установка режима входа с подтяжкой к "плавающему" уровню
    GPIO_Init(GPIOA, &PortObj);                // Инициализация GPIOA с использованием структуры Obj
}

static void initTIM2(void)
{
    TIM_TimeBaseInitTypeDef TIMER;

    /* Включение тактирования таймера TIM2 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* Настройка периода и прескеллера TIM2 */
    TIMER.TIM_Period = 10000;    // Срабатывание каждую секунду
    TIMER.TIM_Prescaler = 10799; // Настройка прескелера 10000 int = 1 сек

    /* Настройка делителя частоты, режим счета таймера TIM2 */
    TIMER.TIM_ClockDivision = 0;                /* Делитель частоты таймера */
    TIMER.TIM_CounterMode = TIM_CounterMode_Up; /* Режим счета вверх */
    TIM_TimeBaseInit(TIM2, &TIMER);             /* Применение настроек */

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); /* Включение прерывания по событию обновления */

    TIM_Cmd(TIM2, ENABLE);     /* Включение таймера TIM2 */
    NVIC_EnableIRQ(TIM2_IRQn); /* Разрешить прерывания от таймера 3 */
    NVIC_SetPriority(TIM2_IRQn, 1);
}

static void initUSART(void) // Функция инициализации USART
{
    USART_InitTypeDef UsartObj;                            // Объявление структуры для инициализации USART
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // Включение тактирования USART2

    UsartObj.USART_BaudRate = 2400;                                      // Установка скорости передачи данных
    UsartObj.USART_WordLength = USART_WordLength_8b;                     // Установка длины слова
    UsartObj.USART_StopBits = USART_StopBits_1;                          // Установка количества стоп-битов
    UsartObj.USART_Parity = USART_Parity_No;                             // Установка контроля четности
    UsartObj.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // Установка аппаратного управления потоком
    UsartObj.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;                 // Установка режима передачи и приема
    USART_Init(USART2, &UsartObj);                                       // Инициализация USART2 с использованием структуры Obj
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); 
	
    USART_Cmd(USART2, ENABLE); // Включение USART2
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_SetPriority(USART2_IRQn, 0);
}

void USART2_IRQHandler(void) // Обработчик прерывания USART2
{		
		if (!USART_GetFlagStatus(USART2, USART_FLAG_RXNE))
			return;
		
    S = readADC();
    char sss[20];
    sprintf(sss, "%f", S);
    USART_SendArray("SIGNAL: ");
    USART_SendArray(sss);
    USART_SendArray("\n");
    V = S * 1.2 / 4096;
    char vvv[20];
    sprintf(vvv, "%f", V);
    USART_SendArray("VOLTAGE: ");
    USART_SendArray(vvv);
    USART_SendArray("\n");
    T = (1.43 - V) / 4.3 + 5 + temperature_delta;
    temperature_delta += 0.3;
    char ttt[20];
    sprintf(ttt, "%f", T);
    USART_SendArray("TEMPERATURE: ");
    USART_SendArray(ttt);
    USART_SendArray("\n");
}

static void initADC(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    GPIOC->CRL &= ~(GPIO_CRL_MODE4 | GPIO_CRL_CNF4);
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    ADC1->SMPR1 |= ADC_SMPR1_SMP14;
    ADC1->CR2 |= ADC_CR2_TSVREFE;
    ADC1->CR2 |= ADC_CR2_EXTSEL;
    ADC1->CR2 |= ADC_CR2_EXTTRIG;
    ADC1->CR2 |= ADC_CR2_ADON;
    ADC1->CR2 |= ADC_CR2_CAL;
    while (!(ADC1->CR2 & ADC_CR2_CAL))
    {
    }
}

void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update); /* Сброс флага прерывания */
        // NVIC_SetPendingIRQ(USART2_IRQn);
    }
}

int main(void)
{
    __enable_irq();
    initGPIO();
    // initTIM2();
    initUSART();
    initADC();
    while (1)
    {
    }
}
