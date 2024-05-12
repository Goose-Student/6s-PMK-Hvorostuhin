#include "stm32f10x.h"       // Подключение библиотеки общих определений для STM32F10x
#include "stm32f10x_gpio.h"  // Подключение библиотеки для работы с GPIO
#include "stm32f10x_rcc.h"   // Подключение библиотеки для работы с RCC
#include "stm32f10x_tim.h"   // Подключение библиотеки для работы с таймерами
#include "stm32f10x_usart.h" // Подключение библиотеки для работы с USART

/* Объявление структуры для инициализации канала таймера */
static TIM_OCInitTypeDef ChObj;
/* Объявление структуры для инициализации таймера */
static TIM_TimeBaseInitTypeDef TimObj;

static char buffer;      // Буфер для хранения принятых данных
static char xbuffer[64]; // Буфер для хранения массива данных
static uint8_t xlen = 0; // Длина буфера
uint16_t argument = 0;

// Функция вывода массива байтов
static void sendArr(char *data)
{
    uint64_t i = 0;
    while (data[i])
    {
        USART_SendData(USART1, data[i]);
        while (!USART_GetFlagStatus(USART1, USART_FLAG_TXE))
        {
        }
        i++;
    }
}

// Функция вывода байта
static void sendChar(char data)
{
    USART_SendData(USART1, data);
    while (!USART_GetFlagStatus(USART1, USART_FLAG_TXE))
    {
    }
}

// Функция очистки буфера
static void clearXBuff(void)
{
    uint8_t i;
    for (i = 0; i <= xlen; i++)
        xbuffer[i] = 0;
    xlen = 0;
    buffer = 0;
}
static uint16_t getArg()
{
    uint8_t i;
    uint8_t begin = 0;
    uint16_t arg = 0;

    for (i = 0; i < xlen; i++)
    {
        if (xbuffer[i] == ' ')
            begin = i + 1;
    }
    if (!begin)
        return arg;

    for (i = begin; i < xlen; i++)
        arg = arg * 10 + (xbuffer[i] - '0');
    return arg;
}

// Функция сравнения массива байтов с буфером массива байтов
static int isXBuff(char *data)
{
    uint8_t i;
    for (i = 0; i < sizeof(data); i++)
    {
        if (data[i] != xbuffer[i])
            return 0;
    }
    return 1;
}

static void initGPIO(void) // Функция инициализации GPIO
{
    GPIO_InitTypeDef PortObj;                             // Объявление структуры для инициализации GPIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // Включение тактирования порта A

    /* Настройка порта PA9 */
    PortObj.GPIO_Pin = GPIO_Pin_9;         // Настройка пина 9
    PortObj.GPIO_Speed = GPIO_Speed_50MHz; // Установка скорости GPIO
    PortObj.GPIO_Mode = GPIO_Mode_AF_PP;   // Установка режима альтернативной функции push-pull
    GPIO_Init(GPIOA, &PortObj);            // Инициализация GPIOA с использованием структуры Obj

    /* Настройка порта PA10 */
    PortObj.GPIO_Pin = GPIO_Pin_10;            // Настройка пина 10
    PortObj.GPIO_Mode = GPIO_Mode_IN_FLOATING; // Установка режима входа с подтяжкой к "плавающему" уровню
    GPIO_Init(GPIOA, &PortObj);                // Инициализация GPIOA с использованием структуры Obj

    /* Настройка порта PA3 */
    PortObj.GPIO_Pin = GPIO_Pin_6;        /* Настройка пина 3 */
    PortObj.GPIO_Mode = GPIO_Mode_AF_PP;  /* Режим альтернативной функции, push-pull */
    PortObj.GPIO_Speed = GPIO_Speed_2MHz; /* Скорость порта 2 МГц */
    GPIO_Init(GPIOA, &PortObj);           /* Применение настроек к порту A */
}

static void initUSART(void) // Функция инициализации USART
{
    USART_InitTypeDef UsartObj;                            // Объявление структуры для инициализации USART
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // Включение тактирования USART1

    UsartObj.USART_BaudRate = 9600;                                      // Установка скорости передачи данных
    UsartObj.USART_WordLength = USART_WordLength_8b;                     // Установка длины слова
    UsartObj.USART_StopBits = USART_StopBits_1;                          // Установка количества стоп-битов
    UsartObj.USART_Parity = USART_Parity_No;                             // Установка контроля четности
    UsartObj.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // Установка аппаратного управления потоком
    UsartObj.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;                 // Установка режима передачи и приема
    USART_Init(USART1, &UsartObj);                                       // Инициализация USART1 с использованием структуры Obj

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // Включение прерывания по приему данных
    USART_Cmd(USART1, ENABLE);                     // Включение USART1
}

static void initNVIC(void) // Функция инициализации NVIC
{
    NVIC_InitTypeDef NvicObj;                      // Объявление структуры для инициализации NVIC
    NvicObj.NVIC_IRQChannel = USART1_IRQn;         // Установка канала прерывания
    NvicObj.NVIC_IRQChannelPreemptionPriority = 0; // Установка приоритета прерывания
    NvicObj.NVIC_IRQChannelCmd = ENABLE;           // Включение канала прерывания
    NvicObj.NVIC_IRQChannelSubPriority = 0;        // Установка подприоритета прерывания
    NVIC_Init(&NvicObj);                           // Инициализация NVIC с использованием структуры Obj

    NvicObj.NVIC_IRQChannel = TIM3_IRQn;           // Установка канала прерывания для TIM3
    NvicObj.NVIC_IRQChannelPreemptionPriority = 1; // Установка приоритета прерывания
    NvicObj.NVIC_IRQChannelCmd = ENABLE;           // Включение канала прерывания
    NvicObj.NVIC_IRQChannelSubPriority = 1;        // Установка подприоритета прерывания
    NVIC_Init(&NvicObj);                           // Инициализация NVIC с использованием структуры Obj для TIM3
}

static void initTIM3(void)
{
    /* Включение тактирования таймера TIM3 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* Настройка периода и прескеллера TIM3 */
    TimObj.TIM_Period = 255;
    TimObj.TIM_Prescaler = 107;

    /* Настройка делителя частоты, режим счета таймера TIM3 */
    TimObj.TIM_ClockDivision = 0;                /* Делитель частоты таймера */
    TimObj.TIM_CounterMode = TIM_CounterMode_Up; /* Режим счета вверх */
    TIM_TimeBaseInit(TIM3, &TimObj);             /* Применение настроек */

    /* Настройка канала 1 в режиме PWM */
    ChObj.TIM_OCMode = TIM_OCMode_PWM1;             /* Режим работы канала - PWM1 */
    ChObj.TIM_OutputState = TIM_OutputState_Enable; /* Включение выхода канала */
    ChObj.TIM_OCPolarity = TIM_OCPolarity_High;     /* Полярность выходного сигнала - прямая */
    ChObj.TIM_Pulse = TimObj.TIM_Period / 2;

    TIM_OC1Init(TIM3, &ChObj);                 /* Применение настроек к каналу 1 таймера TIM3 */
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); /* Включение прерывания на канале 1 */

    TIM_Cmd(TIM3, ENABLE);
}

void USART1_IRQHandler(void) // Обработчик прерывания USART1
{
    if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE)) // Если флаг RXNE установлен
    {
        USART_ClearITPendingBit(USART1, USART_FLAG_RXNE); // Очистка флага прерывания RXNE
        buffer = USART_ReceiveData(USART1);               // Чтение данных из регистра приема USART1

        if (xlen + 1 >= sizeof(xbuffer))
        {
            sendArr("\nOverflow buffer\n");
            clearXBuff();
            return;
        }

        if (buffer != 0x0D)
        {
            xbuffer[xlen] = buffer;
            sendChar(buffer); // Отправка прочитанного символа обратно
            xlen++;
            return;
        }

        if (isXBuff("show"))
        {
            clearXBuff();
            sprintf(xbuffer, "%d", TimObj.TIM_Period);
            sendArr("\nPeriod: ");
            sendArr(xbuffer);

            clearXBuff();
            sprintf(xbuffer, "%d", ChObj.TIM_Pulse);
            sendArr("\nPulse: ");
            sendArr(xbuffer);

            clearXBuff();
            sendArr("\n");
            return;
        }

        if (isXBuff("start"))
        {
            TIM_Cmd(TIM3, ENABLE);
            clearXBuff();
            sendArr("\n");
            return;
        }

        if (isXBuff("stop"))
        {
            TIM_Cmd(TIM3, DISABLE);
            clearXBuff();
            sendArr("\n");
            return;
        }

        if (isXBuff("period"))
        {
            TimObj.TIM_Period = getArg();
            TIM_TimeBaseInit(TIM3, &TimObj);
            TIM_OC1Init(TIM3, &ChObj);
            sendArr("\n");
            clearXBuff();
            return;
        }

        if (isXBuff("pulse"))
        {
            ChObj.TIM_Pulse = getArg();
            TIM_TimeBaseInit(TIM3, &TimObj);
            TIM_OC1Init(TIM3, &ChObj);
            clearXBuff();
            sendArr("\n");
            return;
        }

        clearXBuff();
        sendArr("\nInvalid command\n");
        return;
    }
}

int main(void)
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); // Включение тактирования DMA1
    initGPIO();                                        // Инициализация GPIO
    initUSART();                                       // Инициализация USART
    initNVIC();                                        // Инициализация NVIC
    initTIM3                                           // Инициализация TIM3
        while (1)
    {
    }
}
