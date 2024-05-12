#include "stm32f10x.h"       // Подключение библиотеки общих определений для STM32F10x
#include "stm32f10x_gpio.h"  // Подключение библиотеки для работы с GPIO
#include "stm32f10x_rcc.h"   // Подключение библиотеки для работы с RCC
#include "stm32f10x_tim.h"   // Подключение библиотеки для работы с таймерами
#include "stm32f10x_usart.h" // Подключение библиотеки для работы с USART

static char buffer;      // Буфер для хранения принятых данных
static char xbuffer[64]; // Буфер для хранения массива данных
static uint8_t xlen = 0; // Длина буфера

static void sendArr(char *data)
{
    int i = 0;
    while (data[i])
    {
        USART_SendData(USART1, data[i]);
        while (!USART_GetFlagStatus(USART1, USART_FLAG_TXE))
        {
        }
        i++;
    }
}

static void sendChar(char data)
{
    USART_SendData(USART1, data);
    while (!USART_GetFlagStatus(USART1, USART_FLAG_TXE))
    {
    }
}

static void clear()
{
    uint8_t i;
    for (i = 0; i <= xlen; i++)
        xbuffer[i] = 0;
    xlen = 0;
}
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

static void initGPIO() // Функция инициализации GPIO
{
    GPIO_InitTypeDef PortObj;                             // Объявление структуры для инициализации GPIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // Включение тактирования порта A

    PortObj.GPIO_Pin = GPIO_Pin_9;         // Настройка пина 9
    PortObj.GPIO_Speed = GPIO_Speed_50MHz; // Установка скорости GPIO
    PortObj.GPIO_Mode = GPIO_Mode_AF_PP;   // Установка режима альтернативной функции push-pull
    GPIO_Init(GPIOA, &PortObj);            // Инициализация GPIOA с использованием структуры Obj

    PortObj.GPIO_Pin = GPIO_Pin_10;            // Настройка пина 10
    PortObj.GPIO_Mode = GPIO_Mode_IN_FLOATING; // Установка режима входа с подтяжкой к "плавающему" уровню
    GPIO_Init(GPIOA, &PortObj);                // Инициализация GPIOA с использованием структуры Obj
}

static void initUSART() // Функция инициализации USART
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

static void initNVIC() // Функция инициализации NVIC
{
    NVIC_InitTypeDef NvicObj;                      // Объявление структуры для инициализации NVIC
    NvicObj.NVIC_IRQChannel = USART1_IRQn;         // Установка канала прерывания
    NvicObj.NVIC_IRQChannelPreemptionPriority = 0; // Установка приоритета прерывания
    NvicObj.NVIC_IRQChannelCmd = ENABLE;           // Включение канала прерывания
    NvicObj.NVIC_IRQChannelSubPriority = 0;        // Установка подприоритета прерывания
    NVIC_Init(&NvicObj);                           // Инициализация NVIC с использованием структуры Obj
}

void USART1_IRQHandler() // Обработчик прерывания USART1
{
    if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE)) // Если флаг RXNE установлен
    {
        USART_ClearITPendingBit(USART1, USART_FLAG_RXNE); // Очистка флага прерывания RXNE
        buffer = USART_ReceiveData(USART1);               // Чтение данных из регистра приема USART1

        if (buffer != 0x0D)
        {
            xbuffer[xlen] = buffer;
            sendChar(buffer); // Отправка прочитанного символа обратно
            xlen++;
            return;
        }

        if (isXBuff("show"))
        {
            sendArr("\nshow>>");
            clear();
            return;
        }

        if (isXBuff("start"))
        {
            sendArr("\nstart>>");
            clear();
            return;
        }

        if (isXBuff("stop"))
        {
            sendArr("\nstop>>");
            clear();
            return;
        }

        if (isXBuff("period"))
        {
            sendArr("\nperiod>>");
            clear();
            return;
        }
				
				if (isXBuff("pulse"))
        {
            sendArr("\npulse>>");
            clear();
            return;
        }
				
        clear();
        sendArr("\nInvalid command\n");
        return;
    }
}

int main()
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); // Включение тактирования DMA1
    initGPIO();                                        // Инициализация GPIO
    initUSART();                                       // Инициализация USART
    initNVIC();                                        // Инициализация NVIC

    while (1)
    {
    }
    return 0;
}
