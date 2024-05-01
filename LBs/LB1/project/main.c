#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

uint32_t Seed = 0xABCDEF; // Переменная для анализатора
uint32_t GBit; // Переменная для анализатора

typedef struct {
    uint32_t *polynomial;  // Указатель на переменную seed
    int id1;
    int id2;
    int length;
} Generator;

void initializeGenerator(Generator *gen, uint32_t *seed, int id1, int id2, int length) {
    gen->polynomial = seed;
    gen->id1 = id1;
    gen->id2 = id2;
    gen->length = length;
}

uint32_t nextBit(Generator *gen) {
    uint32_t firstBit = ((*gen->polynomial) >> (gen->id1 - 1)) & 0x1;
    uint32_t secondBit = ((*gen->polynomial) >> (gen->id2 - 1)) & 0x1;
    uint32_t sum = firstBit ^ secondBit;

    *gen->polynomial = ((*gen->polynomial) << 1) | sum;
    *gen->polynomial &= ((1ULL << gen->length) - 1);

    return sum;
}

void initializeGPIO() {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef initInput;
    initInput.GPIO_Pin = GPIO_Pin_7;
    initInput.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOB, &initInput);

    GPIO_InitTypeDef initOutput;
    initOutput.GPIO_Pin = GPIO_Pin_5;
    initOutput.GPIO_Mode = GPIO_Mode_Out_PP;
    initOutput.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &initOutput);
}

int main() {
    Generator gen;
    initializeGenerator(&gen, &Seed, 14, 17, 17); // Инициализация параметров генератора

    initializeGPIO(); // Инициализация портов ввода/вывода

    while (1) {
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7)) {
            GBit = nextBit(&gen); // Генерация следующего бита
            if (GBit) {
                GPIO_SetBits(GPIOA, GPIO_Pin_5); // Установка бита на выходе
            } else {
                GPIO_ResetBits(GPIOA, GPIO_Pin_5); // Сброс бита на выходе
            }
        } else {
            GPIO_ResetBits(GPIOA, GPIO_Pin_5); // Сброс бита на выходе
        }
    }

    return 0;
}