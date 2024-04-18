#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

uint32_t polinom = 0x12345678;
uint32_t selfbit;

uint32_t func()
{
uint32_t t;
t = 0;
if (polinom & 0x00020000)
{
	t = t ^ 0x00000001;
}
if (polinom & 0x00000008)
{
	t = t ^ 0x00000001;
}
if (polinom & 0x00000001)
{
	t = t ^ 0x00000001;
}
return t;
}

void Tact(){
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

GPIO_InitTypeDef my_initB;

my_initB.GPIO_Pin = GPIO_Pin_7;
my_initB.GPIO_Mode = GPIO_Mode_IPD;
GPIO_Init(GPIOB, &my_initB);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

GPIO_InitTypeDef my_initA;

my_initA.GPIO_Pin = GPIO_Pin_5;
my_initA.GPIO_Mode = GPIO_Mode_Out_PP;
my_initA.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOA, &my_initA);
}

int main()
{
Tact();
while (1)
{
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7))
	{
		selfbit = func();
		polinom = (polinom << 1) | selfbit;
		if (selfbit)
		{
			GPIO_SetBits(GPIOA, GPIO_Pin_5);
		}
		else{
			GPIO_ResetBits(GPIOA, GPIO_Pin_5);
		}
	}
	else
	{
		GPIO_ResetBits(GPIOA, GPIO_Pin_5);
	}
}

}
