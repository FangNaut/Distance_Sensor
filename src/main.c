
/* Includes */

#include "stm32l1xx.h"
#define HSI_VALUE    ((uint32_t)16000000)
#include "nucleo152start.h"
#include <stdio.h>

/* Private typedef */
/* Private define  */
/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */

void delay_ms(unsigned long delay);
void delay_10us(void);
void USART_write(char data);
void USART2_Init(void);
/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void)
{
	__disable_irq();			//global disable IRQs, M3_Generic_User_Guide p135.
	/* Configure the system clock to 32 MHz and update SystemCoreClock */
	SetSysClock();
	SystemCoreClockUpdate();
	USART2_Init();
	/* TODO - Add your application code here */

	RCC->AHBENR|=1; 		//GPIOA ABH bus clock ON. p154
	GPIOA->MODER|=0x400; 	//GPIOA pin 5 to output. p184
	GPIOA->MODER|=0x4000;	//GPIOA pin 7 to output.
	RCC->APB2ENR |=1;		//Bit 0 SYSCFGEN: System configuration controller clock enable.

	//PA6 (D12)to external interrupt
	GPIOA->MODER &=~0x3000;			//clear (input state for PA6). p184
	SYSCFG->EXTICR[1] &=~0x0F00;	//select port A (PA6) for EXTI6. p223
	EXTI->IMR |= (1<<6);			//unmask EXTI6 (MR6), p242
	EXTI->RTSR |= (1<<6);			//select risign edge trigger PA6, p243
	EXTI->FTSR |= (1<<6);			//select falling edge trigger, p243
	NVIC_EnableIRQ(EXTI9_5_IRQn); 	//enable IRQ M3_Generic_User_Guide p130

	__enable_irq();					//global enable IRQs, M3_Generic_User_Guide p135


  /* Infinite loop */
  while (1)
  {
	GPIOA->ODR|=0x20;		//0010 0000 xor bit 5. p186
	GPIOA->ODR|=0x80;		//0010 0000 xor bit 7. p186
	delay_10us();			//this have to be calibrated
	GPIOA->ODR&=~0x20;		//0010 0000 xor bit 5. p186
	GPIOA->ODR&=~0x80;		//0010 0000 xor bit 7. p186
	delay_ms(1000);
  }
  return 0;
}

void delay_ms(unsigned long delay)
{
	unsigned long i=0;
	RCC->APB1ENR|=(1<<3); 	//TIM5EN: Timer 5 clock enable. p160
	TIM5->PSC=32-1; 		//32 000 000 Hz / 32 = 1 000 000 Hz. p435
	TIM5->ARR=1000-1; 		//TIM5 counter. 1 000 000 Hz / 1000 = 1000 Hz ~ 1ms. p435
	TIM5->CNT=0;			//counter start value = 0
	TIM5->CR1=1; 			//TIM5 Counter enabled. p421

	  while(i<delay)
	  {
		  while(!((TIM5->SR)&1)){} //Update interrupt flag. p427
		  i++;
		  TIM5->SR &= ~1; 	//flag cleared. p427
		  TIM5->CNT=0;	  	//counter start value = 0
	  }
	  TIM5->CR1=0; 		//TIM5 Counter disabled. p421
}

void delay_10us(void)
{
	RCC->APB1ENR|=(1<<4); 	//TIM6EN: Timer 6 clock enable. p158
	TIM6->PSC=32-1; 		//32 000 000 Hz / 32 = 1 000 000 Hz. T=1us. p435
	TIM6->ARR=8-1; 			//TIM6 counter. 1 000 000 Hz / 10 = 100 000 Hz. T=10us. p435
	//kokeiltiin oskilloskoopilla kohdalle.
	TIM6->CNT=0;			//counter start value = 0
	TIM6->CR1=1; 			//TIM6 Counter enabled. p421

	while(!((TIM6->SR)&1)){} //Update interrupt flag. p427
	TIM6->SR &= ~1; 	//flag cleared. p427
	TIM6->CNT=0;	  	//counter start value = 0
	TIM6->CR1=0; 		//TIM6 Counter disabled. p421
}

void USART2_Init(void)
{
	RCC->APB1ENR|=0x00020000; 	//set bit 17 (USART2 EN)
	RCC->AHBENR|=0x00000001; 	//enable GPIOA port clock bit 0 (GPIOA EN)
	GPIOA->AFR[0]=0x00000700;	//GPIOx_AFRL p.188,AF7 p.177
	GPIOA->MODER|=0x00000020; 	//MODER2=PA2(TX) to mode 10=alternate function mode. p184

	USART2->BRR = 0x00000116;	//115200 BAUD and crystal 32MHz. p710, 116
	USART2->CR1 = 0x00000008;	//TE bit. p739-740. Enable transmit
	USART2->CR1 |= 0x00002000;	//UE bit. p739-740. Uart enable
}

void USART_write(char data)
{
	//wait while TX buffer is empty
	while(!(USART2->SR&0x0080)){} 	//TXE: Transmit data register empty. p736-737
		USART2->DR=(data);			//p739
}

void EXTI9_5_IRQHandler(void)
{
	if((GPIOA->IDR & (1<<6))) //if input PA6 state logical 1 = 3.3V
	{
		RCC->APB1ENR|=(1<<2); 	//TIM4EN: Timer 4 clock enable. p160
		TIM4->PSC=32-1; 		//32 000 000 Hz / 32 = 1 000 000 Hz. p435
		TIM4->CNT=0;			//counter start value = 0
		TIM4->CR1=1; 			//TIM3 Counter enabled. p421
	}
	else
	{
		int timer4=0;
		timer4=TIM4->CNT;
		int distance_cm=timer4/58;
		int distance_mm=(timer4*10/58)%10;
		char buf[100]="";

			sprintf(buf,"timer %d distance %d.%d cm",timer4,distance_cm,distance_mm);

			for(int i=0;i<30;i++)
			{
				USART_write(buf[i]);
			}

			USART_write('\n');
			USART_write('\r');
		TIM4->CR1=0; 			//TIM4 Counter disabled. p421
	}

	EXTI->PR=(1<<6);		//Pending, This bit is cleared by writing a p245
}
