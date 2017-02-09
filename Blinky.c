/*----------------------------------------------------------------------------
 * Name:    Blinky.c
 * Purpose: LED Flasher
 * Note(s): 
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2011 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stm32f4xx.h>
#include <system_stm32f4xx.h>
#include "LED.h"
#include <time.h>

void InitTimers();
void EnableTimerInterrupt();
int blinkSelection(int num);
int brewCoffee(int num);
volatile uint32_t msTicks;                      /* counts 1ms timeTicks       */
/*----------------------------------------------------------------------------
  SysTick_Handler
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void) {
  msTicks++;
}

/*----------------------------------------------------------------------------
  delays number of tick Systicks (happens every 1 ms)
 *----------------------------------------------------------------------------*/
void Delay (uint32_t dlyTicks) {                                              
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks);
}


/*----------------------------------------------------------------------------
  Function that initializes Button pins
 *----------------------------------------------------------------------------*/
void BTN_Init(void) {

  RCC->AHB1ENR  |= ((1UL <<  0) );              /* Enable GPIOA clock         */

  GPIOA->MODER    &= ~((3UL << 2*0)  );         /* PA.0 is input              */
  GPIOA->OSPEEDR  &= ~((3UL << 2*0)  );         /* PA.0 is 50MHz Fast Speed   */
  GPIOA->OSPEEDR  |=  ((2UL << 2*0)  ); 
  GPIOA->PUPDR    &= ~((3UL << 2*0)  );         /* PA.0 is no Pull up         */
}

/*----------------------------------------------------------------------------
  Function that read Button pins
 *----------------------------------------------------------------------------*/
uint32_t BTN_Get(void) {

 return (GPIOA->IDR & (1UL << 0));
}


/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {
	InitTimers();
	EnableTimerInterrupt();
  int32_t num = -1; 
  int32_t dir =  1;
  uint32_t btns = 0;

  SystemCoreClockUpdate();                      /* Get Core Clock Frequency   */
  if (SysTick_Config(SystemCoreClock / 1000)) { /* SysTick 1 msec interrupts  */
    while (1);                                  /* Capture error              */
  }

  LED_Init();
  BTN_Init();                             
 
  while(1) {                                    /* Loop forever               */
    btns = BTN_Get();                           /* Read button states         */

    if (btns != (1UL << 0)) {
      /* Calculate 'num': 0,1,...,LED_NUM-1,LED_NUM-1,...,1,0,0,...  */
      //num += dir;
      //if (num == LED_NUM) { dir = -1; num =  LED_NUM-1; } 
      //else if   (num < 0) { dir =  1; num =  0;         }

      //LED_On (num);
      //Delay( 50);                               /* Delay 50ms                 */
      //LED_Off(num);
      //Delay(200);                               /* Delay 200ms                */
    }
    else {
			Delay(500);   // delay to account for long press (selection of coffee type)
			btns = BTN_Get(); 
			if (btns != (1UL << 0)) { 
				/* increment coffee selection light on short click */
			
					LED_Off(num);
					num += dir;
					if (num == LED_NUM) { num =  0; } 
					LED_On (num);
			} else {
				blinkSelection(num);
				brewCoffee(num);
				}
    }

  }
  
}
int brewCoffee(int num)
{
		
      return 0;
}

int blinkSelection(int num)
{
	int i;
	for (i = 0; i < 3; i++) {
     LED_On(0);
	   LED_On(1);
	   LED_On(2);
	   LED_On(3);
		 Delay(250);
		 LED_Off(0);
	   LED_Off(1);
	   LED_Off(2);
	   LED_Off(3);
		 Delay(250);
	}
	for (i = 0; i < 3; i++) {
				   LED_On(num);
					 Delay(250);
					 LED_Off(num);
				   Delay(250);
			}
	return 0;
}
void InitTimers()
{
  //GPIO_InitTypeDef GPIO_Initstructure;
	//TIM_TimeBaseInitTypeDef timer_InitStructure;
}

