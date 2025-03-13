#include "stm32f103x8.h"


/**
  * @brief:  SysTick handler is implemented in main.c
  *
  */


void SysTick_Init(void)
{
    // Configure SysTick to generate an interrupt every 1ms
    // Assuming 72MHz system clock