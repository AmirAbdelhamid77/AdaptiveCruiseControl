#include "stm32f103x8.h"

// Global variable to count system ticks
static volatile uint32_t SysTickCounter = 0;

/**
  * @brief  SysTick handler
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    SysTickCounter++;
}

/**
  * @brief  Initialize the SysTick timer
  * @param  None
  * @retval None
  */
void SysTick_Init(void)
{
    // Configure SysTick to generate an interrupt every 1ms
    // Assuming 72MHz system clock