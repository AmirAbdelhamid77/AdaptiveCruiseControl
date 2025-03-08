/*
 *   motor_driver.h
 *
 *  Created on: SomeDay
 *      Author: AMiR & Co.
 */

#ifndef INC_STM32F103C6_MOTOR_DRIVER_H_
#define INC_STM32F103C6_MOTOR_DRIVER_H_

//-----------------------------
//Includes from MCAL
#include "stm32f103x6.h"
#include "stm32_F103C6_gpio_driver.h"
#include "hw_config.h"
//-----------------------------

// Motor Directions
#define MOTOR_DIRECTION_FORWARD    0
#define MOTOR_DIRECTION_BACKWARD   1
#define MOTOR_DIRECTION_STOP       2

// Motor Selection
#define MOTOR_1                    0
#define MOTOR_2                    1
#define MOTOR_3                    2
#define MOTOR_4                    3

// Motor Speed Range
#define MOTOR_SPEED_MIN            0
#define MOTOR_SPEED_MAX            100

// Use timer definitions from hw_config.h 
// Primary motors use TIM1, secondary motors use TIM3
#define MOTOR_PWM_TIMER_PRIMARY    MOTOR_PWM_TIMER    // TIM1 from hw_config.h
#define MOTOR_PWM_TIMER_SECONDARY  MOTOR_PWM_TIMER2   // TIM3 from hw_config.h

//-----------------------------
// Motor Configuration Structure
typedef struct
{
    GPIO_TypeDef* IN1_Port;     // IN1 control pin port 
    uint16_t IN1_Pin;           // IN1 control pin

    GPIO_TypeDef* IN2_Port;     // IN2 control pin port 
    uint16_t IN2_Pin;           // IN2 control pin
    
    GPIO_TypeDef* EN_Port;      // Enable/PWM pin port 
    uint16_t EN_Pin;            // Enable/PWM pin
    
    uint8_t PWM_Channel;        // PWM Timer channel for this motor
    TIM_TypeDef* PWM_Timer;     // Added to support multiple timers (TIM1 or TIM3)
} Motor_Config_t;

//-----------------------------
// Car Configuration Structure
typedef struct
{
    Motor_Config_t Motors[4];    // Array of 4 motor configurations
} Car_Config_t;


/**================================================================
 * @Fn          - MOTOR_Init
 * @brief       - Initializes the 4 motors with their GPIO configurations
 */
void MOTOR_Init(Car_Config_t* car_cfg);

/**================================================================
 * @Fn          - MOTOR_SetDirection
 * @brief       - Sets the direction of a specific motor

 */
void MOTOR_SetDirection(uint8_t motor_id, uint8_t direction);

/**================================================================
 * @Fn          - MOTOR_SetSpeed
 * @brief       - Sets the speed of a specific motor (0-100%)

 */
void MOTOR_SetSpeed(uint8_t motor_id, uint8_t speed);

/**================================================================
 * @Fn          - MOTOR_Stop
 * @brief       - Stops a specific motor

 */
void MOTOR_Stop(uint8_t motor_id);

/**================================================================
 * @Fn          - MOTOR_StopAll
 * @brief       - Stops all motors

 */
void MOTOR_StopAll(void);

/**================================================================
 * @Fn          - MOTOR_Move
 * @brief       - Configures all motors to move the car in a specific way

 */
void MOTOR_Move(int8_t forward_speed, int8_t turn_value);

#endif
