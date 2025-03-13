/**
  ******************************************************************************
  * @file           : hw_config.h
  * @brief          : Hardware configuration and pin mapping
  * @author         : AMiR

  ******************************************************************************
  * @attention
  *
  * This file contains all hardware-specific configurations including
  * pin mappings, timer settings, and (peripheral assignments)->some 
    of them need activation in case of need.
  *
  ******************************************************************************
  */

#ifndef HW_CONFIG_H_
#define HW_CONFIG_H_

#include "stm32f103x6.h"
#include "stm32_F103C6_gpio_driver.h"

/*----------------------------------------------------------------------------*/
/* Timer Configurations                                                       */
/*----------------------------------------------------------------------------*/

/* TIM1 Configuration (Motor PWM) */
#define MOTOR_PWM_TIMER            TIM1
#define MOTOR_PWM_PRESCALER        72      /* To get 1 MHz timer clock */
#define MOTOR_PWM_PERIOD           1000    /* To get 1 kHz PWM frequency */

/* TIM2 Configuration (Ultrasonic Echo + Speed Sensors) */
#define ULTRASONIC_TIMER           TIM2
#define ULTRASONIC_PRESCALER       72      /* To get 1 MHz timer clock (1μs resolution) */
#define ULTRASONIC_PERIOD          65535   /* Maximum for 16-bit timer */

/* TIM3 Configuration (Motor PWM - Secondary) */
#define MOTOR_PWM_TIMER2           TIM3
#define MOTOR_PWM_PRESCALER2       72      /* To get 1 MHz timer clock */
#define MOTOR_PWM_PERIOD2          1000    /* To get 1 kHz PWM frequency */

/* TIM4 Configuration (Ultrasonic Echo - Secondary) */
#define ULTRASONIC_TIMER2          TIM4
#define ULTRASONIC_PRESCALER2      72      /* To get 1 MHz timer clock (1μs resolution) */
#define ULTRASONIC_PERIOD2         65535   /* Maximum for 16-bit timer */

/*----------------------------------------------------------------------------*/
/* Motor Driver Pin Configurations                                            */
/*----------------------------------------------------------------------------*/

/* Motor 1 (Front Left) */
#define MOTOR1_IN1_PORT            GPIOA
#define MOTOR1_IN1_PIN             GPIO_PIN_8
#define MOTOR1_IN2_PORT            GPIOA
#define MOTOR1_IN2_PIN             GPIO_PIN_9
#define MOTOR1_PWM_PORT            GPIOA
#define MOTOR1_PWM_PIN             GPIO_PIN_8
#define MOTOR1_PWM_TIMER           TIM1
#define MOTOR1_PWM_CHANNEL         1

/* Motor 2 (Front Right) */
#define MOTOR2_IN1_PORT            GPIOA
#define MOTOR2_IN1_PIN             GPIO_PIN_10
#define MOTOR2_IN2_PORT            GPIOA
#define MOTOR2_IN2_PIN             GPIO_PIN_11
#define MOTOR2_PWM_PORT            GPIOA
#define MOTOR2_PWM_PIN             GPIO_PIN_9
#define MOTOR2_PWM_TIMER           TIM1
#define MOTOR2_PWM_CHANNEL         2

/* Motor 3 (Rear Left) */
#define MOTOR3_IN1_PORT            GPIOA
#define MOTOR3_IN1_PIN             GPIO_PIN_6
#define MOTOR3_IN2_PORT            GPIOA
#define MOTOR3_IN2_PIN             GPIO_PIN_7
#define MOTOR3_PWM_PORT            GPIOA
#define MOTOR3_PWM_PIN             GPIO_PIN_6
#define MOTOR3_PWM_TIMER           TIM3
#define MOTOR3_PWM_CHANNEL         1

/* Motor 4 (Rear Right) */
#define MOTOR4_IN1_PORT            GPIOB
#define MOTOR4_IN1_PIN             GPIO_PIN_0
#define MOTOR4_IN2_PORT            GPIOB
#define MOTOR4_IN2_PIN             GPIO_PIN_1
#define MOTOR4_PWM_PORT            GPIOB
#define MOTOR4_PWM_PIN             GPIO_PIN_0
#define MOTOR4_PWM_TIMER           TIM3
#define MOTOR4_PWM_CHANNEL         3

/*----------------------------------------------------------------------------*/
/* Speed Sensor Pin Configurations                                            */
/*----------------------------------------------------------------------------*/

/* Speed Sensor 1 (Left) */
#define SPEED1_PORT                GPIOA
#define SPEED1_PIN                 GPIO_PIN_2
#define SPEED1_TIMER               TIM2
#define SPEED1_CHANNEL             3

/* Speed Sensor 2 (Right) */
#define SPEED2_PORT                GPIOA
#define SPEED2_PIN                 GPIO_PIN_3
#define SPEED2_TIMER               TIM2
#define SPEED2_CHANNEL             4

/*----------------------------------------------------------------------------*/
/* Ultrasonic Sensor Pin Configurations                                       */
/*----------------------------------------------------------------------------*/

/* Ultrasonic Sensor 1 (Front) */
#define US1_TRIG_PORT              GPIOA
#define US1_TRIG_PIN               GPIO_PIN_0
#define US1_ECHO_PORT              GPIOA
#define US1_ECHO_PIN               GPIO_PIN_0
#define US1_TIMER                  TIM2
#define US1_CHANNEL                1

/* Ultrasonic Sensor 2 (Rear) */
#define US2_TRIG_PORT              GPIOA
#define US2_TRIG_PIN               GPIO_PIN_1
#define US2_ECHO_PORT              GPIOA
#define US2_ECHO_PIN               GPIO_PIN_1
#define US2_TIMER                  TIM2
#define US2_CHANNEL                2

/* Ultrasonic Sensor 3 (Left) */
#define US3_TRIG_PORT              GPIOB
#define US3_TRIG_PIN               GPIO_PIN_6
#define US3_ECHO_PORT              GPIOB
#define US3_ECHO_PIN               GPIO_PIN_6
#define US3_TIMER                  TIM4
#define US3_CHANNEL                1

/* Ultrasonic Sensor 4 (Right) */
#define US4_TRIG_PORT              GPIOB
#define US4_TRIG_PIN               GPIO_PIN_7
#define US4_ECHO_PORT              GPIOB
#define US4_ECHO_PIN               GPIO_PIN_7
#define US4_TIMER                  TIM4
#define US4_CHANNEL                2

/*----------------------------------------------------------------------------*/
/* Communication Interface Pin Configurations                                 */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*  Activate these cofigurations if you Approve them
/*----------------------------------------------------------------------------*/

/*
// CAN Bus 
#define CAN_TX_PORT                GPIOA
#define CAN_TX_PIN                 GPIO_PIN_12
#define CAN_RX_PORT                GPIOA
#define CAN_RX_PIN                 GPIO_PIN_11

I2C for MPU6050 
#define MPU6050_I2C_PORT           GPIOB
#define MPU6050_I2C_SCL_PIN        GPIO_PIN_6
#define MPU6050_I2C_SDA_PIN        GPIO_PIN_7

 UART for GPS 
#define GPS_UART_PORT              GPIOA
#define GPS_UART_TX_PIN            GPIO_PIN_9
#define GPS_UART_RX_PIN            GPIO_PIN_10

#endif   */