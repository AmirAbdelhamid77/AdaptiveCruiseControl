/**

  function for the AdaptiveCruiseControl Function V.1.0
 **/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f103x6.h"
#include "stm32_F103C6_gpio_driver.h"
#include "motor_driver.h"
#include "ultrasonic_driver.h"
#include "speed_sensor.h"
#include "hw_config.h"  
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define SYSTEM_TICK_MS           10        // System tick in milliseconds
#define ACC_UPDATE_RATE_MS       100       // ACC update rate in milliseconds
#define SPEED_CALC_INTERVAL_MS   1000      // Speed calculation interval in milliseconds
#define DISTANCE_SCAN_INTERVAL_MS 200      // Distance scanning interval in milliseconds

// ACC Parameters
#define ACC_MIN_FOLLOW_DISTANCE_CM  30     // Minimum following distance in cm
#define ACC_MAX_FOLLOW_DISTANCE_CM  100    // Maximum following distance in cm
#define ACC_TARGET_SPEED_KMH        10     // Target cruise speed in km/h
#define ACC_SPEED_TOLERANCE_KMH     0.5    // Speed tolerance in km/h

// ACC States
#define ACC_STATE_OFF            0
#define ACC_STATE_ACTIVE         1
#define ACC_STATE_BRAKING        2
#define ACC_STATE_ACCELERATING   3

// Motor Speed Levels (percentage)
#define MOTOR_SPEED_STOP         0
#define MOTOR_SPEED_SLOW         20
#define MOTOR_SPEED_MAINTAIN     30
#define MOTOR_SPEED_MEDIUM       40
#define MOTOR_SPEED_FAST         50

/* Private types -------------------------------------------------------------*/
typedef struct {
  uint8_t active;                  // ACC active flag
  uint16_t targetDistance_cm;      // Target following distance in cm
  float targetSpeed_kmh;           // Target cruise speed in km/h
  uint8_t state;                   // Current ACC state
} ACC_Config_t;

/* Private variables ---------------------------------------------------------*/
static Car_Config_t CarConfig;
static ACC_Config_t AccConfig;
static uint32_t systemTicks = 0;
static uint32_t lastSpeedCalcTime = 0;
static uint32_t lastDistanceScanTime = 0;
static uint32_t lastAccUpdateTime = 0;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void GPIO_Init(void);
static void Motors_Init(void);
static void Ultrasonics_Init(void);
static void SpeedSensors_Init(void);
static void ACC_Init(void);
static void ACC_Update(void);
static void SysTick_Handler(void);
static void EXTI_Config(void);
static void HAL_Init(void);
static void delay_ms(uint32_t ms);

/**
  * @desciption  primary ACC application-> entry point.
  */
int main(void)
{
  HAL_Init();
  
  SystemClock_Config();
  
  GPIO_Init();
  Motors_Init();
  Ultrasonics_Init();
  SpeedSensors_Init();
  
  /* Initialization of ACC system */
  ACC_Init();
  while (1)
  {
    /* Calculate speed at regular times */
    if (systemTicks - lastSpeedCalcTime >= SPEED_CALC_INTERVAL_MS) {
      SPEED_SENSOR_CalculateSpeed(SPEED_SENSOR_LEFT, SPEED_CALC_INTERVAL_MS);
      SPEED_SENSOR_CalculateSpeed(SPEED_SENSOR_RIGHT, SPEED_CALC_INTERVAL_MS);
      lastSpeedCalcTime = systemTicks;
    }
    
    /* Scan distances at regular times */
    if (systemTicks - lastDistanceScanTime >= DISTANCE_SCAN_INTERVAL_MS) {
      uint16_t distances[ULTRASONIC_COUNT];
      HAL_ULTRASONIC_MeasureAllDistances(distances);
      lastDistanceScanTime = systemTicks;
    }
    
    /* Update ACC system */
    if (systemTicks - lastAccUpdateTime >= ACC_UPDATE_RATE_MS) {
      ACC_Update();
      lastAccUpdateTime = systemTicks;
    }
    
    /* Process sensor data for collision detection and obstacle avoidance */
    ProcessSensorData();
  }
}

static void SystemClock_Config(void)
{
  /* Setting up the clock system at 72MHz */
  
  /* HSE Enabling */
  RCC->CR |= RCC_CR_HSEON;
  
  while(!(RCC->CR & RCC_CR_HSERDY));
  
  /* Configure PLL: source=HSE, PLLMUL=9 (8MHz * 9 = 72MHz) */
  RCC->CFGR &= ~RCC_CFGR_PLLSRC;
  RCC->CFGR |= RCC_CFGR_PLLSRC;    // PLL source = HSE
  RCC->CFGR &= ~RCC_CFGR_PLLMUL;
  RCC->CFGR |= RCC_CFGR_PLLMUL9;   // PLL multiplier = 9
  
  /* Enable PLL */
  RCC->CR |= RCC_CR_PLLON;
  
  /* Wait till PLL is ready */
  while(!(RCC->CR & RCC_CR_PLLRDY));
  
  /* Select PLL as system clock source */
  RCC->CFGR &= ~RCC_CFGR_SW;
  RCC->CFGR |= RCC_CFGR_SW_PLL;
  
  /* Wait till PLL is used as system clock source */
  while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
  
  /* Configure AHB, APB1, APB2 prescalers */
  RCC->CFGR &= ~RCC_CFGR_HPRE;     // AHB prescaler = 1
  RCC->CFGR &= ~RCC_CFGR_PPRE1;
  RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; // APB1 prescaler = 2 (36MHz max)
  RCC->CFGR &= ~RCC_CFGR_PPRE2;    // APB2 prescaler = 1
  
  /* Configure SysTick to generate interrupt every 1ms */
  SysTick_Config(72000);  // 72MHz / 1000Hz = 72000
}

/**
  * @desciption GPIO Initialization Function
  */
static void GPIO_Init(void)
{

  MCAL_GPIO_Enable_ClocK(GPIOA);
  MCAL_GPIO_Enable_ClocK(GPIOB);
  MCAL_GPIO_Enable_ClocK(GPIOC);
  
  GPIO_PinConfig_t pinConfig;
  
  /* Configure PC13 as output (onboard LED) */
  pinConfig.GPIO_PinNumber = GPIO_PIN_13;
  pinConfig.GPIO_Mode = GPIO_Mode_Out_push_pull;
  pinConfig.GPIO_Speed = GPIO_Speed_10MHz;
  MCAL_GPIO_Init(GPIOC, &pinConfig);
}

/**
  * @desciption Motors Initialization Function
  */
static void Motors_Init(void)
{
  // Initialize all members of CarConfig to safe defaults first
  memset(&CarConfig, 0, sizeof(Car_Config_t));
  
  /* Motor 1 (Front Left) */
  CarConfig.Motors[MOTOR_1].IN1_Port = MOTOR1_IN1_PORT;
  CarConfig.Motors[MOTOR_1].IN1_Pin = MOTOR1_IN1_PIN;
  CarConfig.Motors[MOTOR_1].IN2_Port = MOTOR1_IN2_PORT;
  CarConfig.Motors[MOTOR_1].IN2_Pin = MOTOR1_IN2_PIN;
  CarConfig.Motors[MOTOR_1].EN_Port = MOTOR1_PWM_PORT;
  CarConfig.Motors[MOTOR_1].EN_Pin = MOTOR1_PWM_PIN;
  CarConfig.Motors[MOTOR_1].PWM_Channel = MOTOR1_PWM_CHANNEL;
  
  /* Motor 2 (Front Right) */
  CarConfig.Motors[MOTOR_2].IN1_Port = MOTOR2_IN1_PORT;
  CarConfig.Motors[MOTOR_2].IN1_Pin = MOTOR2_IN1_PIN;
  CarConfig.Motors[MOTOR_2].IN2_Port = MOTOR2_IN2_PORT;
  CarConfig.Motors[MOTOR_2].IN2_Pin = MOTOR2_IN2_PIN;
  CarConfig.Motors[MOTOR_2].EN_Port = MOTOR2_PWM_PORT;
  CarConfig.Motors[MOTOR_2].EN_Pin = MOTOR2_PWM_PIN;
  CarConfig.Motors[MOTOR_2].PWM_Channel = MOTOR2_PWM_CHANNEL;
  
  /* Motor 3 (Rear Left) */
  CarConfig.Motors[MOTOR_3].IN1_Port = MOTOR3_IN1_PORT;
  CarConfig.Motors[MOTOR_3].IN1_Pin = MOTOR3_IN1_PIN;
  CarConfig.Motors[MOTOR_3].IN2_Port = MOTOR3_IN2_PORT;
  CarConfig.Motors[MOTOR_3].IN2_Pin = MOTOR3_IN2_PIN;
  CarConfig.Motors[MOTOR_3].EN_Port = MOTOR3_PWM_PORT;
  CarConfig.Motors[MOTOR_3].EN_Pin = MOTOR3_PWM_PIN;
  CarConfig.Motors[MOTOR_3].PWM_Channel = MOTOR3_PWM_CHANNEL;
  
  /* Motor 4 (Rear Right) */
  CarConfig.Motors[MOTOR_4].IN1_Port = MOTOR4_IN1_PORT;
  CarConfig.Motors[MOTOR_4].IN1_Pin = MOTOR4_IN1_PIN;
  CarConfig.Motors[MOTOR_4].IN2_Port = MOTOR4_IN2_PORT;
  CarConfig.Motors[MOTOR_4].IN2_Pin = MOTOR4_IN2_PIN;
  CarConfig.Motors[MOTOR_4].EN_Port = MOTOR4_PWM_PORT;
  CarConfig.Motors[MOTOR_4].EN_Pin = MOTOR4_PWM_PIN;
  CarConfig.Motors[MOTOR_4].PWM_Channel = MOTOR4_PWM_CHANNEL;
  
  
  MOTOR_Init(&CarConfig);
}

/**
  * @desciption Ultrasonics Initialization Function  */
static void Ultrasonics_Init(void)
{
  /* Initialize ultrasonic sensors */
  HAL_ULTRASONIC_Init();
}

/**
  * @desciption Speed Sensors Initialization Function  */
static void SpeedSensors_Init(void)
{
  /* left speed sensor */
  SPEED_SENSOR_Init(SPEED_SENSOR_LEFT, GPIOB, GPIO_PIN_12);
  
  /* right speed sensor */
  SPEED_SENSOR_Init(SPEED_SENSOR_RIGHT, GPIOB, GPIO_PIN_13);
  
  /* Reset counters */
  SPEED_SENSOR_ResetCount(SPEED_SENSOR_LEFT);
  SPEED_SENSOR_ResetCount(SPEED_SENSOR_RIGHT);
}

/**
  * @desciption ACC Initialization Function
  */
static void ACC_Init(void)
{
  /* Initialize ACC configuration */
  AccConfig.active = 1;  
  AccConfig.targetDistance_cm = 60;  // Default target distance
  AccConfig.targetSpeed_kmh = ACC_TARGET_SPEED_KMH;  // Default target speed
  AccConfig.state = ACC_STATE_OFF;
}
/**   
*@desciption   PID function for smoother movement
*/
// PID Controller Parameters
#define PID_KP                    2.0     // Proportional gain
#define PID_KI                    0.5     // Integral gain
#define PID_KD                    1.0     // Derivative gain

// Speed ramping parameters
#define SPEED_RAMP_RATE           2       // Maximum speed change per update (percentage)
#define DISTANCE_BUFFER_ZONE      20      // Buffer zone for smoother transitions (cm)

// Additional motor speed levels for finer control
#define MOTOR_SPEED_VERY_SLOW     10
#define MOTOR_SPEED_SLOW_PLUS     25
#define MOTOR_SPEED_MEDIUM_PLUS   45


static float pid_integral = 0;
static float pid_previous_error = 0;
static uint8_t current_motor_speed = 0;    // Current motor speed (0-100)

/**
  * @desciption ACC Update Function -  Adaptive Cruise Control logic
  */
static void ACC_Update(void)
{
  if (!AccConfig.active) {
    return;  // ACC is not active, do nothing
  }
  
  /* Get current speed (average of left and right wheels) */
  float currentSpeed_kmh = (SPEED_SENSOR_GetSpeedKMH(SPEED_SENSOR_LEFT) + 
                           SPEED_SENSOR_GetSpeedKMH(SPEED_SENSOR_RIGHT)) / 2.0f;
  
  /* Get distance to vehicle in front */
  uint16_t frontDistance_cm = HAL_ULTRASONIC_GetLastDistance(ULTRASONIC_FRONT);
  
  /* Calculate target speed based on distance */
  float targetSpeed = AccConfig.targetSpeed_kmh;
  
  if (frontDistance_cm != ULTRASONIC_INVALID_DISTANCE && 
      frontDistance_cm < ACC_MAX_FOLLOW_DISTANCE_CM) {
    /* Adjust target speed based on distance to vehicle in front */
    float distanceRatio = (float)(frontDistance_cm - ACC_MIN_FOLLOW_DISTANCE_CM) / 
                         (float)(ACC_MAX_FOLLOW_DISTANCE_CM - ACC_MIN_FOLLOW_DISTANCE_CM);
    targetSpeed = distanceRatio * AccConfig.targetSpeed_kmh;
    
    /* Limit to minimum safe speed */
    if (targetSpeed < 0) targetSpeed = 0;
  }
  
  /* PID control for smoother speed transitions */
  float error = targetSpeed - currentSpeed_kmh;
  pid_integral += error * (ACC_UPDATE_RATE_MS / 1000.0f);
  float derivative = (error - pid_previous_error) / (ACC_UPDATE_RATE_MS / 1000.0f);
  
  /* Calculate PID output */
  float pid_output = (PID_KP * error) + (PID_KI * pid_integral) + (PID_KD * derivative);
  pid_previous_error = error;
  
  /* Convert PID output to motor speed (0-100%) */
  int target_motor_speed = (int)(pid_output * 10.0f + 30.0f); // Scale and offset
  
  /* Apply speed ramping for smoother transitions */
  if (target_motor_speed > current_motor_speed + SPEED_RAMP_RATE) {
    current_motor_speed += SPEED_RAMP_RATE;
  } else if (target_motor_speed < current_motor_speed - SPEED_RAMP_RATE) {
    current_motor_speed -= SPEED_RAMP_RATE;
  } else {
    current_motor_speed = target_motor_speed;
  }
  
  /* Clamp motor speed to valid range */
  if (current_motor_speed > 100) current_motor_speed = 100;
  if (current_motor_speed < 0) current_motor_speed = 0;
  
  /* Update ACC state based on conditions */
  if (frontDistance_cm != ULTRASONIC_INVALID_DISTANCE && 
      frontDistance_cm < ACC_MIN_FOLLOW_DISTANCE_CM) {
    /* Too close, need to brake */
    AccConfig.state = ACC_STATE_BRAKING;
    MOTOR_Move(MOTOR_SPEED_STOP, 0);  // Stop
  } else if (current_motor_speed < 10) {
    AccConfig.state = ACC_STATE_BRAKING;
    MOTOR_Move(current_motor_speed, 0);
  } else if (current_motor_speed > 40) {
    AccConfig.state = ACC_STATE_ACCELERATING;
    MOTOR_Move(current_motor_speed, 0);
  } else {
    AccConfig.state = ACC_STATE_ACTIVE;
    MOTOR_Move(current_motor_speed, 0);
  }
}

/**
  * @description: Process sensor data and detect potential accidents
  */
static void ProcessSensorData(void)
{
  /* Get ultrasonic distances */
  uint16_t frontDistance = HAL_ULTRASONIC_GetLastDistance(ULTRASONIC_FRONT);
  uint16_t rearDistance = HAL_ULTRASONIC_GetLastDistance(ULTRASONIC_REAR);
  uint16_t leftDistance = HAL_ULTRASONIC_GetLastDistance(ULTRASONIC_LEFT);
  uint16_t rightDistance = HAL_ULTRASONIC_GetLastDistance(ULTRASONIC_RIGHT);
  
  /* Get current speed */
  float leftSpeed = SPEED_SENSOR_GetSpeedKMH(SPEED_SENSOR_LEFT);
  float rightSpeed = SPEED_SENSOR_GetSpeedKMH(SPEED_SENSOR_RIGHT);
  float averageSpeed = (leftSpeed + rightSpeed) / 2.0f;
  /* Collision detection logic */
  // Front collision detection with progressive braking based on distance and speed
  if (frontDistance < 100 && averageSpeed > 0.0f) {
    // Calculate braking intensity based on distance and speed
    float collisionRisk = (averageSpeed * averageSpeed) / (frontDistance * 2.0f);
    
    if (collisionRisk > 2.0f || frontDistance < 20) {
      // High risk of collision - emergency stop
      MOTOR_StopAll();
      // Toggle LED rapidly to indicate emergency
      MCAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      
      // Log collision warning
      // TODO: Send collision warning via CAN to Raspberry Pi
    }
    else if (collisionRisk > 1.0f || frontDistance < 40) {
      // Medium risk - hard braking
      AccConfig.state = ACC_STATE_BRAKING;
      MOTOR_Move(MOTOR_SPEED_VERY_SLOW, 0);
      // Solid LED to indicate braking
      MCAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    }
    else if (collisionRisk > 0.5f || frontDistance < 60) {
      // Low risk - gentle braking
      AccConfig.state = ACC_STATE_BRAKING;
      MOTOR_Move(MOTOR_SPEED_SLOW, 0);
      // Blink LED slowly to indicate caution
      if ((systemTicks % 500) < 250)
        MCAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
      else
        MCAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    }
  }
  
  /* Obstacle avoidance logic */
  int8_t avoidanceTurn = 0;
  
  // Check for obstacles on sides while moving
  if (leftDistance < 30 && rightDistance >= 30) {
    // Obstacle on left side, turn right to avoid
    avoidanceTurn = 30; // 30% turn to the right
  }
  else if (rightDistance < 30 && leftDistance >= 30) {
    // Obstacle on right side, turn left to avoid
    avoidanceTurn = -30; // 30% turn to the left
  }
  else if (leftDistance < 30 && rightDistance < 30 && frontDistance >= 40) {
    // Obstacles on both sides, but front is clear - slow down and proceed carefully
    MOTOR_Move(MOTOR_SPEED_SLOW, 0);
  }
  else if (leftDistance < 20 && rightDistance < 20 && frontDistance < 40) {
    // Boxed in on all sides - stop
    MOTOR_StopAll();
    // Blink LED in SOS pattern
    uint32_t pattern = (systemTicks / 200) % 14;
    MCAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, (pattern < 3 || (pattern > 3 && pattern < 7) || (pattern > 7 && pattern < 11)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
  
  // Apply avoidance steering if needed and not in emergency stop
  if (avoidanceTurn != 0 && AccConfig.state != ACC_STATE_BRAKING) {
    // Get current forward speed and apply the turn
    uint8_t currentForwardSpeed = (AccConfig.state == ACC_STATE_ACCELERATING) ? 
                                 MOTOR_SPEED_MEDIUM : MOTOR_SPEED_SLOW_PLUS;
    MOTOR_Move(currentForwardSpeed, avoidanceTurn);
  }
  
  /* TODO: Process MPU6050 data for accident detection-> To be determined later */
  /* This would detect sudden acceleration changes indicating a collision */
  
  /* TODO: Process GPS data for location tracking-> To be determined later */
}

/**
  * @desciption SysTick Handler - Called every 1ms
  */
void SysTick_Handler(void)
{
  systemTicks++;
  
}

/**
  * @desciption Delay function in milliseconds
  */
static void delay_ms(uint32_t ms)
{
  uint32_t startTick = systemTicks;
  while ((systemTicks - startTick) < ms);
}

/**
  * @desciption External interrupt handler for speed sensor pulses
  */
void EXTI15_10_IRQHandler(void)
{
  /* Check if EXTI Line 12 interrupt occurred (left speed sensor) */
  if (EXTI->PR & (1 << 12)) {
    SPEED_SENSOR_UpdateCount(SPEED_SENSOR_LEFT);
    EXTI->PR |= (1 << 12);  // Clear pending bit
  }
  
  /* Check if EXTI Line 13 interrupt occurred (right speed sensor) */
  if (EXTI->PR & (1 << 13)) {
    SPEED_SENSOR_UpdateCount(SPEED_SENSOR_RIGHT);
    EXTI->PR |= (1 << 13);  // Clear pending bit
  }
}

/**
  * @desciption Configure external interrupts for speed sensors
  */
static void EXTI_Config(void)
{
  /* Enable AFIO clock */
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
  
  /* Configure EXTI line 12 (PB12 - left speed sensor) */
  AFIO->EXTICR[3] = (AFIO->EXTICR[3] & ~AFIO_EXTICR4_EXTI12) | AFIO_EXTICR4_EXTI12_PB;
  
  /* Configure EXTI line 13 (PB13 - right speed sensor) */
  AFIO->EXTICR[3] = (AFIO->EXTICR[3] & ~AFIO_EXTICR4_EXTI13) | AFIO_EXTICR4_EXTI13_PB;
  
  /* Enable rising edge trigger for EXTI lines 12 and 13 */
  EXTI->RTSR |= (1 << 12) | (1 << 13);
  
  /* Enable EXTI lines 12 and 13 */
  EXTI->IMR |= (1 << 12) | (1 << 13);
  
  /* Enable EXTI15_10 interrupt in NVIC */
  NVIC_EnableIRQ(EXTI15_10_IRQn);
  NVIC_SetPriority(EXTI15_10_IRQn, 3);
}

/**
  * @desciption HAL Initialization Function
  */
void HAL_Init(void)
{
  /* Set up priority grouping */
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  
  /* Initialize SysTick */
  SysTick_Config(SystemCoreClock / 1000);  // 1ms tick
  
  /* Configure external interrupts */
  EXTI_Config();
}
/**
  * @desciption TIM3 interrupt handler for ultrasonic sensor timing
  */
void TIM3_IRQHandler(void)
{
  if (TIM3->SR & TIM_SR_UIF) {
    /* Clear update interrupt flag */
    TIM3->SR &= ~TIM_SR_UIF;
    
    /* This is used by the ultrasonic driver to track long pulse durations */
    /* Notify the ultrasonic driver about timer overflow */
    extern volatile uint32_t overflowCount;
    overflowCount++;
  }
}

/**
  * @desciption TIM2 interrupt handler for ultrasonic sensor timing
  */
/**
  * @desciption TIMERS Initialization Function
  */
static void Timers_Init(void)
{
  /* Enable TIM2 clock for PWM generation (motor control) */
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  
  /* Configure TIM2 for PWM generation */
  /* Set prescaler to get a 1MHz clock (implementing on a 72MHz system clock) */
  /* 72MHz / 72 = 1MHz = 1us per count */
  TIM2->PSC = 72 - 1;
  
  /* Set auto-reload register to 1000 for 1kHz PWM frequency */
  /* 1MHz / 1000 = 1kHz */
  TIM2->ARR = 1000 - 1;
  
  /* Configure channels for PWM mode 1 */
  /* Channel 1 */
  TIM2->CCMR1 &= ~TIM_CCMR1_OC1M;
  TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;  // PWM mode 1
  TIM2->CCMR1 |= TIM_CCMR1_OC1PE;  // Preload enable
  
  /* Channel 2 */
  TIM2->CCMR1 &= ~TIM_CCMR1_OC2M;
  TIM2->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;  // PWM mode 1
  TIM2->CCMR1 |= TIM_CCMR1_OC2PE;  // Preload enable
  
  /* Channel 3 */
  TIM2->CCMR2 &= ~TIM_CCMR2_OC3M;
  TIM2->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;  // PWM mode 1
  TIM2->CCMR2 |= TIM_CCMR2_OC3PE;  // Preload enable
  
  /* Channel 4 */
  TIM2->CCMR2 &= ~TIM_CCMR2_OC4M;
  TIM2->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;  // PWM mode 1
  TIM2->CCMR2 |= TIM_CCMR2_OC4PE;  // Preload enable
  
  /* Enable output compare for all channels */
  TIM2->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
  
  /* Enable auto-reload preload */
  TIM2->CR1 |= TIM_CR1_ARPE;
  
  /* Enable counter */
  TIM2->CR1 |= TIM_CR1_CEN;
  
  /* Enable TIM3 clock for ultrasonic sensor timing */
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  
  /* Configure TIM3 for microsecond timing */
  /* Setting prescaler to get a 1MHz clock (implementing 72MHz system clock) */
  /* 72MHz / 72 = 1MHz = 1us per count */
  TIM3->PSC = 72 - 1;
  
  /* Set auto-reload register to maximum */
  TIM3->ARR = 0xFFFF;
  
  /* Enable update interrupt */
  TIM3->DIER |= TIM_DIER_UIE;
  
  /* Enable counter */
  TIM3->CR1 |= TIM_CR1_CEN;
  
  /* Enable TIM3 interrupt in NVIC */
  NVIC_EnableIRQ(TIM3_IRQn);
  NVIC_SetPriority(TIM3_IRQn, 1);  // Higher priority than EXTI
}
