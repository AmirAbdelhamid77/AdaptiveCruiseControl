/*
 * 		ultrasonic_driver.c
 *
 *  Created on: AnotherSomeDay
 *      Author: AMiR & C.
 *      Description: HC-SR04 Ultrasonic driver for STM32F103C8T6
 */

#include "stm32_ultrasonic_driver.h"

//-----------------------------
// Static Variables
//-----------------------------
static ULTRASONIC_Config_t Ultrasonic_Sensors[ULTRASONIC_COUNT];
static volatile uint8_t Initialized = 0;
static volatile uint32_t overflowCount = 0;

//-----------------------------
// Static Function Prototypes
//-----------------------------
static void delay_us(uint32_t us);
static uint32_t micros(void);
static void timer_init(void);

/**
 *@discreption : Initialize all ultrasonic sensors
*/
void HAL_ULTRASONIC_Init(void)
{
    // Configure Front Ultrasonic using hw_config.h definitions
    Ultrasonic_Sensors[ULTRASONIC_FRONT].TRIG_Port = US1_TRIG_PORT;
    Ultrasonic_Sensors[ULTRASONIC_FRONT].TRIG_Pin = US1_TRIG_PIN;
    Ultrasonic_Sensors[ULTRASONIC_FRONT].ECHO_Port = US1_ECHO_PORT;
    Ultrasonic_Sensors[ULTRASONIC_FRONT].ECHO_Pin = US1_ECHO_PIN;
    Ultrasonic_Sensors[ULTRASONIC_FRONT].ID = ULTRASONIC_FRONT;
    Ultrasonic_Sensors[ULTRASONIC_FRONT].LastDistance = ULTRASONIC_INVALID_DISTANCE;

    // Configure Rear Ultrasonic using hw_config.h definitions
    Ultrasonic_Sensors[ULTRASONIC_REAR].TRIG_Port = US2_TRIG_PORT;
    Ultrasonic_Sensors[ULTRASONIC_REAR].TRIG_Pin = US2_TRIG_PIN;
    Ultrasonic_Sensors[ULTRASONIC_REAR].ECHO_Port = US2_ECHO_PORT;
    Ultrasonic_Sensors[ULTRASONIC_REAR].ECHO_Pin = US2_ECHO_PIN;
    Ultrasonic_Sensors[ULTRASONIC_REAR].ID = ULTRASONIC_REAR;
    Ultrasonic_Sensors[ULTRASONIC_REAR].LastDistance = ULTRASONIC_INVALID_DISTANCE;
    
    // Configure Left Ultrasonic using hw_config.h definitions
    Ultrasonic_Sensors[ULTRASONIC_LEFT].TRIG_Port = US3_TRIG_PORT;
    Ultrasonic_Sensors[ULTRASONIC_LEFT].TRIG_Pin = US3_TRIG_PIN;
    Ultrasonic_Sensors[ULTRASONIC_LEFT].ECHO_Port = US3_ECHO_PORT;
    Ultrasonic_Sensors[ULTRASONIC_LEFT].ECHO_Pin = US3_ECHO_PIN;
    Ultrasonic_Sensors[ULTRASONIC_LEFT].ID = ULTRASONIC_LEFT;
    Ultrasonic_Sensors[ULTRASONIC_LEFT].LastDistance = ULTRASONIC_INVALID_DISTANCE;
    
    // Configure Right Ultrasonic using hw_config.h definitions
    Ultrasonic_Sensors[ULTRASONIC_RIGHT].TRIG_Port = US4_TRIG_PORT;
    Ultrasonic_Sensors[ULTRASONIC_RIGHT].TRIG_Pin = US4_TRIG_PIN;
    Ultrasonic_Sensors[ULTRASONIC_RIGHT].ECHO_Port = US4_ECHO_PORT;
    Ultrasonic_Sensors[ULTRASONIC_RIGHT].ECHO_Pin = US4_ECHO_PIN;
    Ultrasonic_Sensors[ULTRASONIC_RIGHT].ID = ULTRASONIC_RIGHT;
    Ultrasonic_Sensors[ULTRASONIC_RIGHT].LastDistance = ULTRASONIC_INVALID_DISTANCE;
    
    // Initialize all sensors
    for (uint8_t i = 0; i < ULTRASONIC_COUNT; i++) {
        HAL_ULTRASONIC_Init_Sensor(&Ultrasonic_Sensors[i]);
    }
    
    // Initialize timer for microsecond counting
    timer_init();
    
    Initialized = 1;
}

/**
 @discreption: Initialize a specific ultrasonic sensor
  */
uint8_t HAL_ULTRASONIC_Init_Sensor(ULTRASONIC_Config_t* Sensor)
{
    GPIO_PinConfig_t GPIO_Pin_Config;

    if (Sensor == NULL) {
        return ULTRASONIC_ERROR;
    }
    
    // Enable GPIO Clock for both ports
    MCAL_GPIO_Enable_ClocK(Sensor->TRIG_Port);
    MCAL_GPIO_Enable_ClocK(Sensor->ECHO_Port);
    
    // Configure Trigger pin as output push-pull
    GPIO_Pin_Config.GPIO_PinNumber = Sensor->TRIG_Pin;
    GPIO_Pin_Config.GPIO_Mode = GPIO_Mode_Out_push_pull;
    GPIO_Pin_Config.GPIO_Speed = GPIO_Speed_10MHz;
    MCAL_GPIO_Init(Sensor->TRIG_Port, &GPIO_Pin_Config);
    
    // Configure Echo pin as input floating
    GPIO_Pin_Config.GPIO_PinNumber = Sensor->ECHO_Pin;
    GPIO_Pin_Config.GPIO_Mode = GPIO_Mode_Inp_Floating;
    MCAL_GPIO_Init(Sensor->ECHO_Port, &GPIO_Pin_Config);
    
    // Set Trigger pin to LOW initially
    MCAL_GPIO_WritePin(Sensor->TRIG_Port, Sensor->TRIG_Pin, GPIO_PIN_RESET);
    
    // Small delay to ensure pin stabilizes
    delay_us(10);
    
    return ULTRASONIC_OK;
}

/**
 * @discreption: Initialize Timer for microsecond timing
 * Using timer configuration from hw_config.h
 */
static void timer_init(void)
{
    // Enable timer clock (using ULTRASONIC_TIMER from hw_config.h)
    if (ULTRASONIC_TIMER == TIM2) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    } else if (ULTRASONIC_TIMER == TIM3) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    } else if (ULTRASONIC_TIMER == TIM4) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    }
    
    // Configure timer for microsecond counting
    // Set prescaler from hw_config.h
    ULTRASONIC_TIMER->PSC = ULTRASONIC_PRESCALER - 1;
    
    // Set auto-reload register to maximum
    ULTRASONIC_TIMER->ARR = ULTRASONIC_PERIOD;
    
    // Enable update interrupt
    ULTRASONIC_TIMER->DIER |= TIM_DIER_UIE;
    
    // Enable the timer
    ULTRASONIC_TIMER->CR1 |= TIM_CR1_CEN;
    
    // Enable timer interrupt in NVIC
    if (ULTRASONIC_TIMER == TIM2) {
        NVIC_EnableIRQ(TIM2_IRQn);
        NVIC_SetPriority(TIM2_IRQn, 0);
    } else if (ULTRASONIC_TIMER == TIM3) {
        NVIC_EnableIRQ(TIM3_IRQn);
        NVIC_SetPriority(TIM3_IRQn, 0);
    } else if (ULTRASONIC_TIMER == TIM4) {
        NVIC_EnableIRQ(TIM4_IRQn);
        NVIC_SetPriority(TIM4_IRQn, 0);
    }
    
    // Reset overflow counter
    overflowCount = 0;
}

/**
 * @discreption: Get current time in microseconds
 * @note Using configured timer to track microseconds
 */
static uint32_t micros(void)
{
    // Return the current timer value
    // Timer is configured to count microseconds (1MHz frequency)
    return ULTRASONIC_TIMER->CNT + (overflowCount * ULTRASONIC_PERIOD);
}

/**
 * @brief Timer interrupt handler
 * Note: This function needs to be properly defined in the startup file
 * or the appropriate IRQ handler file based on which timer is used
 */
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF) {
        // Clear the update interrupt flag
        TIM2->SR &= ~TIM_SR_UIF;
        
        // Increment overflow counter
        overflowCount++;
    }
}

/**
 * @@discreption: Trigger a distance measurement for a single sensor
 */
uint8_t HAL_ULTRASONIC_Trigger(ULTRASONIC_Config_t* Sensor)
{
    if (Sensor == NULL) {
        return ULTRASONIC_ERROR;
    }

    // Set Trigger pin to LOW for clean state
    MCAL_GPIO_WritePin(Sensor->TRIG_Port, Sensor->TRIG_Pin, GPIO_PIN_RESET);
    delay_us(2);
    
    // Send 10us trigger pulse
    MCAL_GPIO_WritePin(Sensor->TRIG_Port, Sensor->TRIG_Pin, GPIO_PIN_SET);
    delay_us(10);
    MCAL_GPIO_WritePin(Sensor->TRIG_Port, Sensor->TRIG_Pin, GPIO_PIN_RESET);
    
    return ULTRASONIC_OK;
}

/**
 * @discreption: Read the distance from a sensor (blocking mode)
  */
uint8_t HAL_ULTRASONIC_Read(ULTRASONIC_Config_t* Sensor, uint16_t* Distance)
{
    uint32_t start_time, current_time, timeout_time;
    uint32_t pulse_duration;
    uint16_t distance;
    
    if (Sensor == NULL || Distance == NULL) {
        return ULTRASONIC_ERROR;
    }
    
    // Wait for echo pin to go HIGH (start of echo pulse)
    timeout_time = micros() + ULTRASONIC_TIMEOUT;
    while (MCAL_GPIO_ReadPin(Sensor->ECHO_Port, Sensor->ECHO_Pin) == GPIO_PIN_RESET) {
        current_time = micros();
        if (current_time > timeout_time) {
            *Distance = ULTRASONIC_INVALID_DISTANCE;
            return ULTRASONIC_TIMEOUT_ERROR;
        }
    }
    
    // Record start time
    start_time = micros();
    
    // Wait for echo pin to go LOW (end of echo pulse)
    timeout_time = start_time + ULTRASONIC_TIMEOUT;
    while (MCAL_GPIO_ReadPin(Sensor->ECHO_Port, Sensor->ECHO_Pin) == GPIO_PIN_SET) {
        current_time = micros();
        if (current_time > timeout_time) {
            *Distance = ULTRASONIC_INVALID_DISTANCE;
            return ULTRASONIC_TIMEOUT_ERROR;
        }
    }
    
    // Calculate pulse duration
    pulse_duration = micros() - start_time;

    // Calculate distance (sound travels at ~343m/s )
    // Distance = (time * speed of sound) / 2 (for Go & back)
    // Speed of sound = 343 m/s = 0.0343 cm/us
    // Distance (cm) = (pulse_duration * 0.0343) / 2
    distance = (pulse_duration * 343) / 20000;
    
    // Validate distance
    if (distance > ULTRASONIC_MAX_DISTANCE) {
        *Distance = ULTRASONIC_MAX_DISTANCE;
    } else {
        *Distance = distance;
    }
    
    // Update last distance
    Sensor->LastDistance = *Distance;
    
    return ULTRASONIC_OK;
}

/**
 * @discreption Measure the distance from a sensor (blocking mode)
 */
uint8_t HAL_ULTRASONIC_MeasureDistance(ULTRASONIC_Config_t* Sensor, uint16_t* Distance)
{
    uint8_t status;
    
    if (Sensor == NULL || Distance == NULL) {
        return ULTRASONIC_ERROR;
    }
    
    // Trigger the sensor
    status = HAL_ULTRASONIC_Trigger(Sensor);
    if (status != ULTRASONIC_OK) {
        return status;
    }
    
    // Small delay to allow the signal to start
    delay_us(50);
    
    // Read the distance
    return HAL_ULTRASONIC_Read(Sensor, Distance);
}

/**
 * @discreption Measure the distance from a sensor by ID (blocking mode)
 */
uint8_t HAL_ULTRASONIC_MeasureDistanceByID(uint8_t SensorID, uint16_t* Distance)
{
    if (!Initialized || SensorID >= ULTRASONIC_COUNT || Distance == NULL) {
        return ULTRASONIC_ERROR;
    }
    
    return HAL_ULTRASONIC_MeasureDistance(&Ultrasonic_Sensors[SensorID], Distance);
}

/**
 * @discreption Measure distances from all sensors (blocking mode)
  */
uint8_t HAL_ULTRASONIC_MeasureAllDistances(uint16_t Distances[ULTRASONIC_COUNT])
{
    uint8_t status = ULTRASONIC_OK;
    uint8_t result;

    if (!Initialized || Distances == NULL) {
        return ULTRASONIC_ERROR;
    }
    
    // Measure distance from each sensor
    for (uint8_t i = 0; i < ULTRASONIC_COUNT; i++) {
        result = HAL_ULTRASONIC_MeasureDistance(&Ultrasonic_Sensors[i], &Distances[i]);
        if (result != ULTRASONIC_OK) {
            Distances[i] = ULTRASONIC_INVALID_DISTANCE;
            status = ULTRASONIC_ERROR;
        }

        // Adding a small delay between measurements to avoid cross-interference
        delay_us(10000);  // 10ms delay
    }
    
    return status;
}

/**
 * @discreption: Get the last measured distance from a sensor
 */
uint16_t HAL_ULTRASONIC_GetLastDistance(uint8_t SensorID)
{
    if (!Initialized || SensorID >= ULTRASONIC_COUNT) {
        return ULTRASONIC_INVALID_DISTANCE;
    }
    
    return Ultrasonic_Sensors[SensorID].LastDistance;
}

/**
 * @discreption Microsecond delay function
 * => Number of microseconds to delay
 * @assuming this function is for system that's clock is 72MHz
 */
static void delay_us(uint32_t us)
{
    uint32_t start = micros();
    uint32_t end = start + us;
    
    // Handle potential overflow
    if (end < start) {
        // Wait until overflow occurs
        while (micros() >= start) {}
        // Then wait until we reach the target time
        while (micros() < end) {}
    } else {
        // Normal case, no overflow
        while (micros() < end) {}
    }
}

/**
 * @discreption: Get current time in microseconds
 * @note Using Timer2 to track microseconds
 */
static uint32_t micros(void)
{
    // Return the current timer value
    // TIM2 is configured to count microseconds (1MHz frequency)
    return TIM2->CNT + (overflowCount * 65536);
}

static volatile uint32_t overflowCount = 0;

/**
 * @discreption: Initialize Timer2 for microsecond timing
 */
static void timer_init(void)
{
    // Enable TIM2 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    
    // Configure TIM2 for microsecond counting
    // Set prescaler to get a 1MHz clock (assuming 72MHz system clock)
    // 72MHz / 72 = 1MHz = 1us per count
    TIM2->PSC = 72 - 1;
    
    // Set auto-reload register to maximum
    TIM2->ARR = 0xFFFF;
    
    // Enable update interrupt
    TIM2->DIER |= TIM_DIER_UIE;
    
    // Enable the timer
    TIM2->CR1 |= TIM_CR1_CEN;
    
    // Enable TIM2 interrupt in NVIC
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 0);
    
    // Reset overflow counter
    overflowCount = 0;
}

/**
 * @brief TIM2 interrupt handler
 */
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF) {
        // Clear the update interrupt flag
        TIM2->SR &= ~TIM_SR_UIF;
        
        // Increment overflow counter
        overflowCount++;
    }
}
