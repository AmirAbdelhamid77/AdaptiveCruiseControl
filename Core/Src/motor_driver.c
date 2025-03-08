/*
 * 		motor_driver.c
 *
 *  Created on: AnotherSomeDay
 *      Author: AMiR & Co.
 */

#include "motor_driver.h"

// Static global variables
static Car_Config_t* g_car_config = NULL;

// Private helper functions forward declarations
static void MOTOR_ConfigureGPIO(Motor_Config_t* motor);
static void MOTOR_ConfigurePWM(void);

/**================================================================
 * @Fn                - MOTOR_Init
 * @description       - Initializes the 4 motors with their GPIO configurations
 */
void MOTOR_Init(Car_Config_t* car_cfg)
{
    if (car_cfg == NULL)
        return;

    g_car_config = car_cfg;
    
    // Configure each motor's GPIO pins
    for (uint8_t i = 0; i < 4; i++)
    {
        MOTOR_ConfigureGPIO(&car_cfg->Motors[i]);
    }
    
    // Configure PWM for speed control
    MOTOR_ConfigurePWM();

    // Stop all motors initially
    MOTOR_StopAll();
}

/**================================================================
 * @Fn          - MOTOR_ConfigureGPIO
 */
static void MOTOR_ConfigureGPIO(Motor_Config_t* motor)
{
    GPIO_PinConfig_t pin_config;

    // Enable clocks for GPIO ports
    MCAL_GPIO_Enable_ClocK(motor->IN1_Port);
    MCAL_GPIO_Enable_ClocK(motor->IN2_Port);
    MCAL_GPIO_Enable_ClocK(motor->EN_Port);
    
    // Configure IN1 pin as output push-pull
    pin_config.GPIO_PinNumber = motor->IN1_Pin;
    pin_config.GPIO_Mode = GPIO_Mode_Out_push_pull;
    pin_config.GPIO_Speed = GPIO_Speed_10MHz;
    MCAL_GPIO_Init(motor->IN1_Port, &pin_config);
    
    // Configure IN2 pin as output push-pull
    pin_config.GPIO_PinNumber = motor->IN2_Pin;
    pin_config.GPIO_Mode = GPIO_Mode_Out_push_pull;
    pin_config.GPIO_Speed = GPIO_Speed_10MHz;
    MCAL_GPIO_Init(motor->IN2_Port, &pin_config);
    
    // Configure EN pin as alternate function push-pull for PWM
    pin_config.GPIO_PinNumber = motor->EN_Pin;
    pin_config.GPIO_Mode = GPIO_Mode_Out_AF_push_pull;
    pin_config.GPIO_Speed = GPIO_Speed_50MHz;
    MCAL_GPIO_Init(motor->EN_Port, &pin_config);
}

/**================================================================
 * @Fn          - MOTOR_ConfigurePWM
 * @brief       - Configures the PWM timer for all motors
*/
static void MOTOR_ConfigurePWM(void)
{

    // Enable the timer clock (assuming TIM2 is used for PWM)
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Setup TIM2 for PWM mode
    // 1.prescaler to get a 1MHz clock (assuming 72MHz system clock)
    MOTOR_PWM_TIMER->PSC = 72 - 1;  

    // 2. Set auto-reload register for PWM period (1000 for 1kHz PWM frequency)
    MOTOR_PWM_TIMER->ARR = 1000 - 1;

    // 3. Configure channels for PWM mode 1 (active when counter < CCR)
    MOTOR_PWM_TIMER->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2); // CH1 PWM mode 1
    MOTOR_PWM_TIMER->CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2); // CH2 PWM mode 1
    MOTOR_PWM_TIMER->CCMR2 |= (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2); // CH3 PWM mode 1
    MOTOR_PWM_TIMER->CCMR2 |= (TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2); // CH4 PWM mode 1

    // 4. Enable output compare preload
    MOTOR_PWM_TIMER->CCMR1 |= (TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);
    MOTOR_PWM_TIMER->CCMR2 |= (TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE);

    // 5. Enable output channels
    MOTOR_PWM_TIMER->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);

    // 6. Set initial duty cycle to 0%
    MOTOR_PWM_TIMER->CCR1 = 0;
    MOTOR_PWM_TIMER->CCR2 = 0;
    MOTOR_PWM_TIMER->CCR3 = 0;
    MOTOR_PWM_TIMER->CCR4 = 0;

    // 7. Enable the timer
    MOTOR_PWM_TIMER->CR1 |= TIM_CR1_CEN;
}

/**================================================================
 * @Fn          - MOTOR_SetDirection
 * @brief       - Sets the direction of a specific motor
  */
void MOTOR_SetDirection(uint8_t motor_id, uint8_t direction)
{
    if (g_car_config == NULL || motor_id >= 4)
        return;
    
    Motor_Config_t* motor = &g_car_config->Motors[motor_id];

    switch (direction)
    {
        case MOTOR_DIRECTION_FORWARD:
            MCAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_SET);
            MCAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_RESET);
            break;
            
        case MOTOR_DIRECTION_BACKWARD:
            MCAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_RESET);
            MCAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_SET);
            break;
            
        case MOTOR_DIRECTION_STOP:
        default:
            MCAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_RESET);
            MCAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_RESET);
            break;
    }
}

/**================================================================
 * @Fn          - MOTOR_SetSpeed
 * @brief       - Sets the speed of a specific motor (0-100%)
 */
void MOTOR_SetSpeed(uint8_t motor_id, uint8_t speed)
{
    if (g_car_config == NULL || motor_id >= 4)
        return;
    
    // Clamp speed value to valid range
    if (speed > MOTOR_SPEED_MAX)
        speed = MOTOR_SPEED_MAX;

    // Calculate PWM duty cycle (0-1000) from speed percentage (0-100)
    uint16_t pwm_value = (speed * 1000) / 100;
    
    // Setting the PWM duty cycle for the selected motor
    switch (g_car_config->Motors[motor_id].PWM_Channel)
    {
        case 1:
            MOTOR_PWM_TIMER->CCR1 = pwm_value;
            break;
        case 2:
            MOTOR_PWM_TIMER->CCR2 = pwm_value;
            break;
        case 3:
            MOTOR_PWM_TIMER->CCR3 = pwm_value;
            break;
        case 4:
            MOTOR_PWM_TIMER->CCR4 = pwm_value;
            break;
        default:
            break;
    }
}

/**================================================================
 * @Fn          - MOTOR_Stop
 * @brief       - Stops a specific motor
 */
void MOTOR_Stop(uint8_t motor_id)
{
    if (g_car_config == NULL || motor_id >= 4)
        return;
    
    // Set direction to stop
    MOTOR_SetDirection(motor_id, MOTOR_DIRECTION_STOP);
    
    // Set speed to zero
    MOTOR_SetSpeed(motor_id, 0);
}

/**================================================================
 * @Fn          - MOTOR_StopAll
 * @brief       - Stops all motors
 */
void MOTOR_StopAll(void)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        MOTOR_Stop(i);
    }
}

/**================================================================
 * @Fn          - MOTOR_Move
 * @brief       - Controls the movement of the car based on forward speed and turn value
 */
void MOTOR_Move(int8_t forward_speed, int8_t turn_value)
{
    int16_t left_speed, right_speed;
    uint8_t direction;
    
    // Calculate left and right side speeds based on forward speed and turn value
    if (forward_speed >= 0)
    {
        // Moving forward
        direction = MOTOR_DIRECTION_FORWARD;

        if (turn_value >= 0)
        {
            // Turn right
            left_speed = forward_speed;
            right_speed = forward_speed - ((forward_speed * turn_value) / 100);
        }
        else
        {
            // Turn left
            left_speed = forward_speed - ((forward_speed * (-turn_value)) / 100);
            right_speed = forward_speed;
        }
    }
    else
    {
        // Moving backward
        direction = MOTOR_DIRECTION_BACKWARD;
        forward_speed = -forward_speed; // Make forward_speed positive for calculations
        
        if (turn_value >= 0)
        {
            // Turn right (while going backward)
            left_speed = forward_speed;
            right_speed = forward_speed - ((forward_speed * turn_value) / 100);
        }
        else
        {
            // Turn left (while going backward)
            left_speed = forward_speed - ((forward_speed * (-turn_value)) / 100);
            right_speed = forward_speed;
        }
    }
    
    // Clamp speeds to valid range
    if (left_speed < 0) left_speed = 0;
    if (left_speed > 100) left_speed = 100;
    if (right_speed < 0) right_speed = 0;
    if (right_speed > 100) right_speed = 100;
    
    //  (MOTOR_1 and MOTOR_3 are left side, MOTOR_2 and MOTOR_4 are right side)
    // Left side motors
    MOTOR_SetDirection(MOTOR_1, direction);
    MOTOR_SetDirection(MOTOR_3, direction);
    MOTOR_SetSpeed(MOTOR_1, left_speed);
    MOTOR_SetSpeed(MOTOR_3, left_speed);
    
    // Right side motors
    MOTOR_SetDirection(MOTOR_2, direction);
    MOTOR_SetDirection(MOTOR_4, direction);
    MOTOR_SetSpeed(MOTOR_2, right_speed);
    MOTOR_SetSpeed(MOTOR_4, right_speed);
}
