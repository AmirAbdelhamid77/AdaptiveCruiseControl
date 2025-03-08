/*
 *      speed_sensor.c
 *      Created on: AnotherSomeDay
 *      Author: AMiR & Co.
 */

#include "speed_sensor.h"

/* Private variables */
static SPEED_SENSOR_Config_t SpeedSensors[SPEED_SENSOR_MAX];
static const float PI = 3.14159265359;
extern uint32_t HAL_GetTick(void); 


uint8_t SPEED_SENSOR_Init(SPEED_SENSOR_Channel_t Channel, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    if (Channel >= SPEED_SENSOR_MAX)
    {
        return SPEED_SENSOR_ERROR;
    }

    /* Store configuration */
    SpeedSensors[Channel].GPIOx = GPIOx;
    SpeedSensors[Channel].GPIO_Pin = GPIO_Pin;
    SpeedSensors[Channel].PulseCount = 0;
    SpeedSensors[Channel].LastPulseTime = 0;
    SpeedSensors[Channel].Speed_RPM = 0.0f;
    SpeedSensors[Channel].Speed_KMH = 0.0f;

    /* Enable GPIO clock */
    MCAL_GPIO_Enable_ClocK(GPIOx);

    /* Configure GPIO pin as input floating (for digital signal from HC-020K) */
    GPIO_PinConfig_t PinConfig;
    PinConfig.GPIO_PinNumber = GPIO_Pin;
    PinConfig.GPIO_Mode = GPIO_Mode_Inp_Floating;
    PinConfig.GPIO_Speed = GPIO_Speed_10MHz; // Using correct GPIO speed constant
    
    MCAL_GPIO_Init(GPIOx, &PinConfig);

    return SPEED_SENSOR_OK;
}

/**================================================================
 * @Fn:    SPEED_SENSOR_UpdateCount
 * @brief: Update the pulse count (to be called in GPIO interrupt handler)
 */
void SPEED_SENSOR_UpdateCount(SPEED_SENSOR_Channel_t Channel)
{
    if (Channel < SPEED_SENSOR_MAX)
    {
        SpeedSensors[Channel].PulseCount++;
        SpeedSensors[Channel].LastPulseTime = HAL_GetTick(); // Use system tick for timing
    }
}

/**================================================================
 * @Fn:    SPEED_SENSOR_ResetCount
 * @brief: Reset the pulse counter for a specific channel
 */
void SPEED_SENSOR_ResetCount(SPEED_SENSOR_Channel_t Channel)
{
    if (Channel < SPEED_SENSOR_MAX)
    {
        SpeedSensors[Channel].PulseCount = 0;
    }
}

/**================================================================
 * @Fn:    SPEED_SENSOR_GetPulseCount
 * @brief: Get the current pulse count for a specific channel
 */
uint32_t SPEED_SENSOR_GetPulseCount(SPEED_SENSOR_Channel_t Channel)
{
    if (Channel < SPEED_SENSOR_MAX)
    {
        return SpeedSensors[Channel].PulseCount;
    }
    return 0;
}

/**================================================================
 * @Fn:    SPEED_SENSOR_CalculateSpeed
 * @brief: Calculate the current speed based on pulse count
*/
void SPEED_SENSOR_CalculateSpeed(SPEED_SENSOR_Channel_t Channel, uint32_t TimeElapsed_ms)
{
    if (Channel < SPEED_SENSOR_MAX)
    {
        /* Calculating RPM: (pulses / slots_per_rev) * (60000 / time_ms) */
        float revolutions = (float)SpeedSensors[Channel].PulseCount / SPEED_SENSOR_SLOTS_PER_REVOLUTION;
        float minutes = (float)TimeElapsed_ms / 60000.0f;

        if (minutes > 0)
        {
            SpeedSensors[Channel].Speed_RPM = revolutions / minutes;

            /* Calculating speed in km/h: RPM * wheel circumference * 60 / 1000000 */
            float wheelCircumference_mm = PI * SPEED_SENSOR_WHEEL_DIAMETER_MM;
            SpeedSensors[Channel].Speed_KMH = (SpeedSensors[Channel].Speed_RPM * wheelCircumference_mm * 60.0f) / 1000000.0f;
        }
        else
        {
            SpeedSensors[Channel].Speed_RPM = 0.0f;
            SpeedSensors[Channel].Speed_KMH = 0.0f;
        }

        /* Reset pulse count for next calculation */
        SpeedSensors[Channel].PulseCount = 0;
    }
}

/**================================================================
 * @Fn:    SPEED_SENSOR_GetSpeedRPM
 * @brief: Get the current speed in RPM for a specific channel
 */
float SPEED_SENSOR_GetSpeedRPM(SPEED_SENSOR_Channel_t Channel)
{
    if (Channel < SPEED_SENSOR_MAX)
    {
        return SpeedSensors[Channel].Speed_RPM;
    }
    return 0.0f;
}

/**================================================================
 * @Fn:    SPEED_SENSOR_GetSpeedKMH
 * @brief: Get the current speed in km/h for a specific channel
 */
float SPEED_SENSOR_GetSpeedKMH(SPEED_SENSOR_Channel_t Channel)
{
    if (Channel < SPEED_SENSOR_MAX)
    {
        return SpeedSensors[Channel].Speed_KMH;
    }
    return 0.0f;
}
