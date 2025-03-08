/*
 *       speed_sensor.h
 *       Created on: SomeDay
 *       Author: AMiR & Co.
 */

#ifndef SPEED_SENSOR_H_
#define SPEED_SENSOR_H_

#include "stm32_F103C6_gpio_driver.h"
#include "stm32f103x8.h"
#include "hw_config.h"  

/* Speed Sensor Configuration */
#define SPEED_SENSOR_SLOTS_PER_REVOLUTION   20      // Number of slots in the encoder disk as counted
#define SPEED_SENSOR_WHEEL_DIAMETER_MM      25      //As measured
#define SPEED_SENSOR_SAMPLE_TIME_MS         1000    // Time between speed calculations in ms

/* Speed Sensor Status */
#define SPEED_SENSOR_OK       0
#define SPEED_SENSOR_ERROR    1

/* Speed Sensor Channels */
typedef enum {
    SPEED_SENSOR_LEFT = 0,
    SPEED_SENSOR_RIGHT,
    SPEED_SENSOR_MAX
} SPEED_SENSOR_Channel_t;

/* Speed Sensor Configuration Structure */
typedef struct {
    GPIO_TypeDef* GPIOx;         // GPIO Port 
    uint16_t GPIO_Pin;            // GPIO Pin
    uint32_t PulseCount;          // Number of pulses counted
    uint32_t LastPulseTime;       // Time of last pulse (for timeout detection)
    float Speed_RPM;              // Speed in RPM
    float Speed_KMH;              // Speed in km/h
} SPEED_SENSOR_Config_t;


/**================================================================
 * @Fn: SPEED_SENSOR_Init
 * @brief -Initialize the speed sensor GPIO pins as input
 */
uint8_t SPEED_SENSOR_Init(SPEED_SENSOR_Channel_t Channel, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

/**================================================================
 * @Fn: SPEED_SENSOR_ResetCount
 * @brief: Reset the pulse counter for a specific channel
 */
void SPEED_SENSOR_ResetCount(SPEED_SENSOR_Channel_t Channel);

/**================================================================
 * @Fn: SPEED_SENSOR_GetPulseCount
 * @brief -Get the current pulse count for a specific channel
  */
uint32_t SPEED_SENSOR_GetPulseCount(SPEED_SENSOR_Channel_t Channel);

/**================================================================
 * @Fn: SPEED_SENSOR_UpdateCount
 * @brief -Update the pulse count (to be called in GPIO interrupt handler)
 */
void SPEED_SENSOR_UpdateCount(SPEED_SENSOR_Channel_t Channel);

/**================================================================
 * @Fn: SPEED_SENSOR_CalculateSpeed
 * @brief: Calculate the current speed based on pulse count
 */
void SPEED_SENSOR_CalculateSpeed(SPEED_SENSOR_Channel_t Channel, uint32_t TimeElapsed_ms);

/**================================================================
 * @Fn: SPEED_SENSOR_GetSpeedRPM
 * @brief: Get the current speed in RPM for a specific channel
 */
float SPEED_SENSOR_GetSpeedRPM(SPEED_SENSOR_Channel_t Channel);

/**================================================================
 * @Fn: SPEED_SENSOR_GetSpeedKMH
 * @brief: Get the current speed in km/h for a specific channel
 */
float SPEED_SENSOR_GetSpeedKMH(SPEED_SENSOR_Channel_t Channel);

#endif 
