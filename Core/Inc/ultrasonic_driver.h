/*
 * stm32_ultrasonic_driver.h
 *
 *  Created on: SomeDay
 *      Author: AMiR & Co.
 */

#ifndef INC_ULTRASONIC_DRIVER_H_
#define INC_ULTRASONIC_DRIVER_H_

//-----------------------------
// Includes
#include "stm32f103x6.h"
#include "stm32_F103C6_gpio_driver.h"
#include "hw_config.h"  // Include hardware configuration
//-----------------------------

// HC-SR04 Configuration Macros
#define ULTRASONIC_TIMEOUT        35000   // Timeout in microseconds (for ~6m range)
#define ULTRASONIC_MAX_DISTANCE   400     // Maximum distance in cm
#define ULTRASONIC_INVALID_DISTANCE 0xFFFF // Invalid distance value

// Ultrasonic Directions/Positions
#define ULTRASONIC_FRONT          0
#define ULTRASONIC_REAR           1
#define ULTRASONIC_LEFT           2
#define ULTRASONIC_RIGHT          3
#define ULTRASONIC_COUNT          4       // Total number of ultrasonic sensors

// Ultrasonic Status
#define ULTRASONIC_OK             0
#define ULTRASONIC_ERROR          1
#define ULTRASONIC_TIMEOUT_ERROR  2

//-----------------------------
// User type definitions (structures)
typedef struct
{
    GPIO_TypeDef* TRIG_Port;    // GPIO Port for Trigger pin (fixed from GPIO_TypedDef*)
    uint16_t TRIG_Pin;          // GPIO Pin for Trigger pin
    GPIO_TypeDef* ECHO_Port;    // GPIO Port for Echo pin (fixed from GPIO_TypedDef*)
    uint16_t ECHO_Pin;          // GPIO Pin for Echo pin
    uint16_t ID;                // Sensor ID (position)
    uint16_t LastDistance;      // Last measured distance in cm
} ULTRASONIC_Config_t;



/**
 * @discreption: Initialize all ultrasonic sensors
 *
 */
void HAL_ULTRASONIC_Init(void);

/**
 * @discreption: Initialize a specific ultrasonic sensor
 * =>It returns the Status of ULTRASONIC_OK or ULTRASONIC_ERROR
 */
uint8_t HAL_ULTRASONIC_Init_Sensor(ULTRASONIC_Config_t* Sensor);

uint8_t HAL_ULTRASONIC_Trigger(ULTRASONIC_Config_t* Sensor);

/**
 * @discreption Read the distance from a sensor (blocking mode)
 * => stores the distance in the Distance pointer
 */
uint8_t HAL_ULTRASONIC_Read(ULTRASONIC_Config_t* Sensor, uint16_t* Distance);

/**
 * @discreption Measure the distance from a sensor (blocking mode)
 * * => stores Distance in Pointer to store measured distance in cm
  */
uint8_t HAL_ULTRASONIC_MeasureDistance(ULTRASONIC_Config_t* Sensor, uint16_t* Distance);

/**
 * @discreption Measure the distance from a sensor by ID (blocking mode)
 * => measure Distance in Pointer to store measured distance in cm -> for every ultrasonic referred to By It's ID
 */
uint8_t HAL_ULTRASONIC_MeasureDistanceByID(uint8_t SensorID, uint16_t* Distance);

/**
 * @discreption Measure distances from all sensors (blocking mode)
*/
uint8_t HAL_ULTRASONIC_MeasureAllDistances(uint16_t Distances[ULTRASONIC_COUNT]);

/**
 * @discreption Get the last measured distance from a sensor
 */
uint16_t HAL_ULTRASONIC_GetLastDistance(uint8_t SensorID);

#endif
