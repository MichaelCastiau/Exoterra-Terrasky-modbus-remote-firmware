/*
 * temp_sensor.c
 *
 *  Created on: Apr 4, 2022
 *      Author: michael
 */

#ifndef INC_TEMP_SENSOR_C_
#define INC_TEMP_SENSOR_C_

#include "stm32f0xx_hal.h"
#include "math.h"

#define HDC1080_TEMP_REGISTER_ADDRESS 0x00
#define HDC1080_HUMIDITY_REGISTER_ADDRESS 0x01
#define HDC1080_CONFIGURATION_REG 0x02
#define HDC1080_DEVICE_ADDRESS 0b1000000

typedef struct{
	double temperatureInCelsius;
	double humidityPercentage;
} TempParameters;

typedef struct {
	I2C_HandleTypeDef *i2c;
	void (*callback)(TempParameters);
} TempSensor;

void temp_sensor_initialize(TempSensor *sensor);
void temp_sensor_measure(TempSensor *sensor);
HAL_StatusTypeDef temp_senor_waitForReady(void);

#endif /* INC_TEMP_SENSOR_C_ */
