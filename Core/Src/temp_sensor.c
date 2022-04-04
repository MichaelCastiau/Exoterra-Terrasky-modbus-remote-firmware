/*
 * temp_sensor.c
 *
 *  Created on: Apr 4, 2022
 *      Author: michael
 */

#include "temp_sensor.h"

void temp_sensor_initialize(TempSensor *sensor) {
	//First read out configuration
	uint8_t initalizationData[3] = {0x02, 0x90, 0x00};
	HAL_I2C_Master_Transmit(sensor->i2c, HDC1080_DEVICE_ADDRESS,
			(uint8_t*) initalizationData, 3, 250);
}

void temp_sensor_measure(TempSensor *sensor) {
	if (!HAL_I2C_IsDeviceReady(sensor->i2c, HDC1080_DEVICE_ADDRESS, 3, 250)) {
		return;
	}

	// Write to register 0x00 to indicate we want to make a measurement
	uint8_t registerPointer[1] = { 0x00 };
	if (HAL_I2C_Master_Transmit_DMA(sensor->i2c, HDC1080_DEVICE_ADDRESS,
			(uint8_t*) registerPointer, 1) != HAL_OK) {
		return;
	}

	// Read out temperature and humidity
	uint8_t dataBuffer[4] = { 0x00, 0x00, 0, 0 };
	if (HAL_I2C_Master_Receive_DMA(sensor->i2c, HDC1080_DEVICE_ADDRESS,
			(uint8_t*) dataBuffer, 4) != HAL_OK) {
		return;
	}

	const uint16_t rawTemp = (dataBuffer[0] << 8) | dataBuffer[1];
	const uint16_t rawHumidity = (dataBuffer[2] << 8) | dataBuffer[3];

	TempParameters params = { temperatureInCelsius: (double) (rawTemp) / (65536)
			* 165 - 40, humidityPercentage : (double) (rawHumidity / pow(2, 16))
			* 100 };

	sensor->callback(params);
}
