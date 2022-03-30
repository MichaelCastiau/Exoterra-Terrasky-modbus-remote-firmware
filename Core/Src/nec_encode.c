/*
 * nec_encode.c
 *
 *  Created on: 30 Mar 2022
 *      Author: michael
 */
#include "nec_encode.h"

static int index;
static const double timerResolution = 0.002;

void NEC_Send(TIM_HandleTypeDef *timer, char address, char command) {
	uint16_t timings[67];

	// Timer runs at 1MHz

	timings[0] = 10 + (9 / timerResolution); //9 ms burst
	timings[1] = timings[0] + ((uint16_t) (((double) 4.5) / timerResolution)); // 4.5ms pauze

	const char addressComplement = ~(address);
	const char commandComplement = ~(command);

	index = 1;

	//encode address
	for (int i = 7; i >= 0; --i) {
		addBit(&timings[0], (address >> i) & 0x1);
	}

	//encode address complement
	for (int i = 7; i >= 0; --i) {
		addBit(&timings[0], (addressComplement >> i) & 0x1);
	}

	//encode command
	for (int i = 7; i >= 0; --i) {
		addBit(&timings[0], (command >> i) & 0x1);
	}

	//encode command complement
	for (int i = 7; i >= 0; --i) {
		addBit(&timings[0], (commandComplement >> i) & 0x1);
	}

	timings[++index] = timings[index] + (((double) 0.5625) / timerResolution); //562.5 us final burst

	HAL_TIM_OC_Start_DMA(timer, TIM_CHANNEL_1, (uint16_t*) timings, sizeof(timings));
}

void addBit(uint16_t *timings, char bit) {
	if (bit == 0x1) {
		//bit 1
		timings[++index] = timings[index]
				+ (((double) 0.5625) / timerResolution); //562.5 us burst
		timings[++index] = timings[index]
				+ (((double) 1.6875) / timerResolution); //1.6875 ms pauze
	} else {
		//bit 0
		timings[++index] = timings[index]
				+ (((double) 0.5625) / timerResolution); //562.5 us burst
		timings[++index] = timings[index]
				+ (((double) 0.5625) / timerResolution); //562.5 us pauze
	}
}
