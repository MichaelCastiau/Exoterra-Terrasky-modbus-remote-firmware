/*
 * nec_encode.c
 *
 *  Created on: 30 Mar 2022
 *      Author: michael
 */
#include "nec_encode.h"

static volatile int index;

// Timer runs at 1MHz
// so the resolution is 0.002 ms
static const double timerResolution = 0.002;

void NEC_Send(TIM_HandleTypeDef *timer, char address, char command) {
	uint16_t timings[NEC_FRAME_LENGTH];

	timings[0] = 1000;	//Get the line high after 1000 ticks since last overflow
	timings[1] = timings[0] + (9 / timerResolution); //9 ms burst
	timings[2] = timings[1] + ((uint16_t) (((double) 4.5) / timerResolution)); // 4.5ms pauze

	const char addressComplement = ~(address);
	const char commandComplement = ~(command);

	index = 2;

	//encode address
	for (int i = 0; i < 8; ++i) {
		addBit(&timings[0], (address >> i) & 0x1);
	}

	//encode address complement
	for (int i = 0; i < 8; ++i) {
		addBit(&timings[0], (addressComplement >> i) & 0x1);
	}

	//encode command
	for (int i = 0; i < 8; ++i) {
		addBit(&timings[0], (command >> i) & 0x1);
	}

	//encode command complement
	for (int i = 0; i < 8; ++i) {
		addBit(&timings[0], (commandComplement >> i) & 0x1);
	}

	timings[++index] = timings[index] + (((double) 0.5625) / timerResolution); //562.5 us final burst
	timings[++index] = timings[index] + 500; // End

	/*
	 * Reset the timer counter
	 */
	__HAL_TIM_SET_COUNTER(timer, 0);

	/*
	 * The line will be pulled high as soon as we start a DMA request
	 * We will wait for it to overflow once (pulling the line low)
	 * For us to start sending the data.
	 *
	 * We don't need to 'explicitly' wait for the overflow event though
	 * since when the timer is running, it won't stumble against our first timing
	 * anyways.
	 */
	HAL_TIM_OC_Start_DMA(timer, TIM_CHANNEL_1, (uint16_t*) timings,
			sizeof(timings));

	// Enable necessary interrupts to stop the timer
	__HAL_TIM_ENABLE_IT(timer, TIM_IT_UPDATE);
	__HAL_TIM_ENABLE_IT(timer, TIM_IT_CC1);
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
