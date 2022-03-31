/*
 * nec_encode.h
 *
 *  Created on: 30 Mar 2022
 *      Author: michael
 */

#include "stm32f0xx_hal.h"
#include "timer_helpers.h"

#ifndef SRC_NEC_ENCODE_H_
#define SRC_NEC_ENCODE_H_

#define NEC_FRAME_LENGTH 69

void NEC_Send(TIM_HandleTypeDef *timer, char address, char command);

void addBit(uint16_t *timings, char bit);

#endif /* SRC_NEC_ENCODE_H_ */
