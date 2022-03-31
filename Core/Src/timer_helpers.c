/*
 * timer_helpers.c
 *
 *  Created on: Mar 31, 2022
 *      Author: michael
 */

#include "timer_helpers.h"

void TIM_ForcedOC1Config(TIM_HandleTypeDef *timer, uint32_t action) {
	uint32_t temporary = timer->Instance->CCMR1;

	temporary &= ~TIM_CCMR1_OC1M;
	temporary |= action;
	timer->Instance->CCMR1 = temporary;
}
