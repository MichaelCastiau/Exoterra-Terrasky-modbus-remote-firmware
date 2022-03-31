/*
 * timer_helpers.h
 *
 *  Created on: Mar 31, 2022
 *      Author: michael
 */

#include "stm32f0xx_hal.h"

#ifndef INC_TIMER_HELPERS_H_
#define INC_TIMER_HELPERS_H_

#define TIM_FORCED_ACTIVE      ((uint16_t)0x0050)
#define TIM_FORCED_INACTIVE    ((uint16_t)0x0040)
#define TIM_TOGGLE    		   ((uint16_t)0x0030)

void TIM_ForcedOC1Config(TIM_HandleTypeDef *timer, uint32_t action);

#endif /* INC_TIMER_HELPERS_H_ */
