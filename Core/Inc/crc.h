/*
 * crc.h
 *
 *  Created on: May 3, 2022
 *      Author: michael
 */

#ifndef INC_CRC_H_
#define INC_CRC_H_

#include <stdint.h>

uint16_t usMBCRC16(uint8_t *pucFrame, uint16_t usLen);

#endif /* INC_CRC_H_ */
