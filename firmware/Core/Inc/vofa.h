/*
 * vofa.h
 *
 *  Created on: 2026年11月1日
 *      Author: 15332
 */

#ifndef INC_VOFA_H_
#define INC_VOFA_H_

#include "main.h"

uint8_t VOFA_Send8(const float *data);

extern volatile uint32_t vofa_drop_count;

#endif /* INC_VOFA_H_ */
