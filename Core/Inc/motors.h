/*
 * motors.h
 *
 *  Created on: Nov 11, 2024
 *      Author: rosso
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include "tim.h"
#include "gpio.h"

void Motor1_Init(float ccr_value);
void Motor2_Init(float ccr_value);

#endif /* INC_MOTORS_H_ */
