/*
 * motors.c
 *
 * Contains functions to initialize motors and set direction and speed
 *
 *  Created on: Nov 11, 2024
 *      Author: rosso
 */
#include "tim.h"
#include "gpio.h"
#include "motors.h"

void Motor1_Init(float ccr_value){
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);

    TIM2 -> CCR1 = ccr_value;

	HAL_GPIO_WritePin(Mot1_Dir1_GPIO_Port, Mot1_Dir1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Mot1_Dir2_GPIO_Port, Mot1_Dir2_Pin, GPIO_PIN_RESET);
	HAL_Delay(2500);

	TIM2 -> CCR1 = 0; // stop motor
	HAL_Delay(1000);

}
void Motor2_Init(float ccr_value){
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);

	TIM2 -> CCR2 = ccr_value;

	HAL_GPIO_WritePin(Mot2_Dir1_GPIO_Port, Mot2_Dir1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Mot2_Dir2_GPIO_Port, Mot2_Dir2_Pin, GPIO_PIN_SET);
	HAL_Delay(2500);

	TIM2 -> CCR2 = 0; // stop motor
	HAL_Delay(1000);
}



