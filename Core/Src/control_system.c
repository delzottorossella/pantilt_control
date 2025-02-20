/*
 * control_system.c
 *
 * PAN-TILT SYSTEM
 *
 * Contains:
 *    	Function to read pan and tilt angles, based on IMU data
 * 		Inverse kinematics function of the pan-tilt system
 * 		Control functions for the two motors
 *		Functions to send data via SWD
 *
 *
 *  Created on: Nov 27, 2024
 *      Author: rosso
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "mpu9265.h"
#include "mpu9265_fun.h"
#include "control_system.h"
#include "motors.h"

/* Compute Pan and Tilt angles in degrees starting from IMU data */
void ReadPanTilt(float *PanAngle, float *TiltAngle, float *PanVel, float *TiltVel){
	float dt = 0.005; // dt TIM6 (f = 200 Hz)
	/* Read IMU Data */
	float accelData[3];
	float gyroData[3];
	static float tilt_prev = 0.0f;
	static float pan_prev = 0.0f;

	MPU9265_ReadAccelGyro(accelData, gyroData);

	/* Compute Pan Angle (Theta1) == Pitch IMU (deg) */
	*PanAngle = pan_prev + gyroData[1] * dt;
	pan_prev = *PanAngle;
	/* Compute Tilt Angle (Theta2) == Roll IMU (deg) */
    float tilt_acc = -(atan2(accelData[1], sqrtf(accelData[0] * accelData[0] + accelData[2] * accelData[2])) * 180.0f / M_PI - OFFSET_THETA2);
    float tilt_gyro = tilt_prev - gyroData[0] * dt;
    *TiltAngle = ALPHA * tilt_gyro + (1.0f - ALPHA) * tilt_acc;  // complementary filter
    tilt_prev = *TiltAngle; // updates tilt_prev
	/* Read Pan Velocity (dps) */
	*PanVel = gyroData[1];
	/* Read Tilt Velocity (dps) */
	*TiltVel = gyroData[0];
}

/* Compute Pan and Tilt (deg) starting from a point (x,y,z) in space */
void InverseKinematics(float targetPoint[3], float *PanAngle, float *TiltAngle){
	float l1 = 106.0f; // link 1 (mm)
	float l2 = 15.9f; // link 2 (mm)

	float x = targetPoint[0];
	float y = targetPoint[1];
	float z = targetPoint[2];
	/* Non reachable points:
	 * z < l1
	 * ************************/

	if (z < l1){
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // toggle led2
	} else {
		if (x<0 && y<0){
			*PanAngle = atan(y/x) * 180.0f / M_PI;
			*TiltAngle = 180 - atan((z - l1) / (sqrt(pow(x,2) + pow(y,2)))) * 180.f / M_PI;
		} else if (x<0 && y>=0){
			*PanAngle = 180 + atan(y/x) * 180.0f / M_PI;
			*TiltAngle = atan((z - l1) /( sqrt(pow(x,2) + pow(y,2)))) * 180.f / M_PI;
		} else if (x>0 && y<0){
			*PanAngle = 180 + atan(y/x) * 180.0f / M_PI;
			*TiltAngle = 180.0 - atan((z - l1) / (sqrt(pow(x,2) + pow(y,2)))) * 180.f / M_PI;
		} else if (x>0 && y>=0){
			*PanAngle = atan(y/x) * 180.0f / M_PI;
			*TiltAngle = atan((z - l1) / (sqrt(pow(x,2) + pow(y,2)))) * 180.f / M_PI;
		} else if (x == 0 && y>0){
			*PanAngle = 90.0;
			*TiltAngle = atan((z - l1) / (sqrt(pow(x,2) + pow(y,2)))) * 180.f / M_PI;
		} else if (x == 0 && y<0){
			*PanAngle = 90.0;
			*TiltAngle = 180 - atan((z - l1) / (sqrt(pow(x,2) + pow(y,2)))) * 180.f / M_PI;
		} else if (x == 0 && y == 0){
			*PanAngle = 90.0f;
			*TiltAngle = 90.0f;
		}
	}
	if (*TiltAngle > 127){
		*TiltAngle = 127; // physical limit of Tilt Angle
	}
}

/* Initialize function for PID_Controller */
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float outputMin, float outputMax){
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->target = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->prevError = 0.0f;
    pid->dt = 0.005f; // dt of TIM6
    pid->outputMin = outputMin; //250 //200
    pid->outputMax = outputMax; //800 //900
}

void Motor1_Control(PID_Controller *position_pid, PID_Controller *velocity_pid, float feedbackPosition, float feedbackVelocity, float targetPosition, ControlState *state, float *errorPosition){
    /* Position Controller (P) */
	*errorPosition = targetPosition - feedbackPosition;
    float targetVelocity = (position_pid->Kp * *errorPosition); // Output of position controller == target velocity
    /* Velocity Controller (PI) */
    float errorVelocity = targetVelocity - feedbackVelocity;
    velocity_pid->integral += errorVelocity * velocity_pid->dt; // Integral Term
    float output = (velocity_pid->Kp * errorVelocity) + (velocity_pid->Ki * velocity_pid->integral);
    // Limit output
    if (output > velocity_pid->outputMax) {
        output = velocity_pid->outputMax;
        // Anti-windup: limit integral term when output is clamped
        velocity_pid->integral = (output - (velocity_pid->Kp * errorVelocity)) / velocity_pid->Ki;
    } else if (output < velocity_pid->outputMin) {
        output = velocity_pid->outputMin;
        // Anti-windup: limit integral term when output is clamped
        velocity_pid->integral = (output - (velocity_pid->Kp * errorVelocity)) / velocity_pid->Ki;
    }
    /* Actuate Motor 1 */
	if (*errorPosition > TOLERANCE){
		TIM2 -> CCR1 = output;
		HAL_GPIO_WritePin(Mot1_Dir1_GPIO_Port, Mot1_Dir1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Mot1_Dir2_GPIO_Port, Mot1_Dir2_Pin, GPIO_PIN_SET);
		*state = PAN_CONTROL;
	} else if (*errorPosition < - TOLERANCE) { // turn motor in the opposite verse
		TIM2 -> CCR1 = output;
		HAL_GPIO_WritePin(Mot1_Dir1_GPIO_Port, Mot1_Dir1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Mot1_Dir2_GPIO_Port, Mot1_Dir2_Pin, GPIO_PIN_RESET);
		*state = PAN_CONTROL;
	} else if (fabsf(*errorPosition) <= TOLERANCE) {
		TIM2 -> CCR1 = 0; //stop motor
		*state = TILT_CONTROL;
	}
}

void Motor2_Control(PID_Controller *position_pid, PID_Controller *velocity_pid, float feedbackPosition, float feedbackVelocity, float targetPosition, ControlState *state, float *errorPosition){
    /* Position Controller (P) */
	*errorPosition = targetPosition - feedbackPosition;
    float targetVelocity = (position_pid->Kp * *errorPosition);  // Output of position controller == target velocity
    /* Velocity Controller (PI) */
    float errorVelocity = targetVelocity - feedbackVelocity;
    velocity_pid->integral += errorVelocity * velocity_pid->dt; // Integral Term
    float output = (velocity_pid->Kp * errorVelocity) + (velocity_pid->Ki * velocity_pid->integral);
    // Limit output
    if (output > velocity_pid->outputMax) {
        output = velocity_pid->outputMax;
    } else if (output < velocity_pid->outputMin) {
        output = velocity_pid->outputMin;
    }
    /* Actuate Motor 2 */
	if (*errorPosition > TOLERANCE){
		TIM2 -> CCR2 = output;
		HAL_GPIO_WritePin(Mot2_Dir1_GPIO_Port, Mot2_Dir1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Mot2_Dir2_GPIO_Port, Mot2_Dir2_Pin, GPIO_PIN_RESET);
		*state = TILT_CONTROL;
	} else if (*errorPosition < - TOLERANCE){ // turn motor in the opposite verse
		TIM2 -> CCR2 = output;
		HAL_GPIO_WritePin(Mot2_Dir1_GPIO_Port, Mot2_Dir1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Mot2_Dir2_GPIO_Port, Mot2_Dir2_Pin, GPIO_PIN_SET);
		*state = TILT_CONTROL;
	} else if (fabsf(*errorPosition) <= TOLERANCE) {
		TIM2 -> CCR2 = 0; //stop motor
		*state = SEQUENCE_DONE;
	}
}

void configure_swo(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // enable tracking
    DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN;              // enable clock of SWO
    ITM->LAR = 0xC5ACCE55;                           // unlock ITM
    ITM->TCR = 0x0001000D;                           // enable ITM
    ITM->TER |= 1UL;                                 // enable channel 0
}

void send_data_to_swv(float PanAngle, float TiltAngle) {
    char buffer[50];
    snprintf(buffer, sizeof(buffer), "Pan:, %.2f, Tilt:, %.2f\n", PanAngle, TiltAngle);
    for (char *ptr = buffer; *ptr != '\0'; ptr++) {
        ITM_SendChar(*ptr);
    }
}


