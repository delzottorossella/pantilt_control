/*
 * control_system.h
 *
 *  Created on: Nov 27, 2024
 *      Author: rosso
 */

#ifndef INC_CONTROL_SYSTEM_H_
#define INC_CONTROL_SYSTEM_H_

#include "motors.h"

#define OFFSET_THETA2 87.5f //85.0f
#define ALPHA 0.95f
#define TOLERANCE 1.0f

/* Control State (to move motors in sequence) */
typedef enum {
    PAN_CONTROL,
    TILT_CONTROL,
    SEQUENCE_DONE
} ControlState;

/* Controller Structure */
typedef struct {
    float Kp;        		// Proportional Gain
    float Ki;        		// Integral Gain
    float Kd;        		// Derivative Gain
    float target; 			// Target Point
    float integral;  		// Integral counter
    float derivative;		// Derivative counter
    float prevError; 		// Previous Error (for derivative counter)
    float dt; 				// Ts - sampling time
    int outputMax;			// Upper limit of the output
    int outputMin;			// Lower limit of the output
} PID_Controller;

void ReadPanTilt(float *PanAngle, float *TiltAngle, float *PanVel, float *TiltVel);
void InverseKinematics(float targetPoint[3], float *PanAngle, float *TiltAngle);
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float outputMin, float outputMax);
void Motor1_Control(PID_Controller *position_pid, PID_Controller *velocity_pid, float feedbackPosition, float feedbackVelocity, float targetPosition, ControlState *state, float *errorPosition);
void Motor2_Control(PID_Controller *position_pid, PID_Controller *velocity_pid, float feedbackPosition, float feedbackVelocity, float targetPosition, ControlState *state, float *errorPosition);

void configure_swo(void);
void send_data_to_swv(float PanAngle, float TiltAngle);

#endif /* INC_CONTROL_SYSTEM_H_ */
