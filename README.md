# Independent Joint Control for a Pan-Tilt System


## Overview
This project implements an independent joint control for a pan-tilt system using an STM32-based development board (Nucleo F446RE). 
The system uses an IMU (MPU9250) mounted on the end effector to measure joint angles and velocities. 
The goal is to define a target point in 3D space (XYZ) and adjust the pan and tilt joints sequentially to align the camera (end effector) with the target.


## Features
- Independent Joint Control: controllers are tuned to achieve stable and accurate responses.

- IMU-based Feedback: Joint angles and velocities are computed based on MPU9250 sensor data.

- Target Tracking: The system computes the required angles with inverse kinematics to align the end effector with a specified target point in XYZ space.

- Sequential Movement Strategy: The pan (joint 1) moves first, followed by the tilt (joint 2).


## Key Components

- Pan-Tilt Mechanism: Two actuated joints (pan and tilt) controlling the end effector.

- MPU9250 IMU Sensor: Provides real-time measurements of orientation and velocity.

- Microcontroller/Embedded System: Nucleo Board F446RE, it processes IMU data and computes joint movements.

- Control Algorithm: Determines the necessary joint movements to align the camera with the target. 


## How It Works

- Target Definition: A point in 3D space (X, Y, Z) is specified.

- Angle Calculation: The required pan and tilt angles are computed based on the target location using inverse kinematics.

- Pan Adjustment: The pan joint moves first to minimize lateral displacement.

- Tilt Adjustment: The tilt joint moves next to align the camera precisely.

- IMU Feedback: The system continuously monitors joint angles to ensure accurate positioning.


## Project Files

- Codes: STM32CubeIDE project files for the control system.
- Report: report.pdf containing detailed project documentation and schematics.


## Authors

Rossella Delzotto
Angela Carnevale