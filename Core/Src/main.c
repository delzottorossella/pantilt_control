/* USER CODE BEGIN Header */
/**
  ********************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ********************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ********************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "mpu9265.h"
#include "mpu9265_fun.h"
#include "motors.h"
#include "control_system.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
PID_Controller PositionController_Motor1;
PID_Controller PositionController_Motor2;
PID_Controller VelocityController_Motor1;
PID_Controller VelocityController_Motor2;

ControlState state = PAN_CONTROL; // starting state

float targetPoint[3] = {-35.0f, -20.0f, 160.0f}; 	// xyz target point (in mm)
float panTarget, tiltTarget; 						// computed value of target angles (in deg)
float panAngle, tiltAngle; 							// current value of angles (in deg)
float panVelocity, tiltVelocity; 					// current value of velocity (in dps)

float errorPosition_Mot1, errorPosition_Mot2; 		// position error (in mm)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  /* Initialize IMU */
  MPU9265_Init();
  /* Initialize motors */
  Motor1_Init(360); // ccr = 360 -> duty_cycle = 0.08 and rotate in dir1
  Motor2_Init(540); // ccr = 540 -> duty_cycle = 0.12 and rotate in dir2 (cam backwards)
  /* Compute Inverse Kinematics */
  InverseKinematics(targetPoint, &panTarget, &tiltTarget);
  /* Initialize controllers */
  PID_Init(&PositionController_Motor1, 18.0, 0.0, 0.0, -INFINITY, INFINITY); // no limit for output
  PID_Init(&VelocityController_Motor1, 5.0, 4.0, 0.0, 0, 675);
  PID_Init(&PositionController_Motor2, 15.0, 0.0, 0.0, -INFINITY, INFINITY); // no limit for output
  PID_Init(&VelocityController_Motor2, 7.5, 3.5, 0.0, 0, 540);
  /* Configure SWO to transfer position data */
  configure_swo();
  /* Starting TIM6 Interrupt */
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
  {
    	/* Check IMU Sensor */
    	//MPU9265_CheckStatus(&id, &status);

    	/* Gyroscope Calibration */
    	//MPU9265_CalibrateGyro(&gyroOffset);
    	//break;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* Control Action each dt_TIM6 = 5 ms */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6){
		ReadPanTilt(&panAngle, &tiltAngle, &panVelocity, &tiltVelocity);
		send_data_to_swv(panAngle, tiltAngle);

		__disable_irq();
		      switch (state){
		      case PAN_CONTROL:
		        Motor1_Control(&PositionController_Motor1, &VelocityController_Motor1, panAngle, panVelocity, panTarget, &state, &errorPosition_Mot1);
		        break;
		      case TILT_CONTROL:
		        Motor2_Control(&PositionController_Motor2, &VelocityController_Motor2, tiltAngle, tiltVelocity, tiltTarget, &state, &errorPosition_Mot2);
		        break;
		      case SEQUENCE_DONE:
		        break;
		      }
		 __enable_irq();

	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
