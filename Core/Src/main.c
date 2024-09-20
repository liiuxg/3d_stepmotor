/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stepmotor_def.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2){
		if(circle_info.circle_lock){
			if((circle_info.circleisrunning) && (++circle_info.curpuls < circle_info.totalpuls)){
				circle_handler_method2();
			} else {
				circle_info.circleisrunning = 0;
				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
			}
			return;
		}

#if 1	//section for normal part
		if((xstep_motor_info.muststop == 0) && (xstep_motor_info.isrunning)){
			if(++xstep_motor_info.cur_pulse >= xstep_motor_info.req_pulse){
				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
				xstep_motor_info.isrunning = 0;
				return;
			}
#if		STARTDRAW
			else {
				/** do nothing **/
				return;
			}
#endif
			switch(xstep_motor_info.dtype){
				case LINE:
					xstep_move_handler(&xstep_motor_info, &xmotor_cnt_info, xstep_motor_info.cur_pulse);
					break;
				case CIRCLE:
					xstep_circle_handler(xstep_motor_info.cur_pulse);
					break;
				default:
					break;
			}
		} else {
			xstep_motor_info.isrunning = 0;
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
			return;
		}
#endif // step move

	} else if(htim->Instance == TIM3){
		if(circle_info.circle_lock){
			if((circle_info.circleisrunning) && (++circle_info.curpuls < circle_info.totalpuls)){
				circle_handler_method2();
			} else {
				circle_info.circleisrunning = 0;
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			}
			return;
		}

#if 1	// section for normal part
		if((ystep_motor_info.muststop == 0) && (ystep_motor_info.isrunning)){
			if(++ystep_motor_info.cur_pulse >= ystep_motor_info.req_pulse){
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
				ystep_motor_info.isrunning = 0;
				return;
			}
#if		STARTDRAW
			else {
				/** do nothing **/
				return;
			}
#endif
			switch(xstep_motor_info.dtype){
				case LINE:
					ystep_move_handler(&ystep_motor_info, &ymotor_cnt_info, ystep_motor_info.cur_pulse);
					break;
				case CIRCLE:
					ystep_circle_handler(ystep_motor_info.cur_pulse);
					break;
				default:
					break;
			}
		} else {
			ystep_motor_info.isrunning = 0;
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			return;
		}
#endif // step move

	} else {
		/* do nothing */
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == XL_Switch_Sensor_Pin){
		if(xstep_motor_info.direction == XLeft_To_LX1){
		  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
		  xstep_motor_info.l_signal = Switch_Limit_On;
		  xstep_motor_info.muststop = 1;
		}
	} else if(GPIO_Pin == XR_Switch_Sensor_Pin){
		if(xstep_motor_info.direction == XRight_To_LX2){
		  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
		  xstep_motor_info.r_signal = Switch_Limit_On;
		  xstep_motor_info.muststop = 1;
		}
	} else if(GPIO_Pin == YSwitch_Bottom_Pin){
		if(ystep_motor_info.direction == YLeft_To_LX1_Back){
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			ystep_motor_info.r_signal = Switch_Limit_On;
			ystep_motor_info.muststop = 1;
		}
	} else if(GPIO_Pin == YSwitch_Front_Pin){
		if(ystep_motor_info.direction == YRight_To_LX2_Front){
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			ystep_motor_info.l_signal = Switch_Limit_On;
			ystep_motor_info.muststop = 1;
		}
	} else {
		/** do nothing **/
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	  static draw_move_param movdraw[] = { [0] = {0.0, XLeft_To_LX1, 2.0, YRight_To_LX2_Front, Z_to_Down, LINE},
										   [1] = {0.0, XLeft_To_LX1, 1.0, YLeft_To_LX1_Back, Z_to_Up, LINE},
										   [2] = {1.0, XRight_To_LX2, 0.0, YLeft_To_LX1_Back, Z_to_Down, LINE},
										   [3] = {0.0, XLeft_To_LX1, 1.0, YRight_To_LX2_Front, Z_to_Up, LINE},
										   [4] = {0.0, XLeft_To_LX1, 2.0, YLeft_To_LX1_Back, Z_to_Down, LINE},  		// H character

										   [5] = {1.0, XRight_To_LX2, 1.0, YRight_To_LX2_Front, Z_to_Up, LINE}, 		// connection between H and e
										   [6] = {1.0, XRight_To_LX2, 0.0, YRight_To_LX2_Front, Z_to_Down, LINE},
										   [7] = {0.5, XLeft_To_LX1, 0.5, YRight_To_LX2_Front, Z_to_Down, CIRCLE},
										   [8] = {0.5, XLeft_To_LX1, 0.5, YLeft_To_LX1_Back, Z_to_Down, CIRCLE},
										   [9] = {0.5, XRight_To_LX2, 0.5, YLeft_To_LX1_Back, Z_to_Down, CIRCLE},		// e character

										   [10] = {1.5, XRight_To_LX2, 1.5, YRight_To_LX2_Front, Z_to_Up, LINE},		// connection between e and l
										   [11] = {0.0, XLeft_To_LX1, 1.5, YLeft_To_LX1_Back, Z_to_Down, LINE},
										   [12] = {0.5, XRight_To_LX2, 0.5, YLeft_To_LX1_Back, Z_to_Down, CIRCLE},
										   [13] = {0.5, XRight_To_LX2, 0.5, YRight_To_LX2_Front, Z_to_Down, CIRCLE}, 	// character l

										   [14] = {1.0, XRight_To_LX2, 1.5, YRight_To_LX2_Front, Z_to_Up, LINE},		// connection between l and l
										   [15] = {0.0, XLeft_To_LX1, 1.5, YLeft_To_LX1_Back, Z_to_Down, LINE},
										   [16] = {0.5, XRight_To_LX2, 0.5, YLeft_To_LX1_Back, Z_to_Down, CIRCLE},
										   [17] = {0.5, XRight_To_LX2, 0.5, YRight_To_LX2_Front, Z_to_Down, CIRCLE}, 	// character l

										   [18] = {1.5, XRight_To_LX2, 0.5, YRight_To_LX2_Front, Z_to_Up, LINE},		// connection between l and o
										   [19] = {0.5, XLeft_To_LX1, 0.5, YRight_To_LX2_Front, Z_to_Down, CIRCLE},
										   [20] = {0.5, XLeft_To_LX1, 0.5, YLeft_To_LX1_Back, Z_to_Down, CIRCLE},
										   [21] = {0.5, XRight_To_LX2, 0.5, YLeft_To_LX1_Back, Z_to_Down, CIRCLE},
										   [22] = {0.5, XRight_To_LX2, 0.5, YRight_To_LX2_Front, Z_to_Down, CIRCLE},	// character o

	  	  	  	  	  	  	  	  	  	   [23] = {8.0, XLeft_To_LX1, 6.0, YLeft_To_LX1_Back, Z_to_Up, LINE},			// connection between o and 2
										   [24] = {1.0, XLeft_To_LX1, 0.0, YRight_To_LX2_Front, Z_to_Down, LINE},
										   [25] = {1.0, XRight_To_LX2, 2.0, YRight_To_LX2_Front, Z_to_Down, LINE},
										   [26] = {0.5, XLeft_To_LX1, 0.5, YRight_To_LX2_Front, Z_to_Down, CIRCLE},
										   [27] = {0.5, XLeft_To_LX1, 0.5, YLeft_To_LX1_Back, Z_to_Down, CIRCLE},		// num 2

										   [28] = {3.0, XRight_To_LX2, 0.0, YLeft_To_LX1_Back, Z_to_Up, LINE},		 	// connection between 2 to 9
										   [29] = {0.5, XLeft_To_LX1, 0.5, YRight_To_LX2_Front, Z_to_Down, CIRCLE},
										   [30] = {0.5, XLeft_To_LX1, 0.5, YLeft_To_LX1_Back, Z_to_Down, CIRCLE},
										   [31] = {0.5, XRight_To_LX2, 0.5, YLeft_To_LX1_Back, Z_to_Down, CIRCLE},
										   [32] = {0.5, XRight_To_LX2, 0.5, YRight_To_LX2_Front, Z_to_Down, CIRCLE},
										   [33] = {0.0, XRight_To_LX2, 2.0, YLeft_To_LX1_Back, Z_to_Down, LINE},		// num 9

	  	  	  	  	  	  	  	  	  	   [34] = {2.0, XRight_To_LX2, 0.0, YLeft_To_LX1_Back, Z_to_Up, LINE},			// connection between o and 2
										   [35] = {1.0, XLeft_To_LX1, 0.0, YRight_To_LX2_Front, Z_to_Down, LINE},
										   [36] = {1.0, XRight_To_LX2, 2.0, YRight_To_LX2_Front, Z_to_Down, LINE},
										   [37] = {0.5, XLeft_To_LX1, 0.5, YRight_To_LX2_Front, Z_to_Down, CIRCLE},
										   [38] = {0.5, XLeft_To_LX1, 0.5, YLeft_To_LX1_Back, Z_to_Down, CIRCLE},		// num 2

										   [39] = {4.0, XRight_To_LX2, 1.0, YLeft_To_LX1_Back, Z_to_Up, LINE},			// connection between 2 and o
										   [40] = {1.0, XLeft_To_LX1, 1.0, YRight_To_LX2_Front, Z_to_Down, CIRCLE},
										   [41] = {1.0, XLeft_To_LX1, 1.0, YLeft_To_LX1_Back, Z_to_Down, CIRCLE},
										   [42] = {1.0, XRight_To_LX2, 1.0, YLeft_To_LX1_Back, Z_to_Down, CIRCLE},
										   [43] = {1.0, XRight_To_LX2, 1.0, YRight_To_LX2_Front, Z_to_Down, CIRCLE},	// character o

	  	  	  	  	  	  	  	  	  	   [44] = {1.0, XRight_To_LX2, 1.0, YRight_To_LX2_Front, Z_to_Up, LINE},		// connection between o and f
										   [45] = {1.0, XRight_To_LX2, 0.0, YLeft_To_LX1_Back, Z_to_Down, LINE},
										   [46] = {1.0, XLeft_To_LX1, 0.0, YLeft_To_LX1_Back, Z_to_Up, LINE},
										   [47] = {0.0, XLeft_To_LX1, 1.0, YLeft_To_LX1_Back, Z_to_Down, LINE},
										   [48] = {1.0, XRight_To_LX2, 0.0, YLeft_To_LX1_Back, Z_to_Down, LINE},
										   [49] = {1.0, XLeft_To_LX1, 0.0, YLeft_To_LX1_Back, Z_to_Up, LINE},
										   [50] = {0.0, XLeft_To_LX1, 1.0, YLeft_To_LX1_Back, Z_to_Down, LINE},

	  };

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  uint8_t buttom_sig = 0;
  while(!buttom_sig){
	buttom_sig = HAL_GPIO_ReadPin(User_Bottom_GPIO_Port, User_Bottom_Pin);
	HAL_Delay(20);
	if(buttom_sig){
		buttom_sig = 0;
		break;
	}
  }

#if		CALIBRATION
  calibrate_xymotor();
#endif

  calibrate_pen();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    while(!buttom_sig){
	  buttom_sig = HAL_GPIO_ReadPin(User_Bottom_GPIO_Port, User_Bottom_Pin);
	  HAL_Delay(30);
	  if(buttom_sig){
		buttom_sig = 0;
		break;
	  }
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if 1
    int sz = sizeof(movdraw) / sizeof(draw_move_param);
    for(int i = 0; i < sz; i++){
    	draw_move_param cur = movdraw[i];
    	switch(cur.dtype){
    		case LINE:
//    			line_move(cur.xlen, cur.xdir, cur.ylen, cur.ydir, cur.pendir);
    			line_move_method2(cur.xlen, cur.xdir, cur.ylen, cur.ydir, cur.pendir);

    			while((xstep_motor_info.isrunning) || (ystep_motor_info.isrunning)){
    				HAL_Delay(20);
    			}
    			break;

    		case CIRCLE:
    			circle_move_method2(cur.xlen, cur.xdir, cur.ylen, cur.ydir, cur.pendir);

    			while(circle_info.circleisrunning){
    				HAL_Delay(20);
    			}
    			circle_info.circle_lock = 0;
    			break;

    		default:
    			/* do nothing */
    		break;
    	}
    }
#endif

#if 0	// draw a circle
	circle_move_method2(3.0, XLeft_To_LX1, 3.0, YRight_To_LX2_Front, Z_to_Down);
	while(circle_info.circleisrunning){
		HAL_Delay(50);
	}
	circle_info.circle_lock = 0;

	circle_move_method2(3.0, XLeft_To_LX1, 3.0, YLeft_To_LX1_Back, Z_to_Down);
	while(circle_info.circleisrunning){
		HAL_Delay(50);
	}
	circle_info.circle_lock = 0;

	circle_move_method2(3.0, XRight_To_LX2, 3.0, YLeft_To_LX1_Back, Z_to_Down);
	while(circle_info.circleisrunning){
		HAL_Delay(50);
	}
	circle_info.circle_lock = 0;

	circle_move_method2(3.0, XRight_To_LX2, 3.0, YRight_To_LX2_Front, Z_to_Down);
	while(circle_info.circleisrunning){
		HAL_Delay(50);
	}
	circle_info.circle_lock = 0;
#endif

	line_move(0.0, XRight_To_LX2, 0.0, YRight_To_LX2_Front, Z_to_Up);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
