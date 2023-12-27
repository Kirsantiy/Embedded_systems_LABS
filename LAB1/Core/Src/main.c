/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "disp1color.h"
#include "font.h"
#include <locale.h>
#include "median.h"
#include "FlappyBird.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define f6x8_MONO_WIDTH         6
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int16_t X = 32;
uint16_t Period = 70;
uint16_t TickNum = 1; //150
extern uint8_t data1[16], data2[16], data3[16], data4[16];
uint8_t x0_bird = 4;
volatile uint8_t y0_bird;
uint8_t State;

volatile int8_t rectangle_x1;
volatile uint8_t rectangle_y1;
volatile int8_t rectangle_x2;
volatile uint8_t rectangle_y2;
int rectangle_y1_array[11] = {4, 8, 6, 0, 10, 2, 5, 7, 3, 1, 9};
int rectangle_y2_array[11] = {9, 13, 11, 5, 15, 7, 10, 12, 8, 6, 14};

median_filter_t median_filter_MIC;
FlappyBird_t FlappyBird;
uint16_t MIC_median_filter;
volatile int16_t adc_reg_value;
volatile int16_t volume;

uint8_t lose;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void disp_row(int row);
void draw_game(int8_t rectangle_x1,int8_t rectangle_x2, uint8_t rectangle_y1, uint8_t rectangle_y2);
void Flappy_Bird(int16_t x0, int16_t y0, uint8_t State);
void game_over(char *pMyStr);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	rectangle_x1 = 31;
	rectangle_x2 = 35;
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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);

	median_filter_init(&median_filter_MIC);
	FlappyBird_init(&FlappyBird);

	HAL_GPIO_WritePin(nOE_GPIO_Port, nOE_Pin, GPIO_PIN_RESET);
	char *pMyStr = "GAME OVER, Press Reset to continue";  // string for input
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		State = 1;
		if ((x0_bird == rectangle_x1 && y0_bird <= rectangle_y1)
				|| (x0_bird == rectangle_x1 && y0_bird >= rectangle_y2) || lose == 1) {
			game_over(pMyStr);
			lose = 1;
		} else {
			Flappy_Bird(x0_bird, y0_bird, State);
			draw_game(rectangle_x1, rectangle_x2, rectangle_y1, rectangle_y2);
	  	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void disp_row(int row){


	  if (row == 0){

		  for(uint8_t i=0; i<6; i++){
		  		HAL_SPI_Transmit(&hspi1, &data1, 16, 10);
		  }

		  HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_SET);

		  HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET);
	  }
	  if (row == 1){

		  for(uint8_t i=0; i<6; i++){
		  		HAL_SPI_Transmit(&hspi1, &data2, 16, 10);
		  }

		  HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_SET);

		  HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET);
	  }

	  if (row == 2){

		  for(uint8_t i=0; i<6; i++){
		  		HAL_SPI_Transmit(&hspi1, &data3, 16, 10);
		  }

		  HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_SET);

		  HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
	  }

	  if (row == 3){

		  for(uint8_t i=0; i<6; i++){
		  		HAL_SPI_Transmit(&hspi1, &data4, 16, 10);
		  }

		  HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(SCLK_GPIO_Port, SCLK_Pin, GPIO_PIN_SET);

		  HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
	  }


	  HAL_GPIO_WritePin(nOE_GPIO_Port, nOE_Pin, GPIO_PIN_SET);
		  for(uint32_t x=0; x<=500; x++) {};
	 HAL_GPIO_WritePin(nOE_GPIO_Port, nOE_Pin, GPIO_PIN_RESET);
  }

void draw_game(int8_t rectangle_x1, int8_t rectangle_x2, uint8_t rectangle_y1, uint8_t rectangle_y2) {
	disp1color_DrawLine(0, 0, 32, 0);
	disp1color_DrawLine(0, 15, 32, 15);
	disp1color_DrawRectangle(rectangle_x1, 0, rectangle_x2, rectangle_y1);
	disp1color_DrawRectangle(rectangle_x1, rectangle_y2, rectangle_x2, 15);

	disp1color_UpdateFromBuff(); // converting calculated data into an array
	prepare_data(); // splitting the array into arrays for each row

	// Switching-on the matrix, row by row
	for (uint8_t i = 0; i < 20; i++) {
		disp_row(0);
		disp_row(1);
		disp_row(2);
		disp_row(3);
	}
}

void Flappy_Bird(int16_t x0, int16_t y0, uint8_t State) {
	disp1color_DrawPixel(x0, y0, State);
	disp1color_DrawPixel((x0 + 1), y0, State);
	disp1color_DrawPixel((x0 + 2), y0, State);
	disp1color_DrawPixel(x0, (y0 - 1), State);
	disp1color_DrawPixel((x0 + 1), (y0 - 1), State);
}

void game_over(char *pMyStr) {
	uint16_t strSize = strlen(pMyStr);
	uint8_t symbolDelay = 0;
	disp1color_FillScreenbuff(0);
	for (int16_t x = 32; x > -((strSize + symbolDelay) * f6x8_MONO_WIDTH);
			x--) {

		disp1color_printf(x, 4, FONTID_6X8M, pMyStr); // calculation of data for output

		disp1color_UpdateFromBuff(); // converting calculated data into an array
		prepare_data();// splitting the array into arrays for each row

		// Switching-on the matrix, row by row
		for (uint8_t i = 0; i < 20; i++) {
			disp_row(0);
			disp_row(1);
			disp_row(2);
			disp_row(3);
		}
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
