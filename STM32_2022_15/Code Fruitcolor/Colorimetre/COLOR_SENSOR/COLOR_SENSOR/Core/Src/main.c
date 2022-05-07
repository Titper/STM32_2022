/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
#include "lib_lcd.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

static rgb_lcd lcdData;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



#define COLOR_SENSOR_ADDR  0x39 << 1
#define CONTROL 0x00                    // Control of basic functions
#define TIMING 0x01                     // Integration time/gain control
#define INTERRUPT 0x02                  // Interrupt control
#define INT_SOURCE 0x03                 // Interrupt source
#define ID 0x04 << 1                    // Part number / Rev ID
#define GAIN 0x07                       // ADC gain control
#define LOW_THRESH_LOW_BYTE 0x08        // Low byte of low interrupt threshold
#define LOW_THRESH_HIGH_BYTE 0x09       // High byte of low interrupt threshold
#define HIGH_THRESH_LOW_BYTE 0x0A       // Low byte of high interrupt threshold
#define HIGH_THRESH_HIGH_BYTE 0x0B      // High byte of high interrupt threshold
#define DATA1LOW 0x10                   // Low byte of ADC green channel
#define DATA1HIGH 0x11                  // High byte of ADC green channel
#define DATA2LOW 0x12                   // Low byte of ADC red channel
#define DATA2HIGH 0x13                  // High byte of ADC red channel
#define DATA3LOW 0x14                   // Low byte of ADC blue channel
#define DATA3HIGH 0x15                  // High byte of ADC blue channel
#define DATA4LOW 0x16                   // Low byte of ADC clear channel
#define DATA4HIGH 0x17                  // High byte of ADC clear channel

#define REG_CTL 0x80
#define TIM_REG 0x81

#define CH1_LOWER 0xB0
#define CH2_LOWER 0xB2
#define CH3_LOWER 0xB4
#define CH4_LOWER 0xB8
#define DATA_T 0x01


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */

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

	uint8_t reg_ctl[8];
	uint8_t tim_reg[8];
	uint8_t intsource[8];
	uint8_t buf[16];
	uint8_t ch1_low[8];
	uint8_t ch2_low[8];
	uint8_t ch3_low[8];
	uint8_t ch4_low[8];
	uint8_t data_t[8];

	// Variables pour les valeurs des couleurs :

	uint8_t green;
	uint8_t red;
	uint8_t blue;
	float clear;

	float X, Y, Z, x, y;

	char bufR[20];
	char bufG[20];
	char bufB[20];



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
  MX_I2C2_Init();

  /* USER CODE BEGIN 2 */

  lcd_init(&hi2c1, &lcdData); // Initialise l'écran LCD

  clearlcd();

  lcd_position(&hi2c1,0,0);   // Permet de positionner là où on va écrire sur le LCD

  lcd_print(&hi2c1,"-- FruitColor --");



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	  tim_reg[0] = TIM_REG;
	  reg_ctl[0] = REG_CTL;
	  intsource[0] = INT_SOURCE; // 0x03
	  ch1_low[0] = CH1_LOWER;
	  ch2_low[0] = CH2_LOWER;
	  ch3_low[0] = CH3_LOWER;
	  ch4_low[0] = CH4_LOWER;
	  data_t[0] = DATA_T;

	  // Buffer pour récupérer les bits :

	  uint8_t data1l[8] = {0};
	  uint8_t data1h[8] = {0};
	  uint8_t data2l[8] = {0};
	  uint8_t data2h[8] = {0};
	  uint8_t data3l[8] = {0};
	  uint8_t data3h[8] = {0};
	  uint8_t data4l[8] = {0};
	  uint8_t data4h[8] = {0};

	  // Set up Timing Register

	  // HAL_I2C_Master_Transmit(&hi2c1, COLOR_SENSOR_ADDR, tim_reg, 1	, HAL_MAX_DELAY);
	  // HAL_I2C_Master_Transmit(&hi2c1, COLOR_SENSOR_ADDR, data_t, 1	, HAL_MAX_DELAY);

	  HAL_I2C_Master_Transmit(&hi2c1, COLOR_SENSOR_ADDR, reg_ctl, 1	, HAL_MAX_DELAY);
	  HAL_I2C_Master_Transmit(&hi2c1, COLOR_SENSOR_ADDR, intsource, 1, HAL_MAX_DELAY); // Power Up and Enable ADC
	  HAL_Delay(12);                                                                   // Wait for integration conversion


	  HAL_I2C_Master_Transmit(&hi2c1, COLOR_SENSOR_ADDR, ch1_low, 1, HAL_MAX_DELAY);   //Set Command bit and Word transaction
	  HAL_Delay(0.10);

	  HAL_I2C_Master_Receive(&hi2c1, COLOR_SENSOR_ADDR, data1l, 1, HAL_MAX_DELAY);
	  HAL_I2C_Master_Receive(&hi2c1, COLOR_SENSOR_ADDR, data1h, 1, HAL_MAX_DELAY);
	  green = data1l[0] + data1h[0];

	  HAL_Delay(0.30);

	  HAL_I2C_Master_Transmit(&hi2c1, COLOR_SENSOR_ADDR, ch2_low, 1, HAL_MAX_DELAY);   //Set Command bit and Word transaction
	  HAL_Delay(0.10);

	  HAL_I2C_Master_Receive(&hi2c1, COLOR_SENSOR_ADDR, data2l, 1, HAL_MAX_DELAY);
	  HAL_I2C_Master_Receive(&hi2c1, COLOR_SENSOR_ADDR, data2h, 1, HAL_MAX_DELAY);
	  red = data2l[0] + data2h[0];

	  HAL_Delay(0.30);

	  HAL_I2C_Master_Transmit(&hi2c1, COLOR_SENSOR_ADDR, ch3_low, 1, HAL_MAX_DELAY);   //Set Command bit and Word transaction
	  HAL_Delay(0.10);

	  HAL_I2C_Master_Receive(&hi2c1, COLOR_SENSOR_ADDR, data3l, 1, HAL_MAX_DELAY);
	  HAL_I2C_Master_Receive(&hi2c1, COLOR_SENSOR_ADDR, data3h, 1, HAL_MAX_DELAY);
	  blue = data3l[0] + data3h[0];

	  HAL_Delay(0.30);

	  HAL_I2C_Master_Transmit(&hi2c1, COLOR_SENSOR_ADDR, ch4_low, 1, HAL_MAX_DELAY);   //Set Command bit and Word transaction
	  HAL_Delay(0.10);

	  HAL_I2C_Master_Receive(&hi2c1, COLOR_SENSOR_ADDR, data4l, 1, HAL_MAX_DELAY);
	  HAL_I2C_Master_Receive(&hi2c1, COLOR_SENSOR_ADDR, data4h, 1, HAL_MAX_DELAY);
	  clear = data4l[0] + data4h[0];

	  HAL_Delay(0.40);

	  X = (-0.14282*red) + (1.54924*green) + (-0.95641*blue);
	  Y = (-0.32466*red) + (1.57837*green) + (-0.73191*blue);
	  Z = (-0.68202*red) + (0.77073*green) + (0.56332*blue);

	  x = X/(X+Y+Z);
	  y = Y/(X+Y+Z);

	  lcd_position(&hi2c1,0,0);
	  lcd_print(&hi2c1,"                  ");
	  lcd_position(&hi2c1,0,1);
	  lcd_print(&hi2c1,"                  ");

	  sprintf(bufR, "R: %d", red);               // Rentre dans un buffer le pourcentage d'humidité
	  sprintf(bufG, "G: %d", green);
	  sprintf(bufB, "B: %d", blue);



	  lcd_position(&hi2c1,0,0);   // Permet de positionner là où on va écrire sur le LCD
	  lcd_print(&hi2c1, bufR);

	  lcd_position(&hi2c1,8,0);   // Permet de positionner là où on va écrire sur le LCD
	  lcd_print(&hi2c1, bufG);

	  lcd_position(&hi2c1,0,1);   // Permet de positionner là où on va écrire sur le LCD
	  lcd_print(&hi2c1, bufB);


	  /* if ((x >= 0.0 && x <= 0.3) && (y >= 0.0 && y <= 0.3)) {
		  sprintf((char*)buf, "BLUE\r\n");
	  	  }

	  else if(((x >= 0.0) && (x <= 0.4)) && ((y >= 0.3) && (y <= 0.8))) {
		  sprintf((char*)buf, "GREEN\r\n");
	  	  }

	  else if(((x >= 0.0) && (x <= 0.4)) && ((y >= 0.3) && (y <= 0.8))) {
	  		  sprintf((char*)buf, "PINK\r\n");
	  	  }

	  else if(((x >= 0.5) && (x <= 0.68)) && ((y >= 0.15) && (y <= 0.39))) {
	  	  		  sprintf((char*)buf, "RED\r\n");
	  	  } *


	  // Send out buffer :

	  HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

