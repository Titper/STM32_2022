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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lib_lcd.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static rgb_lcd lcdData;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float Temperature = 0, Humidite = 0;
int k = 0, reponse = 0; //variable dans le main
uint8_t HH;
uint8_t HL;
uint8_t TH;
uint8_t TL;
uint8_t SUM;
uint8_t check;
char tabH[20]; //tableau de caractère pour l'humidité à afficher sur le LCD
char tabT[20]; //tableau de caractère pour la température à afficher sur le LCD
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Delay_us(uint16_t us);//timer en microsecondes
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);//Mise en sortie d'une broche
void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);//Mise en entrée d'une broche
void DHT22_start(void);//Initialiser DHT22
int DHT22_Answer(void);//Attendre réponse duu DHT22 avant de relever des données
void DHT22_Read_Data (uint8_t *data);//Relever donnée d'humidité puis de température
void DHT22_Display_Data(void);//Afficher humidité et température sur LCD
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Delay_us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	while(__HAL_TIM_GET_COUNTER(&htim2) < us);
}

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DHT22_start(void)
{
	Set_Pin_Output(DHT22_GPIO_Port, DHT22_Pin);//Mettre broche en sortie pour initialiser DHT22
	HAL_GPIO_WritePin(DHT22_GPIO_Port, DHT22_Pin, GPIO_PIN_RESET);
	Delay_us(1200);
	HAL_GPIO_WritePin(DHT22_GPIO_Port, DHT22_Pin, GPIO_PIN_SET);
	Delay_us(30);
	Set_Pin_Input(DHT22_GPIO_Port, DHT22_Pin); //Mettre broche en entrée pour recevoir réponse DHT22
}

int DHT22_Answer(void)
{
	//Je choisit toujours 10 us de plus pour les temps pour une marge d'erreur
	int temps = 0, reponse = 0;
	//verifier qu'on est bien a 0 pendant 80 us
	while((HAL_GPIO_ReadPin(DHT22_GPIO_Port, DHT22_Pin) == GPIO_PIN_RESET) && (temps < 90))
	{
		Delay_us(1);
		temps++;
	}

	if(temps < 90)//si on est bien rester à 0 pendant de 80 us alors OK
	{
		temps = 0;
		//verifier qu'on est bien a 1 pendant 80 us
		while((HAL_GPIO_ReadPin(DHT22_GPIO_Port, DHT22_Pin) == GPIO_PIN_SET) && (temps < 90))
		{
			Delay_us(1);
			temps++;
		}

		if(temps < 90)//si on est bient rester à 1 pendant 80 us alors OK
		{
			//début de la trame de donnée 50 us à 0 puis 1er bit de donnée
			temps = 0;

			//vérifier la reception du premier bit
			while((HAL_GPIO_ReadPin(DHT22_GPIO_Port, DHT22_Pin) == GPIO_PIN_RESET) && (temps < 60))
			{
				Delay_us(1);
				temps++;
			}
			if(temps < 60)
			{
				reponse = 1;
			}
		}
	}

	Delay_us(40);

	return reponse;
}

void DHT22_Read_Data (uint8_t *data)
{
 	int i, k;
  	for (i=0;i<8;i++)
  	{
  		if (HAL_GPIO_ReadPin (DHT22_GPIO_Port, DHT22_Pin) == GPIO_PIN_RESET)
  		{
  			(*data)&= ~(1<<(7-i)); //Mettre bit à 0
  			while(!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)));
  			Delay_us(40);
  		}
  		else
  		{
  			(*data)|= (1<<(7-i)); //Mettre bit à 1
  			for (k=0;k<1000;k++)
  			{
  				if (HAL_GPIO_ReadPin (DHT22_GPIO_Port, DHT22_Pin) == GPIO_PIN_RESET)
  				  {
  				  	break;
  				  }
  			}
  			while(!(HAL_GPIO_ReadPin(DHT22_GPIO_Port, DHT22_Pin)));
  			Delay_us(40);
  		}
  	 }
}

void DHT22_Display_Data(void)
{
	sprintf(tabH,"Humidite: %.1f ", Humidite);
	sprintf(tabT, "Temp.: %.1f C   ", Temperature);
	lcd_position(&hi2c1, 0, 0);
	lcd_print(&hi2c1, tabH);
	lcd_print(&hi2c1, "%");
	lcd_position(&hi2c1, 0, 1);
	lcd_print(&hi2c1, tabT);
}
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
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);//initialisation timer
  lcd_init(&hi2c1, &lcdData); //initialisation lcd
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	 HAL_Delay(1000);//Attente 1s = refaire la boucle toute les secondes
	 DHT22_start();
	 reponse = DHT22_Answer();

	 /*DHT22 à une trame de 40 bit, 8 premier bit humidité haute HH, 8 bit d'après humidité basse HL,
	 8 bit bit d'après température haute, 8 bit d'après température basse, 8 dernier bit pour verification
	 de l'acquisition */

	 if(reponse == 1)
	 {
	 	 //Commencer aquérir les 5 différents octets
	 	 DHT22_Read_Data(&HH);
	 	 DHT22_Read_Data(&HL);
	 	 DHT22_Read_Data(&TH);
	 	 DHT22_Read_Data(&TL);
	 	 DHT22_Read_Data(&SUM);

	 	 check = HH + HL + TH + TL;
	 	 if(check == SUM)//vérifier qu'il n'y a pas d'erreur dans les données reçu
	 	 {
	 	  	//combiner 2 octets d'humité et diviser resultat par 10 pour avoir humidité en %
	 	  	Humidite = (float) ((HH<<8) | HL) / 10;
	 	  	//combiner 2 octets de température et diviser resultat par 10 pour avoir température en °C
	 	  	Temperature = (float) ((TH<<8) | TL) / 10;

	 	  	DHT22_Display_Data();//afficher température sur LCD
	 	 }
	 }
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

