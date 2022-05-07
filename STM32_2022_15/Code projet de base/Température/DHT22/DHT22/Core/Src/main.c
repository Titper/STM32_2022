/* USER CODE BEGIN Header */
/*
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
#include "HT.h"
#include "timer.h"

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

static rgb_lcd lcdData;

float Temperature = 0, Humidite = 0;   // Initialisation des variables où l'on va stocker le résultat final

uint16_t RH = 0, TEMP = 0;             // Entiers de 16 bits contenant les bits
uint8_t dataH1;
uint8_t dataH2;
uint8_t dataT1;
uint8_t dataT2;
uint8_t SUM;
uint8_t check;

char bufRH[20];                        // Permet de stocker un taux d'humidité et de l'afficher sur le LCD
char bufT[20];                         // Permet de stocker une température et de l'afficher sur le LCD

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
  **/

int main(void)
{
  /* USER CODE BEGIN 1 */

  int k = 0; // Variable pour les boucles "while"

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

  DWT_Delay_Init();
  lcd_init(&hi2c1, &lcdData); // Initialise l'écran LCD

  lcd_position(&hi2c1,0,0);   // Permet de positionner là où on va écrire sur le LCD

  lcd_print(&hi2c1,"-- FruitColor --");

  HAL_Delay(2000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /** -------------------------------------------- Début de la communication avec le DHT22 -------------------------------------------- **/

	  HAL_Delay(2000);
	  Data_Output(GPIOA, GPIO_PIN_1);                          // Configure la broche en mode "sortie" pour communiquer avec le capteur

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);    // Envoie un premier signal de commande vers le capteur (signal logique bas)
	  DWT_Delay_us(1200);                                      // D'une durée de 1200 us

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);      // Envoie le second signal de commande vers le capteur (signal logique haut)
	  DWT_Delay_us(30);                                        // D'une durée de 30 us

	  Data_Input(GPIOA, GPIO_PIN_1);                           // Configure la broche en mode "entrée" pour que le capteur communique avec la STM32

	  /** ------------------------------------------------- Réception des données du DHT22 ------------------------------------------------ **/

	  while(!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)));

	  for (k=0; k < 1000; k++)                                 // Permet de récupérer tous les bits de la trames de données provenant du DHT22
	  {
		  if (HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET)
		  {
	  	  	break;
	  	  }
	  }

	  while(!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)));
	  DWT_Delay_us(40);

	  Read_data(&dataH1);                                      // Récupère les 8 premiers bits de l'humidité
	  Read_data(&dataH2);                                      // Récupère les 8 derniers bits de l'humidité
	  Read_data(&dataT1);                                      // Récupère les 8 premiers bits de la température
	  Read_data(&dataT2);                                      // Récupère les 8 derniers bits de l'humidité
	  Read_data(&SUM);                                         // Récupère les bits du "Checksum" qui vérifie la bonne transmission de la trame

	  check = dataH1 + dataH2 + dataT1 + dataT2;               // Permet de vérifier si nous avons l'intégralité des bits

	  RH = (dataH1<<8) | dataH2;                               // On assemble ensuite tous les bits, en n'oubliant pas d'effectuer un décalage de 8
	  TEMP = (dataT1<<8) | dataT2;                             // sur la gauche pour le premier octet

	  Humidite = RH / 10.0;                                    // On divise par 10.0 les valeurs pour obtenir un nombre décimal
	  Temperature = TEMP / 10.0;

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);      // Recommence une lecture

	  /** ---------------------------------------------------- Transmission vers le LCD --------------------------------------------------- **/

	  clearlcd();                                              // Clear l'écran LCD

	  sprintf(bufRH,"Humidite: %.1f ", Humidite);               // Rentre dans un buffer le pourcentage d'humidité
	  sprintf(bufT, "Temp.: %.1f C    ", Temperature);          // Rentre dans un buffer la température

	  lcd_position(&hi2c1,0,0);                                // Affichage du taux d'humidité
	  lcd_print(&hi2c1,bufRH);                                 //
	  lcd_print(&hi2c1,"%  ");                                 //

	  lcd_position(&hi2c1,0,1);                                // Affichage de la température
	  lcd_print(&hi2c1,bufT);                                  //

  }
  /* USER CODE END 3 */
}

  /**
  * @brief System Clock Configuration
  * @retval None
  **/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage **/

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters in the RCC_OscInitTypeDef structure. **/

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
  /** Initializes the CPU, AHB and APB buses clocks **/

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
  **/

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
  **/
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

