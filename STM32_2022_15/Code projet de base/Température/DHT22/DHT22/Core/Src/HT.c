#include "HT.h"
#include "timer.h"
#include "stm32l4xx_hal.h"

void Data_Output (GPIO_TypeDef *PORT, uint16_t PIN) // Transmission du microcontrôleur vers le capteur
  {
  	GPIO_InitTypeDef GPIO_InitStruct = {0};

  	GPIO_InitStruct.Pin = PIN;
  	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Pin en Pushpull
  	HAL_GPIO_Init(PORT, &GPIO_InitStruct);
  }

void Data_Input (GPIO_TypeDef *PORT, uint16_t PIN) // Transmission du capteur vers le microcontrôleur
  {
  	GPIO_InitTypeDef GPIO_InitStruct = {0};

  	GPIO_InitStruct.Pin = PIN;
  	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  	HAL_GPIO_Init(PORT, &GPIO_InitStruct);
  }

void Read_data (uint8_t *data) // Permet de récupérer les bits de la trame de données du DHT22
  {
  	int i, k;
  	for (i=0; i<8; i++)
  	{
  		if (HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET)
  		{
  			(*data)&= ~(1<<(7-i)); // Si le bit est un '0'
  			while(!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)));
  			DWT_Delay_us(40);
  		}
  		else                       // Si le bit est un '1'
  		{
  			(*data)|= (1<<(7-i));
  			for (k=0; k<1000; k++)
  			{
  				if (HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET)
  				  {
  				  	break;
  				  }
  			}
  			while(!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)));
  			DWT_Delay_us(40);
  		}
  	 }
  }
