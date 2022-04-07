#include <ht.h>
#include "timer.h"
#include "stm32l4xx_hal.h"

void Data_Output (GPIO_TypeDef *PORT, uint16_t PIN) //On va se diriger vers le capteur
  {
  	GPIO_InitTypeDef GPIO_InitStruct = {0};

  	GPIO_InitStruct.Pin = PIN;
  	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; //Initialisation on pushpull
  	HAL_GPIO_Init(PORT, &GPIO_InitStruct);
  }

void Data_Input (GPIO_TypeDef *PORT, uint16_t PIN) //On va se diriger vers le microcontroleur
  {
  	GPIO_InitTypeDef GPIO_InitStruct = {0};

  	GPIO_InitStruct.Pin = PIN;
  	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  	HAL_GPIO_Init(PORT, &GPIO_InitStruct); //Initialisation classique
  }

void Read_data (uint8_t *data)
  {
  	int i, k;
  	for (i=0;i<8;i++)
  	{
  		if (HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET)
  		{
  			(*data)&= ~(1<<(7-i)); //data bit est a 0 si la longueur entre 26-28us
  			while(!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)));
  			DWT_Delay_us(40);
  		}
  		else                       //data bit est a 1 si la longueur 70us
  		{
  			(*data)|= (1<<(7-i));
  			for (k=0;k<1000;k++)
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
