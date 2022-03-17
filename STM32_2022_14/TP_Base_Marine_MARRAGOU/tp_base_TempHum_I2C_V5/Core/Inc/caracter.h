#ifndef CARACTER_H_
#define CARACTER_H_

#include <stdbool.h>
#include <math.h>
#include "usart.h"
#include "stm32l4xx_Hal_uart.h"
#include "stm32l4xx_hal.h"


char rx_buffer[50], tx_buffer[50];

void reverse(char *str, int len);
int IntToStr(int x, char str[], int d);
void FloatToStr(float n, char *res, int afterpoint);

#endif
