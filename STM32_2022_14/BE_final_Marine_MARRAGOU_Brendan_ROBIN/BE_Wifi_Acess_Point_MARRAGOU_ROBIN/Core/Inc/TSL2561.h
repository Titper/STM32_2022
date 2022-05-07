#ifndef __TSL2561_H
#define __TSL2561_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "main.h"

extern uint16_t Channel_0, Channel_1;	 
#define Write_success 1
#define Write_fail 0;	 
	 
	 
// I2C address options
#define ADDR_LOW          0x39
#define ADDR_FLOAT        0x29    // Default address (pin left floating)
#define ADDR_HIGH         0x49

#define ADDR_FLOAT_Write 0x53
#define ADDR_FLOAT_Read	 0x52
	 
//---------------------------------------------------
// x       xx      xxxxx 
//CMD TRANSACTION ADDRESS
#define COMMAND_CMD   0xC0
#define TRANSACTION_SPECIAL 0X40
#define TRANSACTION 0x50

//ADDRESS
#define CONTROL   0x00
#define TIMING    0x01
#define INTERRUPT 0X06
#define THLLOW 0x02
#define THLHIGH 0X03
#define THHLOW 0x04
#define THHHIGH 0X05
#define ID 0x0A

#define DATA0LOW 0X8C
#define DATA0HIGH 0X8D
#define DATA1LOW 0X8E
#define DATA1HIGH 0X8F
//---------------------------------------------------

#define CONTROL_POWERON   0x03
#define CONTROL_POWEROFF  0x00
#define INTR_INTER_MODE 0X30

//INTERRUPT
#define INTEGRATIONTIME_400MS1X 0x02
#define INTEGRATIONTIME_400MS16X 0x12

//---------------------------------------------------
// Integration time scaling factors
//---------------------------------------------------
#define CH_SCALE 16 // scale channel values by 2^16

// Nominal 400 ms integration. 
// Specifies the integration time in 2.7-ms intervals
// 400/2.7 = 148
#define NOM_INTEG_CYCLE 148
//---------------------------------------------------
// Gain scaling factors
//---------------------------------------------------
#define CH0GAIN128X 107 // 128X gain scalar for Ch0
#define CH1GAIN128X 115 // 128X gain scalar for Ch1

//---------------------------------------------------

#define LUX_SCALE 14	// 2^14
#define RATIO_SCALE 9	// scale ratio to 2^5=512
#define K1T 0x0040 		// 0.125 * 2^RATIO_SCALE
#define B1T 0x01f2 		// 0.0304 * 2^LUX_SCALE
#define M1T 0x01be 		// 0.0272 * 2^LUX_SCALE
#define K2T 0x0080 		// 0.250 * 2^RATIO_SCALE
#define B2T 0x0214 		// 0.0325 * 2^LUX_SCALE
#define M2T 0x02d1		// 0.0440 * 2^LUX_SCALE
#define K3T 0x00c0		// 0.375 * 2^RATIO_SCALE
#define B3T 0x023f 		// 0.0351 * 2^LUX_SCALE
#define M3T 0x037b 		// 0.0544 * 2^LUX_SCALE
#define K4T 0x0100 		// 0.50 * 2^RATIO_SCALE
#define B4T 0x0270 		// 0.0381 * 2^LUX_SCALE
#define M4T 0x03fe 		// 0.0624 * 2^LUX_SCALE
#define K5T 0x0138 		// 0.61 * 2^RATIO_SCALE
#define B5T 0x016f 		// 0.0224 * 2^LUX_SCALE
#define M5T 0x01fc 		// 0.0310 * 2^LUX_SCALE
#define K6T 0x019a 		// 0.80 * 2^RATIO_SCALE
#define B6T 0x00d2 		// 0.0128 * 2^LUX_SCALE
#define M6T 0x00fb 		// 0.0153 * 2^LUX_SCALE
#define K7T 0x029a 		// 1.3 * 2^RATIO_SCALE
#define B7T 0x0018 		// 0.00146 * 2^LUX_SCALE
#define M7T 0x0012 		// 0.00112 * 2^LUX_SCALE
#define K8T 0x029a 		// 1.3 * 2^RATIO_SCALE
#define B8T 0x0000 		// 0.000 * 2^LUX_SCALE
#define M8T 0x0000 		// 0.000 * 2^LUX_SCALE
//---------------------------------------------------

uint8_t I2C_DEV_Write(uint16_t I2C_Addr,uint16_t Register_Addr,uint8_t Register_Data);
uint8_t I2C_DEV_Read(uint16_t Register_Addr);

void I2C_DEV_init(void);
void Reload_register(void);
void SET_Interrupt_Threshold(uint16_t min,uint16_t max);
void Read_Channel(void);
uint32_t calculateLux(uint16_t iGain,uint16_t tIntCycles);

#endif


