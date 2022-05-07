#include <TSL2561.h>
#include "i2c.h"
#include "usart.h"
#include "stm32l4xx_hal.h"

uint16_t Channel_0, Channel_1;

/**********************************************************************************************
* @brief  I2C_DEV_Write(uint16_t Register_Addr,uint8_t Register_Data)
* @param  Register_Addr : Register Address	
* @param  Register_Data : Register data
* @param  return Write state,Write success is 1,Write fail is 0
**********************************************************************************************/
uint8_t I2C_DEV_Write(uint16_t I2C_Addr,uint16_t Register_Addr,uint8_t Register_Data)
{
	if(HAL_I2C_Mem_Write(&hi2c1, I2C_Addr, Register_Addr, I2C_MEMADD_SIZE_8BIT, &Register_Data, 1, 1000) == HAL_OK)
	{
		return Write_success;
	} else{
		return Write_fail;
	}
}

/**********************************************************************************************
* @brief  I2C_DEV_Read(uint16_t Register_Addr)
* @param  Register_Addr : Register Address	
* @param  
* @param  return read data (uchar)
**********************************************************************************************/
uint8_t I2C_DEV_Read(uint16_t Register_Addr)
{
	uint8_t ReadBuffer[1];
	HAL_I2C_Mem_Read(&hi2c1, ADDR_FLOAT_Read, Register_Addr, I2C_MEMADD_SIZE_8BIT,ReadBuffer,1, 1000);
	return ReadBuffer[0];
}
 
/**********************************************************************************************
* @brief  	I2C_DEV_init()
* @param   	power on ,set gain is 16,interrupt is 402ms
* @param   
* @param    This field selects the integration time for each conversion.
**********************************************************************************************/
void I2C_DEV_init(void)
{ 
	/* write date from tsl2561 */
	I2C_DEV_Write(ADDR_FLOAT_Write,COMMAND_CMD | CONTROL,CONTROL_POWERON);//power on
	HAL_Delay(2000);//Wait 2 seconds for power on
	
	I2C_DEV_Write(ADDR_FLOAT_Write,COMMAND_CMD | TIMING, INTEGRATIONTIME_400MS16X);  //400MS
	I2C_DEV_Write(ADDR_FLOAT_Write,COMMAND_CMD | CONTROL, CONTROL_POWERON); //Every ADC cycle generates interrupt
	I2C_DEV_Write(ADDR_FLOAT_Write,COMMAND_CMD | INTERRUPT, INTR_INTER_MODE);	//TEST MODE
}
/**********************************************************************************************
* @brief  	Read_Channel()
* @param    
* @param   	read two ADC data
* @param     
**********************************************************************************************/
void Read_Channel()
{	
	uint8_t DataLow,DataHigh;
	DataLow = I2C_DEV_Read(COMMAND_CMD | DATA0LOW);
	DataHigh = I2C_DEV_Read(COMMAND_CMD | DATA0HIGH);
	Channel_0 = 256 * DataHigh + DataLow ;
		
	DataLow = I2C_DEV_Read(COMMAND_CMD | DATA1LOW);
	DataHigh = I2C_DEV_Read(COMMAND_CMD | DATA1HIGH);
	Channel_1 = 256 * DataHigh + DataLow ;
}

/**********************************************************************************************
* @brief  	calculateLux()
* @param    Channel_0 and Channel_1 is for TSL2561_Read_Channel();
* @param   	// Arguments: unsigned int iGain - gain, where 0:1X, 1:8X, 2:16X, 3:128X
* @param   	// unsigned int tIntCycles - INTEG_CYCLES defined in Timing Register
**********************************************************************************************/
uint32_t calculateLux(uint16_t iGain,uint16_t tIntCycles)
{
  unsigned long chScale0;
  unsigned long chScale1;
  unsigned long channel1;
  unsigned long channel0;
  unsigned long temp;
  unsigned long ratio1 = 0;
  unsigned long ratio;
  unsigned long lux_temp;
  unsigned int b, m;

  // No scaling if nominal integration (148 cycles or 400 ms) is used
  if (tIntCycles == NOM_INTEG_CYCLE)
  {
//     chScale0 = 65536;
	  chScale0 = (1 << (CH_SCALE));
  }
  else
    chScale0 = (NOM_INTEG_CYCLE << CH_SCALE) / tIntCycles;
  switch (iGain)
  {
    case 0: // 1x gain
      chScale1 = chScale0; // No scale. Nominal setting
      break;
    case 1: // 8x gain
      chScale0 = chScale0 >> 3; // Scale/multiply value by 1/8
      chScale1 = chScale0;
      break;
    case 2: // 16x gain
      chScale0 = chScale0 >> 4; // Scale/multiply value by 1/16
      chScale1 = chScale0;
      break;
    case 3: // 128x gain
      chScale1 = chScale0 / CH1GAIN128X;
      chScale0 = chScale0 / CH0GAIN128X;
      break;
  }
  // Read Channel for ADC
  Read_Channel();
  // scale the channel values
  channel0 = (Channel_0 * chScale0) >>  CH_SCALE;
  channel1 = (Channel_1 * chScale1) >>  CH_SCALE;

  // find the ratio of the channel values (Channel1/Channel0)
  if (channel0 != 0)
    ratio1 = (channel1 << (RATIO_SCALE + 1)) / channel0;
  ratio = (ratio1 + 1) >> 1;	  									 // round the ratio value

  if (ratio >= 0 && ratio <= K1T) {
  		b = B1T;
  		m = M1T;
  	} else if (ratio <= K2T) {
  		b = B2T;
  		m = M2T;
  	} else if (ratio <= K3T) {
  		b = B3T;
  		m = M3T;
  	} else if (ratio <= K4T) {
  		b = B4T;
  		m = M4T;
  	} else if (ratio <= K5T) {
  		b = B5T;
  		m = M5T;
  	} else if (ratio <= K6T) {
  		b = B6T;
  		m = M6T;
  	} else if (ratio <= K7T) {
  		b = B7T;
  		m = M7T;
  	} else if (ratio <= K8T) {
  		b = B8T;
  		m = M8T;
  	}

  temp = ((channel0 * b) - (channel1 * m));
  temp += (1 << (LUX_SCALE - 1));			// round lsb (2^(LUX_SCALE-1))
  //temp = temp + 32768;
  lux_temp = temp >> LUX_SCALE;			// strip off fractional portion
  return (lux_temp);		  							// Signal I2C had no errors
}

