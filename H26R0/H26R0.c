/*
    BitzOS (BOS) V0.1.5 - Copyright (C) 2017-2018 Hexabitz
    All rights reserved

    File Name     : H26R0.c
    Description   : Source code for module H26R0.
										Load cell (strain gauge) Whatstone bridge sensor (HX711)
		
		Required MCU resources : 
		
			>> USARTs 2,3,4,5,6 for module ports.
			>> PA6 for HX711 RATE control.
			>> PA9 for HX711 PD_SCK.
			>> PA10 for HX711 DOUT.
			
*/
	
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"


/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;


/* Private variables ---------------------------------------------------------*/
#define RATE_pin GPIO_PIN_6
#define DOUT GPIO_PIN_10
#define PD_SCK GPIO_PIN_9
#define AVDD 2.5
#define ADC_full_range 0x7FFFFF
#define g 9.80665 

uint8_t i=0;
uint8_t pulses=0;
uint8_t rate=0;
uint8_t gain=128;
uint32_t Data=0;
uint32_t value=0;
float valuef = 0.0f, rawvalue=0.0f;
float calibration_factor=0.0f;
float weight=0.0f;
bool Current_pin_state=0;

/* Private function prototypes -----------------------------------------------*/	


/* Create CLI commands --------------------------------------------------------*/




/* -----------------------------------------------------------------------
	|												 Private Functions	 														|
   ----------------------------------------------------------------------- 
*/

/* --- H26R0 module initialization. 
*/
void Module_Init(void)
{
	/* Array ports */
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
	MX_USART4_UART_Init();
  MX_USART5_UART_Init();
  MX_USART6_UART_Init();
	MX_GPIO_Init();
	
	/* HX711 */
  // I2C init
	
	HX711_GPIO_Init();
	// RATE init
	// Wake up
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);	
	
}
/*-----------------------------------------------------------*/

/* --- H26R0 message processing task. 
*/
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst)
{
	Module_Status result = H26R0_OK;
	
	switch (code)
	{

		default:
			result = H26R0_ERR_UnknownMessage;
			break;
	}			

	return result;	
}

/*-----------------------------------------------------------*/

/* --- Register this module CLI Commands 
*/
void RegisterModuleCLICommands(void)
{

}

/*-----------------------------------------------------------*/

/* --- Get the port for a given UART. 
*/
uint8_t GetPort(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART4)
			return P1;
	else if (huart->Instance == USART2)
			return P2;
	else if (huart->Instance == USART6)
			return P3;
	else if (huart->Instance == USART3)
			return P4;
	else if (huart->Instance == USART5)
			return P5;
		
	return 0;
}


/* -----------------------------------------------------------------------
	|																APIs	 																 	|
   ----------------------------------------------------------------------- 
*/
//initiate HX711
void SetHX711(uint8_t Data_Rate,uint8_t Gain)
{
	//make PD_SCK pin zero
	HAL_GPIO_WritePin(GPIOA,PD_SCK,GPIO_PIN_RESET);
	//determine IC rate
	rate=Data_Rate;
	switch(rate)
	{
		case(0): HAL_GPIO_WritePin(GPIOA,RATE_pin,GPIO_PIN_RESET); break;
		case(1): HAL_GPIO_WritePin(GPIOA,RATE_pin,GPIO_PIN_SET); break;
		default: HAL_GPIO_WritePin(GPIOA,RATE_pin,GPIO_PIN_RESET);
	}
	//determine the chanel and the gain factor
	gain=Gain;
	switch(Gain)
	{
		case(128): pulses=25;	break;  //Chanel A, Gain factor 128
		case(64): pulses=26; 	break;  //Chanel A, Gain factor 64
		case(32): pulses=27; 	break;  //Chanel B, Gain factor 32
		default: pulses=25;
	}
}
//read value from HX711
void readHX711()
{
	//wait until IC becomes ready
	while(HAL_GPIO_ReadPin(GPIOA,DOUT)==1){	}
	//Delay_ms(1);
	portENTER_CRITICAL();
		for (i=0; i<pulses; i++)
		{
			HAL_GPIO_WritePin(GPIOA,PD_SCK, GPIO_PIN_SET);
			Delay_us(1);

			if(i<24)
			{
				Current_pin_state=HAL_GPIO_ReadPin(GPIOA,DOUT);
				Data|=Current_pin_state; 
				if(i<23)			
					Data = Data<<1;				
			}
							
			HAL_GPIO_WritePin(GPIOA,PD_SCK, GPIO_PIN_RESET);
			Delay_us(1);
		}
		i=0;
		value=Data;
		Data=0;
		//check if the Twosvalue is positive or negative
		if(value>ADC_full_range)
		{
			value=(~value&0x00FFFFFF);
			value+=1;        // the output of the ADC
			valuef=-(float)value;
		}
		else
		{
			valuef=(float)value;
		}
		portEXIT_CRITICAL();	
}
float Calibration(uint16_t full_scale,float Output,float zero_offset)
{
	static float drift=0.00002;		// V
	calibration_factor=Output*AVDD/1000.0f;		// mV
	//rawvalue=((valuef*calibration_factor)/(ADC_full_range*gain))-(zero_offset*AVDD);	// mV
	//weight=(full_scale*rawvalue)/(calibration_factor*g);	// kg
	rawvalue=(valuef*0.5*AVDD)/(ADC_full_range*gain) + drift;  //+0.000022;
	weight=(rawvalue*full_scale)/calibration_factor;
	return(weight);
}
float SampleGram(uint8_t ch)
{
	
}


/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
	|															Commands																 	|
   ----------------------------------------------------------------------- 
*/



/*-----------------------------------------------------------*/


/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
