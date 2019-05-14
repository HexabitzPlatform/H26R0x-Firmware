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
			>> Gain of ch1 is fixed at 128.
			>> Gain of ch2 is fixed at 32.
			
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
#define Kg2Pound_ratio 2.20462262 
#define Kg2Ounce_ratio 35.274
#define IC_drift 0.0           //00004

uint8_t pulses=0;
uint8_t rate=0;
uint8_t gain=128;
uint16_t full_scale=0;
uint32_t Data=0;
uint32_t value=0;
float valuef = 0.0f, rawvalue=0.0f;
float calibration_factor=0.0f;
static float cell_output=0.0;
static float cell_drift=0.00002;
float weight=0.0f;
bool Current_pin_state=0;
float weightGram=0.0f;
float weightKGram=0.0f;
float weightOunce=0.0f;
float weightPound=0.0f;
float w=0.0f;

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
  // GPIO init
	HX711_GPIO_Init();
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

/*-----------------------------------------------------------*/
//read value from HX711
void readHX711()
{
	uint8_t j=0;
	//wait until IC becomes ready
	while(HAL_GPIO_ReadPin(GPIOA,DOUT)==1){	}
	//Delay_ms(1);
	portENTER_CRITICAL();
		for (j=0; j<pulses; j++)
		{
			HAL_GPIO_WritePin(GPIOA,PD_SCK, GPIO_PIN_SET);
			Delay_us(1);

			if(j<24)
			{
				Current_pin_state=HAL_GPIO_ReadPin(GPIOA,DOUT);
				Data|=Current_pin_state; 
				if(j<23)			
					Data = Data<<1;				
			}
							
			HAL_GPIO_WritePin(GPIOA,PD_SCK, GPIO_PIN_RESET);
			Delay_us(1);
		}
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
 
/*-----------------------------------------------------------*/

//calculate the weight
float weightCalculation()
{
	rawvalue=(valuef*0.5*AVDD)/(ADC_full_range*gain) + cell_drift + IC_drift;  //+0.000022;
	weight=(rawvalue*full_scale)/calibration_factor;
	return(weight);	
}


/* -----------------------------------------------------------------------
	|																APIs	 																 	|
   ----------------------------------------------------------------------- 
*/
//set HX711 Rate sample per second
void SetHX711Rate(uint8_t Data_Rate)
{
	//make PD_SCK pin zero
	HAL_GPIO_WritePin(GPIOA,PD_SCK,GPIO_PIN_RESET);
	//determine IC rate
	rate=Data_Rate;
	switch(rate)
	{
		case(10): HAL_GPIO_WritePin(GPIOA,RATE_pin,GPIO_PIN_RESET); break;
		case(80): HAL_GPIO_WritePin(GPIOA,RATE_pin,GPIO_PIN_SET); break;
		default: HAL_GPIO_WritePin(GPIOA,RATE_pin,GPIO_PIN_RESET);
	}
}

/*-----------------------------------------------------------*/

//Set the gain factor of the used channel
void SetHX711Gain(uint8_t ch)
{
	//determine the chanel and the gain factor
	switch(ch)
	{
		case(1): pulses=25;	gain=128; break;  //Chanel A, Gain factor 128
		//case(64): pulses=26; 	break;  //Chanel A, Gain factor 64
		case(2): pulses=27; gain=32;	break;  //Chanel B, Gain factor 32
		default: pulses=25;
	}
}

/*-----------------------------------------------------------*/

//enter the calibration values of the load cell
float Calibration(uint16_t Full_Scale,float Cell_Output,float Cell_Drift)
{
	cell_output=Cell_Output;
	full_scale=Full_Scale;
	cell_drift=Cell_Drift/1000.0f;
	calibration_factor=cell_output*AVDD/1000.0f;		// mV	
	return 0;
}

/*-----------------------------------------------------------*/

//read weight value from channel ch of HX711 and return weight in Gram
float SampleGram(uint8_t ch)
{
	uint8_t i=0;
	SetHX711Gain(ch);
	for(i=0; i<2; i++) 		readHX711();
	weightGram=weightCalculation()*1000;
	return(weightGram);
}

/*-----------------------------------------------------------*/

//read weight value from channel ch of HX711 and return weight in KGram
float SampleKGram(uint8_t ch)
{
	uint8_t i=0;
	SetHX711Gain(ch);
	for(i=0; i<2; i++) 		readHX711();
	weightKGram=weightCalculation();
	return(weightKGram);
}

/*-----------------------------------------------------------*/

//read weight value from channel ch of HX711 and return weight in Ounce
float SampleOunce(uint8_t ch)
{
	uint8_t i=0;
	SetHX711Gain(ch);
	for(i=0; i<2; i++)		readHX711();
	weightOunce=weightCalculation()*Kg2Ounce_ratio;
	return(weightKGram);
}

/*-----------------------------------------------------------*/

//read weight value from channel ch of HX711 and return weight in Pound
float SamplePound(uint8_t ch)
{
	uint8_t i=0;
	SetHX711Gain(ch);
	for(i=0; i<2; i++) 		readHX711();
	weightPound=weightCalculation()*Kg2Pound_ratio;  
	return(weightKGram);
}

/*-----------------------------------------------------------*/

//stream weightvalue from channel ch to port
void StreamGramToPort(uint8_t Ch, uint8_t Port, uint8_t Module, uint32_t Period, uint32_t Timeout)
{
uint8_t ch=Ch;
	uint8_t port=Port;
	uint8_t module=Module;
	uint32_t period=Period;
	uint32_t timeout=Timeout;
	
}	

/*-----------------------------------------------------------*/

//stream weightvalue from channel ch to port
void StreamKGramToPort(uint8_t Ch, uint8_t Port, uint8_t Module, uint32_t Period, uint32_t Timeout)
{
uint8_t ch=Ch;
	uint8_t port=Port;
	uint8_t module=Module;
	uint32_t period=Period;
	uint32_t timeout=Timeout;
	
}

/*-----------------------------------------------------------*/

//stream weightvalue from channel ch to port
void StreamOunceToPort(uint8_t Ch, uint8_t Port, uint8_t Module, uint32_t Period, uint32_t Timeout)
{
uint8_t ch=Ch;
	uint8_t port=Port;
	uint8_t module=Module;
	uint32_t period=Period;
	uint32_t timeout=Timeout;
	
}

/*-----------------------------------------------------------*/

//stream weightvalue from channel ch to port
void StreamPoundToPort(uint8_t Ch, uint8_t Port, uint8_t Module, uint32_t Period, uint32_t Timeout)
{
uint8_t ch=Ch;
	uint8_t port=Port;
	uint8_t module=Module;
	uint32_t period=Period;
	uint32_t timeout=Timeout;
	
}

/*-----------------------------------------------------------*/

//stream weightvalue from channel ch to buffer
void StreamGramToBuffer(uint8_t Ch, float *buffer, uint32_t Period, uint32_t Timeout)
{
	uint8_t ch=Ch;
	uint8_t period=Period;
	uint8_t timeout=Timeout;
	
}

/*-----------------------------------------------------------*/

//stream weightvalue from channel ch to buffer
void StreamKGramToBuffer(uint8_t Ch, float *buffer, uint32_t Period, uint32_t Timeout)
{
	uint8_t ch=Ch;
	uint8_t period=Period;
	uint8_t timeout=Timeout;
	
}

/*-----------------------------------------------------------*/

//stream weightvalue from channel ch to buffer
void StreamOunceToBuffer(uint8_t Ch, float *buffer, uint32_t Period, uint32_t Timeout)
{
	uint8_t ch=Ch;
	uint8_t period=Period;
	uint8_t timeout=Timeout;
	
}

/*-----------------------------------------------------------*/

//stream weightvalue from channel ch to buffer
void StreamPoundToBuffer(uint8_t Ch, float *buffer, uint32_t Period, uint32_t Timeout)
{
	uint8_t ch=Ch;
	uint8_t period=Period;
	uint8_t timeout=Timeout;
	
}

/*-----------------------------------------------------------*/

//HX711 Power Down
void PowerDown(void)
{
//make PD_SCK pin high
	HAL_GPIO_WritePin(GPIOA,PD_SCK,GPIO_PIN_SET);
	
}

/*-----------------------------------------------------------*/

//HX711 Power Down
void PowerOn(void)
{
//make PD_SCK pin high
	HAL_GPIO_WritePin(GPIOA,PD_SCK,GPIO_PIN_RESET);
	
}


/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
	|															Commands																 	|
   ----------------------------------------------------------------------- 
*/



/*-----------------------------------------------------------*/


/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
