/*
    BitzOS (BOS) V0.1.5 - Copyright (C) 2017-2018 Hexabitz
    All rights reserved

    File Name     : H1DR1.c
    Description   : Source code for module H1DR1.
										RS-485 Serial Transceiver (MAX14840EASA+)
		
		Required MCU resources : 
		
			>> USARTs 1,2,3,5,6 for module ports.
			>> USART 4 for MAX14840EASA+.
			>> PB0 for DE (driver output enable).
			>> PA7 for \RE (receiver output enable).
			
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


/* Private function prototypes -----------------------------------------------*/	


/* Create CLI commands --------------------------------------------------------*/




/* -----------------------------------------------------------------------
	|												 Private Functions	 														|
   ----------------------------------------------------------------------- 
*/

/* --- H1DR1 module initialization. 
*/
void Module_Init(void)
{
	/* Array ports */
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART5_UART_Init();
  MX_USART6_UART_Init();
	
	/* RS485 port */
  MX_USART4_UART_Init();
	RS485_DE_RE_Init();
	RS485_RECEIVER_EN();
	//RS485_DRIVER_EN();
	
}
/*-----------------------------------------------------------*/

/* --- H1DR1 message processing task. 
*/
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst)
{
	Module_Status result = H1DR1_OK;
	
	switch (code)
	{

		default:
			result = H1DR1_ERR_UnknownMessage;
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
	if (huart->Instance == USART5)
			return P1;
	else if (huart->Instance == USART2)
			return P2;
	else if (huart->Instance == USART6)
			return P3;
	else if (huart->Instance == USART3)
			return P4;
	else if (huart->Instance == USART1)
			return P5;
	else if (huart->Instance == USART4)
			return P6;
		
	return 0;
}


/* -----------------------------------------------------------------------
	|																APIs	 																 	|
   ----------------------------------------------------------------------- 
*/



/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
	|															Commands																 	|
   ----------------------------------------------------------------------- 
*/



/*-----------------------------------------------------------*/


/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
