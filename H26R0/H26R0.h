/*
    BitzOS (BOS) V0.1.5 - Copyright (C) 2017-2018 Hexabitz
    All rights reserved
		
    File Name     : H1DR1.h
    Description   : Header file for module H1DR1.
										RS-485 Serial Transceiver (MAX14840EASA+)
*/
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef H1DR1_H
#define H1DR1_H

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H1DR1_uart.h"	
#include "H1DR1_gpio.h"	
#include "H1DR1_dma.h"	
	
	
/* Exported definitions -------------------------------------------------------*/

#define	modulePN		_H1DR1

/* Port-related definitions */
#define	NumOfPorts		5
#define P_PROG 				P2						/* ST factory bootloader UART */

/* Define available ports */
#define _P1 
#define _P2 
#define _P3
#define _P4 
#define _P5 
#define _P6 

/* Define available USARTs */
#define _Usart1 1
#define _Usart2 1
#define _Usart3 1
#define _Usart4 1
#define _Usart5 1
#define _Usart6	1

/* Port-UART mapping */
#define P1uart 			&huart5
#define P2uart 			&huart2	
#define P3uart 			&huart6
#define P4uart 			&huart3
#define P5uart 			&huart1	
#define P6uart 			&huart4	
#define P_RS485uart 		P6uart

/* Port Definitions */
#define	USART1_TX_PIN		GPIO_PIN_9
#define	USART1_RX_PIN		GPIO_PIN_10
#define	USART1_TX_PORT	GPIOA
#define	USART1_RX_PORT	GPIOA
#define	USART1_AF				GPIO_AF1_USART1

#define	USART2_TX_PIN		GPIO_PIN_2
#define	USART2_RX_PIN		GPIO_PIN_3
#define	USART2_TX_PORT	GPIOA
#define	USART2_RX_PORT	GPIOA
#define	USART2_AF				GPIO_AF1_USART2

#define	USART3_TX_PIN		GPIO_PIN_10
#define	USART3_RX_PIN		GPIO_PIN_11
#define	USART3_TX_PORT	GPIOB
#define	USART3_RX_PORT	GPIOB
#define	USART3_AF				GPIO_AF4_USART3

#define	USART4_TX_PIN		GPIO_PIN_0
#define	USART4_RX_PIN		GPIO_PIN_1
#define	USART4_TX_PORT	GPIOA
#define	USART4_RX_PORT	GPIOA
#define	USART4_AF				GPIO_AF4_USART4

#define	USART5_TX_PIN		GPIO_PIN_3
#define	USART5_RX_PIN		GPIO_PIN_4
#define	USART5_TX_PORT	GPIOB
#define	USART5_RX_PORT	GPIOB
#define	USART5_AF				GPIO_AF4_USART5

#define	USART6_TX_PIN		GPIO_PIN_4
#define	USART6_RX_PIN		GPIO_PIN_5
#define	USART6_TX_PORT	GPIOA
#define	USART6_RX_PORT	GPIOA
#define	USART6_AF				GPIO_AF5_USART6

/* Module-specific Definitions */
#ifdef H1DR1
	#define _P_RS485 		_P6
	#define P_RS485 		P6
	#define	RS485_RE_PIN		GPIO_PIN_7
	#define	RS485_DE_PIN		GPIO_PIN_0
	#define	RS485_RE_PORT		GPIOA
	#define	RS485_DE_PORT		GPIOB
	// \RE = 1, DE = 0 Low power shutdown mode
	#define	RS485_SHUTDOWN()		HAL_GPIO_WritePin(RS485_RE_PORT, RS485_RE_PIN, GPIO_PIN_SET); HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET)
	// \RE = 0 Enable receiver output
	#define	RS485_RECEIVER_EN()		HAL_GPIO_WritePin(RS485_RE_PORT, RS485_RE_PIN, GPIO_PIN_RESET)
	// DE = 1 Enable driver output
	#define	RS485_DRIVER_EN()		HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_SET)
	// \RE = 1 Disable receiver output
	#define	RS485_RECEIVER_DIS()		HAL_GPIO_WritePin(RS485_RE_PORT, RS485_RE_PIN, GPIO_PIN_SET)
	// DE = 0 Disable driver output
	#define	RS485_DRIVER_DIS()		HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET)
#endif

/* H01R0_Status Type Definition */  
typedef enum 
{
  H1DR1_OK = 0,
	H1DR1_ERR_UnknownMessage = 1,
	H1DR1_ERROR = 255
} Module_Status;

/* Indicator LED */
#define _IND_LED_PORT		GPIOB
#define _IND_LED_PIN		GPIO_PIN_14


/* Export UART variables */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

/* Define UART Init prototypes */
extern void MX_USART1_UART_Init(void);
extern void MX_USART2_UART_Init(void);
extern void MX_USART3_UART_Init(void);
extern void MX_USART4_UART_Init(void);
extern void MX_USART5_UART_Init(void);
extern void MX_USART6_UART_Init(void);



/* -----------------------------------------------------------------------
	|														Message Codes	 														 	|
   ----------------------------------------------------------------------- 
*/



	
/* -----------------------------------------------------------------------
	|																APIs	 																 	|
   ----------------------------------------------------------------------- 
*/



/* -----------------------------------------------------------------------
	|															Commands																 	|
   ----------------------------------------------------------------------- 
*/




#endif /* H1DR1_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
