/*
    BitzOS (BOS)V0.2.3 - Copyright (C) 2017-2020 Hexabitz
    All rights reserved
		
    File Name     : H26R0.h
    Description   : Header file for module H26R0.
										Load cell (strain gauge) Whatstone bridge sensor (HX711)
*/
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef H26R0_H
#define H26R0_H

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H26R0_MemoryMap.h"	
#include "H26R0_uart.h"	
#include "H26R0_gpio.h"	
#include "H26R0_dma.h"	
	
	
/* Exported definitions -------------------------------------------------------*/

#define	modulePN		_H26R0

/* Port-related definitions */
#define	NumOfPorts		5
#define P_PROG 				P2						/* ST factory bootloader UART */

/* Define available ports */
#define _P1 
#define _P2 
#define _P3
#define _P4 
#define _P5  

/* Define available USARTs */
#define _Usart2 1
#define _Usart3 1
#define _Usart4 1
#define _Usart5 1
#define _Usart6	1

/* Port-UART mapping */
#define P1uart 			&huart4
#define P2uart 			&huart2	
#define P3uart 			&huart6
#define P4uart 			&huart3
#define P5uart 			&huart5	

/* Port Definitions */
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

#define STOP_MEASUREMENT_RANGING      0
#define START_MEASUREMENT_RANGING     1

/* Module_Status Type Definition */
#define NUM_MODULE_PARAMS		3

/* Module-specific Definitions */
#define RATE_pin             GPIO_PIN_6
#define DOUT                 GPIO_PIN_10
#define PD_SCK               GPIO_PIN_9
#define TIMERID_TIMEOUT_MEASUREMENT   0xFF

// Module EEPROM variables addresses - Module Addressing Space 500 - 599
#define _EE_cell_full_scale		500
#define _EE_cell_drift_LSB		501
#define _EE_cell_drift_MSB		502
#define _EE_cell_output_LSB		503
#define _EE_cell_output_MSB		504
#define _EE_zero_drift_LSB		505
#define _EE_zero_drift_MSB		506


/* H26R0_Status Type Definition */  
typedef enum 
{
  H26R0_OK = 0,
	H26R0_ERR_UnknownMessage = 1,
	H26R0_ERR_WrongParams,
	H26R0_ERROR = 255
} Module_Status;

/* Indicator LED */
#define _IND_LED_PORT		GPIOA
#define _IND_LED_PIN		GPIO_PIN_11


/* Export UART variables */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

/* Define UART Init prototypes */
extern void MX_USART2_UART_Init(void);
extern void MX_USART3_UART_Init(void);
extern void MX_USART4_UART_Init(void);
extern void MX_USART5_UART_Init(void);
extern void MX_USART6_UART_Init(void);


	
/* -----------------------------------------------------------------------
	|																APIs	 																 	|
   ----------------------------------------------------------------------- 
*/
void SetHX711Rate(uint8_t Data_Rate);
float Calibration(uint16_t Full_Scale, float Cell_Output, float Cell_Drift);
float SampleGram(uint8_t ch);
float SampleKGram(uint8_t ch);
float SampleOunce(uint8_t ch);
float SamplePound(uint8_t ch);
int StreamGramToPort(uint8_t Ch, uint8_t Port, uint8_t Module, uint32_t Period, uint32_t Timeout);
int StreamKGramToPort(uint8_t Ch, uint8_t Port, uint8_t Module, uint32_t Period, uint32_t Timeout);
int StreamOunceToPort(uint8_t Ch, uint8_t Port, uint8_t Module, uint32_t Period, uint32_t Timeout);
int StreamPoundToPort(uint8_t Ch, uint8_t Port, uint8_t Module, uint32_t Period, uint32_t Timeout);
int StreamGramToBuffer(uint8_t Ch, float *buffer, uint32_t Period, uint32_t Timeout);
int StreamKGramToBuffer(uint8_t Ch, float *buffer, uint32_t Period, uint32_t Timeout);
int StreamOunceToBuffer(uint8_t Ch, float *buffer, uint32_t Period, uint32_t Timeout);
int StreamPoundToBuffer(uint8_t Ch, float *buffer, uint32_t Period, uint32_t Timeout);
float Average(uint8_t ch, uint8_t samples);
int ZeroCal(uint8_t Ch);
int Stop(void);
int PowerDown(void);
int PowerOn(void);

/* -----------------------------------------------------------------------
	|															Commands																 	|
   ----------------------------------------------------------------------- 
*/



#endif /* H26R0_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
