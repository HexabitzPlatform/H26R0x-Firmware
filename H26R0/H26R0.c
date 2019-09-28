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
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

/* Module exported parameters ------------------------------------------------*/
float H26R0_Weight1 = 0.0f;
float H26R0_Weight2 = 0.0f;
uint8_t H26R0_DATA_FORMAT = FMT_FLOAT;
module_param_t modParam[NUM_MODULE_PARAMS] = {{.paramPtr=&H26R0_Weight1, .paramFormat=FMT_FLOAT, .paramName="weight1"} ,
{.paramPtr=&H26R0_Weight2, .paramFormat=FMT_FLOAT, .paramName="weight2"},
{.paramPtr=&H26R0_DATA_FORMAT, .paramFormat=FMT_UINT8, .paramName="format"}};

/* Private variables ---------------------------------------------------------*/
/* Define HX711 pins */
#define AVDD                     3 //2.44

/* Define preprocessor variables */
#define ADC_full_range           0x7FFFFF
#define Kg2Gram_ratio            1000
#define Kg2Pound_ratio           2.20462262 
#define Kg2Ounce_ratio           35.274
#define Gram                     1
#define KGram                    2
#define Ounce                    3
#define Pound                    4
#define IC_drift                 0.00000938     //00000938  the best value  
#define STREAM_CLI_CASE          1
#define STREAM_PORT_CASE         2
#define STREAM_BUFFER_CASE       3
#define STREAM_CLI_VERBOSE_CASE  4
#define SAMPLE_CLI_CASE          6
#define SAMPLE_PORT_CASE         7
#define SAMPLE_BUFFER_CASE       8
#define SAMPLE_CLI_VERBOSE_CASE  9
#define IDLE_CASE                0
#define RAW                      5

/*Define private variables*/
uint8_t pulses=0, rate=0, gain=128;
uint16_t full_scale=0;
uint32_t Data=0, value=0;
uint8_t global_ch, global_port, global_module, mode, unit=KGram;
uint32_t global_period, global_timeout;
float weight1_buffer, weight2_buffer; float *ptr_weight_buffer;
float valuef = 0.0f, rawvalue=0.0f;
float calibration_factor=0.0f, Zero_Drift=0.0f;
static float cell_output=0.0;
static float cell_drift=0.00002;
static float weight=0.0f;
float DATA_To_SEND=0.0f;     //float
bool Current_pin_state=0;
float weightGram=0.0f, weightKGram=0.0f, weightOunce=0.0f, weightPound=0.0f;
float Sample[256]={0.0};
TaskHandle_t LoadcellHandle = NULL;
TimerHandle_t xTimer = NULL;
uint8_t startMeasurementRanging = STOP_MEASUREMENT_RANGING;
uint16_t EE_full_scale=0;
uint16_t word_LSB=0, word_MSB=0;
uint32_t temp32=0;


/* Private function prototypes -----------------------------------------------*/	
float readHX711(void);
float weightCalculation(void);
int SendResults(float message, uint8_t Mode, uint8_t unit, uint8_t Port, uint8_t Module, float *Buffer);
void LoadcellTask(void * argument);
void TimerTask(void * argument);
static void CheckForEnterKey(void);
static void HandleTimeout(TimerHandle_t xTimer);
int StreamRawToPort(uint8_t Ch, uint8_t Port, uint8_t Module, uint32_t Period, uint32_t Timeout);

/* Create CLI commands --------------------------------------------------------*/
static portBASE_TYPE demoCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE sampleCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE streamCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE stopCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE unitCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE rateCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE calibrationCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE zerocalCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE weight1ModParamCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE weight2ModParamCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE formatModParamCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* CLI command structure : demo */
const CLI_Command_Definition_t demoCommandDefinition =
{
	( const int8_t * ) "demo", /* The command string to type. */
	( const int8_t * ) "demo:\r\n Run a demo program to test module functionality\r\n\r\n",
	demoCommand, /* The function to run. */
	1 /* one parameter is expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : sample */
const CLI_Command_Definition_t sampleCommandDefinition =
{
  ( const int8_t * ) "sample", /* The command string to type. */
  ( const int8_t * ) "sample:\r\n Take one sample from ch (1 or 2)\r\n\r\n",
  sampleCommand, /* The function to run. */
  1 /* one parameter is expected. */
};

/* CLI command structure : stream */
const CLI_Command_Definition_t streamCommandDefinition =
{
  ( const int8_t * ) "stream", /* The command string to type. */
  ( const int8_t * ) "stream:\r\n Stream from ch (1 or 2) to the CLI, buffer or port with period (ms) and total time (ms). \r\n\r\n",
  streamCommand, /* The function to run. */
  -1 /* Multiparameters are expected. */
};

/* CLI command structure : stop */
const CLI_Command_Definition_t stopCommandDefinition =
{
  ( const int8_t * ) "stop", /* The command string to type. */
  ( const int8_t * ) "stop:\r\n Stop streaming and put HX711 into sleep mode\r\n\r\n",
  stopCommand, /* The function to run. */
  0 /* No parameters are expected. */
};

/* CLI command structure : unit */
const CLI_Command_Definition_t unitCommandDefinition =
{
  ( const int8_t * ) "unit", /* The command string to type. */
  ( const int8_t * ) "unit:\r\n Set the measurement unit (g, kg, ounce, lb)\r\n\r\n",
  unitCommand, /* The function to run. */
  1 /* one parameter is expected. */
};

/* CLI command structure : rate */
const CLI_Command_Definition_t rateCommandDefinition =
{
  ( const int8_t * ) "rate", /* The command string to type. */
  ( const int8_t * ) "rate:\r\n Set HX711 measurement rate in sample per second (10, 80)\r\n\r\n",
  rateCommand, /* The function to run. */
  1 /* one parameter is expected. */
};


/* CLI command structure : calibration */
const CLI_Command_Definition_t calibrationCommandDefinition =
{
  ( const int8_t * ) "calibration", /* The command string to type. */
		( const int8_t * ) "calibration:\r\n Set load cell calibration values: (1st) full scale in Kg, (2nd) cell output in mV, (3rd) cell drift in mV\r\n\r\n",
  calibrationCommand, /* The function to run. */
  3 /* three parameters are expected. */
};

/* CLI command structure : zerocal */
const CLI_Command_Definition_t zerocalCommandDefinition =
{
  ( const int8_t * ) "zerocal", /* The command string to type. */
  ( const int8_t * ) "zerocal:\r\n Choose the channel to make zero calibration for the load cell\r\n\r\n",
  zerocalCommand, /* The function to run. */
  1 /* one parameter is expected. */
};

/*-----------------------------------------------------------*/
/* CLI command structure : weight */
const CLI_Command_Definition_t weight1CommandDefinition =
{
  ( const int8_t * ) "weight1", /* The command string to type. */
		( const int8_t * ) "weight1:\r\nDisplay the value of module parameter: channel_1's weight\r\n\r\n",
  weight1ModParamCommand, /* The function to run. */
  0 /* No parameters are expected. */
};

/*-----------------------------------------------------------*/
/* CLI command structure : weight */
const CLI_Command_Definition_t weight2CommandDefinition =
{
  ( const int8_t * ) "weight2", /* The command string to type. */
		( const int8_t * ) "weight2:\r\nDisplay the value of module parameter: channel_2's weight\r\n\r\n",
  weight2ModParamCommand, /* The function to run. */
  0 /* No parameters are expected. */
};

/*-----------------------------------------------------------*/
/* CLI command structure : weight */
const CLI_Command_Definition_t dataformatCommandDefinition =
{
  ( const int8_t * ) "format", /* The command string to type. */
		( const int8_t * ) " format:\r\nSelect Data format for sending\r\n\r\n",
  formatModParamCommand, /* The function to run. */
  1 /* No parameters are expected. */
};

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
	
	/* HX711 */
	HX711_GPIO_Init();     // GPIO init
	
	/* Creat load cell task */
	xTaskCreate(LoadcellTask, (const char*) "LoadcellTask", (2*configMINIMAL_STACK_SIZE), NULL, osPriorityNormal-osPriorityIdle, &LoadcellHandle);	
	
	/* load saved var*/
	EE_ReadVariable(_EE_cell_full_scale, &full_scale);
	EE_ReadVariable(_EE_cell_drift_LSB, &word_LSB);
	EE_ReadVariable(_EE_cell_drift_MSB, &word_MSB);
	temp32=(uint32_t)word_LSB+((uint32_t)word_MSB<<16);
	cell_drift=*(float*)&temp32;
	EE_ReadVariable(_EE_cell_output_LSB, &word_LSB);
	EE_ReadVariable(_EE_cell_output_MSB, &word_MSB);
	temp32=(uint32_t)word_LSB+((uint32_t)word_MSB<<16);
	cell_output=*(float*)&temp32;
	EE_ReadVariable(_EE_zero_drift_LSB, &word_LSB);
	EE_ReadVariable(_EE_zero_drift_MSB, &word_MSB);
	temp32=(uint32_t)word_LSB+((uint32_t)word_MSB<<16);
	Zero_Drift=*(float*)&temp32;
	calibration_factor=cell_output*AVDD/1000.0f;		// mV

	
}
/*-----------------------------------------------------------*/

/* --- H26R0 message processing task. 
*/
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst, uint8_t shift)
{
	Module_Status result = H26R0_OK;
  uint32_t period = 0;
  uint32_t timeout = 0;
	
	switch (code)
	{
		case (CODE_H26R0_SET_RATE):
			SetHX711Rate(cMessage[port-1][shift]);
			break;
		
		case (CODE_H26R0_STREAM_PORT_GRAM):
			period = ( (uint32_t) cMessage[port-1][1+shift] << 24 ) + ( (uint32_t) cMessage[port-1][2+shift] << 16 ) + ( (uint32_t) cMessage[port-1][3+shift] << 8 ) + cMessage[port-1][4+shift];
			timeout = ( (uint32_t) cMessage[port-1][5+shift] << 24 ) + ( (uint32_t) cMessage[port-1][6+shift] << 16 ) + ( (uint32_t) cMessage[port-1][7+shift] << 8 ) + cMessage[port-1][8+shift];
			StreamGramToPort(cMessage[port-1][shift], cMessage[port-1][9+shift], cMessage[port-1][10+shift], period, timeout);
			break;
		
		case (CODE_H26R0_STREAM_PORT_KGRAM):
			period = ( (uint32_t) cMessage[port-1][1+shift] << 24 ) + ( (uint32_t) cMessage[port-1][2+shift] << 16 ) + ( (uint32_t) cMessage[port-1][3+shift] << 8 ) + cMessage[port-1][4+shift];
			timeout = ( (uint32_t) cMessage[port-1][5+shift] << 24 ) + ( (uint32_t) cMessage[port-1][6+shift] << 16 ) + ( (uint32_t) cMessage[port-1][7+shift] << 8 ) + cMessage[port-1][8+shift];
			StreamKGramToPort(cMessage[port-1][shift], cMessage[port-1][9+shift], cMessage[port-1][10+shift], period, timeout);
			break;
		
    case (CODE_H26R0_STREAM_PORT_OUNCE):
			period = ( (uint32_t) cMessage[port-1][1+shift] << 24 ) + ( (uint32_t) cMessage[port-1][2+shift] << 16 ) + ( (uint32_t) cMessage[port-1][3+shift] << 8 ) + cMessage[port-1][4+shift];
			timeout = ( (uint32_t) cMessage[port-1][5+shift] << 24 ) + ( (uint32_t) cMessage[port-1][6+shift] << 16 ) + ( (uint32_t) cMessage[port-1][7+shift] << 8 ) + cMessage[port-1][8+shift];
			StreamOunceToPort(cMessage[port-1][shift], cMessage[port-1][9+shift], cMessage[port-1][10+shift], period, timeout);
			break;
		
		case (CODE_H26R0_STREAM_PORT_POUND):
			period = ( (uint32_t) cMessage[port-1][1+shift] << 24 ) + ( (uint32_t) cMessage[port-1][2+shift] << 16 ) + ( (uint32_t) cMessage[port-1][3+shift] << 8 ) + cMessage[port-1][4+shift];
			timeout = ( (uint32_t) cMessage[port-1][5+shift] << 24 ) + ( (uint32_t) cMessage[port-1][6+shift] << 16 ) + ( (uint32_t) cMessage[port-1][7+shift] << 8 ) + cMessage[port-1][8+shift];
			StreamPoundToPort(cMessage[port-1][shift], cMessage[port-1][9+shift], cMessage[port-1][10+shift], period, timeout);
			break;
		
		case (CODE_H26R0_STOP):
			mode=IDLE_CASE;
			PowerDown();
			xTimerStop( xTimer, portMAX_DELAY );
			break;
		
		case (CODE_H26R0_SAMPLE_GRAM):
			if (cMessage[port-1][shift] == 1)
				H26R0_Weight1=SampleGram(cMessage[port-1][shift]);
			else
				H26R0_Weight2=SampleGram(cMessage[port-1][shift]);
			break;
			
		case (CODE_H26R0_SAMPLE_KGRAM):
			if (cMessage[port-1][shift] == 1)
				H26R0_Weight1=SampleKGram(cMessage[port-1][shift]);
			else
				H26R0_Weight2=SampleKGram(cMessage[port-1][shift]);	
			break;
			
		case (CODE_H26R0_SAMPLE_OUNCE):
			if (cMessage[port-1][shift] == 1)
				H26R0_Weight1=SampleOunce(cMessage[port-1][shift]);
			else
				H26R0_Weight2=SampleOunce(cMessage[port-1][shift]);	
			break;
			
		case (CODE_H26R0_SAMPLE_POUND):
			if (cMessage[port-1][shift] == 1)
				H26R0_Weight1=SamplePound(cMessage[port-1][shift]);
			else
				H26R0_Weight2=SamplePound(cMessage[port-1][shift]);
			break;
			
		case (CODE_H26R0_ZEROCAL):
				ZeroCal(cMessage[port-1][shift]);
			break;
			
		case (CODE_H26R0_STREAM_RAW):
			period = ( (uint32_t) cMessage[port-1][1+shift] << 24 ) + ( (uint32_t) cMessage[port-1][2+shift] << 16 ) + ( (uint32_t) cMessage[port-1][3+shift] << 8 ) + cMessage[port-1][4+shift];
			timeout = ( (uint32_t) cMessage[port-1][5+shift] << 24 ) + ( (uint32_t) cMessage[port-1][6+shift] << 16 ) + ( (uint32_t) cMessage[port-1][7+shift] << 8 ) + cMessage[port-1][8+shift];
			StreamRawToPort(cMessage[port-1][shift], cMessage[port-1][9+shift], cMessage[port-1][10+shift], period, timeout);
			H26R0_Weight2=Average(cMessage[port-1][shift],1);	
			break;
			
		case (CODE_H26R0_SAMPLE_RAW):
			H26R0_Weight2=Average(cMessage[port-1][shift],1);	
			break;
		
		case (CODE_H26R0_STREAM_FORMAT):
			if (cMessage[port-1][shift] == 0)
				H26R0_DATA_FORMAT = FMT_UINT32;
			else
				H26R0_DATA_FORMAT = FMT_FLOAT;
			break;
			
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
	FreeRTOS_CLIRegisterCommand( &demoCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &sampleCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &streamCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &stopCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &unitCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &rateCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &calibrationCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &zerocalCommandDefinition);
	FreeRTOS_CLIRegisterCommand( &weight1CommandDefinition);
	FreeRTOS_CLIRegisterCommand( &weight2CommandDefinition);
	FreeRTOS_CLIRegisterCommand( &dataformatCommandDefinition);
	
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
float readHX711(void)
{
	uint8_t j=0;
	//wait until HX711 becomes ready
	while(HAL_GPIO_ReadPin(GPIOA,DOUT)==1){	}
	
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
	portEXIT_CRITICAL();
		
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
	return (valuef);		
}
 
/*-----------------------------------------------------------*/

/* --- calculate the weight
*/
float weightCalculation(void)
{
	rawvalue=(valuef*0.5*AVDD)/(ADC_full_range*gain) + cell_drift - IC_drift - Zero_Drift;  //+0.000022;
	weight=(rawvalue*full_scale)/calibration_factor;
	return(weight);	
}
 
/*-----------------------------------------------------------*/

/* --- send results to x
*/
int SendResults(float message, uint8_t Mode, uint8_t Unit, uint8_t Port, uint8_t Module, float *Buffer)
{
	float Raw_Msg=0.0f;
	uint32_t RawMsgInt=0;
  int8_t *pcOutputString;
  static const int8_t *pcWeightMsg = ( int8_t * ) "Weight (%s): %d\r\n";
	static const int8_t *pcWeightVerboseMsg = ( int8_t * ) "%d\r\n";
  char *strUnit;
	static uint8_t temp[4];
  /* specify the unit */
	switch (unit)
	{
		case Gram: 
			Raw_Msg=message*Kg2Gram_ratio; break;
		case KGram:
			Raw_Msg=message; break;
		case Ounce:
			Raw_Msg=message*Kg2Ounce_ratio; break;
		case Pound:
			Raw_Msg=message*Kg2Pound_ratio; break;
		case RAW:
			Raw_Msg=Average(global_ch, 1);
		default:
			Raw_Msg=message; break;
	}

  /* Get CLI output buffer */
  pcOutputString = FreeRTOS_CLIGetOutputBuffer();

	if (mode != STREAM_CLI_VERBOSE_CASE && mode != STREAM_PORT_CASE)
	{
		strUnit = malloc(6*sizeof(char));
		memset(strUnit, 0, (6*sizeof(char)));
		if (unit == Gram)
		{
			sprintf( ( char * ) strUnit, "Gram");
		}
		else if (unit == KGram)
		{
			sprintf( ( char * ) strUnit, "Kg");
		}
		else if (unit == Ounce)
		{
			sprintf( ( char * ) strUnit, "Ounce");
		}
		else if (unit == Pound)
		{
			sprintf( ( char * ) strUnit, "Pound");
		}
		else if (unit == RAW)
		{
			sprintf( ( char * ) strUnit, "Raw Data");
			
		}
		else
		{
			sprintf( ( char * ) strUnit, "Kg");
		}
	}


	// Send the value to appropriate outlet
  switch(mode)
  {
    case SAMPLE_CLI_CASE:
    case STREAM_CLI_CASE:
      sprintf( ( char * ) pcOutputString, ( char * ) pcWeightMsg, strUnit, Raw_Msg);
      writePxMutex(PcPort, (char *)pcOutputString, strlen((char *)pcOutputString), cmd500ms, HAL_MAX_DELAY);
			CheckForEnterKey();
      break;

    case SAMPLE_CLI_VERBOSE_CASE:
    case STREAM_CLI_VERBOSE_CASE:
      sprintf( ( char * ) pcOutputString, ( char * ) pcWeightVerboseMsg, Raw_Msg);
      writePxMutex(PcPort, (char *)pcOutputString, strlen((char *)pcOutputString), cmd500ms, HAL_MAX_DELAY);
			CheckForEnterKey();
      break;
		
    case SAMPLE_PORT_CASE:
    case STREAM_PORT_CASE:
			if (H26R0_DATA_FORMAT == FMT_UINT32)
			{
				RawMsgInt=Raw_Msg*10;
				if (Module==myID){
						temp[0] = *((__IO uint8_t *)(&RawMsgInt)+3);
						temp[1] = *((__IO uint8_t *)(&RawMsgInt)+2);
						temp[2] = *((__IO uint8_t *)(&RawMsgInt)+1);
						temp[3] = *((__IO uint8_t *)(&RawMsgInt)+0);
						writePxMutex(Port, (char *)&temp, 4*sizeof(uint8_t), 10, 10);
				}
				else{
						messageParams[0] = Port;
					  messageParams[1] = *((__IO uint8_t *)(&RawMsgInt)+3);
						messageParams[2] = *((__IO uint8_t *)(&RawMsgInt)+2);
						messageParams[3] = *((__IO uint8_t *)(&RawMsgInt)+1);
						messageParams[4] = *((__IO uint8_t *)(&RawMsgInt)+0);
						SendMessageToModule(Module, CODE_PORT_FORWARD, sizeof(uint32_t)+1);
				}
			
			}
			else if (H26R0_DATA_FORMAT == FMT_FLOAT)
			{
				
						temp[0] = *((__IO uint8_t *)(&Raw_Msg)+3);
						temp[1] = *((__IO uint8_t *)(&Raw_Msg)+2);
						temp[2] = *((__IO uint8_t *)(&Raw_Msg)+1);
						temp[3] = *((__IO uint8_t *)(&Raw_Msg)+0);
						writePxMutex(Port, (char *)&temp, 4*sizeof(uint8_t), 10, 10);
				}
				else{
						messageParams[0] = Port;
					  messageParams[1] = *((__IO uint8_t *)(&Raw_Msg)+3);
						messageParams[2] = *((__IO uint8_t *)(&Raw_Msg)+2);
						messageParams[3] = *((__IO uint8_t *)(&Raw_Msg)+1);
						messageParams[4] = *((__IO uint8_t *)(&Raw_Msg)+0);
						SendMessageToModule(Module, CODE_PORT_FORWARD, sizeof(float)+1);
			}

      break;
		
    case SAMPLE_BUFFER_CASE:
		case STREAM_BUFFER_CASE:
      memset(Buffer, 0, sizeof(float));
      memcpy(Buffer, &Raw_Msg, sizeof(float));
      break;
	}	
	if (mode != STREAM_CLI_VERBOSE_CASE && mode != STREAM_PORT_CASE){
		free(strUnit);
	}
	return (H26R0_OK);
}

/*-----------------------------------------------------------*/

/* ---  Check for CLI stop key
*/
static void CheckForEnterKey(void)
{
	// Look for ENTER key to stop the stream
	for (uint8_t chr=0 ; chr<MSG_RX_BUF_SIZE ; chr++)
	{
		if (UARTRxBuf[PcPort-1][chr] == '\r') {
			UARTRxBuf[PcPort-1][chr] = 0;
			startMeasurementRanging = STOP_MEASUREMENT_RANGING;
			mode = IDLE_CASE;		                // Stop the streaming task
			xTimerStop( xTimer, 0 );            // Stop the timeout timer
			break;
		}
	}
}


/*-----------------------------------------------------------*/

/* --- load cell stream task
*/
void LoadcellTask(void * argument)
{
	uint32_t t0=0;
	while(1)
	{
		switch(mode)
		{
			case STREAM_CLI_CASE:
				t0=HAL_GetTick();
				DATA_To_SEND=SampleKGram(global_ch);		
				SendResults(DATA_To_SEND, mode, unit, 0, 0, NULL);
				while(HAL_GetTick()-t0<(global_period-1)) {taskYIELD();}
				break;
				
			case STREAM_CLI_VERBOSE_CASE:
				t0=HAL_GetTick();
				DATA_To_SEND=SampleKGram(global_ch);	
				SendResults(DATA_To_SEND, mode, unit, 0, 0, NULL);
				while(HAL_GetTick()-t0<global_period) {taskYIELD();}
				break;
				
			case STREAM_PORT_CASE:
				t0=HAL_GetTick();
				DATA_To_SEND=SampleKGram(global_ch);	
				SendResults(DATA_To_SEND, mode, unit, global_port, global_module, NULL);
				while(HAL_GetTick()-t0<global_period) {taskYIELD();}
				break;
				
			case STREAM_BUFFER_CASE: 
				t0=HAL_GetTick();
				DATA_To_SEND=SampleKGram(global_ch);	
				SendResults(DATA_To_SEND, unit, mode, 0, 0, ptr_weight_buffer);
				while(HAL_GetTick()-t0<global_period) {taskYIELD();}
				break;
				
			default: mode = IDLE_CASE; break;
		}
		
		taskYIELD();
	}
}

/*-----------------------------------------------------------*/

/* --- software timer callback function
*/
static void HandleTimeout(TimerHandle_t xTimer)
{
  uint32_t tid = 0;

  /* Get Timer ID */
  tid = ( uint32_t ) pvTimerGetTimerID( xTimer );
  if (TIMERID_TIMEOUT_MEASUREMENT == tid)
  {
		mode = IDLE_CASE;		                                    // Stop the streaming task
		startMeasurementRanging = STOP_MEASUREMENT_RANGING;     // stop streaming
  }
}

/* -----------------------------------------------------------------------
	|																APIs	 																 	|
   ----------------------------------------------------------------------- 
*/

/* --- set HX711 Rate sample per second
*/
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

/* --- Set the gain factor of the used channel
*/
void SetHX711Gain(uint8_t ch)
{
	//determine the chanel and the gain factor
	switch(ch)
	{
		case(1): pulses=25;	gain=128; break;  //Chanel A, Gain factor 128
		case(2): pulses=27; gain=32;	break;  //Chanel B, Gain factor 32
		default: pulses=25;
	}
}

/*-----------------------------------------------------------*/

/* --- enter the calibration values of the load cell
*/
float Calibration(uint16_t Full_Scale,float Cell_Output,float Cell_Drift)
{
	cell_output=Cell_Output;
	full_scale=Full_Scale;
	cell_drift=Cell_Drift/1000.0f;
	calibration_factor=cell_output*AVDD/1000.0f;		// mV
	EE_WriteVariable(_EE_cell_full_scale, full_scale);
	word_LSB=*(uint16_t*)&cell_drift;
	word_MSB=*(((uint16_t*)&cell_drift)+1);
	EE_WriteVariable(_EE_cell_drift_LSB, word_LSB);
	EE_WriteVariable(_EE_cell_drift_MSB, word_MSB);
	word_LSB=*(uint16_t*)&cell_output;
	word_MSB=*(((uint16_t*)&cell_output)+1);
	EE_WriteVariable(_EE_cell_output_LSB, word_LSB);
	EE_WriteVariable(_EE_cell_output_MSB, word_MSB);
	return H26R0_OK;
}

/*-----------------------------------------------------------*/

/* --- read weight value from channel ch of HX711 and return weight in Gram
*/
float SampleGram(uint8_t ch)
{
	uint8_t i=0;
	SetHX711Gain(ch);
	PowerOn();
	for(i=0; i<2; i++) 		readHX711();
	weightGram=weightCalculation()*Kg2Gram_ratio;
	return(weightGram);
}

/*-----------------------------------------------------------*/

/* --- read weight value from channel ch of HX711 and return weight in KGram
*/
float SampleKGram(uint8_t ch)
{
	uint8_t i=0;
	SetHX711Gain(ch);
	PowerOn();
	for(i=0; i<2; i++) 		readHX711();
	weightKGram=weightCalculation();
	return(weightKGram);
}

/*-----------------------------------------------------------*/

/* --- read weight value from channel ch of HX711 and return weight in Ounce
*/
float SampleOunce(uint8_t ch)
{
	uint8_t i=0;
	SetHX711Gain(ch);
	PowerOn();
	for(i=0; i<2; i++)		readHX711();
	weightOunce=weightCalculation()*Kg2Ounce_ratio;
	return(weightOunce);
}

/*-----------------------------------------------------------*/

/* --- read weight value from channel ch of HX711 and return weight in Pound
*/
float SamplePound(uint8_t ch)
{
	uint8_t i=0;
	SetHX711Gain(ch);
	PowerOn();
	for(i=0; i<2; i++) 		readHX711();
	weightPound=weightCalculation()*Kg2Pound_ratio;  
	return(weightPound);
}

/*-----------------------------------------------------------*/

/* --- stream weightvalue from channel ch to port
*/
int StreamGramToPort(uint8_t Ch, uint8_t Port, uint8_t Module, uint32_t Period, uint32_t Timeout)
{
	global_ch=Ch;
	global_port=Port;
	global_module=Module;
	global_period=Period;
	global_timeout=Timeout;
	mode=STREAM_PORT_CASE;
	unit=Gram;
	if ((global_timeout > 0) && (global_timeout < 0xFFFFFFFF))
  {
	  /* start software timer which will create event timeout */
		/* Create a timeout timer */
		xTimer = xTimerCreate( "Timeout Measurement", pdMS_TO_TICKS(global_timeout), pdFALSE, ( void * ) TIMERID_TIMEOUT_MEASUREMENT, HandleTimeout );
		/* Start the timeout timer */
		xTimerStart( xTimer, portMAX_DELAY );
	}
	return (H26R0_OK);
}	

/*-----------------------------------------------------------*/

/* --- stream weightvalue from channel ch to port
*/
int StreamKGramToPort(uint8_t Ch, uint8_t Port, uint8_t Module, uint32_t Period, uint32_t Timeout)
{
	global_ch=Ch;
	global_port=Port;
	global_module=Module;
	global_period=Period;
	global_timeout=Timeout;
	mode=STREAM_PORT_CASE;
	unit=KGram;
	if ((global_timeout > 0) && (global_timeout < 0xFFFFFFFF))
  {
	  /* start software timer which will create event timeout */
		/* Create a timeout timer */
		xTimer = xTimerCreate( "Measurement Timeout", pdMS_TO_TICKS(global_timeout), pdFALSE, ( void * ) TIMERID_TIMEOUT_MEASUREMENT, HandleTimeout );
		/* Start the timeout timer */
		xTimerStart( xTimer, portMAX_DELAY );
	}
	return (H26R0_OK);
}

/*-----------------------------------------------------------*/

/* --- stream weightvalue from channel ch to port
*/
int StreamOunceToPort(uint8_t Ch, uint8_t Port, uint8_t Module, uint32_t Period, uint32_t Timeout)
{
	global_ch=Ch;
	global_port=Port;
	global_module=Module;
	global_period=Period;
	global_timeout=Timeout;
	mode=STREAM_PORT_CASE;
	unit=Ounce;
	if ((global_timeout > 0) && (global_timeout < 0xFFFFFFFF))
  {
	  /* start software timer which will create event timeout */
		/* Create a timeout timer */
		xTimer = xTimerCreate( "Measurement Timeout", pdMS_TO_TICKS(global_timeout), pdFALSE, ( void * ) TIMERID_TIMEOUT_MEASUREMENT, HandleTimeout );
		/* Start the timeout timer */
		xTimerStart( xTimer, portMAX_DELAY );
	}
	return (H26R0_OK);
}

/*-----------------------------------------------------------*/

/* --- stream weightvalue from channel ch to port
*/
int StreamPoundToPort(uint8_t Ch, uint8_t Port, uint8_t Module, uint32_t Period, uint32_t Timeout)
{
	global_ch=Ch;
	global_port=Port;
	global_module=Module;
	global_period=Period;
	global_timeout=Timeout;
	mode=STREAM_PORT_CASE;
	unit=Pound;
	if ((global_timeout > 0) && (global_timeout < 0xFFFFFFFF))
  {
	  /* start software timer which will create event timeout */
		/* Create a timeout timer */
		xTimer = xTimerCreate( "Measurement Timeout", pdMS_TO_TICKS(global_timeout), pdFALSE, ( void * ) TIMERID_TIMEOUT_MEASUREMENT, HandleTimeout );
		/* Start the timeout timer */
		xTimerStart( xTimer, portMAX_DELAY );
	}
	return (H26R0_OK);
}

/*-----------------------------------------------------------*/

/* --- stream weightvalue from channel ch to CLI
*/
int StreamKGramToCLI(uint8_t Ch, uint32_t Period, uint32_t Timeout)
{
	global_ch=Ch;
	global_period=Period;
	global_timeout=Timeout;
	mode=STREAM_CLI_CASE;
	if ((global_timeout > 0) && (global_timeout < 0xFFFFFFFF))
  {
		/* start software timer which will create event timeout */
		/* Create a timeout timer */
		xTimer = xTimerCreate( "Measurement Timeout", pdMS_TO_TICKS(global_timeout), pdFALSE, ( void * ) TIMERID_TIMEOUT_MEASUREMENT, HandleTimeout );
		/* Start the timeout timer */
		xTimerStart( xTimer, portMAX_DELAY );
	}
	if (global_timeout > 0)
	{
		startMeasurementRanging = START_MEASUREMENT_RANGING;
	}
	
	return (H26R0_OK);
}


/*-----------------------------------------------------------*/

/* --- stream weightvalue from channel ch to CLI
*/
int StreamKGramToVERBOSE(uint8_t Ch, uint32_t Period, uint32_t Timeout)
{
	global_ch=Ch;
	global_period=Period;
	global_timeout=Timeout;
	mode=STREAM_CLI_VERBOSE_CASE;
	if ((global_timeout > 0) && (global_timeout < 0xFFFFFFFF))
  {
	  /* start software timer which will create event timeout */
		/* Create a timeout timer */
		xTimer = xTimerCreate( "Measurement Timeout", pdMS_TO_TICKS(global_timeout), pdFALSE, ( void * ) TIMERID_TIMEOUT_MEASUREMENT, HandleTimeout );
		/* Start the timeout timer */
		xTimerStart( xTimer, portMAX_DELAY );
	}
	if (global_timeout > 0)
	{
		startMeasurementRanging = START_MEASUREMENT_RANGING;
	}
	return (H26R0_OK);
}

/* --- stream raw value from channel ch to Port
*/
int StreamRawToPort(uint8_t Ch, uint8_t Port, uint8_t Module, uint32_t Period, uint32_t Timeout)
{
	global_ch=Ch;
	global_period=Period;
	global_timeout=Timeout;
	mode=STREAM_PORT_CASE;
	unit=RAW;
	if ((global_timeout > 0) && (global_timeout < 0xFFFFFFFF))
  {
	  /* start software timer which will create event timeout */
		/* Create a timeout timer */
		xTimer = xTimerCreate( "Measurement Timeout", pdMS_TO_TICKS(global_timeout), pdFALSE, ( void * ) TIMERID_TIMEOUT_MEASUREMENT, HandleTimeout );
		/* Start the timeout timer */
		xTimerStart( xTimer, portMAX_DELAY );
	}
	if (global_timeout > 0)
	{
		startMeasurementRanging = START_MEASUREMENT_RANGING;
	}
	return (H26R0_OK);
}

/*-----------------------------------------------------------*/

/* --- stream weightvalue from channel ch to buffer
*/
int StreamGramToBuffer(uint8_t Ch, float *Buffer, uint32_t Period, uint32_t Timeout)
{
	global_ch=Ch;
	global_period=Period;
	global_timeout=Timeout;
	ptr_weight_buffer=Buffer;
	mode=STREAM_BUFFER_CASE;
	unit=Gram;
	if ((global_timeout > 0) && (global_timeout < 0xFFFFFFFF))
  {
	  /* start software timer which will create event timeout */
		/* Create a timeout timer */
		xTimer = xTimerCreate( "Measurement Timeout", pdMS_TO_TICKS(global_timeout), pdFALSE, ( void * ) TIMERID_TIMEOUT_MEASUREMENT, HandleTimeout );
		/* Start the timeout timer */
		xTimerStart( xTimer, portMAX_DELAY );
	}
	
	return (H26R0_OK);
}

/*-----------------------------------------------------------*/

/* --- stream weightvalue from channel ch to buffer
*/
int StreamKGramToBuffer(uint8_t Ch, float *Buffer, uint32_t Period, uint32_t Timeout)
{
	global_ch=Ch;
	global_period=Period;
	global_timeout=Timeout;
	ptr_weight_buffer=Buffer;
	mode=STREAM_BUFFER_CASE;
	unit=KGram;
	if ((global_timeout > 0) && (global_timeout < 0xFFFFFFFF))
  {
	  /* start software timer which will create event timeout */
		/* Create a timeout timer */
		xTimer = xTimerCreate( "Measurement Timeout", pdMS_TO_TICKS(global_timeout), pdFALSE, ( void * ) TIMERID_TIMEOUT_MEASUREMENT, HandleTimeout );
		/* Start the timeout timer */
		xTimerStart( xTimer, portMAX_DELAY );
	}
	
	return (H26R0_OK);
}

/*-----------------------------------------------------------*/

/* --- stream weightvalue from channel ch to buffer
*/
int StreamOunceToBuffer(uint8_t Ch, float *Buffer, uint32_t Period, uint32_t Timeout)
{
	global_ch=Ch;
	global_period=Period;
	global_timeout=Timeout;
	ptr_weight_buffer=Buffer;
	mode=STREAM_BUFFER_CASE;
	unit=Ounce;
	if ((global_timeout > 0) && (global_timeout < 0xFFFFFFFF))
  {
	  /* start software timer which will create event timeout */
		/* Create a timeout timer */
		xTimer = xTimerCreate( "Measurement Timeout", pdMS_TO_TICKS(global_timeout), pdFALSE, ( void * ) TIMERID_TIMEOUT_MEASUREMENT, HandleTimeout );
		/* Start the timeout timer */
		xTimerStart( xTimer, portMAX_DELAY );
	}
	
	return (H26R0_OK);
}

/*-----------------------------------------------------------*/

/* --- stream weightvalue from channel ch to buffer
*/
int StreamPoundToBuffer(uint8_t Ch, float *Buffer, uint32_t Period, uint32_t Timeout)
{
	global_ch=Ch;
	global_period=Period;
	global_timeout=Timeout;
	ptr_weight_buffer=Buffer;
	mode=STREAM_BUFFER_CASE;
	unit=Pound;
	if ((global_timeout > 0) && (global_timeout < 0xFFFFFFFF))
  {
	  /* start software timer which will create event timeout */
		/* Create a timeout timer */
		xTimer = xTimerCreate( "Measurement Timeout", pdMS_TO_TICKS(global_timeout), pdFALSE, ( void * ) TIMERID_TIMEOUT_MEASUREMENT, HandleTimeout );
		/* Start the timeout timer */
		xTimerStart( xTimer, portMAX_DELAY );
	}
	
	return (H26R0_OK);
}

/*-----------------------------------------------------------*/

/* --- take the average of samples from channel ch
*/
float Average(uint8_t ch, uint8_t samples)
{
	uint8_t N=samples;
	uint8_t i=0;
	uint8_t ii=0;
	float average=0.0;
	PowerOn();
	SetHX711Gain(ch);
	for(i=0; i<=N; i++)
	{
		readHX711();	
		if (i>=1)
		{
			Sample[i]=valuef;
		}
	}
	for (ii=0; ii<N; ii++)
	{
		average+=Sample[ii+1];
	}
	average/=N;
	return(average);
}

/*-----------------------------------------------------------*/

/* --- Perform zero weight calibration on the load cell
*/
int ZeroCal(uint8_t Ch)
{
	uint8_t ch;
	ch=Ch;
	IND_ON();
	SetHX711Rate(80);
	Zero_Drift=(Average(ch,100)*0.5*AVDD)/(ADC_full_range*gain);
	temp32=*(uint32_t*)&Zero_Drift;
	SetHX711Rate(10);
	word_LSB=0x0000FFFF & temp32;
	word_MSB=0xFFFF0000 & temp32;
	EE_WriteVariable(_EE_cell_output_MSB, word_LSB);
	EE_WriteVariable(_EE_cell_output_MSB, word_MSB);
	IND_OFF();
	
	return (H26R0_OK);
}

/*-----------------------------------------------------------*/

/* --- HX711 Power Down
*/
int Stop(void)
{
	mode=IDLE_CASE;
  PowerDown();
	xTimerStop( xTimer, 0 );
	weight1_buffer=0;
	weight2_buffer=0;
  return H26R0_OK;	
}

/*-----------------------------------------------------------*/

/* --- HX711 Power Down
*/
int PowerDown(void)
{
	//make PD_SCK pin high
	HAL_GPIO_WritePin(GPIOA,PD_SCK,GPIO_PIN_SET);
	
	return (H26R0_OK);
}

/*-----------------------------------------------------------*/

/* --- HX711 Power On
*/
int PowerOn(void)
{
	//make PD_SCK pin high
	HAL_GPIO_WritePin(GPIOA,PD_SCK,GPIO_PIN_RESET);
	
	return (H26R0_OK);
}

/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
	|															Commands																 	|
   ----------------------------------------------------------------------- 
*/

portBASE_TYPE demoCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	static const int8_t *pcMessage = ( int8_t * ) "Streaming weight measurements at 2 Hz for 10 seconds\r\n";
	static const int8_t *pcMessageError = ( int8_t * ) "Wrong parameter\r\n";
	int8_t *pcParameterString1; /* ch */
	portBASE_TYPE xParameterStringLength1 = 0;
	uint8_t channel = 0;
	Module_Status result = H26R0_OK;
	
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );
	
	  /* Obtain the 1st parameter string: channel */
  pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);

  if (NULL != pcParameterString1)
  {
    channel = atoi( (char *)pcParameterString1);
  }
  else
  {
    result = H26R0_ERR_WrongParams;
  }
	
	/* Respond to the command */
		if (channel == 1 || channel == 2)
	{
			strcpy(( char * ) pcWriteBuffer, ( char * ) pcMessage);
		writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
		StreamKGramToCLI(channel, 500, 10000);
		/* Wait till the end of stream */
		while(startMeasurementRanging != STOP_MEASUREMENT_RANGING){ Delay_ms(1); };
	}
	
	if (result != H26R0_OK || channel != 1 || channel != 2){
		strcpy(( char * ) pcWriteBuffer, ( char * ) pcMessageError);
	}

	/* clean terminal output */
	memset((char *) pcWriteBuffer, 0, strlen((char *)pcWriteBuffer));
			
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE sampleCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	static const int8_t *pcMessageError = ( int8_t * ) "Wrong parameter\r\n";
	int8_t *pcParameterString1; /* ch */
	portBASE_TYPE xParameterStringLength1 = 0;
	uint8_t channel = 0;
	Module_Status result = H26R0_OK;
	
	/* Remove compile time warnings about unused parameters, and check the
  write buffer is not NULL.  NOTE - for simplicity, this example assumes the
  write buffer length is adequate, so does not check for buffer overflows. */
  ( void ) pcCommandString;
  ( void ) xWriteBufferLen;
  configASSERT( pcWriteBuffer );
	
	  /* Obtain the 1st parameter string: channel */
  pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);

  if (NULL != pcParameterString1)
  {
    channel = atoi( (char *)pcParameterString1);
  }
  else
  {
    result = H26R0_ERR_WrongParams;
  }
	
	if (channel == 1 || channel == 2)
	{
		DATA_To_SEND=SampleKGram(channel);
		mode=SAMPLE_CLI_CASE;
		SendResults(DATA_To_SEND, mode, unit, 0, 0, NULL);
	}
	
		if (result != H26R0_OK || channel != 1 || channel != 2 )
		strcpy(( char * ) pcWriteBuffer, ( char * ) pcMessageError);
		
  /* clean terminal output */
  memset((char *) pcWriteBuffer, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);

  /* There is no more data to return after this single string, so return pdFALSE. */
  return pdFALSE;	
}

/*-----------------------------------------------------------*/

static portBASE_TYPE streamCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	static const int8_t *pcMessageBuffer = ( int8_t * ) "Streaming measurements to internal buffer. Access in the CLI using module parameters: weight1 or weight2\n\r";
	static const int8_t *pcMessageModule = ( int8_t * ) "Streaming measurements to port P%d in module #%d\n\r";
	static const int8_t *pcMessageCLI = ( int8_t * ) "Streaming measurements to the CLI\n\n\r";
	static const int8_t *pcMessageError = ( int8_t * ) "Wrong parameter\r\n";
	static const int8_t *pcMessageWrongName = ( int8_t * ) "Wrong module name\r\n";
  int8_t *pcParameterString1; /* ch */
	int8_t *pcParameterString2; /* period */
  int8_t *pcParameterString3; /* timeout */
  int8_t *pcParameterString4; /* port or buffer */
  int8_t *pcParameterString5; /* module */
  portBASE_TYPE xParameterStringLength1 = 0;
  portBASE_TYPE xParameterStringLength2 = 0;
  portBASE_TYPE xParameterStringLength3 = 0;
  portBASE_TYPE xParameterStringLength4 = 0;
	portBASE_TYPE xParameterStringLength5 = 0;
  uint32_t period = 0;
  uint32_t timeout = 0;
  uint8_t port = 0;
  uint8_t module = 0;
	uint8_t channel = 1;
  Module_Status result = H26R0_OK;

  /* Remove compile time warnings about unused parameters, and check the
  write buffer is not NULL.  NOTE - for simplicity, this example assumes the
  write buffer length is adequate, so does not check for buffer overflows. */
  ( void ) xWriteBufferLen;
  configASSERT( pcWriteBuffer );

  /* Obtain the 1st parameter string: channel */
  pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);
  /* Obtain the 2nd parameter string: period */
  pcParameterString2 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 2, &xParameterStringLength2);
  /* Obtain the 3rd parameter string: timeout */
  pcParameterString3 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 3, &xParameterStringLength3);
  /* Obtain the 4th parameter string: port */
  pcParameterString4 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 4, &xParameterStringLength4);
	/* Obtain the 5th parameter string: module */
	pcParameterString5 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 5, &xParameterStringLength5);

  if (NULL != pcParameterString1)
  {
    channel = atoi( (char *)pcParameterString1);
  }
  else
  {
    result = H26R0_ERR_WrongParams;
  }
	if (NULL != pcParameterString2)
  {
    period = atoi( (char *)pcParameterString2);
  }
  else
  {
    result = H26R0_ERR_WrongParams;
  }
  if (NULL != pcParameterString3)
  {
    if (!strncmp((const char *)pcParameterString3, "inf", 3))
    {
      timeout = portMAX_DELAY;
    }
    else
    {
      timeout = atoi( (char *)pcParameterString3);
    }
  }
  else
  {
    result = H26R0_ERR_WrongParams;
  }
	/* streaming data to internal buffer (module parameter) */
	if (NULL != pcParameterString4 && !strncmp((const char *)pcParameterString4, "buffer", 6)) 
	{
		strcpy(( char * ) pcWriteBuffer, ( char * ) pcMessageBuffer);
		if (channel==1){
		StreamKGramToBuffer(channel, &weight1_buffer, period, timeout);
		}
		else {
		StreamKGramToBuffer(channel, &weight2_buffer, period, timeout);
		}			
		// Return right away here as we don't want to block the CLI
		return pdFALSE;
	} 
	/* streaming data to port */
	else if (NULL != pcParameterString4 && NULL != pcParameterString5 && pcParameterString4[0] == 'p') 
	{
		port = ( uint8_t ) atol( ( char * ) pcParameterString4+1 );
		module = (uint8_t) GetID((char *)pcParameterString5);
		if (module != (uint8_t) BOS_ERR_WrongName) {
			sprintf( ( char * ) pcWriteBuffer, ( char * ) pcMessageModule, port, module);
			StreamKGramToPort(channel, port, module, period, timeout);
			// Return right away here as we don't want to block the CLI
			return pdFALSE;
		} else {
			strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageWrongName);
		}
	} 
	/* Stream to the CLI */
	else if (NULL == pcParameterString5) 
	{	
		if (NULL != pcParameterString4 && !strncmp((const char *)pcParameterString4, "-v", 2)) {
			StreamKGramToVERBOSE(channel, period, timeout);
		} else {
			if (channel == 1 || channel == 2)
			{
			strcpy(( char * ) pcWriteBuffer, ( char * ) pcMessageCLI);
			writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
			StreamKGramToCLI(channel, period, timeout);
			/* Wait till the end of stream */
			while(startMeasurementRanging != STOP_MEASUREMENT_RANGING){taskYIELD();}
			/* clean terminal output */
			memset((char *) pcWriteBuffer, 0, strlen((char *)pcWriteBuffer));
			}
			else
				 strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageError);
		}		

	}		
	else 
	{
		result = H26R0_ERR_WrongParams;
	}	

  if (H26R0_ERR_WrongParams == result)
  {
    strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageError);
  }
	
  /* There is no more data to return after this single string, so return pdFALSE. */
  return pdFALSE;

}

/*-----------------------------------------------------------*/

static portBASE_TYPE stopCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	
  /* Remove compile time warnings about unused parameters, and check the
  write buffer is not NULL.  NOTE - for simplicity, this example assumes the
  write buffer length is adequate, so does not check for buffer overflows. */
  ( void ) xWriteBufferLen;
  configASSERT( pcWriteBuffer );
	
	Stop();
	
	/* There is no more data to return after this single string, so return pdFALSE. */
  return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE unitCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	Module_Status result = H26R0_OK;
  int8_t *pcParameterString1;
  portBASE_TYPE xParameterStringLength1 = 0;
  static const int8_t *pcMessageWrongParam = ( int8_t * ) "Wrong parameter!\r\n";

  /* Remove compile time warnings about unused parameters, and check the
  write buffer is not NULL.  NOTE - for simplicity, this example assumes the
  write buffer length is adequate, so does not check for buffer overflows. */
  ( void ) xWriteBufferLen;
  configASSERT( pcWriteBuffer );

  /* 1st parameter for naming of uart port: P1 to P6 */
  pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);
  if (!strncmp((const char *)pcParameterString1, "g", 1))
  {
    unit = Gram;
    strcpy( ( char * ) pcWriteBuffer, ( char * ) "Used measurement unit: Gram\r\n" );
  }
  else if (!strncmp((const char *)pcParameterString1, "kg", 2))
  {
    unit = KGram;
    strcpy( ( char * ) pcWriteBuffer, ( char * ) "Used measurement unit: KGram\r\n" );
  }
  else if (!strncmp((const char *)pcParameterString1, "ounce", 5))
  {
    unit = Ounce;
    strcpy( ( char * ) pcWriteBuffer, ( char * ) "Used measurement unit: Ounce\r\n" );
  }
	else if (!strncmp((const char *)pcParameterString1, "lb", 2))
  {
    unit = Pound;
    strcpy( ( char * ) pcWriteBuffer, ( char * ) "Used measurement unit: Pound\r\n" );
  }
	else if (!strncmp((const char *)pcParameterString1, "raw", 3))
  {
    unit = RAW;
    strcpy( ( char * ) pcWriteBuffer, ( char * ) "Used measurement unit: Raw\r\n" );
  }
  else
  {
    result = H26R0_ERR_WrongParams;
  }

  /* Respond to the command */
  if (H26R0_ERR_WrongParams == result)
  {
    strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageWrongParam );
  }

  /* There is no more data to return after this single string, so return pdFALSE. */
  return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE rateCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	//Module_Status result = H26R0_OK;
  int8_t *pcParameterString1;
  portBASE_TYPE xParameterStringLength1 = 0;
  static const int8_t *pcMessageWrongParam = ( int8_t * ) "Wrong parameter!\r\n";

  /* Remove compile time warnings about unused parameters, and check the
  write buffer is not NULL.  NOTE - for simplicity, this example assumes the
  write buffer length is adequate, so does not check for buffer overflows. */
  ( void ) xWriteBufferLen;
  configASSERT( pcWriteBuffer );

  /* 1st parameter for naming of uart port: P1 to P6 */
  pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);
  if (!strncmp((const char *)pcParameterString1, "10", 2))
  {
    rate = 10;      // 10SPS
    strcpy( ( char * ) pcWriteBuffer, ( char * ) "Used measurement rate: 10\r\n" );
  }
  else if (!strncmp((const char *)pcParameterString1, "80", 2))
  {
    rate = 80;      // 80SPS
    strcpy( ( char * ) pcWriteBuffer, ( char * ) "Used measurement rate: 80\r\n" );
  }
	else
	{
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageWrongParam );
	}
	SetHX711Rate(rate);
	return 0;	
}

/*-----------------------------------------------------------*/

static portBASE_TYPE calibrationCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	Module_Status result = H26R0_OK;
  int8_t *pcParameterString1;
	int8_t *pcParameterString2;
	int8_t *pcParameterString3;
  portBASE_TYPE xParameterStringLength1 = 0;
	portBASE_TYPE xParameterStringLength2 = 0;
	portBASE_TYPE xParameterStringLength3 = 0;
  static const int8_t *pcMessage = ( int8_t * ) "Calibrating the load cell parameters!\r\n";
	static const int8_t *pcMessageWrongParam = ( int8_t * ) "Wrong parameter!\r\n";
	uint16_t load_cell_scale=0;
	float load_cell_output=0.0f;
	float load_cell_drift=0.0f;

  /* Remove compile time warnings about unused parameters, and check the
  write buffer is not NULL.  NOTE - for simplicity, this example assumes the
  write buffer length is adequate, so does not check for buffer overflows. */
  ( void ) xWriteBufferLen;
  configASSERT( pcWriteBuffer );

  /* 1st parameter for the full scale of the load cell */
  pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);
	/* 2nd parameter for the output of the load cell */
  pcParameterString2 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 2, &xParameterStringLength2);
	/* 3rd parameter for the drift of the load cell */
  pcParameterString3 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 3, &xParameterStringLength3);
	
  if (NULL != pcParameterString1)
  {
    load_cell_scale = atoi( (char *)pcParameterString1);
  }
  else
  {
    result = H26R0_ERR_WrongParams;
  }
	if (NULL != pcParameterString2)
  {
    load_cell_output = atof( (char *)pcParameterString2);
  }
  else
  {
    result = H26R0_ERR_WrongParams;
  }
  if (NULL != pcParameterString3)
  {
      load_cell_drift = atof( (char *)pcParameterString3);
  }
	else
	{
		result = H26R0_ERR_WrongParams;
	}
	
	  /* Respond to the command */
  if (H26R0_ERR_WrongParams == result)
  {
    strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageWrongParam );
  }
	// execute the the calibation API
	strcpy(( char * ) pcWriteBuffer, ( char * ) pcMessage);
	Calibration(load_cell_scale, load_cell_output, load_cell_drift);
	return H26R0_OK;	
}

/*-----------------------------------------------------------*/

static portBASE_TYPE zerocalCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	//Module_Status result = H26R0_OK;
	uint8_t channel;
  int8_t *pcParameterString1;
  portBASE_TYPE xParameterStringLength1 = 0;
  static const int8_t *pcMessageWrongParam = ( int8_t * ) "Wrong parameter!\r\n";

  /* Remove compile time warnings about unused parameters, and check the
  write buffer is not NULL.  NOTE - for simplicity, this example assumes the
  write buffer length is adequate, so does not check for buffer overflows. */
  ( void ) xWriteBufferLen;
  configASSERT( pcWriteBuffer );

  /* 1st parameter for naming of uart port: P1 to P6 */
  pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);
  if (!strncmp((const char *)pcParameterString1, "1", 1))
  {
    channel=1;
		ZeroCal(channel);
    strcpy( ( char * ) pcWriteBuffer, ( char * ) "Zero calibration for channel 1\r\n" );
  }
  else if (!strncmp((const char *)pcParameterString1, "2", 1))
  {
    channel=2;
		ZeroCal(channel);
    strcpy( ( char * ) pcWriteBuffer, ( char * ) "Zero calibration for channel 2\r\n" );
  }
	else
	{
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageWrongParam );
	}
	
	return 0;	
}

/*-----------------------------------------------------------*/

static portBASE_TYPE weight1ModParamCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
  static const int8_t *pcWeightVerboseMsg = ( int8_t * ) "%.2f\r\n";

  /* Remove compile time warnings about unused parameters, and check the
  write buffer is not NULL.  NOTE - for simplicity, this example assumes the
  write buffer length is adequate, so does not check for buffer overflows. */
  ( void ) xWriteBufferLen;
  configASSERT( pcWriteBuffer );

  sprintf( ( char * ) pcWriteBuffer, ( char * ) pcWeightVerboseMsg, weight1_buffer);

  /* There is no more data to return after this single string, so return pdFALSE. */
  return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE weight2ModParamCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
  static const int8_t *pcWeightVerboseMsg = ( int8_t * ) "%.2f\r\n";

  /* Remove compile time warnings about unused parameters, and check the
  write buffer is not NULL.  NOTE - for simplicity, this example assumes the
  write buffer length is adequate, so does not check for buffer overflows. */
  ( void ) xWriteBufferLen;
  configASSERT( pcWriteBuffer );

  sprintf( ( char * ) pcWriteBuffer, ( char * ) pcWeightVerboseMsg, weight2_buffer);

  /* There is no more data to return after this single string, so return pdFALSE. */
  return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE formatModParamCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{	
	//Module_Status result = H26R0_OK;
  int8_t *pcParameterString1;
  portBASE_TYPE xParameterStringLength1 = 0;
  static const int8_t *pcMessageWrongParam = ( int8_t * ) "Wrong parameter!\r\n";

  /* Remove compile time warnings about unused parameters, and check the
  write buffer is not NULL.  NOTE - for simplicity, this example assumes the
  write buffer length is adequate, so does not check for buffer overflows. */
  ( void ) xWriteBufferLen;
  configASSERT( pcWriteBuffer );

  /* 1st parameter for naming of uart port: P1 to P6 */
  pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);
  if (!strncmp((const char *)pcParameterString1, "0", 1))
  {
    H26R0_DATA_FORMAT = FMT_UINT32;      
    strcpy( ( char * ) pcWriteBuffer, ( char * ) "Used data format: uint\r\n" );
  }
  else if (!strncmp((const char *)pcParameterString1, "1", 1))
  {
    H26R0_DATA_FORMAT = FMT_FLOAT;        
    strcpy( ( char * ) pcWriteBuffer, ( char * ) "Used data format: float\r\n" );
  }
	else
	{
		strcpy( ( char * ) pcWriteBuffer, ( char * ) pcMessageWrongParam );
	}
	SetHX711Rate(rate);
	return pdFALSE;	
}

/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
