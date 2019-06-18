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
float h08r6_range = 0.0f;
module_param_t modParam[NUM_MODULE_PARAMS] = {{.paramPtr=&h08r6_range, .paramFormat=FMT_FLOAT, .paramName="weight"}};

/* Private variables ---------------------------------------------------------*/
/* Define HX711 pins */
#define AVDD                 3 //2.44

/* Define preprocessor variables */
#define ADC_full_range       0x7FFFFF
#define Kg2Gram_ratio        1000
#define Kg2Pound_ratio       2.20462262 
#define Kg2Ounce_ratio       35.274
#define Gram                 1
#define KGram                2
#define Ounce                3
#define Pound                4
#define IC_drift             0.00000938         
#define STREAM_CLI_CASE      1
#define STREAM_PORT_CASE     2
#define STREAM_BUFFER_CASE   3
#define STREAM_VERBOSE_CASE  4
#define SAMPLE_CLI_CASE      6
#define SAMPLE_PORT_CASE     7
#define SAMPLE_BUFFER_CASE   8
#define SAMPLE_VERBOSE_CASE  9
#define IDLE_CASE            0
//#define LoadcellTask

uint8_t pulses=0, rate=0, gain=128;
uint16_t full_scale=0;
uint32_t Data=0, value=0;
uint8_t global_ch, global_port, global_module, mode, unit;
uint32_t global_period, global_timeout;
float weight1_buffer, weight2_buffer; float *ptr_weight_buffer;
float valuef = 0.0f, rawvalue=0.0f;
float calibration_factor=0.0f, Zero_Drift=0.0f;
static float cell_output=0.0;
static float cell_drift=0.00002;
static float weight=0.0f;
float DATA_To_SEND=0.0f;
bool Current_pin_state=0;
float weightGram=0.0f, weightKGram=0.0f, weightOunce=0.0f, weightPound=0.0f;
float w=0.0f;
float Sample[256]={0.0};
TaskHandle_t LoadcellHandle = NULL;
TimerHandle_t xTimer = NULL;
uint8_t startMeasurementRanging = STOP_MEASUREMENT_RANGING;

/* Private function prototypes -----------------------------------------------*/	
float readHX711(void);
float weightCalculation(void);
int SendResults(float message, uint8_t Mode, uint8_t unit, uint8_t Port, uint8_t Module, float *Buffer);
void LoadcellTask(void * argument);
void TimerTask(void * argument);
static void CheckForEnterKey(void);
static void HandleTimeout(TimerHandle_t xTimer);

/* Create CLI commands --------------------------------------------------------*/
static portBASE_TYPE demoCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE sampleCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE streamCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE stopCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE unitCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE rateCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE weight1ModParamCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE weight2ModParamCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* CLI command structure : demo */
const CLI_Command_Definition_t demoCommandDefinition =
{
	( const int8_t * ) "demo", /* The command string to type. */
	( const int8_t * ) "(H26R0) demo:\r\n Run a demo program to test module functionality\r\n\r\n",
	demoCommand, /* The function to run. */
	1 /* one parameter is expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : sample */
const CLI_Command_Definition_t sampleCommandDefinition =
{
  ( const int8_t * ) "sample", /* The command string to type. */
  ( const int8_t * ) "(H26R0) sample:\r\n Take one sample from ch (1 or 2)\r\n\r\n",
  sampleCommand, /* The function to run. */
  1 /* one parameter is expected. */
};

/* CLI command structure : stream */
const CLI_Command_Definition_t streamCommandDefinition =
{
  ( const int8_t * ) "stream", /* The command string to type. */
  ( const int8_t * ) "(H26R0) stream:\r\n Stream from ch (1 or 2) to the CLI with period (ms) and total time (ms). \r\n\r\n",
  streamCommand, /* The function to run. */
  -1 /* Multiparameters are expected. */
};

/* CLI command structure : stop */
const CLI_Command_Definition_t stopCommandDefinition =
{
  ( const int8_t * ) "stop", /* The command string to type. */
  ( const int8_t * ) "(H26R0) stop:\r\n Stop the HX711\r\n\r\n",
  stopCommand, /* The function to run. */
  0 /* No parameters are expected. */
};

/* CLI command structure : unit */
const CLI_Command_Definition_t unitCommandDefinition =
{
  ( const int8_t * ) "unit", /* The command string to type. */
  ( const int8_t * ) "(H26R0) unit:\r\n Set the measurement unit (g, kg, ounce, lb)\r\n\r\n",
  unitCommand, /* The function to run. */
  1 /* one parameter is expected. */
};

/* CLI command structure : rate */
const CLI_Command_Definition_t rateCommandDefinition =
{
  ( const int8_t * ) "rate", /* The command string to type. */
  ( const int8_t * ) "(H26R0) rate:\r\n Set HX711 measurement rate in sample per second (10, 80)\r\n\r\n",
  rateCommand, /* The function to run. */
  1 /* one parameter is expected. */
};

/*-----------------------------------------------------------*/
/* CLI command structure : weight */
const CLI_Command_Definition_t weight1CommandDefinition =
{
  ( const int8_t * ) "weight1", /* The command string to type. */
		( const int8_t * ) "(H26R0) weight1:\r\nDisplay the value of module parameter: channel_1's weight\r\n\r\n",
  weight1ModParamCommand, /* The function to run. */
  0 /* No parameters are expected. */
};

/*-----------------------------------------------------------*/
/* CLI command structure : weight */
const CLI_Command_Definition_t weight2CommandDefinition =
{
  ( const int8_t * ) "weight2", /* The command string to type. */
		( const int8_t * ) "(H26R0) weight2:\r\nDisplay the value of module parameter: channel_2's weight\r\n\r\n",
  weight2ModParamCommand, /* The function to run. */
  0 /* No parameters are expected. */
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
	MX_GPIO_Init();
	
	/* HX711 */
  // GPIO init
	HX711_GPIO_Init();
	
	/* Creat load cell task */
	xTaskCreate(LoadcellTask, (const char*) "LoadcellTask", (2*configMINIMAL_STACK_SIZE), NULL, osPriorityNormal, &LoadcellHandle);	
	
	
}
/*-----------------------------------------------------------*/

/* --- H26R0 message processing task. 
*/
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst)
{
	Module_Status result = H26R0_OK;
  uint32_t period = 0;
  uint32_t timeout = 0;
	//TimerHandle_t xTimer = NULL;
	
	switch (code)
	{
		case (CODE_H26R0_SET_RATE):
			SetHX711Rate(messageParams[0]);
		break;
		case (CODE_H26R0_STREAM_PORT_GRAM):
			period = ( (uint32_t) cMessage[port-1][1] << 24 ) + ( (uint32_t) cMessage[port-1][2] << 16 ) + ( (uint32_t) cMessage[port-1][3] << 8 ) + cMessage[port-1][4];
			timeout = ( (uint32_t) cMessage[port-1][5] << 24 ) + ( (uint32_t) cMessage[port-1][6] << 16 ) + ( (uint32_t) cMessage[port-1][7] << 8 ) + cMessage[port-1][8];
			StreamGramToPort(messageParams[0], period, timeout, messageParams[9], messageParams[10]);
		break;
		case (CODE_H26R0_STREAM_PORT_KGRAM):
			period = ( (uint32_t) cMessage[port-1][1] << 24 ) + ( (uint32_t) cMessage[port-1][2] << 16 ) + ( (uint32_t) cMessage[port-1][3] << 8 ) + cMessage[port-1][4];
			timeout = ( (uint32_t) cMessage[port-1][5] << 24 ) + ( (uint32_t) cMessage[port-1][6] << 16 ) + ( (uint32_t) cMessage[port-1][7] << 8 ) + cMessage[port-1][8];
			StreamKGramToPort(messageParams[0], period, timeout, messageParams[9], messageParams[10]);
		break;
      case (CODE_H26R0_STREAM_PORT_OUNCE):
			period = ( (uint32_t) cMessage[port-1][1] << 24 ) + ( (uint32_t) cMessage[port-1][2] << 16 ) + ( (uint32_t) cMessage[port-1][3] << 8 ) + cMessage[port-1][4];
			timeout = ( (uint32_t) cMessage[port-1][5] << 24 ) + ( (uint32_t) cMessage[port-1][6] << 16 ) + ( (uint32_t) cMessage[port-1][7] << 8 ) + cMessage[port-1][8];
			StreamOunceToPort(messageParams[0], period, timeout, messageParams[9], messageParams[10]);
		break;
		case (CODE_H26R0_STREAM_PORT_POUND):
			period = ( (uint32_t) cMessage[port-1][1] << 24 ) + ( (uint32_t) cMessage[port-1][2] << 16 ) + ( (uint32_t) cMessage[port-1][3] << 8 ) + cMessage[port-1][4];
			timeout = ( (uint32_t) cMessage[port-1][5] << 24 ) + ( (uint32_t) cMessage[port-1][6] << 16 ) + ( (uint32_t) cMessage[port-1][7] << 8 ) + cMessage[port-1][8];
			StreamPoundToPort(messageParams[0], period, timeout, messageParams[9], messageParams[10]);
		break;
		case (CODE_H26R0_STOP):
			mode=IDLE_CASE;
			PowerDown();
			xTimerStop( xTimer, portMAX_DELAY );
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
	FreeRTOS_CLIRegisterCommand( &weight1CommandDefinition);
	FreeRTOS_CLIRegisterCommand( &weight2CommandDefinition);
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

//calculate the weight
float weightCalculation(void)
{
	rawvalue=(valuef*0.5*AVDD)/(ADC_full_range*gain) + cell_drift - IC_drift - Zero_Drift;  //+0.000022;
	weight=(rawvalue*full_scale)/calibration_factor;
	return(weight);	
}
 
/*-----------------------------------------------------------*/

//send results to x
int SendResults(float message, uint8_t Mode, uint8_t Unit, uint8_t Port, uint8_t Module, float *Buffer)
{
	float Raw_Msg=0.0f;
  int8_t *pcOutputString;
	//uint8_t stream_mode;
  static const int8_t *pcWeightMsg = ( int8_t * ) "Weight (%s): %.2f\r\n";
	static const int8_t *pcWeightVerboseMsg = ( int8_t * ) "%.2f\r\n";
  //static const int8_t *pcOutMaxRange = ( int8_t * ) "MAX\r\n";
	//static const int8_t *pcOutTimeout = ( int8_t * ) "TIMEOUT\r\n";
  char *strUnit;
	//char * Msg_To_Send[100];
  //specify the unit
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
		default:
			Raw_Msg=message; break;
	}

  /* Get CLI output buffer */
  pcOutputString = FreeRTOS_CLIGetOutputBuffer();

	if (mode != STREAM_VERBOSE_CASE && mode != STREAM_PORT_CASE)
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
		else
		{
			sprintf( ( char * ) strUnit, "Kg");
			/* nothing to do here */
		}
	}

	// If the value is out of range
  /*if (Raw_Msg >= full_scale)
  {
    switch(mode)
    {
      case REQ_SAMPLE_CLI:
      case REQ_STREAM_PORT_CLI:
        mode = REQ_OUT_RANGE_CLI;
        break;
			case REQ_STREAM_MEMORY:
				break;
      case REQ_SAMPLE_ARR:
      case REQ_STREAM_PORT_ARR:
        mode = REQ_OUT_RANGE_ARR;
        break;
      default:        
        break;
    }
  }*/

	// If measurement timeout occured 
/*	if (tofState == REQ_TIMEOUT)
	{
    switch(request)
    {
      case REQ_SAMPLE_CLI:
      case REQ_STREAM_PORT_CLI:
        request = REQ_TIMEOUT_CLI;
        break;
			case REQ_SAMPLE_VERBOSE_CLI:
			case REQ_STREAM_VERBOSE_PORT_CLI:
				request = REQ_TIMEOUT_VERBOSE_CLI;
				break;
			case REQ_STREAM_MEMORY:
				request = REQ_TIMEOUT_MEMORY;
				break;
      case REQ_SAMPLE_ARR:
      case REQ_STREAM_PORT_ARR:
        request = REQ_TIMEOUT_ARR;
        break;
      default:        
        break;
    }				
	}*/
	
	// Send the value to appropriate outlet
  switch(mode)
  {
    case SAMPLE_CLI_CASE:
    case STREAM_CLI_CASE:
      sprintf( ( char * ) pcOutputString, ( char * ) pcWeightMsg, strUnit, Raw_Msg);
      writePxMutex(PcPort, (char *)pcOutputString, strlen((char *)pcOutputString), cmd500ms, HAL_MAX_DELAY);
			CheckForEnterKey();
      break;

    case SAMPLE_VERBOSE_CASE:
    case STREAM_VERBOSE_CASE:
      sprintf( ( char * ) pcOutputString, ( char * ) pcWeightVerboseMsg, Raw_Msg);
      writePxMutex(PcPort, (char *)pcOutputString, strlen((char *)pcOutputString), cmd500ms, HAL_MAX_DELAY);
			CheckForEnterKey();
      break;
		
    case SAMPLE_PORT_CASE:
    case STREAM_PORT_CASE:
		if (Module==myID){
				writePxITMutex(Port, (char *)&Raw_Msg, sizeof(Raw_Msg), 10);
		}
		else{
			//Raw_Msg=123.4;
		    messageParams[0]=Port;
			  memcpy(&messageParams[1], &Raw_Msg, sizeof(float));
			  SendMessageToModule(Module, CODE_port_forward, sizeof(float)+1);
		}
      break;
		
    case SAMPLE_BUFFER_CASE:
		case STREAM_BUFFER_CASE:
      memset(Buffer, 0, sizeof(float));
      memcpy(Buffer, &Raw_Msg, sizeof(float));
      break;
		/*
    case REQ_OUT_RANGE_CLI:
      strcpy( ( char * ) pcOutputString, ( char * ) pcOutMaxRange);
      writePxMutex(PcPort, (char *)pcOutputString, strlen((char *)pcOutputString), cmd500ms, HAL_MAX_DELAY);
			CheckForEnterKey();
      break;
		
    case REQ_OUT_RANGE_ARR:
      SendMessageFromPort(global_port, myID, global_module, CODE_H08R6_MAX_RANGE, 0);
      break;

    case REQ_TIMEOUT_CLI:
      strcpy( ( char * ) pcOutputString, ( char * ) pcOutTimeout);
      writePxMutex(PcPort, (char *)pcOutputString, strlen((char *)pcOutputString), cmd500ms, HAL_MAX_DELAY);
			CheckForEnterKey();
      break;

    case REQ_TIMEOUT_VERBOSE_CLI:
      sprintf( ( char * ) pcOutputString, ( char * ) pcDistanceVerboseMsg, 0);
      writePxMutex(PcPort, (char *)pcOutputString, strlen((char *)pcOutputString), cmd500ms, HAL_MAX_DELAY);
			CheckForEnterKey();
      break;
		
		case REQ_TIMEOUT_MEMORY:
      memset(buffer, 0, sizeof(float));
      break;

    case REQ_TIMEOUT_ARR:
      SendMessageFromPort(global_port, myID, global_module, CODE_H08R6_TIMEOUT, 0);
      break;
		
    default:
      break;
  */
	}
	
	if (mode != STREAM_VERBOSE_CASE && mode != STREAM_PORT_CASE){
		free(strUnit);
	}
	return (H26R0_OK);
}

/* --- Check for CLI stop key*/

static void CheckForEnterKey(void)
{
  int8_t *pcOutputString;

  pcOutputString = FreeRTOS_CLIGetOutputBuffer();
  readPxMutex(PcPort, (char *)pcOutputString, sizeof(char), cmd500ms, 100);
  if ('\r' == pcOutputString[0])
  {
    startMeasurementRanging = STOP_MEASUREMENT_RANGING;
		mode = IDLE_CASE;		// Stop the streaming task
	  xTimerStop( xTimer, 0 );
  }
}


/*-----------------------------------------------------------*/

//load cell stream task
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
				case STREAM_VERBOSE_CASE:
				t0=HAL_GetTick();
				DATA_To_SEND=SampleKGram(global_ch);	
				while(HAL_GetTick()-t0<global_period) {taskYIELD();}
					SendResults(DATA_To_SEND, mode, unit, 0, 0, NULL);
		  break;
			case STREAM_PORT_CASE:
				t0=HAL_GetTick();
				DATA_To_SEND=SampleKGram(global_ch);	
				while(HAL_GetTick()-t0<global_period) {taskYIELD();}
					SendResults(DATA_To_SEND, mode, unit, global_port, global_module, NULL);
		  break;
			case STREAM_BUFFER_CASE: 
				t0=HAL_GetTick();
				DATA_To_SEND=SampleKGram(global_ch);	
				while(HAL_GetTick()-t0<global_period) {taskYIELD();}
				SendResults(DATA_To_SEND, unit, mode, 0, 0, ptr_weight_buffer); break;
			default: mode = IDLE_CASE; break;
		}
		
		taskYIELD();
	}
}

/*-----------------------------------------------------------*/

//software timer callback function
static void HandleTimeout(TimerHandle_t xTimer)
{
  uint32_t tid = 0;

  /* close DMA stream */
  tid = ( uint32_t ) pvTimerGetTimerID( xTimer );
  if (TIMERID_TIMEOUT_MEASUREMENT == tid)
  {
		mode = IDLE_CASE;		// Stop the streaming task
		startMeasurementRanging = STOP_MEASUREMENT_RANGING;     // stop streaming
  }
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
	return H26R0_OK;
}

/*-----------------------------------------------------------*/

//read weight value from channel ch of HX711 and return weight in Gram
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

//read weight value from channel ch of HX711 and return weight in KGram
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

//read weight value from channel ch of HX711 and return weight in Ounce
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

//read weight value from channel ch of HX711 and return weight in Pound
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

//stream weightvalue from channel ch to port
int StreamGramToPort(uint8_t Ch, uint8_t Port, uint8_t Module, uint32_t Period, uint32_t Timeout)
{
	global_ch=Ch;
	global_port=Port;
	global_module=Module;
	global_period=Period;
	global_timeout=Timeout;
	mode=STREAM_PORT_CASE;
	unit=Gram;
	//TimerHandle_t xTimer = NULL;
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

//stream weightvalue from channel ch to port
int StreamKGramToPort(uint8_t Ch, uint8_t Port, uint8_t Module, uint32_t Period, uint32_t Timeout)
{
	global_ch=Ch;
	global_port=Port;
	global_module=Module;
	global_period=Period;
	global_timeout=Timeout;
	mode=STREAM_PORT_CASE;
	unit=KGram;
		//TimerHandle_t xTimer = NULL;
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

//stream weightvalue from channel ch to port
int StreamOunceToPort(uint8_t Ch, uint8_t Port, uint8_t Module, uint32_t Period, uint32_t Timeout)
{
	global_ch=Ch;
	global_port=Port;
	global_module=Module;
	global_period=Period;
	global_timeout=Timeout;
	mode=STREAM_PORT_CASE;
	unit=Ounce;
	//	TimerHandle_t xTimer = NULL;
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

//stream weightvalue from channel ch to port
int StreamPoundToPort(uint8_t Ch, uint8_t Port, uint8_t Module, uint32_t Period, uint32_t Timeout)
{
	global_ch=Ch;
	global_port=Port;
	global_module=Module;
	global_period=Period;
	global_timeout=Timeout;
	mode=STREAM_PORT_CASE;
	unit=Pound;
		//TimerHandle_t xTimer = NULL;
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

//stream weightvalue from channel ch to CLI
int StreamKGramToCLI(uint8_t Ch, uint32_t Period, uint32_t Timeout)
{
	global_ch=Ch;
	global_period=Period;
	global_timeout=Timeout;
	mode=STREAM_CLI_CASE;
	//unit=KGram;
		//TimerHandle_t xTimer = NULL;
	  if ((global_timeout > 0) && (global_timeout < 0xFFFFFFFF))
  {
	  /* start software timer which will create event timeout */
  /* Create a timeout timer */
  xTimer = xTimerCreate( "Timeout Measurement", pdMS_TO_TICKS(global_timeout), pdFALSE, ( void * ) TIMERID_TIMEOUT_MEASUREMENT, HandleTimeout );
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

//stream weightvalue from channel ch to CLI
int StreamKGramToVERBOSE(uint8_t Ch, uint32_t Period, uint32_t Timeout)
{
	global_ch=Ch;
	global_period=Period;
	global_timeout=Timeout;
	mode=STREAM_VERBOSE_CASE;
	//unit=KGram;
		//TimerHandle_t xTimer = NULL;
	  if ((global_timeout > 0) && (global_timeout < 0xFFFFFFFF))
  {
	  /* start software timer which will create event timeout */
  /* Create a timeout timer */
  xTimer = xTimerCreate( "Timeout Measurement", pdMS_TO_TICKS(global_timeout), pdFALSE, ( void * ) TIMERID_TIMEOUT_MEASUREMENT, HandleTimeout );
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

//stream weightvalue from channel ch to buffer
int StreamGramToBuffer(uint8_t Ch, float *Buffer, uint32_t Period, uint32_t Timeout)
{
	global_ch=Ch;
	global_period=Period;
	global_timeout=Timeout;
	ptr_weight_buffer=Buffer;
	mode=STREAM_BUFFER_CASE;
	unit=Gram;
	//	TimerHandle_t xTimer = NULL;
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

//stream weightvalue from channel ch to buffer
int StreamKGramToBuffer(uint8_t Ch, float *Buffer, uint32_t Period, uint32_t Timeout)
{
	global_ch=Ch;
	global_period=Period;
	global_timeout=Timeout;
	ptr_weight_buffer=Buffer;
	mode=STREAM_BUFFER_CASE;
	unit=KGram;
	//	TimerHandle_t xTimer = NULL;
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

//stream weightvalue from channel ch to buffer
int StreamOunceToBuffer(uint8_t Ch, float *Buffer, uint32_t Period, uint32_t Timeout)
{
	global_ch=Ch;
	global_period=Period;
	global_timeout=Timeout;
	ptr_weight_buffer=Buffer;
	mode=STREAM_BUFFER_CASE;
	unit=Ounce;
	//	TimerHandle_t xTimer = NULL;
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

//stream weightvalue from channel ch to buffer
int StreamPoundToBuffer(uint8_t Ch, float *Buffer, uint32_t Period, uint32_t Timeout)
{
	global_ch=Ch;
	global_period=Period;
	global_timeout=Timeout;
	ptr_weight_buffer=Buffer;
	mode=STREAM_BUFFER_CASE;
	unit=Pound;
	//	TimerHandle_t xTimer = NULL;
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

//take the average of samples from channel ch
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

//Perform zero weight calibration on the load cell
int ZeroCal(uint8_t Ch)
{
uint8_t ch;
	ch=Ch;
	Zero_Drift=(Average(ch,10)*0.5*AVDD)/(ADC_full_range*gain);
	
		return (H26R0_OK);
}

/*-----------------------------------------------------------*/

//HX711 Power Down
int PowerDown(void)
{
//make PD_SCK pin high
	HAL_GPIO_WritePin(GPIOA,PD_SCK,GPIO_PIN_SET);
	
		return (H26R0_OK);
}

/*-----------------------------------------------------------*/

//HX711 Power On
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
	int8_t *pcParameterString1; /* ch */
	portBASE_TYPE xParameterStringLength1 = 0;
	uint8_t channel = 1;
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
	strcpy(( char * ) pcWriteBuffer, ( char * ) pcMessage);
	writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
	StreamKGramToCLI(channel, 500, 10000);
	
	/* Wait till the end of stream */
	while(startMeasurementRanging != STOP_MEASUREMENT_RANGING){};
	/* clean terminal output */
	memset((char *) pcWriteBuffer, 0, strlen((char *)pcWriteBuffer));
			
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static portBASE_TYPE sampleCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	/* Remove compile time warnings about unused parameters, and check the
  write buffer is not NULL.  NOTE - for simplicity, this example assumes the
  write buffer length is adequate, so does not check for buffer overflows. */
  ( void ) pcCommandString;
  ( void ) xWriteBufferLen;
  configASSERT( pcWriteBuffer );
	
	DATA_To_SEND=SampleKGram(global_ch);
	mode=SAMPLE_CLI_CASE;
	SendResults(DATA_To_SEND, mode, unit, 0, 0, NULL);

	//strcpy( ( char * ) pcWriteBuffer, ( char * ) "Weight: \r\n" );
  /* clean terminal output */
  memset((char *) pcWriteBuffer, 0, configCOMMAND_INT_MAX_OUTPUT_SIZE);

  /* There is no more data to return after this single string, so return pdFALSE. */
  return pdFALSE;	
}

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
		//buffer = atoi( (char *)pcParameterString3);
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
			strcpy(( char * ) pcWriteBuffer, ( char * ) pcMessageCLI);
			writePxMutex(PcPort, (char *)pcWriteBuffer, strlen((char *)pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
			StreamKGramToCLI(channel, period, timeout);
		}		
		/* Wait till the end of stream */
		while(startMeasurementRanging != STOP_MEASUREMENT_RANGING){taskYIELD();}
		/* clean terminal output */
		memset((char *) pcWriteBuffer, 0, strlen((char *)pcWriteBuffer));
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

static portBASE_TYPE stopCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
	mode=IDLE_CASE;
  PowerDown();
	xTimerStop( xTimer, 0 );
	weight1_buffer=0;
	weight2_buffer=0;
  return H26R0_OK;	
}

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


/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
