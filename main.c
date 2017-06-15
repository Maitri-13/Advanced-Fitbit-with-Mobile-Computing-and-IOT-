/*
 * File name  :	main.c
 *
 *  Created on: March 25, 2017
 *      Author: Richard Noronha and Maitri Chattopadhyay
 * Description: This is the main file that contains the main code of the Cloud Connected smart wearable system
 * 				The Devices can be switched on or off for testing purposes
 */
#include "main.h"

#if ACMP_TEMP==ON
// Global variables
#define NumberOfSamples 100
int OnTimeCali;
int	periodCali; // Buffer array
int count =0;
int32_t average=0;
int32_t sum=0;
float temperature;
uint16_t array[NumberOfSamples+1];// Buffer array
#endif

#if HEART_SENSOR
int ACMPstatus;
//Heart
uint16_t ACMP_count=0;
/* GPio*/
#define ACMP0_CHANNEL0_PORT gpioPortC
#define ACMP0_CHANNEL0_PIN 0

DMA_CB_TypeDef ADC0_callback;

// Configuration structure for ACMP globally
static ACMP_Init_TypeDef acmpInit =
	  {
	    .fullBias                 = false, /* The lightsensor is slow acting, */
	    .halfBias                 = false,  /* comparator bias current can be set to lowest setting.*/
	    .biasProg                 = 7,   /* Analog comparator will still be fast enough */
	    .interruptOnFallingEdge   = false, /* No comparator interrupt, lesense will issue interrupts. */
	    .interruptOnRisingEdge    = true,
	    .warmTime                 = acmpWarmTime256, /* Not applicable, lesense controls this. */
	    .hysteresisLevel          = acmpHysteresisLevel7, /* Some hysteresis will prevent excessive toggling. */
	    .inactiveValue            = 0, /* Not applicable, lesense controls this. */
	    .lowPowerReferenceEnabled = false, /* Can be enabled for even lower power. */
	    .vddLevel                 = 38,  /* Not applicable, lesense controls this through .acmpThres value. */
	    .enable                   = false  /* Not applicable, lesense controls this. */
	  };

#endif

//Format of the data to be sent
#define SEND_THIS	"\r\ntmr.alarm(1, 5000, 0, function() postThingSpeak(100,26,20) end)\r\n"

#define ESP_WRITE_HEAD	"\r\ntmr.alarm(1, 5000, 0, function() postThingSpeak("
#define ESP_COMMA		','
#define ESP_WRITE_TAIL	") end)\r\n"

uint8_t transmit_circ_buff[300];
uint8_t * circ_buff_wtr;
uint8_t * circ_buff_rd;

typedef enum{
	buffer_empty, buffer_full
}circ_buffer_state_t;

circ_buffer_state_t my_buffer=buffer_empty;								//By default the circular buffer is empty

/*
 * Pin PB9 is TX from the board
 * PB10 is RX from the board
 */

/* Declare some strings */
const char     welcomeString[]  = "Energy Micro RS-232 - Please press a key\n";
const char     overflowString[] = "\n---RX OVERFLOW---\n";
const uint32_t welLen           = sizeof(welcomeString) - 1;
const uint32_t ofsLen           = sizeof(overflowString) - 1;

/* Define termination character */
#define TERMINATION_CHAR    '.'

/* Declare a circular buffer structure to use for Rx and Tx queues */
#define BUFFERSIZE          1024


/* Setup UART1 in async mode for RS232*/
static USART_TypeDef           * uart   = UART1;
static USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;


volatile struct circularBuffer
{
  uint8_t  data[BUFFERSIZE];  /* data buffer */
  uint32_t rdI;               /* read index */
  uint32_t wrI;               /* write index */
  uint32_t pendingBytes;      /* count of how many bytes are not yet handled */
  bool     overflow;          /* buffer overflow indicator */
} rxBuf, txBuf = { {0}, 0, 0, 0, false };

/*****************************************************************************************/
/*
 *  Sleep functions
 */
/***************************************************************************
 * This is the function which blocks the sleep mode to the minmode set
 * Eg. If minmode is EM2, the device cannot go below EM2
 */
/**************************************************************************************/
/*
 * This function is used for unblocking the minimum sleep level that a peripheral needs.
 * This is an atomic function as it should not be interrupted when being executed
 * Input variables: minimumMode
 * Global Variables: sleep_block_counter
 * Output Variables: None required
 ********************************************************************************
 * @section License
 * <b>Copyright 2016 Silicon Laboratories, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/
void blockSleepMode(sleepstate_t minimumMode)
{
	INT_Disable();
	sleep_block_counter[minimumMode]++;
	INT_Enable();
}
/*
 * This is the function which unblocks the sleep mode to the minmode set
 * Eg. If minmode is EM2, the device cannot go below EM2
 */
/**************************************************************************************/
/*
 * This function is used for unblocking the minimum sleep level that a peripheral needs
 * Input variables: minimumMode
 * Global Variables: sleep_block_counter
 * Output Variables: None required
 ********************************************************************************
 * @section License
 * <b>Copyright 2016 Silicon Laboratories, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/
void unblockSleepMode(sleepstate_t minimumMode)
{
	INT_Disable();
	if(sleep_block_counter[minimumMode] > 0)
	{
		sleep_block_counter[minimumMode]--;
	}
	INT_Enable();
}
/***************************************************************************/
/**************************************************************************************/
/*
 * This function contains the sleep routine to put the board to sleep . This routine
 * takes the board to the minimum sleep mode possible
 * Input variables: None required
 * Global Variables: sleep_block_counter
 * Output Variables: None required
 ********************************************************************************
 * @section License
 * <b>Copyright 2016 Silicon Laboratories, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/
void sleep(void)
{
	if (sleep_block_counter[0] > 0)
	{
		return; 						// BlockedeveryingbelowEM0
	}
	else if(sleep_block_counter[1] > 0)
	{
		EMU_EnterEM1();					// BlockedeveryithingbelowEM1, enterEM1
	}
	else if(sleep_block_counter[2] > 0)
	{
		EMU_EnterEM2(true);				// BlockedeverythingbelowEM2, enterEM2
	}
	else if(sleep_block_counter[3] > 0)
	{
		EMU_EnterEM3(true);				// Block every thing below EM3, enterEM3
	}
	else
	{
		EMU_EnterEM3(true);				// Nothingisblocked, enterEM3  as EM4 is dangerous
	}
	return;

}

/***************************************************************************************
 * Circular buffer functions
 ***************************************************************************************/

/**************************************************************************************/
/*
 * This function is used for writing to the transmit circular buffer
 * Input variables: None
 * Global Variables: none
 * Output Variables: None required

 ******************************************************************************/
#if CIRCULAR_BUFFER==ON
void write_to_transmit_circ_bufer(uint8_t data)
{
	INT_Disable();
	*circ_buff_wtr=data;
	my_buffer=buffer_full;
	circ_buff_wtr++;
	INT_Enable();
}
/**************************************************************************************/
/*
 * This function is used for reading out from the transmit circular buffer
 * Two includes are present as the circular buffer either works for the UART or LEUART not both
 * Input variables: None
 * Global Variables: none
 * Output Variables: None required

 ******************************************************************************/
#if UART_ESP==ON
void read_from_transmit_circ_buffer()
{
	INT_Disable();
	UART1->TXDATA = *circ_buff_rd;			//This starts the writing out to UART.
	//This pointer is incremented in the interrupt handler routine
	INT_Enable();
}
#elif LEUART0_ESP==ON
void read_from_transmit_circ_buffer()
{
	INT_Disable();
	LEUART0->TXDATA = *circ_buff_rd;			//This starts the writing out to UART.
	//This pointer is incremented in the interrupt handler routine
	INT_Enable();
}
#endif

/**************************************************************************************/
/*
 *This function is to write data of length to the circular buffer
 * Input variables: None
 * Global Variables: none
 * Output Variables: None required

 ******************************************************************************/
void write_data_to_cb(char *data, uint8_t length)
{
	uint16_t s;
	for(s=0;s<length;s++)
	{
		write_to_transmit_circ_bufer(*data);
		data++;
	}
	/*
	 * For circular buffer, check end condition
	 */

}
/*
 * Miscelleous functions
 */
/**************************************************************************************/
/*Function to reverse the string
 * Input variables: None
 * Global Variables: none
 * Output Variables: None required

 ******************************************************************************/
void reverse(char *str, int len)
{
    int i=0, j=len-1, temp;
    while (i<j)
    {
    	temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}
/**************************************************************************************

 Converts a given integer x to string str[].  d is the number of digits required in output. If d is more than the number
  of digits in x, then 0s are added at the beginning.
 * Input variables: None
 * Global Variables: none
 * Output Variables: None required

 ******************************************************************************/
void intToStr(int theInteger, char the_str[], int d)
{
    int the_index = 0;
    while (theInteger)
    {
    	the_str[the_index++] = (theInteger%10) + '0';
        theInteger = theInteger/10;
    }
    /*If number of digits required is more, then add 0s at the beginning*/
    while (the_index < d)
    	{the_str[the_index] = '0';
    	the_index++;
    	}

    reverse(the_str, the_index);
    the_str[the_index] = '\0';
}
#endif
/*
 * Functions for various setups
 */
#if UART_ESP_FUNCTIONS==ON
/**************************************************************************************/
/*
 * This function is used for setting up the UART
 * Input variables: None
 * Global Variables: none
 * Output Variables: None required

 ******************************************************************************/
void uartSetup(void)
{
	CMU_ClockEnable(cmuClock_USART1, true);
	CMU_ClockEnable(cmuClock_UART1, true);

  /* Enable clock for GPIO module (required for pin configuration) */
  CMU_ClockEnable(cmuClock_GPIO, true);
  /* Configure GPIO pins */
  GPIO_PinModeSet(gpioPortB, 9, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortB, 10, gpioModeInput, 0);


  /* Prepare struct for initializing UART in asynchronous mode*/
  uartInit.enable       = usartDisable;   /* Don't enable UART upon intialization */
  uartInit.refFreq      = 0;              /* Provide information on reference frequency. When set to 0, the reference frequency is */
  uartInit.baudrate     = 9600; /* Baud rate */
  uartInit.oversampling = usartOVS16;     /* Oversampling. Range is 4x, 6x, 8x or 16x */
  uartInit.databits     = usartDatabits8; /* Number of data bits. Range is 4 to 10 */
  uartInit.parity       = usartNoParity;  /* Parity mode */
  uartInit.stopbits     = usartStopbits1; /* Number of stop bits. Range is 0 to 2 */
  uartInit.mvdis        = false;          /* Disable majority voting */
  uartInit.prsRxEnable  = false;          /* Enable USART Rx via Peripheral Reflex System */
  uartInit.prsRxCh      = usartPrsRxCh0;  /* Select PRS channel if enabled */

  /* Initialize USART with uartInit struct */
  USART_InitAsync(uart, &uartInit);

  /* Prepare UART Rx and Tx interrupts */
  USART_IntClear(uart, _UART_IF_MASK);
  USART_IntEnable(uart, UART_IF_RXDATAV);
  NVIC_ClearPendingIRQ(UART1_RX_IRQn);
  NVIC_ClearPendingIRQ(UART1_TX_IRQn);

  UART1->IEN=USART_IEN_TXC|USART_IEN_RXDATAV;

  NVIC_EnableIRQ(UART1_RX_IRQn);
  NVIC_EnableIRQ(UART1_TX_IRQn);

  	 UART1->CTRL= USART_CTRL_LOOPBK;

  /* Enable I/O pins at UART1 location #2 */
  uart->ROUTE = UART_ROUTE_RXPEN | UART_ROUTE_TXPEN | UART_ROUTE_LOCATION_LOC2;


  /* Enable UART */
  USART_Enable(uart, usartEnable);
}

#endif

/**************************************************************************************
*	This function sets up the LEUART
 * Input variables: None
 * Global Variables: none
 * Output Variables: None required

 ******************************************************************************/
#if LEUART0_ESP==ON

  void LEUART0_Setup(void)
 {
	  LEUART_Init_TypeDef my_leuart=
	  {
			  .baudrate=LEU0_BAUD,
			  .databits=LEU0_DATAB,
			  .enable=leuartDisable,
			  .parity=LEU0_PARITY_VALUE,
			  .refFreq=LEU0_REF_FREQ,
			  .stopbits=LEU0_STOPB
	  };
	  CMU_ClockSelectSet(cmuClock_LFB,cmuSelect_LFXO);
	  CMU_ClockEnable(cmuClock_LFB,true);										//Select the clock
	  CMU_ClockEnable(cmuClock_LEUART0,true);										//Select the clock

 	GPIO_PinModeSet(LEUART0_TX_PORT,LEUART0_TX_PIN, gpioModePushPull, 1);			//Initialize the LED0
 	GPIO_PinModeSet(LEUART0_RX_PORT,LEUART0_RX_PIN, gpioModeInputPull, 1);			//Initialize the LED0

 	LEUART_Init(LEUART0,&my_leuart);
 	LEUART0->ROUTE=LEUART_ROUTE_RXPEN|LEUART_ROUTE_TXPEN |LEUART_ROUTE_LOCATION_DEFAULT;
#if LEAURT_LOOPBACK==ON
 	LEUART0->CTRL|=LEUART_CTRL_LOOPBK;
#endif
 	//Interrupts part
 	LEUART0->IFC =0xFFFF ;					//Clear all interrupts
#if LEUART_INTERRUPTS==ON
 	LEUART0->IEN =LEUART_IEN_RXDATAV;
#if LEUART_TX_INTERRUPT==ON
 	LEUART0->IEN |= LEUART_IEN_TXC ;
#endif

 	NVIC_EnableIRQ(LEUART0_IRQn);
#endif

 	LEUART_Enable(LEUART0,leuartEnable);
 	LEUART0->CMD=LEUART_CMD_TXEN;
 }

  /**************************************************************************************/
    /*
     * This is the LEUART0 interrupt handler routine
     * This is an atomic function as it should not be interrupted when being executed
     * Input variables: None required
     * Global Variables: my_buffer, circ_buff_rd
     * Output Variables: None required
     ********************************************************************************/
   void LEUART0_IRQHandler(void)
   {
  	//static char i=42;
  	static uint8_t leuart_recvd;
  	int intFlags=0;
  	INT_Disable();								//Make routine atomic
  	intFlags=LEUART_IntGet(LEUART0);
  	LEUART_IntClear(LEUART0 ,intFlags);			//Clear the interrupts

  	static int loc=0;
  	if(intFlags & _LEUART_IF_TXC_MASK)
  	{

  		circ_buff_rd++;
  		if(circ_buff_wtr==circ_buff_rd)
  					{
  						my_buffer=buffer_empty;
  					}
  		if(my_buffer==buffer_full)
  		{
  			LEUART0->TXDATA = *circ_buff_rd;

  		}


  	}
  	/*if(intFlags & _LEUART_IF_RXDATAV_MASK)
  	{
  		huge_buffer_receive[loc]=LEUART0->RXDATA;
  		loc++;
  	}*/

  	INT_Enable();

   }

 #endif

/*************************************************************************************************/
/*
 * Peripheral function
 */
/*************************************************************************************************/
/*
 * This function initializes the GPIO LEDS
 * Input variables: None required
 * Global Variables: None required
 * Output Variables: None required
 **************************************************************************************************/
void GPIO_LedsInit(void)
{
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_GPIO, true);

  GPIO_PinModeSet(LED0_PORT,LED0_PIN, gpioModePushPull, 0);			//Initialize the LED0
  GPIO_PinModeSet(LED1_PORT,LED1_PIN, gpioModePushPull, 0);			//Initialize the LED1

}
/**************************************************************************************/
/*
 * This function turns on the LED that is specified
 * Input variables: LED_no
 * Global Variables: None required
 * Output Variables: None required
 ********************************************************************************
  ******************************************************************************/
void Turn_on_LED(uint8_t LED_no)
{
	if(LED_no==0)
		GPIO_PinOutSet(LED0_PORT,LED0_PIN);
	else
		GPIO_PinOutSet(LED1_PORT,LED1_PIN);
}

/**************************************************************************************/
/*
 * This function turns off the LED that is specified
 * Input variables: LED_no
 * Global Variables: None required
 * Output Variables: None required
 ********************************************************************************
 ******************************************************************************/
void Turn_off_LED(uint8_t LED_no)
{
	if(LED_no==0)
		GPIO_PinOutClear(LED0_PORT,LED0_PIN);		//LED0 is on portE pin2
	else
		GPIO_PinOutClear(LED1_PORT,LED1_PIN);		//LED1 is on portE pin3
}
/**************************************************************************************/
/*
 * This function toggles the LED that is specified
 * Input variables: LED_no
 * Global Variables: None required
 * Output Variables: None required
 ********************************************************************************/
void Toggle_LED(uint8_t LED_no)
{
	if(LED_no==0)
		GPIO_PinOutToggle(LED0_PORT,LED0_PIN);
	else
		GPIO_PinOutToggle(LED1_PORT,LED1_PIN);
}

/**************************************************************************************/
/*
 * This function is the LESENSE interrupt handler routine
 * Input variables: LED_no
 * Global Variables: None required
 * Output Variables: None required
 ********************************************************************************
  ******************************************************************************/
void LESENSE_IRQHandler( void )
{
  uint8_t channel;
  uint32_t interrupt_flags;

  INT_Disable();
  /* Get interrupt flag */
  		    interrupt_flags = LESENSE_IntGet();
  		    /* Clear interrupt flag */
  		    LESENSE_IntClear(interrupt_flags);

  buttons_pressed=0;		//Reset the which button is pressed variable
  /* Interrupt handles only one channel at a time */
  /* therefore only first active channel found is handled by the interrupt. */
  for(channel = 0; channel < 16/*NUM_LESENSE_CHANNELS*/; channel++){
    if( (interrupt_flags >> channel) & 0x1 ){
      break;
    }
  }

  buttons_pressed |= (1 << channel);

		    /* Get channels that are pressed, result is or'ed together */
  //buttons_pressed = LETOUCH_GetChannelsTouched();		//Runs on LESENSE interrupt. INT IS DIABLED IN THIS IRQHANDLER

		    /* Check if any channels are in the touched state. */
		if(buttons_pressed==256 || buttons_pressed==0){				//First slot
		      /* Turn off LED */
		    	Turn_off_LED(0);
		    	LETIMER_Enable(LETIMER0, false);		//Start the timer

		    }
		    else if (buttons_pressed==512){		//Second slot

		    	LETIMER_Enable(LETIMER0, false);		//Start the timer
		    	ledBrightness=0.019;
		    	LETIMER_Setup_PWM(1.0);
		    }
		    else if (buttons_pressed==1024){		//Third slot
		    	LETIMER_Enable(LETIMER0, false);		//Start the timer
		    	ledBrightness=0.012;
		    	LETIMER_Setup_PWM(1.0);
		    }
		    else /*if(buttons_pressed==2048)*/{		//Fourth slot
		    	LETIMER_Enable(LETIMER0, false);		//Start the timer
		    	ledBrightness=0.004;
		    	LETIMER_Setup_PWM(1.0);
		    }
		    /* Get interrupt flag */
		    interrupt_flags = LESENSE_IntGet();
		    /* Clear interrupt flag */
		    LESENSE_IntClear(interrupt_flags);
INT_Enable();

}
/**************************************************************************************/
/*
 * This function is the Letimer interrupt handler routine
 * Input variables: LED_no
 * Global Variables: None required
 * Output Variables: None required
 ********************************************************************************
  ******************************************************************************/
void LETIMER0_IRQHandler(){
	int flag;
	 INT_Disable();


	  flag = LETIMER_IntGet(LETIMER0);
	  LETIMER_IntClear(LETIMER0, LETIMER_IFC_COMP0 | LETIMER_IFC_COMP1);

	  // the PWM is here. Depending on that, the interrupt will be handled
	 if(flag & LETIMER_IF_COMP0)
	 {
		 Turn_on_LED(0);
		 // Turn_on_LED(1);
		// ACMP0->CTRL &= ~ACMP_CTRL_EN;
		 LETIMER0 ->IFC = flag;
	 }
		 else
		 {
			 Turn_off_LED(0);
			 //ACMP0->CTRL |=ACMP_CTRL_EN;
			// Turn_off_LED(1);
		 LETIMER0->IFC = flag;
		// while(!(ACMP0->STATUS & ACMP_STATUS_ACMPACT));
		 }
	 INT_Enable();

 }

/**************************************************************************************/
/*
 * This function initializes the clock based on sleep mode
 * Input variables: None required
 * Global Variables: sleep_block_counter
 * Output Variables: None required
 ********************************************************************************
  ******************************************************************************/
void Clock_setup(sleepstate_t lowest_energy_mode)
{
	if(lowest_energy_mode==EM3)
			{
				CMU_OscillatorEnable(cmuOsc_ULFRCO,true, true);		//use ULFRCO for EM3
				CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_ULFRCO);
			//	CMU_ClockEnable(cmuClock_HFPER,true);
				CMU_ClockEnable(cmuClock_CORELE, true);		//Supposed to be CORELE. Do a search in the enum part to know

				blockSleepMode(EM3);
			}
		else
			{
				CMU_OscillatorEnable(cmuOsc_LFXO,true, true);		//use ULFRCO for EM3
				CMU_ClockEnable(cmuClock_HFPER,true);
				CMU_OscillatorEnable(cmuOsc_HFRCO,true, true);		//use ULFRCO for EM3
				CMU_OscillatorEnable(cmuOsc_HFXO,true, true);		//use ULFRCO for EM3
				CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_LFXO);
				CMU_ClockSelectSet(cmuClock_HFPER,cmuSelect_HFRCO);
				CMU_ClockEnable(cmuClock_CORELE, true);

				blockSleepMode(lowest_energy_mode);
			}
}

/**************************************************************************************/
/*
 * This function sets up the RTC
 * Input variables: LED_no
 * Global Variables: None required
 * Output Variables: None required
 ********************************************************************************
  ******************************************************************************/
void RTC_Setup()
{
	CMU_ClockEnable(cmuClock_RTC,true);			//LFA is needed
	RTC_Init_TypeDef my_rtc={
			.comp0Top=true,
			.debugRun=false,
			.enable=false
	};
	RTC_Init(&my_rtc);
   // RTC_CounterReset();
	RTC_CompareSet(0,RTC_COMP0);					//in em3: 220ms:256, 12.5ms:16, 3.73s:4300, 5.10s:5800
	RTC_CompareSet(1,300000);					//in em3: 220ms:256, 12.5ms:16, 3.73s:4300, 5.10s:5800
	//Interrupts part
		RTC->IFC=0xFFFF ;					//Clear all interrupts
		RTC->IEN|=RTC_IEN_COMP0;
	#if RTC_COM1==ON
		RTC->IEN|=RTC_IEN_COMP1;
	#endif

	NVIC_EnableIRQ(RTC_IRQn);
	blockSleepMode(EM3);
	RTC_Enable(true);
}
/**************************************************************************************/
/*
 * This function sets up the Lesense module
 * Input variables: LED_no
 * Global Variables: None required
 * Output Variables: None required
 ********************************************************************************
  ******************************************************************************/
void LESENSE_TOUCH_Setup()
{

	/* Four pins active, PC8, PC9, PC10 and PC11. A value of 0.0 indicates inactive channel. */
	float sensitivity[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0};

	/* Init Capacitive touch for channels configured in sensitivity array */
	LETOUCH_Init(sensitivity);

	  /* If any channels are touched while starting, the calibration will not be correct. */
	  /* Wait while channels are touched to be sure we continue in a calibrated state. */
	//while(LETOUCH_GetChannelsTouched() != 0);

}
/**************************************************************************************/
/*
 * This function sets up the pwm for the LETIMEr
 * Input variables: LED_no
 * Global Variables: None required
 * Output Variables: None required
 ********************************************************************************
  ******************************************************************************/
void LETIMER_Setup_PWM(float calib_if_needed)
{
	float ledBrightness=0.010;

	CMU_ClockEnable(cmuClock_LETIMER0,true);

	int comp0_value, comp1_value;
	LETIMER_Init_TypeDef my_letimer;
	my_letimer.enable=false;
	my_letimer.debugRun=false;
	my_letimer.rtcComp0Enable=false;
	my_letimer.rtcComp1Enable=false;
	my_letimer.comp0Top=true;
	my_letimer.bufTop=false;
	my_letimer.out0Pol=0;
	my_letimer.out1Pol=0;
	my_letimer.ufoa0=letimerUFOANone;
	my_letimer.ufoa1=letimerUFOANone;
	my_letimer.repMode=letimerRepeatFree;

	LETIMER_Init(LETIMER0, &my_letimer);

	int Desired_Period;
	int LETIMER0_period=TIME_TO_SENSE_LIGHT;
	int LETIMER0_LFXO_count=LFXO_MAX_CNT;
	int LETIMER0_prescalar,temp,Prescaled_two_power;
	int LETIMER0_Max_Count=LE0_MAX_CNT;							//Max count for the timer is 2^16
	Desired_Period= LETIMER0_period * LETIMER0_LFXO_count;
	LETIMER0_prescalar = 0;
	temp = Desired_Period;
	Prescaled_two_power= 1;

	while (temp > LETIMER0_Max_Count)
	{
		LETIMER0_prescalar++;
		Prescaled_two_power= Prescaled_two_power*2;
		temp = Desired_Period/ Prescaled_two_power;
	}

	if(LETIMER0_prescalar>15)
	{
		//EM_ASSERT();										//Dont let value of prescalar be more than size of the register
	}
	//LETIMER0_prescalar=3;
	CMU->LFAPRESC0=LETIMER0_prescalar << 8;					//Prescalar to the clock. Shifted by 8 according to the cmu.h file

	/*
	 * ADJUST COMP1 and COMP0 based on the prescalar
	 */
	/****************************************************/

	int freq_in_hz,led_time_count,led_time_duty_on;
	freq_in_hz=CMU_ClockFreqGet(cmuClock_LFA);
	led_time_count=((freq_in_hz*TIME_TO_SENSE_LIGHT)/Prescaled_two_power);
	led_time_duty_on=((freq_in_hz*LIGHT_SENSOR_ON_TIME)/Prescaled_two_power);

/*	if(ULFRCO_SELF_CALIBRATE==1&&LOWEST_ENERGY_MODE==EM3)
	{
	comp0_value=2092;
	comp1_value=1;
	}*/
	//else

	{
		comp0_value=led_time_count * calib_if_needed;
		comp1_value=led_time_duty_on * calib_if_needed;
	}

 	LETIMER_CompareSet(LETIMER0, 0, comp0_value);
	LETIMER_CompareSet(LETIMER0, 1, comp1_value);

	//Interrupts part
	LETIMER0->IFC=0xFFFF ;					//Clear all interrupts
	LETIMER0->IEN|=LETIMER_IEN_COMP0;
#if 1
	LETIMER0->IEN|=LETIMER_IEN_COMP1;
#endif

	NVIC_EnableIRQ(LETIMER0_IRQn);
	blockSleepMode(EM2);
	LETIMER_Enable(LETIMER0, true);		//Start the timer
}
/**************************************************************************************/
/*
 * This function Sets up the I2C for both light sensor and the BME280
 * Input variables: LED_no
 * Global Variables: None required
 * Output Variables: None required
 ********************************************************************************
  ******************************************************************************/
void I2C_setup(void)
{
	CMU_ClockEnable(cmuClock_I2C1,true);			//Select the clock
	I2C1->ROUTE=I2C_ROUTE_LOCATION_LOC0;			//Select the location 0 of the sda and scl pins
	I2C1->ROUTE|=I2C_ROUTE_SDAPEN;					//Enable the sda
	I2C1->ROUTE|=I2C_ROUTE_SCLPEN;					//Enable the Scl

	I2C_Init_TypeDef my_i2c1=
	{
		.clhr=I2C_SPEED_MODE,						//Set high speed mode
		.enable=false,								//Dont enable immediately
		.freq=I2C_FREQ_MAX,							//Run at maximum frequency
		.master=true,								//Leopard Gecko is in the master mode
		.refFreq=0
	};

	I2C_Init(I2C1,&my_i2c1);

	//Interrupts part. interrupts are being used only to check status
	I2C1->IFC=0xFFFF ;								//Clear all interrupt flags

	//blockSleepMode(EM1);
	I2C_Enable(I2C1,true);
	if (I2C1->STATE & I2C_STATE_BUSY)			//Exit the busy state.  The I2Cn will be in this state out of RESET
	{
		I2C1->CMD = I2C_CMD_ABORT;
	}
}

/**************************************************************************//**
 *This function Sets up I2C for the Active light sensor. The Leopard Gecko is
 *set in the master mode and at high speed I2C
 *
 * Input variables: none
 * Global Variables: sleep_block_counter, Temperature_samples,temp_offset
 * Output Variables: none
 *****************************************************************************/
#if ACTIVE_LIT_SENSR_TSL2651==ON
/**************************************************************************//**
 *This function is the read driver of the I2C bus based on the requirements for the
 *active light sensor TSL2561
 *
 * Input variables: none
 * Global Variables: none
 * Output Variables: none

 *****************************************************************************/
uint8_t I2C1_read_register(uint8_t device_register_address)
{
	//blockSleepMode(I2C_MIN_SLEEP_BLOCK);
	uint8_t data=0;
	I2C1->TXDATA=(ACTIVE_LS_I2C_ADDRESS<<1)|WRITE_TO_I2C;			//Send slave address and the write bit
	I2C1->CMD=I2C_CMD_START;										//Send start command
	while((I2C1->IF & I2C_IF_ACK) ==0);								//Wait for acknowlegement
	I2C1->IFC=I2C_IFC_ACK;											//Clear ack flag

	I2C1->TXDATA=device_register_address;							//send command code and register address
	while((I2C1->IF & I2C_IF_ACK) ==0);								//Wait for ack
	I2C1->IFC=I2C_IFC_ACK;											//clear ack flag

	I2C1->CMD=I2C_CMD_START;										//Resend start command
	I2C1->TXDATA=(ACTIVE_LS_I2C_ADDRESS<<1)|READ_FROM_I2C;			//Send slave address and the read bit
	while((I2C1->IF & I2C_IF_ACK) ==0);								//Wait for ack
	I2C1->IFC=I2C_IFC_ACK;											//Clear ack flag

	while ((I2C1->IF & I2C_IF_RXDATAV) == 0);						//Wait till data is ready in the receive register
	data=I2C1->RXDATA;												//Store data in a variable

	I2C1->CMD=I2C_CMD_NACK;											//Send NACK from the master
	I2C1->CMD=I2C_CMD_STOP;											//Send stop bit

	//unblockSleepMode(I2C_MIN_SLEEP_BLOCK);
	return data;
}
/**************************************************************************//**
 *This function is the write driver of the I2C bus based on the requirements for the
 *active light sensor TSL2561
 *
 * Input variables: none
 * Global Variables: none
 * Output Variables: none

 *****************************************************************************/

void I2C1_write_register(uint8_t device_register_address,uint8_t data)
{
	//blockSleepMode(I2C_MIN_SLEEP_BLOCK);
	I2C1->TXDATA=(ACTIVE_LS_I2C_ADDRESS<<1)|WRITE_TO_I2C;		//Send slave address and write bit
	I2C1->CMD=I2C_CMD_START;									//Send start command
	while((I2C1->IF & I2C_IF_ACK) ==0);							//Wait for acknowledgement
	I2C1->IFC=I2C_IFC_ACK;										//Clear Acknowlegment flag

	I2C1->TXDATA=device_register_address;						//Write value to the bus
	while((I2C1->IF & I2C_IF_ACK) ==0);							//Wait for acknowledgment
	I2C1->IFC=I2C_IFC_ACK;										//Clear acknowledgement flag

	I2C1->TXDATA=data;											//Transmit data to be written to the device
	while((I2C1->IF & I2C_IF_ACK) ==0);							//Wait for acknowledgement
	I2C1->IFC=I2C_IFC_ACK;										//Clear acknowlegement
	I2C1->CMD=I2C_CMD_STOP;										//Send stop bit
	//unblockSleepMode(I2C_MIN_SLEEP_BLOCK);
}

/**************************************************************************//**
 *This function Sets up the Active sensor TSL2561 using the I2C routines
 *
 * Input variables: none
 * Global Variables: none
 * Output Variables: none

 *****************************************************************************/

 void TSL2561_Setup(void)
{
	I2C1_write_register(ACTIVE_LS_COMMAND|ACTIVE_LS_CONTROL, ACTIVE_LS_POWER_UP);		//Power up the TSL2561
	I2C1_write_register(ACTIVE_LS_COMMAND|ACTIVE_LS_TIMING, ACTIVE_LS_INTEGRATE_TIME | ACTIVE_LS_GAIN );//Write to timing register
	I2C1_write_register(ACTIVE_LS_COMMAND|ACTIVE_LS_THRESHLOWLOW, ACTIVE_LS_TH_LOWLOW_VAL );//Write low threshold Lower byte
	I2C1_write_register(ACTIVE_LS_COMMAND|ACTIVE_LS_THRESHLOWHIGH, ACTIVE_LS_TH_LOWHIGH_VAL);//Write low threshold higher byte
	I2C1_write_register(ACTIVE_LS_COMMAND|ACTIVE_LS_THRESHHIGHLOW, ACTIVE_LS_TH_HIGHLOW_VAL);//Write hugh threshold lower byte
	I2C1_write_register(ACTIVE_LS_COMMAND|ACTIVE_LS_THRESHHIGHHIGH, ACTIVE_LS_TH_HIGHHIGH_VAL);	//Write high threshold higher byte
	I2C1_write_register(ACTIVE_LS_COMMAND|ACTIVE_LS_INTERRUPT, ACTIVE_LS_PERSIST_VAL |ACTIVE_LS_INT_LEVEL);
}
#endif


#if HEART_SENSOR==ON
/***************************************************/
 /* This routine is used to initialize the ACMP */
 /*Input variables:  None
 *Global variables:  None
 *Returned variables: None
 ***************************************************/
void ACMP_Setup()
{
	CMU_ClockEnable(cmuClock_ACMP0, true);
  /* Initialize ACMP */
  ACMP_Init(ACMP0, &acmpInit);
  /* Set up ACMP negSel to VDD, posSel is controlled by LESENSE. */
  ACMP_ChannelSet(ACMP0, acmpChannelVDD, acmpChannel0);
  //Enabled the ACMP
  ACMP_Enable(ACMP0);
  /* Enable interrupts */
      NVIC_ClearPendingIRQ(ACMP0_IRQn);
      NVIC_EnableIRQ(ACMP0_IRQn);
}

uint32_t AComp0Voltage=0;
/**************************************************************************************/
/*
 * This function is ACMP irq handler
 * Input variables: None
 * Global Variables: none
 * Output Variables: None required
 *********************************************************************************/

void ACMP0_IRQHandler()
{
	INT_Disable();
	/* Clear interrupt flag */
	ACMP0 ->IFC = ACMP_IFC_WARMUP;
		ACMP0 ->IFC |= ACMP_IFC_EDGE;
		AComp0Voltage = ((ACMP0->STATUS & ACMP_STATUS_ACMPOUT) >>  _ACMP_STATUS_ACMPOUT_SHIFT);
		if(AComp0Voltage)
        ACMP_count++;
       INT_Enable();
}
#endif

#if ACMP_TEMP==ON
/***************************************************/
/*  This routine is used to convert the temperature into Celsius  */
/*  Input variables: adcSample(It is used to get the temperature into the function as a parameter) */
/*Global variables:  No global variables  */
/*Returned variables: temp(This is the actual value in Celsius that is being returned)   */
/***************************************************/
float convertToCelsius(int32_t adcSample)
 	 {
		float temp;
/* Factory calibration temperature from device information page. */
		float cal_temp_0 = ( float)((DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK)>> _DEVINFO_CAL_TEMP_SHIFT);
		float cal_value_0 = (float)((DEVINFO->ADC0CAL2 & _DEVINFO_ADC0CAL2_TEMP1V25_MASK)>> _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);
/* Temperature gradient( from datasheet ) */
		float t_grad = GRADIENT;
		temp = (cal_temp_0 -((cal_value_0 -adcSample)  / t_grad));
		return temp;
}
/*******************************************************************************
 * This routine is used to handle the action to be taken when the DMA is called (The call back function)
 *Input variables:  None
 *Global variables: array[] (a buffer to store the 750 ADC values),
  * NumberOfSamples (750),sum(Taking the summation of all the ADC values), average(Sum/750), temperature(Variable storing
  * the celsius value of the average),
 *Returned variables: None
 *******************************************************************************/

void DMA_Callback(unsigned int channel, bool primary, void *user){

    INT_Disable();

    ADC0->CMD = ADC_CMD_SINGLESTOP;   // closing the ADC channel since count is done
    unblockSleepMode(1);  // Unblocking since ADC work is over
     		for (int i =0; i<NumberOfSamples; i++)
     		{
     			sum = sum+ array[i];
     		}
     	average = sum/NumberOfSamples;
     	temperature = 100*(convertToCelsius(average));
     	sum =ZERO;
     	INT_Enable();
}

/*******************************************************************************
  *This routine is used to Setup the DMA
 *Input variables:  None
 Global variables: None
 Returned variables: None
 *******************************************************************************/

void DMA_Setup(void){

	CMU_ClockEnable(cmuClock_DMA, true);
    DMA_Init_TypeDef DMA_init;
    DMA_init.controlBlock = dmaControlBlock;
    DMA_init.hprot = 0;
    DMA_Init(&DMA_init);    //passing pointer as the address of the structure itself


// The call back block
    	DMA_CfgChannel_TypeDef DMA_channel_ADC0;
    	ADC0_callback.cbFunc =  DMA_Callback;  // Calling the callback function
    	ADC0_callback.userPtr = NULL;
    	ADC0_callback.primary = true;

 // Setting DMA descriptors
        DMA_CfgDescr_TypeDef DMA_cfg;
        DMA_cfg.arbRate = dmaArbitrate1 ,         //ADC0_DMA_Arbitration;
        DMA_cfg.dstInc = dmaDataInc2;
        DMA_cfg.hprot = 0;
        DMA_cfg.size = dmaDataSize2;
        DMA_cfg.srcInc = dmaDataIncNone;
        DMA_CfgDescr(0 , true, &DMA_cfg);        //ADC0_DMA_Channel

  // Setting  DMA channel
       DMA_channel_ADC0.cb = &ADC0_callback;   // linking to the call back
       DMA_channel_ADC0.enableInt = true;
       DMA_channel_ADC0.highPri = true;
       DMA_channel_ADC0.select = DMAREQ_ADC0_SINGLE;


    DMA_CfgChannel(0, &DMA_channel_ADC0); //ADC0_DMA_Channel

}
/**************************************************************************************/
/*
 * This function is used for setting up the ADC for temperature reads
 * Input variables: None
 * Global Variables: none
 * Output Variables: None required
 *********************************************************************************/


void ADC_Setup(){
	CMU_ClockEnable(cmuClock_ADC0, true);
ADC_Init_TypeDef ADC0Init =
		    {
		    		.ovsRateSel = adcOvsRateSel2, // find default
					.lpfMode= adcLPFilterBypass,                /* 2x oversampling (if enabled). */
					.warmUpMode=adcWarmupNormal,               /* ADC shutdown after each conversion. */
					.timebase=ADC_CTRL_TIMEBASE_DEFAULT,  /* Use HW default value. */
					.prescale= ADCPrescalar ,
					.tailgate=false
		    };

		  ADC_Init(ADC0, &ADC0Init);

 ADC_InitSingle_TypeDef ADC0SingleInit  =
{

		.acqTime=adcAcqTime2,               /* 1 ADC_CLK cycle acquisition time. */
		.reference=adcRef1V25,                /* 1.25V internal reference. */
		.resolution=adcRes12Bit,               /* 12 bit resolution. */
		.input=adcSingleInpTemp,           /* CH0 input selected. */
		.diff=false,                     /* Single ended input. */
		. prsEnable=false,                     /* PRS disabled. */
		.leftAdjust=false,                     /* Right adjust. */
		.rep=true                      /* Deactivate conversion after one scan sequence. */
};

 ADC_InitSingle(ADC0, &ADC0SingleInit);
 ADC_IntEnable(ADC0, ADC_IEN_SINGLE);

}


#endif


/**************************************************************************************/
/*
 * This is the GPIO odd interrupt handler routine. If the Active sensor
 * sends an interrupt to the port D pin 1, an interrupt is generated.
 * This function reads the value of the active sensor and based on that value assigns the
 * current state of the light
 * This is an atomic function as it should not be interrupted when being executed
 * Input variables: None required
 * Global Variables: NOne required, current_light_state
 * Output Variables: None required
 ********************************************************************************
 */
#if ACTIVE_LIT_SENSR_TSL2651==ON
void GPIO_ODD_IRQHandler(void)
{

	//Check GPIO->IF register to see which pin fired an interrupt
	INT_Disable();
	GPIO->IFC=0xFFFF;

	uint8_t opl,oph;
	uint16_t op;
	opl=I2C1_read_register(ACTIVE_LS_COMMAND|0x0C);
	oph=I2C1_read_register(ACTIVE_LS_COMMAND|0x0D);
	op=(oph<<8) | opl;
	if(op>=0x0800)
	{
		Turn_off_LED(0);
		//current_light_state=IT_IS_BRIGHT;				//Uncomment if the both sensors are to work together
	}
	else
	{
		Turn_on_LED(0);
		//current_light_state=IT_IS_DARK;				//Uncomment if the both sensors are to work together
	}

	INT_Enable();
}
#endif

#if BLE_CONTROL==ON
void GPIO_EVEN_IRQHandler(void)
{

	//Check GPIO->IF register to see which pin fired an interrupt
	INT_Disable();
	GPIO->IFC=0xFFFF;
	if(my_iot_system==systemIsOn)
		{
		//Turn_off_LED(1);
		RTC_Enable(true);
		my_iot_system=systemIsOff;
		}
	else
		{
		//Turn_on_LED(1);
		RTC_Enable(false);
		/*
		 * Execute everything that is in the RTC Com1 interrupt
		 */
#if ACTIVE_LIT_SENSR_TSL2651==ON
		{

				//Load Power Management OFF routine
				GPIO_ExtIntConfig(I2C1_INTERRUPT_PORT,I2C1_INTERRUPT_PIN,1,false,true,false);		//Setting the gpio interrupt on
				NVIC_DisableIRQ(GPIO_ODD_IRQn);
				NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
		}
#endif
#if DISPLAY_ON_LCD==ON || DISPLAY_HEART_RATE_ON_LCD==ON
		SegmentLCD_Disable();
#endif

#if DISPLAY_ON_LCD==ON
		unblockSleepMode(EM2);
#endif


		LETIMER_Enable(LETIMER0, false);		//Start the timer

		LESENSE_ScanStop();
		my_iot_system=systemIsOn;

		}

	INT_Enable();
}

void setup_control_gpio()
{
	GPIO_PinModeSet(BLE_CTRL_PORT,BLE_CTRL_PIN, gpioModeInput, 0);			//Initialize the interrupt pin
	GPIO_ExtIntConfig(BLE_CTRL_PORT,BLE_CTRL_PIN,2,true,true,true);		//Setting the gpio interrupt on
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}
#endif
/**************************************************************************************/
/*
 * This function is the uart rx irq handler
 * Input variables: None
 * Global Variables: none
 * Output Variables: None required
 *********************************************************************************/

void UART1_RX_IRQHandler(void)
{
	INT_Disable();
  /* Check for RX data valid interrupt */
  if (uart->STATUS & UART_STATUS_RXDATAV)
  {
    /* Copy data into RX Buffer */
    uint8_t rxData = USART_Rx(uart);
    rxBuf.data[rxBuf.wrI] = rxData;
    rxBuf.wrI             = (rxBuf.wrI + 1) % BUFFERSIZE;
    rxBuf.pendingBytes++;

    /* Flag Rx overflow */
    if (rxBuf.pendingBytes > BUFFERSIZE)
    {
      rxBuf.overflow = true;
    }

    /* Clear RXDATAV interrupt */
    USART_IntClear(UART1, UART_IF_RXDATAV);

    INT_Enable();
  }
}
/**************************************************************************************/
/*
 * This function is the UART tx irq handler
 * Input variables: None
 * Global Variables: none
 * Output Variables: None required
 *********************************************************************************/

void UART1_TX_IRQHandler(void)
{
  /* Clear interrupt flags by reading them. */
  USART_IntGet(UART1);
  INT_Disable();
  /* Check TX buffer level status */
  if (uart->STATUS & UART_STATUS_TXBL)
  {
	  USART_IntClear(UART1, UART_IF_TXC);
	USART_IntClear(UART1, UART_IF_TXBL);

	circ_buff_rd++;
	if(circ_buff_wtr==circ_buff_rd)
	{
	my_buffer=buffer_empty;
	unblockSleepMode(EM1);
	}
	if(my_buffer==buffer_full)
	{
	UART1->TXDATA = *circ_buff_rd;

	}
	 UART1->IFC=USART_IFC_TXC ; // clear the interupt

INT_Enable();
  }
}

/*
 * Interrupt handler routines
 */
/*
 * This function is the main RTC interrupt handler routine
 * In the Com0 all the Sensors are started
 * In the com1 all sensors are turned off and the sleep is unblocked to work in EM3
 * If all Sensors and display is on the following order is followed
 * 			0. Read light sensor value
 * 			1. Initiate On board temperature read
 * 			2. Read heart rate value
 * 			3. Read value from the BME280. The pressure, temperature and humidity are recorded in pointers
 * 			4.Display the data on to the LCD.
 * 			5. Transmit the data to the ESP8266 which writes it to the cloud
 */
void RTC_IRQHandler(void)
{
	INT_Disable();
	uint32_t flags;
	flags=RTC_IntGet();
	RTC_IntClear(flags);
	//LETOUCH_Calibration();					//LESENSE needs callibration to eliminate effect of humidity

	if(flags & RTC_IF_COMP0)
		{
#if ACTIVE_LIT_SENSR_TSL2651==ON
		{
				//Load Power Management ON routine
				//GPIO_PinModeSet(I2C1_POWER_PORT,I2C1_POWER_PIN, gpioModePushPull, 1);			//Power to the active light sensor
				//for(int i=0;i<500;i++);		//3ms													//Wait for the warm up
				GPIO_PinModeSet(I2C1_INTERRUPT_PORT,I2C1_INTERRUPT_PIN, gpioModeInput, 0);			//Initialize the interrupt pin
				TSL2561_Setup();
				GPIO_ExtIntConfig(I2C1_INTERRUPT_PORT,I2C1_INTERRUPT_PIN,1,false,true,true);		//Setting the gpio interrupt on
				NVIC_EnableIRQ(GPIO_ODD_IRQn);
		}
#endif

#if ACMP_TEMP==ON
	// Temperature record
	DMA_ActivateBasic(0, true, false, (void *)array, (void *)&(ADC0->SINGLEDATA),NumberOfSamples-1);
	blockSleepMode(1);
	ADC0->CMD|=ADC_CMD_SINGLESTART;
#endif

#if HEART_SENSOR==ON
	//Heart Rate Record
	ACMP_IntEnable(ACMP0, ACMP_IEN_EDGE); /* Enable edge interrupt */
	ACMP_Enable(ACMP0);

	/* Wait for warmup */
	while (!(ACMP0 ->STATUS & ACMP_STATUS_ACMPACT));
#endif

		//GPIO_PinModeSet(ESP_POWER_PORT,ESP_POWER_PIN, gpioModePushPull, 1);			//Power to the active light sensor
#if I2C_FOR_BME280==ON
  //GPIO_PinModeSet(I2C1_POWER_PORT,I2C1_POWER_PIN, gpioModePushPull, 1);			//Power to the active light sensor
	//for(int i=0;i<500;i++);		//3ms													//Wait for the warm up

 // GPIO_PinModeSet(I2C1_SCL_PORT,I2C1_SCL_PIN, gpioModeWiredAndPullUp, 1);			//Initialize the LED0
 // GPIO_PinModeSet(I2C1_SDA_PORT,I2C1_SDA_PIN, gpioModeWiredAndPullUp, 1);			//Initialize the LED0

	/*
	GPIO_PinModeSet(I2C1_SCL_PORT,I2C1_SCL_PIN, gpioModeDisabled, 0);			//Initialize the LED0
	GPIO_PinModeSet(I2C1_SDA_PORT,I2C1_SDA_PIN, gpioModeDisabled, 0);			//Initialize the LED0
*/

int32_t temperature_BME280[2]={0,0};
uint32_t pressure_BME280[2]={0,0};
uint32_t humidity_BME280[2]={0,0};

bme280_data_readout_template(&temperature_BME280[0],&pressure_BME280[0],&humidity_BME280[0]);

#endif

#if DISPLAY_ON_LCD==ON || DISPLAY_HEART_RATE_ON_LCD==ON

//You can use frame counter interrupt to turn on the lCD. Need to read more about this feature
	    char string[8];

	    //Clock_setup(EM0);

	    /* Initialize LCD controller without boost */
	      SegmentLCD_Init(false);
	      SegmentLCD_AllOff();
#endif

#if DISPLAY_ON_LCD==ON

	  /*snprintf(string,8,"TEMPERA");
	  SegmentLCD_Write(string);
	  for(uint32_t mm=0;mm<1000000;mm++);
	  snprintf(string, 8, "%2d,%1d%%C", (int)(temperature_BME280[1]/100), abs(temperature_BME280[1]%100));
	  SegmentLCD_Write(string);
	  for(uint32_t mm=0;mm<1000000;mm++);*/
int bme_pressure=(int)(pressure_BME280[1]);
int bme_temperature=(int)temperature_BME280[1];
int bme_humidity=(int)humidity_BME280[1];
	  snprintf(string,8,"PRESSRE");
	  SegmentLCD_Write(string);
	  for(uint32_t mm=0;mm<200000;mm++);
	  snprintf(string, 8, "%2d,%1dmB",(bme_pressure/100), abs(bme_pressure%100)/10);
	  SegmentLCD_Write(string);
	  for(uint32_t mm=0;mm<1000000;mm++);

	  snprintf(string,8,"HUMIDTY");
	  SegmentLCD_Write(string);
	  for(uint32_t mm=0;mm<200000;mm++);
	  snprintf(string, 8, "%2d,%1d%%",(bme_humidity/1000), abs(bme_humidity%1000));
	  SegmentLCD_Write(string);

	  SegmentLCD_Number(bme_temperature);
	  SegmentLCD_Symbol(LCD_SYMBOL_DP10, 1);
	  SegmentLCD_Symbol(LCD_SYMBOL_DEGC, 1);
	  blockSleepMode(EM2);
	  for(uint32_t mm=0;mm<1000000;mm++);

	  //SegmentLCD_AllOff();
	  //SegmentLCD_Disable();

#endif

#if DISPLAY_HEART_RATE_ON_LCD==ON
	  //for(uint32_t mm=0;mm<1000000;mm++);

	  /*snprintf(string,8,"TEMPERA");
	  SegmentLCD_Write(string);
	  for(uint32_t mm=0;mm<1000000;mm++);
	  snprintf(string, 8, "%2d,%1d%%C", (int)(temperature_BME280[1]/100), abs(temperature_BME280[1]%100));
	  SegmentLCD_Write(string);
	  for(uint32_t mm=0;mm<1000000;mm++);*/

	  snprintf(string,8,"HRT RAT");
	  SegmentLCD_Write(string);
	  for(uint32_t mm=0;mm<500000;mm++);
	  snprintf(string, 8, "%2d bpm",ACMP_count);
	  SegmentLCD_Write(string);

	  SegmentLCD_Number((int)(temperature));
	  SegmentLCD_Symbol(LCD_SYMBOL_DP10, 1);
	  SegmentLCD_Symbol(LCD_SYMBOL_DEGC, 1);


#endif



#if UART_ESP==ON
	  /*
	   * make another string and pass the data. The ESP doesnt take decimal places. Floats are not allowed in the function being
	   * passed
	   */
	  GPIO_PinModeSet(ESP_ENABLE_PORT,ESP_ENABLE_PIN, gpioModePushPull, 1);			//Power to the active light sensor
	  			  //for(int i=0;i<500;i++);		//3ms													//Wait for the warm up

	  blockSleepMode(EM1);
	  int temp=20;
	  int pres=732;
	  int humi=50;
	  char temperature_bme_ascii[4];
	  char pressure_bme_ascii[4];
	  char humidity_bme_ascii[4];

	  intToStr(temp,&temperature_bme_ascii[0],4);
	  intToStr(pres,&pressure_bme_ascii[0],4);
	  intToStr(humi,&humidity_bme_ascii[0],4);

    circ_buff_wtr=&transmit_circ_buff;
      circ_buff_rd=&transmit_circ_buff;
     //write_data_to_cb(SEND_THIS,strlen(SEND_THIS));
      write_data_to_cb(ESP_WRITE_HEAD,strlen(ESP_WRITE_HEAD));
      write_data_to_cb(&temperature_bme_ascii[2],2);
      write_data_to_cb(",",strlen(","));
      write_data_to_cb(&pressure_bme_ascii[1],3);
      write_data_to_cb(",",strlen(","));
      write_data_to_cb(&humidity_bme_ascii[2],2);
      write_data_to_cb(ESP_WRITE_TAIL,strlen(ESP_WRITE_TAIL));

      read_from_transmit_circ_buffer();

#endif

#if LEUART0_ESP==ON
	  /*
	   * make another string and pass the data. The ESP doesnt take decimal places. Floats are not allowed in the function being
	   * passed
	   */
	  			  //for(int i=0;i<500;i++);		//3ms													//Wait for the warm up
      GPIO_PinModeSet(ESP_ENABLE_PORT,ESP_ENABLE_PIN, gpioModePushPull, 1);			//Power to the active light sensor

	  blockSleepMode(EM2);
	 /* int temp=29;
	  int pres=132;
	  int humi=90;*/
		INT_Enable();
	  char temperature_bme_ascii[5];
	  char pressure_bme_ascii[5];
	  char humidity_bme_ascii[5];

	  intToStr(bme_temperature,&temperature_bme_ascii[0],4);
	  intToStr(bme_pressure,&pressure_bme_ascii[0],5);
	  intToStr(bme_humidity,&humidity_bme_ascii[0],5);

	  circ_buff_wtr=&transmit_circ_buff;
      circ_buff_rd=&transmit_circ_buff;

      /*
       * This part generates the string to be written to the ESP8266. The string has to be perfect to work
       */
    // write_data_to_cb(SEND_THIS,strlen(SEND_THIS));
      write_data_to_cb(ESP_WRITE_HEAD,strlen(ESP_WRITE_HEAD));
      write_data_to_cb(&temperature_bme_ascii[0],2);
      write_data_to_cb(",",strlen(","));
      write_data_to_cb(&pressure_bme_ascii[0],3);
      write_data_to_cb(",",strlen(","));
      write_data_to_cb(&humidity_bme_ascii[0],2);
      write_data_to_cb(ESP_WRITE_TAIL,strlen(ESP_WRITE_TAIL));

      read_from_transmit_circ_buffer();

#endif

	}
	else					//Com1
	{
		//Make sure there is enough time for the ESP to write to the WIFI before disabling it
		//GPIO_PinModeSet(ESP_ENABLE_PORT,ESP_ENABLE_PIN, gpioModePushPull, 0);			//Power to the active light sensor
	//	GPIO_PinModeSet(ESP_POWER_PORT,ESP_POWER_PIN, gpioModePushPull, 0);			//Power to the active light sensor
#if ACTIVE_LIT_SENSR_TSL2651==ON
		{

				//Load Power Management OFF routine
				GPIO_ExtIntConfig(I2C1_INTERRUPT_PORT,I2C1_INTERRUPT_PIN,1,false,true,false);		//Setting the gpio interrupt on
				NVIC_DisableIRQ(GPIO_ODD_IRQn);
				NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
		}
#endif
#if DISPLAY_ON_LCD==ON || DISPLAY_HEART_RATE_ON_LCD==ON
		SegmentLCD_Disable();
#endif

#if DISPLAY_ON_LCD==ON
		unblockSleepMode(EM2);
#endif
		INT_Enable();
	}

}

/**************************************************************************************/
/*
 * This function is the main which first executes.
 * Input variables: None
 * Global Variables: none
 * Output Variables: None required
 *********************************************************************************/

int main(void)
{
  /* Chip errata */
  CHIP_Init();
#if BLE_CONTROL==ON
  my_iot_system=systemIsOff;
#endif
  GPIO_LedsInit();
  Clock_setup(LOWEST_ENERGY_MODE);
#if BLE_CONTROL==ON
  setup_control_gpio();

  my_iot_system=systemIsOn;
#endif
  GPIO_PinModeSet(ESP_ENABLE_PORT,ESP_ENABLE_PIN, gpioModePushPull, 1);			//Power to the active light sensor

  RTC_Enable(false);
  blockSleepMode(EM2);

#if I2C_FOR_BME280==ON || ACTIVE_LIT_SENSR_TSL2651==ON
	I2C_setup();								//Setup the I2C
	GPIO_PinModeSet(I2C1_POWER_PORT,I2C1_POWER_PIN, gpioModePushPull, 1);			//Power to the active light sensor
	for(int i=0;i<500;i++);
	GPIO_PinModeSet(I2C1_SCL_PORT,I2C1_SCL_PIN, gpioModeWiredAndPullUp, 1);			//Initialize the LED0
	GPIO_PinModeSet(I2C1_SDA_PORT,I2C1_SDA_PIN, gpioModeWiredAndPullUp, 1);			//Initialize the LED0

#endif


#if UART_ESP==ON
  /* Initialize UART peripheral */
    uartSetup( );
#elif LEUART0_ESP==ON
    LEUART0_Setup();
#endif

#if LESENSE_CAP_SENSE==ON
  LESENSE_TOUCH_Setup();

#endif

#if ACMP_TEMP==ON

  DMA_Setup();
  ADC_Setup();

  // Temperature record
  	DMA_ActivateBasic(0, true, false, (void *)array, (void *)&(ADC0->SINGLEDATA),NumberOfSamples-1);
  	//blockSleepMode(1);
  	ADC0->CMD|=ADC_CMD_SINGLESTART;

#endif

#if HEART_SENSOR==ON
    ACMP_Setup();
#endif


#if RTC_DEVICE==ON
  RTC_Setup();
#endif


 // while(1);
  /* Infinite loop */
while (1)
	{
		sleep();
	}
}
