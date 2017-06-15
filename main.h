/*
 * File name  :	main.h
 *
 *  Created on: March 25, 2017
 *      Author: Richard Noronha and Maitri Chattopadhyay
 * Description: This is the main header file that contains most of the includes needed by the main.c file
 * 				The Devices can be switched on or off for testing purposes
 */

#ifndef SRC_MAIN_H_
#define SRC_MAIN_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_int.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_rtc.h"
#include "em_letimer.h"
#include "em_gpio.h"
#include "em_acmp.h"
#include "em_timer.h"
#include "em_adc.h"
#include "dmactrl.h"
#include "em_dma.h"
#include "em_i2c.h"
#include "em_leuart.h"

#include "em_lcd.h"
#include "bme280.h"
#include "bme280_support.h"
#include "em_prs.h"
#include "em_system.h"
#include "em_usart.h"
#include "em_lesense.h"
#include "lesense_letouch.h"
#include "lesense_letouch_config.h"
#include "segmentlcd.h"
#include "em_usart.h"

#define ON							1
#define OFF							0

#define LOWEST_ENERGY_MODE 			EM2			//Set the default minimum Energy mode here

#if LOWEST_ENERGY_MODE
#define RTC_COMP0					5740
#else
#define RTC_COMP0					500000
#endif
/*
 * Sleep variables
 */
typedef enum
{
	EM0=0, EM1, EM2, EM3
}sleepstate_t;

uint8_t sleep_block_counter[4];

/*
 * LED variables
 */
#define LED0_PORT					gpioPortE
#define LED1_PORT					gpioPortE
#define LED0_PIN					2
#define LED1_PIN					3

/*
 * I2C for BME variables
 */
#define I2C1_SDA_PORT				gpioPortC
#define I2C1_SCL_PORT				gpioPortC
#define I2C1_SDA_PIN				4
#define I2C1_SCL_PIN				5

#define BME280_I2C_ADDRESS			0x76							//0x77 if SDO is connected to high
#define WRITE_TO_I2C				0
#define READ_FROM_I2C				1

#define I2C_SPEED_MODE				i2cClockHLRFast					//i2cClockHLRStandard
#define I2C_FREQ_MAX				I2C_FREQ_STANDARD_MAX			//I2C_FREQ_FAST_MAX

#define I2C_MIN_SLEEP_BLOCK			EM1


/*
 * Devices
 */

//Final devices that were displayed for the demo are left on.
//The light sensor did not work reliably and is therefore off

#define I2C_FOR_BME280				ON			//only turn this on to test the BME280
#define LESENSE_CAP_SENSE			ON
#define PWM_LETIMER					ON			//Turn this and lesense cap on for PWM testing
#define RTC_DEVICE					ON
#define RTC_COM1					ON
#define DISPLAY_ON_LCD				ON
#define CIRCULAR_BUFFER				ON

#define LEUART0_ESP 				ON
#define ACMP_TEMP					ON
#define HEART_SENSOR				ON
#define BLE_CONTROL					ON

#define DISPLAY_HEART_RATE_ON_LCD	ON

#define ACTIVE_LIT_SENSR_TSL2651	OFF
#define UART_ESP					OFF


/*********************************/
float ledBrightness;			//Default is 50%duty cycle
#define LIGHT_SENSOR_ON_TIME 		ledBrightness		//in seconds

#define TIME_TO_SENSE_LIGHT		0.020				//DUTY_CYCLE

#define LFXO_MAX_CNT			32768
#define LE0_MAX_CNT				65535


void LETIMER_Setup_PWM(float calib_if_needed);
void I2C_setup(void);
void unblockSleepMode(sleepstate_t minimumMode);

#define ESP_ENABLE_PORT		gpioPortD				//EXT pin 16
#define ESP_ENABLE_PIN		6

#define LEUART0_TX_PORT				gpioPortD			//EXT pin 12
#define LEUART0_TX_PIN				4
#define LEUART0_RX_PORT				gpioPortD			//EXT pin 14
#define LEUART0_RX_PIN				5

//LEUART0 Initialization values
#define	LEU0_PARITY_VALUE				leuartNoParity
#define LEU0_BAUD						9600
#define	LEU0_STOPB						leuartStopbits1
#define LEU0_DATAB						leuartDatabits8
#define LEU0_REF_FREQ					0

#define LEUART_INTERRUPTS				ON
#define LEUART_TX_INTERRUPT				ON
#define LEAURT_LOOPBACK					OFF
#define LEUART_POLL_TEST				OFF

static volatile uint16_t buttons_pressed;

/*
 *UART part
 */
#define UART_ESP_FUNCTIONS				ON


/*
 * Device 2 parts
 */
#if ACMP_TEMP==ON
#define DMA_Enable 1 // 0 means without DMA
#define GRADIENT -6.27
#define ADCPrescalar 49
#define TEMPPIN 3
#define ZERO 0x00
#endif

#if HEART_SENSOR==ON
#define ACMPVDD acmpChannelVDD
#define ACMPCHANNEL16 acmpChannel6

#define NUMBER 16

#endif

#if BLE_CONTROL==ON
#define BLE_CTRL_PORT			gpioPortD
#define BLE_CTRL_PIN			2					//external pin 8

typedef enum
{
	systemIsOff,systemIsOn
}bike_system;
bike_system my_iot_system;
#endif

#define I2C1_POWER_PORT				gpioPortD				//Ext Pin 4
#define I2C1_POWER_PIN				0
#define I2C1_GND_PORT				gpioPortC				//Ext Pin 3
#define I2C1_GND_PIN				0

#define I2C1_INTERRUPT_PORT			gpioPortD				//Ext pin 6
#define I2C1_INTERRUPT_PIN			1


#if ACTIVE_LIT_SENSR_TSL2651==ON
/*********************************************************************************************/


#define ACTIVE_LS_I2C_ADDRESS		0b0111001
#define READ_FROM_I2C				1
#define WRITE_TO_I2C				0

/*Macros for the TSL2561 Luminosity Light sensor*/
#define ACTIVE_LS_COMMAND			0xC0
#define ACTIVE_LS_CONTROL			0x00
#define ACTIVE_LS_TIMING			0x01
#define ACTIVE_LS_THRESHLOWLOW		0x02
#define ACTIVE_LS_THRESHLOWHIGH		0x03
#define ACTIVE_LS_THRESHHIGHLOW		0x04
#define ACTIVE_LS_THRESHHIGHHIGH	0x05
#define ACTIVE_LS_INTERRUPT			0x06
#define ACTIVE_LS_RESERVED			0x07
#define ACTIVE_LS_CRC				0x08
#define ACTIVE_LS_RESERVED1			0x09
#define ACTIVE_LS_ID				0x0A
#define ACTIVE_LS_RESERVED2			0x0B
#define ACTIVE_LS_DATA0LOW			0x0C
#define ACTIVE_LS_DATA0HIGH			0x0D
#define ACTIVE_LS_DATA1LOW			0x0E
#define ACTIVE_LS_DATA1HIGH			0x0F

//Set the lower threshold value
#define ACTIVE_LS_TH_LOWHIGH_VAL	0x00			//Higher byte
#define ACTIVE_LS_TH_LOWLOW_VAL		0x0F			//Lower byte

#define ACTIVE_LS_TH_HIGHHIGH_VAL	0X08			//Higher byte
#define ACTIVE_LS_TH_HIGHLOW_VAL	0X00			//Lower byte

#define ACTIVE_LS_PERSIST_VAL		0X04
#define ACTIVE_LS_INTEGRATE_TIME	0X01			//Timing register, value of integration time set to 101ms
#define ACTIVE_LS_GAIN				0x00			//Gain set to Low=0, write 0x10 for high gain
#define ACTIVE_LS_POWER_UP			0x03			//Write 0x00 for ppower ddown
#define ACTIVE_LS_POWER_DOWN		0x00
#define ACTIVE_LS_INT_DISABLED		0x00
#define ACTIVE_LS_INT_LEVEL			0x10			//This is to turn on the level interrupt in the Interrupt control register
#define ACTIVE_LS_INT_SMB			0x20


#define I2C_MIN_SLEEP_BLOCK			EM1
#define I2C_SPEED_MODE				i2cClockHLRFast					//i2cClockHLRStandard
#define I2C_FREQ_MAX				I2C_FREQ_STANDARD_MAX			//I2C_FREQ_FAST_MAX

#endif


/*
 * Function declarations
 */
void uartSetup(void);
#endif /* SRC_MAIN_H_ */
