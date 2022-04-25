/**-------------------------------------------------------------------------
@file	board.h

@brief	Board specific definitions

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.

@author	Hoang Nguyen Hoan
@date	Nov. 16, 2016

@license

Copyright (c) 2016, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/

#ifndef __BOARD_H__
#define __BOARD_H__

#include "blueio_board.h"

//#define NORDIC_DK

//#ifdef NORDIC_DK
//// Nordic DK PCA10040 board
//
//#define UART_RX_PORT		0
//#define UART_RX_PIN			8
//#define UART_RX_PINOP		1	//
//#define UART_TX_PORT		0
//#define UART_TX_PIN			6//7
//#define UART_TX_PINOP		0
//#define UART_CTS_PORT		0
//#define UART_CTS_PIN		7//12
//#define UART_CTS_PINOP		0
//#define UART_RTS_PORT		0
//#define UART_RTS_PIN		5//11
//#define UART_RTS_PINOP		0
//
//#define BUTTON1_PORT		0
//#define BUTTON1_PIN			13
//#define BUTTON2_PORT		0
//#define BUTTON2_PIN			14
//
//#else
/// BlueIO breakout board

#define DEVICE_NAME     "UartBleBridge"				/**< Name of device. Will be included in the advertising data. */
#define MODEL_NAME      "BlystNano"               	/**< Model number. Will be passed to Device Information Service. */


// LEDs
#define LED1_PORT		BLUEIO_LED1_PORT
#define LED1_PIN		BLUEIO_LED1_PIN
#define LED1_PINOP		BLUEIO_LED1_PINOP

#define LED2_PORT		BLUEIO_LED2_PORT
#define LED2_PIN		BLUEIO_LED2_PIN
#define LED2_PINOP		BLUEIO_LED2_PINOP

#define LED3_PORT		BLUEIO_LED3_PORT
#define LED3_PIN		BLUEIO_LED3_PIN
#define LED3_PINOP		BLUEIO_LED3_PINOP

#define LED4_PORT		BLUEIO_LED4_PORT
#define LED4_PIN		BLUEIO_LED4_PIN
#define LED4_PINOP		BLUEIO_LED4_PINOP

#define LED_PIN_MAP 	{\
	{BLUEIO_LED_BLUE_PORT, BLUEIO_LED_BLUE_PIN, BLUEIO_LED_BLUE_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{BLUEIO_LED_GREEN_PORT, BLUEIO_LED_GREEN_PIN, BLUEIO_LED_GREEN_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{BLUEIO_LED_RED_PORT, BLUEIO_LED_RED_PIN, BLUEIO_LED_RED_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{BLUEIO_LED4_PORT, BLUEIO_LED4_PIN, BLUEIO_LED4_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
}


// Button pins
#define BUTTON1_PORT		BLUEIO_BUT1_PORT
#define BUTTON1_PIN			BLUEIO_BUT1_PIN
#define BUTTON2_PORT		BLUEIO_BUT2_PORT
#define BUTTON2_PIN			BLUEIO_BUT2_PIN

#define BUTTON_PIN_MAP  	{\
	{BUTTON1_PORT, BUTTON1_PIN, 0, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},\
	{BUTTON2_PORT, BUTTON2_PIN, 0, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},\
}



// UART pins
#define UART_RX_PORT		BLUEIO_UART_RX_PORT
#define UART_RX_PIN			BLUEIO_UART_RX_PIN
#define UART_RX_PINOP		BLUEIO_UART_RX_PINOP

#define UART_TX_PORT		BLUEIO_UART_TX_PORT
#define UART_TX_PIN			BLUEIO_UART_TX_PIN
#define UART_TX_PINOP		BLUEIO_UART_TX_PINOP

#define UART_CTS_PORT		BLUEIO_UART_CTS_PORT
#define UART_CTS_PIN		BLUEIO_UART_CTS_PIN
#define UART_CTS_PINOP		BLUEIO_UART_CTS_PINOP

#define UART_RTS_PORT		BLUEIO_UART_RTS_PORT
#define UART_RTS_PIN		BLUEIO_UART_RTS_PIN
#define UART_RTS_PINOP		BLUEIO_UART_RTS_PINOP

/// UART pins definitions
#define UART_PIN_MAP  		{\
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
}


#endif // __BOARD_H__

