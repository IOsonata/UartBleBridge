/**-------------------------------------------------------------------------
@file	BlueIOCtrl.h

@brief	Header file for the BlueIO832Mini and BlueIO840Wizard projects

This file contains declarations of functions and variables used in the BlueIO832Mini.cpp and BlueIO840Wizard.cpp

@author	Thinh Tran
@date	Apr. 25, 2022

@license

Copyright (c) 2022, I-SYST inc., all rights reserved

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

#ifndef __BLUEIOCTRL_H__
#define __BLUEIOCTRL_H__

#include "blueio_board.h"
#include "coredev/timer.h"
#include "coredev/uart.h"
#include "cfifo.h"
#include "ble_app.h"
#include "ble_service.h"
#include "ble_intrf.h"
#include "bluetooth/blueio_blesrvc.h"



#define MANUFACTURER_NAME               "I-SYST inc."                       /**< Manufacturer. Will be passed to Device Information Service. */
#define MANUFACTURER_ID                 ISYST_BLUETOOTH_ID                  /**< Manufacturer ID, part of System ID. Will be passed to Device Information Service. */
#define ORG_UNIQUE_ID                   ISYST_BLUETOOTH_ID                  /**< Organizational Unique ID, part of System ID. Will be passed to Device Information Service. */

#define APP_ADV_INTERVAL                MSEC_TO_UNITS(64, UNIT_0_625_MS)	/**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_TIMEOUT					MSEC_TO_UNITS(0, UNIT_10_MS)		/**< The advertising timeout (in units of 10ms seconds). */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(10, UNIT_1_25_MS)     /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(40, UNIT_1_25_MS)     /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */

/// BlueIO device UUIDs
#define BLE_UART_UUID_BASE			BLUEIO_UUID_BASE				// Base UUID of the device
#define BLE_UART_UUID_SERVICE			BLUEIO_UUID_UART_SERVICE		// BlueIO UART service

#define BLE_UART_UUID_TX_CHAR			BLUEIO_UUID_UART_TX_CHAR		// UART Tx characteristic
#define BLE_UART_UUID_TX_CHAR_PROP		(BLESVC_CHAR_PROP_WRITE | BLESVC_CHAR_PROP_WRAUTH | BLESVC_CHAR_PROP_WRITEWORESP | BLESVC_CHAR_PROP_VARLEN) // Property of Tx characteristic

#define BLE_UART_UUID_RX_CHAR			BLUEIO_UUID_UART_RX_CHAR		// UART Rx characteristic
#define BLE_UART_UUID_RX_CHAR_PROP		(BLESVC_CHAR_PROP_READ | BLESVC_CHAR_PROP_NOTIFY | BLESVC_CHAR_PROP_VARLEN) // Property of Tx characteristic

#define BLE_UART_UUID_CONFIG_CHAR		BLUEIO_UUID_UART_CONFIG_CHAR 							// UART configuration characteristic
#define BLE_UART_UUID_CONFIG_CHAR_PROP	(BLESVC_CHAR_PROP_WRITE | BLESVC_CHAR_PROP_WRAUTH | BLESVC_CHAR_PROP_WRITEWORESP | BLESVC_CHAR_PROP_VARLEN | \
											BLESVC_CHAR_PROP_READ | BLESVC_CHAR_PROP_NOTIFY ) // Property of UART confg. char.

#define BLE_MTU_SIZE			256//byte
#define PACKET_SIZE				(BLE_MTU_SIZE)

// UART
#define UART_MAX_DATA_LEN  		(PACKET_SIZE)
#define UARTFIFOSIZE			CFIFO_MEMSIZE(UART_MAX_DATA_LEN * 4)

// BLE Interface buffer
#define BLEINTRF_PKTSIZE		(BLE_MTU_SIZE)
#define BLEINTRF_FIFOSIZE		BLEINTRF_CFIFO_TOTAL_MEMSIZE(15, BLEINTRF_PKTSIZE)//(NbPkt, PktSize)

#define BLESRV_READ_CHAR_IDX		0
#define BLESRV_WRITE_CHAR_IDX		1



// BLE Interface event handler
int BleIntrfEvtCallback(DevIntrf_t *pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int BufferLen);
int BleCfgIntrfEvtCallback(DevIntrf_t *pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int BufferLen);

// Button event handler
void ButEvent(int IntNo, void *pCtx);

#endif // __BLUEIOCTRL_H__

