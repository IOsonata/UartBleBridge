/**-------------------------------------------------------------------------
@example	UartBle.cpp


@brief	UART-BLE functions for BlueIO832Mini and BlueIO840Wizard projects


@author Thinh Tran
@date	Apr. 13, 2022

@license

Copyright (c) 2017-2022, I-SYST inc., all rights reserved

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
#include "board.h"
#include "UartBle.h"
#include "coredev/uart.h"
#include "idelay.h"
#include "iopinctrl.h"
#include "coredev/timer.h"
#include "fds.h"

uint8_t g_UartRxBuff[PACKET_SIZE];
//uint8_t s_UartRxFifo[UARTFIFOSIZE];
//uint8_t s_UartTxFifo[UARTFIFOSIZE];

UART g_Uart;

extern UARTCfg_t g_UartCfg;
extern IOPinCfg_t s_Leds[];
extern Timer g_Timer2;
extern BleIntrf g_BleIntrf;
extern volatile int g_UartRxBuffLen;

volatile uint8_t g_TimeoutCnt = MAX_COUNT;

/// UART Tx BLE Service callback function
void UartTxSrvcCallback(BLESRVC *pBlueIOSvc, uint8_t *pData, int Offset, int Len)
{
	IOPinToggle(s_Leds[1].PortNo, s_Leds[1].PinNo);
	g_Uart.printf("UART Tx callback: ");
	g_Uart.Tx(pData, Len);
	g_Uart.printf("\r\n");
	IOPinToggle(s_Leds[1].PortNo, s_Leds[1].PinNo);
}


/// UART Configure BLE Service callback function
void UartCfgSrvcCallback(BLESRVC *pBlueIOSvc, uint8_t *pData, int Offset, int Len)
{
	IOPinToggle(s_Leds[1].PortNo, s_Leds[1].PinNo);
	g_Uart.printf("UART Config Characteristic: Callback received %d bytes data\r\n", Len);
	//g_Uart.printf("UART Config callback received data: ");
	//g_Uart.Tx(pData, Len);
	uint8_t buff[PACKET_SIZE];
	uint8_t l;
	int8_t rc=1;
	//l = sprintf((char *)buff, "UART Config callback received data: 0X%x \r\n", pData);
	//g_Uart.Tx(buff, l);

	if (Len == 6)
	{
#if 0
		for (int i = 0; i<6; i++)
		{
			g_Uart.printf("byte[%d] = 0x%x \r\n", i, pData[i]);
		}
#endif
		bool isCorrectRate = false;
		/* This byte order is valid for the nRF BLE desktop app */
		//uint32_t uartRate = (pData[0] << 24) | (pData[1] << 16) | (pData[2] << 8) | pData[3];

		/* This by order is valid for the mobile app */
		uint32_t uartRate = (pData[3] << 24) | (pData[2] << 16) | (pData[1] << 8) | pData[0];
//		g_Uart.printf("uartRate = %d = 0x%x\r\n",uartRate, uartRate);
//		g_Uart.printf("Flow control byte = 0x%x", pData[4]);
//		g_Uart.printf("Parity byte = 0x%x", pData[5]);

		switch (uartRate)
		{
		case 1200:
		case 2400:
		case 4800:
		case 9600:
		case 14400:
		case 19200:
		case 28800:
		case 38400:
		case 57600:
		case 76800:
		case 115200:
		case 230400:
		case 250000:
		case 460800:
		case 921600:
		case 1000000:
			g_Uart.printf("Correct baudrate\r\n");
			/* Update UART configuration */
			g_UartCfg.Rate = uartRate;
			g_UartCfg.FlowControl = (pData[4] > 0) ? UART_FLWCTRL_HW : UART_FLWCTRL_NONE;
			g_UartCfg.Parity = (pData[5] > 0) ? UART_PARITY_EVEN : UART_PARITY_NONE;
			g_UartRxBuffLen = 0;

			/* Apply new UART configuration to nRF5x MCU */
			g_Uart.Disable();
			g_Uart.Init(g_UartCfg);
			msDelay(100);
			break;
		default:
			g_Uart.printf("Not supported baudrate!\r\n");
			break;
		}


	}
	else
	{
		g_Uart.printf("Invalid configuration format\r\n");
	}

	IOPinToggle(s_Leds[1].PortNo, s_Leds[1].PinNo);
}

/**@brief Function for handling event
 *
  * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
int nRFUartEvtHandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	int cnt = 0;

	switch (EvtId)
	{
		case UART_EVT_RXTIMEOUT:
		case UART_EVT_RXDATA:
			app_sched_event_put(NULL, 0, UartRxSchedHandler);
			break;
		case UART_EVT_TXREADY:
			break;
		case UART_EVT_LINESTATE:
			break;
		default:
			break;
	}

	return cnt;
}


/**@brief Function for Scheduling UART Rx event
 *
 * @details
 *
 * @param[in] p_event_data
 * @param[in] p_event_size
 */
// TODO: Add timer to UartRxSchedHandler()
void UartRxSchedHandler(void * p_event_data, uint16_t event_size)
{
	bool flush = false;

	IOPinToggle(s_Leds[1].PortNo, s_Leds[1].PinNo);//LED2_GREEN

	//g_Timer2.DisableTimerTrigger(0);

	int l = g_Uart.Rx(&g_UartRxBuff[g_UartRxBuffLen], PACKET_SIZE - g_UartRxBuffLen);

	if (l > 0)
	{
		g_UartRxBuffLen += l;
		g_TimeoutCnt = MAX_COUNT;

		if (g_UartRxBuffLen >= PACKET_SIZE)
		{
			flush = true;

		}
		else
		{
			//g_Timer2.EnableTimerTrigger(0, 500UL, TIMER_TRIG_TYPE_SINGLE);
//			if (g_TimeoutCnt == 0)
//			{
//				g_BleIntrf.Tx(0, g_UartRxBuff, g_UartRxBuffLen);
//				//g_Uart.printf("Target board: ");
//				g_Uart.Tx(g_UartRxBuff, g_UartRxBuffLen);
//				//g_Uart.printf("\r\n");
//				g_UartRxBuffLen = 0;
//
//				g_TimeoutCnt = MAX_COUNT;
//				app_sched_event_put(NULL, 0, UartRxSchedHandler);
//			}

		}
	}
	else
	{
		if (g_UartRxBuffLen > 0)
		{
			flush = true;
			//g_Uart.printf("#3: l = %d | g_UartRxBuffLen = %d | flush = true", l, g_UartRxBuffLen);
		}
	}
	if (flush)
	{
		//g_Uart.printf("Buffer is full ==> Flush it!\r\n");

		g_BleIntrf.Tx(0, g_UartRxBuff, g_UartRxBuffLen);
		g_Uart.Tx(g_UartRxBuff, g_UartRxBuffLen);
		g_UartRxBuffLen = 0;


//		if (BleSrvcCharNotify(&g_UartBleSrvc, 0, g_UartRxBuff, g_UartRxBuffLen) == 0)
//		{
//			g_UartRxBuffLen = 0;
//		}

		app_sched_event_put(NULL, 0, UartRxSchedHandler);

	}

	IOPinToggle(s_Leds[1].PortNo, s_Leds[1].PinNo);//LED2_GREEN
}
