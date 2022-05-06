/**-------------------------------------------------------------------------
@file	Timers.h

@brief	Header file for Timers functionalities in the BlueIO832Mini and BlueIO840Wizard projects

This file contains declarations of functions and variables used in the BlueIO832Mini.cpp and BlueIO840Wizard.cpp

@author	Thinh Tran
@date	Apr. 13, 2022

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

#ifndef __UARTS_H__
#define __UARTS_H__

#include "app_scheduler.h"
#include "blueio_board.h"
#include "coredev/uart.h"
#include "ble_app.h"
#include "ble_service.h"
#include "bluetooth/blueio_blesrvc.h"
#include "ble_intrf.h"
#include "LedsButtons.h"


#include "coredev/timer.h"
#include "UartBle.h"
#include "board.h"

Timer g_Timer2;
extern volatile uint8_t g_TimeoutCnt;

extern UART g_Uart;
extern BleIntrf g_BleIntrf;
extern volatile int g_UartRxBuffLen;
extern IOPinCfg_t s_Leds[];
extern uint8_t g_UartRxBuff[PACKET_SIZE];


void TestTimerTriggers()
{
	uint64_t period;
	/* Timer trigger #0 */
//	uint64_t period = g_Timer2.EnableTimerTrigger(0, 100UL, TIMER_TRIG_TYPE_CONTINUOUS);
//	if (period == 0)
//	{
//		g_Uart.printf("Timer trigger #0 failed\r\n");
//	}
//	else
//	{
//		g_Uart.printf("Timer trigger #0 with period = %d ms: Test OK!\r\n", period);
//		//g_Timer2.DisableTimerTrigger(0);
//	}

	/* Timer trigger #1 */
	period = g_Timer2.EnableTimerTrigger(1, 1500UL, TIMER_TRIG_TYPE_CONTINUOUS);
	if (period == 0)
	{
		g_Uart.printf("Timer trigger #1 failed\r\n");
	}
	else
	{
		g_Uart.printf("Timer trigger #1 with period = %d ms: Test OK!\r\n", period);
	}
}


void Timer2Handler(TimerDev_t *pTimer, uint32_t Evt)
{
	//static uint64_t precnt[4] = {0, };
	//uint64_t c = TimerGetMilisecond(pTimer);

	/* Trigger #0 */
    if (Evt & TIMER_EVT_TRIGGER(0))
    {
    	//IOPinToggle(s_Leds[0].PortNo, s_Leds[0].PinNo);//LED1
    	//IOPinToggle(s_Leds[1].PortNo, s_Leds[1].PinNo);//LED2_GREEN
    	//IOPinToggle(s_Leds[2].PortNo, s_Leds[2].PinNo);//LED2_RED
    	//IOPinToggle(s_Leds[3].PortNo, s_Leds[3].PinNo);//LED2_BLUE

    	g_TimeoutCnt--;

		if (g_TimeoutCnt == 0)
		{
			int cnt = g_BleIntrf.Tx(0, g_UartRxBuff, g_UartRxBuffLen);
			if (cnt == g_UartRxBuffLen)
			{
				//g_Uart.printf("Target board: ");
				g_Uart.Tx(g_UartRxBuff, g_UartRxBuffLen);
				//g_Uart.printf("\r\n");
				g_UartRxBuffLen = 0;
			}
			else
			{
				g_UartRxBuffLen -= cnt;
				app_sched_event_put(NULL, 0, UartRxSchedHandler);
			}

			g_TimeoutCnt = MAX_COUNT;

		}


    }

    /* Trigger #1 */
    if (Evt & TIMER_EVT_TRIGGER(1))
    {
    	IOPinToggle(s_Leds[2].PortNo, s_Leds[2].PinNo);//LED2_RED
    }

}



#endif //__UARTS_H__
