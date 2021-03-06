/*--------------------------------------------------------------------------
File   : ble_intrf.cpp

Author : Hoang Nguyen Hoan          Feb. 6, 2017

Desc   : Implementation allow the creation of generic serial interface of
		 a custom Bluetooth Smart service with multiple user defined
		 characteristics.

Copyright (c) 2017, I-SYST inc., all rights reserved

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

----------------------------------------------------------------------------
Modified by          Date              Description

----------------------------------------------------------------------------*/
#include <string.h>

#include "app_util_platform.h"

#include "istddef.h"
#include "cfifo.h"
#include "ble_intrf.h"
#include "ble_app.h"
#include "interrupt.h"

#define NRFBLEINTRF_PACKET_SIZE		(NRF_BLE_MAX_MTU_SIZE - 3)// + sizeof(BLEINTRF_PKT) - 1)
#define NRFBLEINTRF_CFIFO_SIZE		BLEINTRF_CFIFO_TOTAL_MEMSIZE(2, NRFBLEINTRF_PACKET_SIZE)

alignas(4) static uint8_t s_nRFBleRxFifoMem[NRFBLEINTRF_CFIFO_SIZE];
alignas(4) static uint8_t s_nRFBleTxFifoMem[NRFBLEINTRF_CFIFO_SIZE];

/**
 * @brief - Disable
 * 		Turn off the interface.  If this is a physical interface, provide a
 * way to turn off for energy saving. Make sure the turn off procedure can
 * be turned back on without going through the full init sequence
 *
 * @param
 * 		pDevIntrf : Pointer to an instance of the Device Interface
 *
 * @return None
 */
void BleIntrfDisable(DevIntrf_t *pDevIntrf)
{
	// TODO:
}

/**
 * @brief - Enable
 * 		Turn on the interface.
 *
 * @param
 * 		pDevIntrf : Pointer to an instance of the Device Interface
 *
 * @return None
 */
void BleIntrfEnable(DevIntrf_t *pDevIntrf)
{
	// TODO:
}

/**
 * @brief - GetRate
 * 		Get data rate of the interface in Hertz.  This is not a clock frequency
 * but rather the transfer frequency (number of transfers per second). It has meaning base on the
 * implementation as bits/sec or bytes/sec or whatever the case
 *
 * @param
 * 		pDevIntrf : Pointer to an instance of the Device Interface
 *
 * @return Transfer rate per second
 */
uint32_t BleIntrfGetRate(DevIntrf_t *pDevIntrf)
{
	return 0;	// BLE has no rate
}

/**
 * @brief - SetRate
 * 		Set data rate of the interface in Hertz.  This is not a clock frequency
 * but rather the transfer frequency (number of transfers per second). It has meaning base on the
 * implementation as bits/sec or bytes/sec or whatever the case
 *
 * @param
 * 		pDevIntrf : Pointer to an instance of the Device Interface
 * 		Rate 	  : Data rate to be set in Hertz (transfer per second)
 *
 * @return 	Actual transfer rate per second set.  It is the real capable rate
 * 			closes to rate being requested.
 */
uint32_t BleIntrfSetRate(DevIntrf_t *pDevIntrf, uint32_t Rate)
{
	return 0; // BLE has no rate
}

/**
 * @brief - StartRx
 * 		Prepare start condition to receive data with subsequence RxData.
 * This can be in case such as start condition for I2C or Chip Select for
 * SPI or precondition for DMA transfer or whatever requires it or not
 * This function must check & set the busy state for re-entrancy
 *
 * @param
 * 		pDevIntrf : Pointer to an instance of the Device Interface
 * 		DevAddr   : The device selection id scheme
 *
 * @return 	true - Success
 * 			false - failed.
 */
bool BleIntrfStartRx(DevIntrf_t *pDevIntrf, uint32_t DevAddr)
{
	return true;
}

/**
 * @brief - RxData : retrieve 1 packet of received data
 * 		Receive data into pBuff passed in parameter.  Assuming StartRx was
 * called prior calling this function to get the actual data. BufferLen
 * to receive data must be at least 1 packet in size.  Otherwise remaining
 * bytes are dropped.
 *
 * @param
 * 		pDevIntrf : Pointer to an instance of the Device Interface
 * 		pBuff 	  : Pointer to memory area to receive data.
 * 		BuffLen   : Length of buffer memory in bytes. Must be at least 1 packet
 * 		            in size.  Otherwise remaining bytes are dropped.
 *
 * @return	Number of bytes read
 */
int BleIntrfRxData(DevIntrf_t *pDevIntrf, uint8_t *pBuff, int BuffLen)
{
	BleIntrf_t *intrf = (BleIntrf_t*)pDevIntrf->pDevData;
	BleIntrfPkt_t *pkt;
	int cnt = 0;

	pkt = (BleIntrfPkt_t *)CFifoGet(intrf->hRxFifo);
	if (pkt != NULL)
	{
	    cnt = min(BuffLen, pkt->Len);
		memcpy(pBuff, pkt->Data, cnt);
	}

	return cnt;
}

/**
 * @brief - StopRx
 * 		Completion of read data phase. Do require post processing
 * after data has been received via RxData
 * This function must clear the busy state for re-entrancy
 *
 * @param
 * 		pDevIntrf : Pointer to an instance of the Device Interface
 *
 * @return	None
 */
void BleIntrfStopRx(DevIntrf_t *pSerDev)
{
	// TODO:
}

/**
 * @brief - StartTx
 * 		Prepare start condition to transfer data with subsequence TxData.
 * This can be in case such as start condition for I2C or Chip Select for
 * SPI or precondition for DMA transfer or whatever requires it or not
 * This function must check & set the busy state for re-entrancy
 *
 * @param
 * 		pDevIntrf : Pointer to an instance of the Device Interface
 * 		DevAddr   : The device selection id scheme
 *
 * @return 	true - Success
 * 			false - failed
 */
bool BleIntrfStartTx(DevIntrf_t *pDevIntrf, uint32_t DevAddr)
{
	return true;
}

bool BleIntrfNotify(BleIntrf_t *pIntrf)
{
	BleIntrfPkt_t *pkt = NULL;
    uint32_t res = NRF_SUCCESS;

    if (pIntrf->TransBuffLen > 0)
    {
        res = BleSrvcCharNotify(pIntrf->pBleSrv, pIntrf->TxCharIdx, pIntrf->TransBuff, pIntrf->TransBuffLen);
    }

    while (res == NRF_SUCCESS)
	{
		pIntrf->TransBuffLen = 0;
		uint32_t state = DisableInterrupt();
		pkt = (BleIntrfPkt_t *)CFifoGet(pIntrf->hTxFifo);
		EnableInterrupt(state);
		if (pkt == NULL)
		{
			return true;
		}
		res = BleSrvcCharNotify(pIntrf->pBleSrv, pIntrf->TxCharIdx, pkt->Data, pkt->Len);
	}

	if (pkt != NULL)
	{
		memcpy(pIntrf->TransBuff, pkt->Data, pkt->Len);
		pIntrf->TransBuffLen = pkt->Len;
	}

	return false;
}

/**
 * @brief - TxData
 * 		Transfer data from pData passed in parameter.  Assuming StartTx was
 * called prior calling this function to send the actual data
 *
 * @param
 * 		pDevIntrf : Pointer to an instance of the Device Interface
 * 		pData 	: Pointer to memory area of data to send.
 * 		DataLen : Length of data memory in bytes
 *
 * @return	Number of bytes sent
 */
int BleIntrfTxData(DevIntrf_t *pDevIntrf, uint8_t *pData, int DataLen)
{
	BleIntrf_t *intrf = (BleIntrf_t*)pDevIntrf->pDevData;
	BleIntrfPkt_t *pkt;
    int maxlen = intrf->PacketSize - BLEINTRF_PKHDR_LEN;
	int cnt = 0;

	while (DataLen > 0)
	{
	    uint32_t state = DisableInterrupt();
		pkt = (BleIntrfPkt_t *)CFifoPut(intrf->hTxFifo);
		EnableInterrupt(state);
		if (pkt == NULL)
		{
			intrf->TxDropCnt++;
			break;
		}
		int l = min(DataLen, maxlen);
		memcpy(pkt->Data, pData, l);
		pkt->Len = l;
		DataLen -= l;
		pData += l;
		cnt += l;
	}

    BleIntrfNotify(intrf);

	return cnt;
}

/**
 * @brief - StopTx
 * 		Completion of sending data via TxData.  Do require post processing
 * after all data was transmitted via TxData.
 * This function must clear the busy state for re-entrancy
 *
 * @param
 * 		pDevIntrf : Pointer to an instance of the Device Interface
 *
 * @return	None
 */
void BleIntrfStopTx(DevIntrf_t *pDevIntrf)
{

}

/**
 * @brief - Reset
 *      This function perform a reset of interface.  Must provide empty
 * function of not used.
 *
 * @param
 *      pDevIntrf : Pointer to an instance of the Device Interface
 *
 * @return  None
 */
void BleIntrfReset(DevIntrf_t *pDevIntrf)
{

}

/**
 *
 *
 */
void BleIntrfTxComplete(BleSrvc_t *pBleSvc, int CharIdx)
{
    BleIntrfNotify((BleIntrf_t*)pBleSvc->pContext);
}

void BleIntrfRxWrCB(BleSrvc_t *pBleSvc, uint8_t *pData, int Offset, int Len)
{
	BleIntrf_t *intrf = (BleIntrf_t*)pBleSvc->pContext;
	BleIntrfPkt_t *pkt;
    //int maxlen = intrf->hTxFifo->BlkSize - sizeof(pkt->Len);

	while (Len > 0) {
		pkt = (BleIntrfPkt_t *)CFifoPut(intrf->hRxFifo);
		if (pkt == NULL)
		{
			intrf->RxDropCnt++;
			break;
		}
		int l = min(intrf->PacketSize - (int)BLEINTRF_PKHDR_LEN, Len);
		memcpy(pkt->Data, pData, l);
		pkt->Len = l;
		Len -= l;
		pData += l;
	}

	if (CFifoUsed(intrf->hRxFifo) > 0 && intrf->DevIntrf.EvtCB != NULL)
	{
		intrf->DevIntrf.EvtCB(&intrf->DevIntrf, DEVINTRF_EVT_RX_DATA, NULL, 0);
	}
}

bool BleIntrfInit(BleIntrf_t *pBleIntrf, const BleIntrfCfg_t *pCfg)
{
	if (pBleIntrf == NULL || pCfg == NULL)
		return false;

	if (pCfg->PacketSize <= 0)
	{
		pBleIntrf->PacketSize = NRFBLEINTRF_PACKET_SIZE + BLEINTRF_PKHDR_LEN;
	}
	else
	{
		pBleIntrf->PacketSize = pCfg->PacketSize + BLEINTRF_PKHDR_LEN;
	}

	if (pCfg->pRxFifoMem == NULL || pCfg->pTxFifoMem == NULL)
	{
		pBleIntrf->hRxFifo = CFifoInit(s_nRFBleRxFifoMem, NRFBLEINTRF_CFIFO_SIZE, pBleIntrf->PacketSize, pCfg->bBlocking);
		pBleIntrf->hTxFifo = CFifoInit(s_nRFBleTxFifoMem, NRFBLEINTRF_CFIFO_SIZE, pBleIntrf->PacketSize, pCfg->bBlocking);
	}
	else
	{
		pBleIntrf->hRxFifo = CFifoInit(pCfg->pRxFifoMem, pCfg->RxFifoMemSize, pBleIntrf->PacketSize, pCfg->bBlocking);
		pBleIntrf->hTxFifo = CFifoInit(pCfg->pTxFifoMem, pCfg->TxFifoMemSize, pBleIntrf->PacketSize, pCfg->bBlocking);
	}

	pBleIntrf->DevIntrf.pDevData = (void*)pBleIntrf;
	pBleIntrf->pBleSrv = pCfg->pBleSrv;
	pBleIntrf->pBleSrv->pContext = pBleIntrf;

	pBleIntrf->RxCharIdx = pCfg->RxCharIdx;
	pBleIntrf->TxCharIdx = pCfg->TxCharIdx;

	pBleIntrf->pBleSrv->pCharArray[pBleIntrf->RxCharIdx].WrCB = BleIntrfRxWrCB;
	pBleIntrf->pBleSrv->pCharArray[pBleIntrf->TxCharIdx].TxCompleteCB = BleIntrfTxComplete;

	pBleIntrf->DevIntrf.Type = DEVINTRF_TYPE_BLE;
	pBleIntrf->DevIntrf.Enable = BleIntrfEnable;
	pBleIntrf->DevIntrf.Disable = BleIntrfDisable;
	pBleIntrf->DevIntrf.GetRate = BleIntrfGetRate;
	pBleIntrf->DevIntrf.SetRate = BleIntrfSetRate;
	pBleIntrf->DevIntrf.StartRx = BleIntrfStartRx;
	pBleIntrf->DevIntrf.RxData = BleIntrfRxData;
	pBleIntrf->DevIntrf.StopRx = BleIntrfStopRx;
	pBleIntrf->DevIntrf.StartTx = BleIntrfStartTx;
	pBleIntrf->DevIntrf.TxData = BleIntrfTxData;
	pBleIntrf->DevIntrf.StopTx = BleIntrfStopTx;
	pBleIntrf->DevIntrf.MaxRetry = 0;
	pBleIntrf->DevIntrf.EvtCB = pCfg->EvtCB;
	pBleIntrf->TransBuffLen = 0;
	pBleIntrf->RxDropCnt = 0;
	pBleIntrf->TxDropCnt = 0;
	atomic_flag_clear(&pBleIntrf->DevIntrf.bBusy);

	DeviceIntrfEnable(&pBleIntrf->DevIntrf);

	return true;
}

bool BleIntrf::Init(const BleIntrfCfg_t &Cfg)
{
	return BleIntrfInit(&vBleIntrf, &Cfg);
}

bool BleIntrf::RequestToSend(int NbBytes)
{
	bool retval = false;

	// ****
	// Some threaded application firmware may stop sending when queue full
	// causing lockup
	// Try to send to free up the queue before validating.
	//
	BleIntrfNotify(&vBleIntrf);

	if (vBleIntrf.hTxFifo)
	{
		int avail = CFifoAvail(vBleIntrf.hTxFifo);
		if ((avail * (vBleIntrf.PacketSize - BLEINTRF_PKHDR_LEN)) > NbBytes)
			retval = true;
	}
	else
	{
		retval = true;
	}

	return retval;
}

