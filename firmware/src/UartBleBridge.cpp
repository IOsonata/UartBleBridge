/**-------------------------------------------------------------------------
@example	UartBleBridge.cpp


@brief	UartBleBridge 52832 & 52840 boards main firmware


@author Thinh Tran
@date	Apr. 25, 2022

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

#include "app_util_platform.h"
#include "app_scheduler.h"

#include "istddef.h"
#include "ble_app.h"
#include "ble_service.h"
#include "ble_intrf.h"
#include "bluetooth/blueio_blesrvc.h"
#include "blueio_board.h"
#include "coredev/uart.h"
#include "custom_board.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "nrf_sdh.h"

#include "board.h"
//#include "coredev/timer.h"
#include "idelay.h"
#include "UartBle.h"
#include "Timers.h"
#include "LedsButtons.h"


IOPinCfg_t s_ButPins[] = BUTTON_PIN_MAP;
static int s_NbButPins = sizeof(s_ButPins) / sizeof(IOPinCfg_t);

IOPinCfg_t s_Leds[] = LED_PIN_MAP;
static int s_NbLeds = sizeof(s_Leds) / sizeof(IOPinCfg_t);

IOPinCfg_t s_UartPins[] = UART_PIN_MAP;
static int s_NbUartPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t);

/*
 * Declaration of Timer2
 * */
extern Timer g_Timer2;

/*
 * BLE Interface object
 */
BleIntrf g_BleIntrf;
alignas(4) static uint8_t s_BleIntrfRxFifo[BLEINTRF_FIFOSIZE];
alignas(4) static uint8_t s_BleIntrfTxFifo[BLEINTRF_FIFOSIZE];

/*
 * UART object instance
 */
extern UART g_Uart;
//volatile int g_UartRxBuffLen = 0;
std::atomic<int> g_UartRxBuffLen(0);
volatile bool g_bUartState = false;


/*
 * UART section
 */
static const char s_UartRxCharDescString[] = {
	"UART Rx characteristic",
};
static const char s_UartTxCharDescString[] = {
	"UART Tx characteristic",
};
static const char s_UartCfgDescString[] = {
	"UART Configuration characteristic",
};

uint8_t g_ManData[8];// Manufacture advertise data

/// UART characteristic definitions
BleSrvcChar_t g_UartChars[] = {
	{
		// Read characteristic
		.Uuid = BLE_UART_UUID_RX_CHAR,
		.MaxDataLen = UART_MAX_DATA_LEN,
		.Property = BLE_UART_UUID_RX_CHAR_PROP,
		.pDesc = s_UartRxCharDescString,	// char UTF-8 description string
		.WrCB = NULL,						// Callback for write char, set to NULL for read char
		.SetNotifCB = NULL,					// Callback on set notification
		.TxCompleteCB = NULL,				// Tx completed callback
		.pDefValue = NULL,					// pointer to char default values
		.ValueLen = 0,						// Default value length in bytes
	},
	{
		// Write characteristic
		.Uuid = BLE_UART_UUID_TX_CHAR,		// char UUID
		.MaxDataLen = UART_MAX_DATA_LEN,			// char max data length
		.Property = BLE_UART_UUID_TX_CHAR_PROP,// char properties define by BLUEIOSVC_CHAR_PROP_...
		.pDesc = s_UartTxCharDescString,		// char UTF-8 description string
		.WrCB = NULL,//UartTxSrvcCallback,			// Callback for write char, set to NULL for read char
		.SetNotifCB = NULL,					// Callback on set notification
		.TxCompleteCB = NULL,				// Tx completed callback
		.pDefValue = NULL,					// pointer to char default values
		.ValueLen = 0						// Default value length in bytes
	},
};

// Number of BLE UART characteristics
static const int s_BleUartNbChar = sizeof(g_UartChars) / sizeof(BLESRVC_CHAR);

uint8_t g_UartLWrBuffer[512];

/// BlueIO UART service definition
static const BleSrvcCfg_t s_UartSrvcCfg = {
	.SecType = BLESRVC_SECTYPE_NONE,			// Secure or Open service/char
	.UuidBase = {BLE_UART_UUID_BASE,},			// Base UUID
	.NbUuidBase = 1,
	.UuidSvc = BLE_UART_UUID_SERVICE,			// Service UUID
	.NbChar = s_BleUartNbChar,					// Total number of characteristics for the service
	.pCharArray = g_UartChars,					// Pointer a an array of characteristic
	.pLongWrBuff = g_UartLWrBuffer,				// Pointer to user long write buffer
	.LongWrBuffSize = sizeof(g_UartLWrBuffer),	// long write buffer size
};

BleSrvc_t g_UartBleSrvc;

static const BleIntrfCfg_t s_BleIntrfCfg = {
	.pBleSrv = &g_UartBleSrvc,
	.RxCharIdx = BLESRV_WRITE_CHAR_IDX,
	.TxCharIdx = BLESRV_READ_CHAR_IDX,
	.PacketSize = BLEINTRF_PKTSIZE,			// Packet size : use default
	.bBlocking = true,
	.RxFifoMemSize = BLEINTRF_FIFOSIZE,			// Rx Fifo mem size
	.pRxFifoMem = s_BleIntrfRxFifo,		// Rx Fifo mem pointer
	.TxFifoMemSize = BLEINTRF_FIFOSIZE,			// Tx Fifo mem size
	.pTxFifoMem = s_BleIntrfTxFifo,		// Tx Fifo mem pointer
	.EvtCB = BleIntrfEvtCallback
};


/*
 * BlueIO BLE App configuration
 */
const BLEAPP_DEVDESC s_UartBleDevDesc = {
	MODEL_NAME,       			// Model name
	MANUFACTURER_NAME,			// Manufacturer name
	"0609YYXXXXXX",					// Serial number string
	"1.0",						// Firmware version string
	"1.0",						// Hardware version string
};

//BLE_UUID_TYPE_BLE
const ble_uuid_t s_UartBleSrvAdvUuid = {
	.uuid = BLE_UART_UUID_SERVICE,
	.type = BLE_UUID_TYPE_BLE,
};

/* BlueIO BLE App configuration */
const BleAppCfg_t s_BleAppCfg = {
	.ClkCfg = { NRF_CLOCK_LF_SRC_XTAL, 0, 0, NRF_CLOCK_LF_ACCURACY_20_PPM},
	.CentLinkCount = 0, 					// Number of central link
	.PeriLinkCount = 1, 					// Number of peripheral link
	.AppMode = BLEAPP_MODE_APPSCHED,		// Use scheduler
	.pDevName = DEVICE_NAME,				// Device name
	.VendorID = ISYST_BLUETOOTH_ID,			// PnP Bluetooth/USB vendor id
	.ProductId = 1,							// PnP Product ID
	.ProductVer = 0,						// Pnp prod version
	.bEnDevInfoService = true,				// Enable device information service (DIS)
	.pDevDesc = &s_UartBleDevDesc,			// App description
	.pAdvManData = g_ManData,				// Manufacture specific data to advertise
	.AdvManDataLen = sizeof(g_ManData),		// Length of manufacture specific data
	.pSrManData = NULL,
	.SrManDataLen = 0,
	.SecType = BLEAPP_SECTYPE_NONE,//BLEAPP_SECTYPE_STATICKEY_MITM,			//BLEAPP_SECTYPE_STATICKEY_MITM,//BLEAPP_SECTYPE_NONE,    // Secure connection type
	.SecExchg = BLEAPP_SECEXCHG_NONE,		// Security key exchange
	.pAdvUuids = NULL,//&s_UartBleSrvAdvUuid, //NULL,      				// Service uuids to advertise
	.NbAdvUuid = 0, 						// Total number of uuids
	.AdvInterval = APP_ADV_INTERVAL,		// Advertising interval in msec
	.AdvTimeout = APP_ADV_TIMEOUT,			// Advertising timeout in sec
	.AdvSlowInterval = 0,					// Slow advertising interval, if > 0, fallback to
											// slow interval on adv timeout and advertise until connected
	.ConnIntervalMin = MIN_CONN_INTERVAL,
	.ConnIntervalMax = MAX_CONN_INTERVAL,
	.ConnLedPort = LED1_PORT,				// Led port nuber
	.ConnLedPin = LED1_PIN,					// Led pin number
	.TxPower = 0,							// Tx power
	.SDEvtHandler = NULL,					// RTOS Softdevice handler
	.MaxMtu = BLE_MTU_SIZE,
};


/// UART operation mode config
alignas(4) uint8_t s_UartRxFifo[UARTFIFOSIZE];
alignas(4) uint8_t s_UartTxFifo[UARTFIFOSIZE];

UARTCfg_t g_UartCfg = {
	.DevNo = 0,									// Device number zero based
	.pIOPinMap = s_UartPins,					// UART assigned pins
	.NbIOPins = s_NbUartPins,					// Total number of UART pins used
	.Rate = 115200,								// Baudrate
	.DataBits = 8,								// Data bits
	.Parity = UART_PARITY_NONE,			// Parity
	.StopBits = 1,								// Stop bit
	.FlowControl = UART_FLWCTRL_NONE,	// Flow control
	.bIntMode = true,							// Interrupt mode
	.IntPrio = APP_IRQ_PRIORITY_LOW,			// Interrupt priority
	.EvtCallback = nRFUartEvtHandler,			// UART event handler
	.bFifoBlocking = true,						// Blocking FIFO
	.RxMemSize = UARTFIFOSIZE,
	.pRxMem = s_UartRxFifo,
	.TxMemSize = UARTFIFOSIZE,
	.pTxMem = s_UartTxFifo,
	.bDMAMode = true,
};


/**
 * Timer configuration
 * @return
 */
TimerCfg_t s_Timer2Cfg = {
    .DevNo = 2, // !!!: TIMER0 and TIMER1 are occupied by SoftDevice
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,			// 0 => Default frequency
	.IntPrio = APP_IRQ_PRIORITY_LOW,
	.EvtHandler = Timer2Handler
};


/**
 * Register the user' BLE services into the BLE stack
 * */
void BleAppInitUserServices()
{
    uint32_t err_code;

    err_code = BleSrvcInit(&g_UartBleSrvc, &s_UartSrvcCfg);
    APP_ERROR_CHECK(err_code);

    msDelay(100);
}


/*
 * Register user's peripheral evt handler into BLE stack
 * */
void BlePeriphEvtUserHandler(ble_evt_t * p_ble_evt)
{
    BleSrvcEvtHandler(&g_UartBleSrvc, p_ble_evt);// UART service
}


/**
 * Initialize hardware components
 */
void HardwareInit()
{
	// UART
	g_Uart.Init(g_UartCfg);
	msDelay(100);

	// LEDs
	IOPinCfg(s_Leds, s_NbLeds);
	IOPinSet(BLUEIO_LED_BLUE_PORT, BLUEIO_LED_BLUE_PIN);//LED1

	IOPinSet(BLUEIO_LED_GREEN_PORT, BLUEIO_LED_GREEN_PIN);//LED2_GREEN
	IOPinSet(BLUEIO_LED_RED_PORT, BLUEIO_LED_RED_PIN);//LED2_RED
	IOPinSet(s_Leds[3].PortNo, s_Leds[3].PinNo);//LED2_BLUE

	// Buttons
	IOPinCfg(s_ButPins, s_NbButPins);

	// Timer
	g_Timer2.Init(s_Timer2Cfg);
	msDelay(100);

	// Interrupts
	IOPinEnableInterrupt(0, APP_IRQ_PRIORITY_LOW, s_ButPins[0].PortNo, s_ButPins[0].PinNo, IOPINSENSE_LOW_TRANSITION, ButEvent, NULL);

	g_Uart.printf("UART-BLE bridge is on!\r\n");

	TestTimerTriggers();
}


/**
 * Initialize BLE's dependencies such as
 * bluetooth passkey
 * timer
 * etc.
 */
void BleAppInitUserData()
{
//	// Add passkey pairing
//    ble_opt_t opt;
//    opt.gap_opt.passkey.p_passkey = (uint8_t*)"123456";
//    uint32_t err_code =  sd_ble_opt_set(BLE_GAP_OPT_PASSKEY, &opt);
//    APP_ERROR_CHECK(err_code);

}


/*
 * Buffer for BLE_Rx data
 */
int BleIntrfEvtCallback(DevIntrf_t *pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	int cnt = 0;
	IOPinToggle(s_Leds[1].PortNo, s_Leds[1].PinNo);
	if (EvtId == DEVINTRF_EVT_RX_DATA)
	{
		uint8_t BleIntrfBuff[BLEINTRF_PKTSIZE];
		int l = g_BleIntrf.Rx(0, BleIntrfBuff, BLEINTRF_PKTSIZE);
		// Forward data to UART interface
		g_Uart.printf("Mobile app [%d bytes]: ", l);
		if (l > 0)
		{
			g_Uart.Tx(BleIntrfBuff, l);
		}
		cnt += l;
	}
	g_Uart.printf("\r\n");
	IOPinToggle(s_Leds[1].PortNo, s_Leds[1].PinNo);

	return cnt;
}



//
// Print a greeting message on standard output and exit.
//
// On embedded platforms this might require semi-hosting or similar.
//
// For example, for toolchains derived from GNU Tools for Embedded,
// to enable semi-hosting, the following was added to the linker:
//
// --specs=rdimon.specs -Wl,--start-group -lgcc -lc -lm -lrdimon -Wl,--end-group
//
// Adjust it for other toolchains.
//

int main()
{
    HardwareInit();
    g_Uart.printf("Default UART Configuration: %d, %d, %d\r\n", \
        		g_UartCfg.Rate, g_UartCfg.FlowControl, g_UartCfg.Parity);

    BleAppInit((const BLEAPP_CFG *)&s_BleAppCfg, true);

    g_BleIntrf.Init(s_BleIntrfCfg);

    BleAppRun();

	return 0;
}
