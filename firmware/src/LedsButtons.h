/**-------------------------------------------------------------------------
@file	Leds.h

@brief	Header file for LEDs and Buttons functionalities in the BlueIO832Mini and BlueIO840Wizard projects

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

#ifndef __LEDS_H__
#define __LEDS_H__

#include "app_scheduler.h"
#include "blueio_board.h"
#include "iopinctrl.h"
#include "coredev/iopincfg.h"
#include "coredev/uart.h"
#include "ble_app.h"
#include "ble_service.h"
#include "bluetooth/blueio_blesrvc.h"
#include "ble_intrf.h"


void ButEvent(int IntNo, void *pCtx);



#endif //__LEDS_H__
