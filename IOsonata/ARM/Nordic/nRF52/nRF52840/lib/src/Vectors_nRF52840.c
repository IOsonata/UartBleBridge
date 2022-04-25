/*--------------------------------------------------------------------------
File   : Vector_nRF52840.c

Author : Hoang Nguyen Hoan          Apr. 26, 2017

Desc   : Interrupt Vectors table for ARM Cortex-M4 specific nRF52840
		 CMSIS & GCC compiler
		 linker section name .Vectors is used for the table

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
#include <stdint.h>
#include "nrf.h"

extern unsigned long __StackTop;
extern void ResetEntry(void);
extern char Image$$ER_ZI$$Base[];
extern char Image$$ARM_LIB_STACK$$ZI$$Base[];

void DEF_IRQHandler(void) { while(1); }
__attribute__((weak, alias("DEF_IRQHandler"))) void NMI_Handler(void);
/*__attribute__((weak, alias("DEF_IRQHandler")))*/ __WEAK void HardFault_Handler(void) { while(1); }
__attribute__((weak, alias("DEF_IRQHandler"))) void MemoryManagement_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void BusFault_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void UsageFault_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SVC_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void DebugMonitor_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PendSV_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SysTick_Handler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void POWER_CLOCK_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RADIO_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void UARTE0_UART0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void NFCT_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void GPIOTE_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SAADC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER2_IRQHandler(void);
__attribute__((weak/*, alias("DEF_IRQHandler")*/)) void RTC0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TEMP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void RNG_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void ECB_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CCM_AAR_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void WDT_IRQHandler(void);
__attribute__((weak/*, alias("DEF_IRQHandler")*/)) void RTC1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void QDEC_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void COMP_LPCOMP_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWI0_EGU0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWI1_EGU1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWI2_EGU2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWI3_EGU3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWI4_EGU4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SWI5_EGU5_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void TIMER4_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PWM0_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PDM_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void MWU_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PWM1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PWM2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPIM2_SPIS2_SPI2_IRQHandler(void);
__attribute__((weak/*, alias("DEF_IRQHandler")*/)) void RTC2_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void I2S_IRQHandler(void);
#if (__FPU_USED == 1)
__WEAK void FPU_IRQHandler(void);
#else
__attribute__((weak, alias("DEF_IRQHandler"))) void FPU_IRQHandler(void);
#endif
__attribute__((weak, alias("DEF_IRQHandler"))) void USBD_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void UARTE1_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void QSPI_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void CRYPTOCELL_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void SPIM3_IRQHandler(void);
__attribute__((weak, alias("DEF_IRQHandler"))) void PWM3_IRQHandler(void);

/**
 * This interrupt vector is by default located in FLASH. Though it can not be
 * changed at runtime. All functions in the vector are weak.  it can be
 * overloaded by application function
 *
 */
#ifdef __ICCARM__
__attribute__ ((section(".intvec"), used))
void (* const __vector_table[])(void) = {
#else
__attribute__ ((section(".vectors"), used))
void (* const __Vectors[100])(void) = {
#endif
#if defined ( __ARMCC_VERSION )
	(void (*)(void) )((uint32_t)0x20000000 + 0x10000),
	Reset_Handler,
#else
	(void (*)(void) )((uint32_t)&__StackTop),
	ResetEntry,
#endif
	NMI_Handler,
	HardFault_Handler,
	MemoryManagement_Handler,
	BusFault_Handler,
	UsageFault_Handler,
	0, 0, 0, 0,
	SVC_Handler,
	DebugMonitor_Handler,
	0,
	PendSV_Handler,
	SysTick_Handler,

/* External Interrupts */
    POWER_CLOCK_IRQHandler,
    RADIO_IRQHandler,
	UARTE0_UART0_IRQHandler,
	SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler,
	SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler,
	NFCT_IRQHandler,
    GPIOTE_IRQHandler,
	SAADC_IRQHandler,
    TIMER0_IRQHandler,
    TIMER1_IRQHandler,
    TIMER2_IRQHandler,
    RTC0_IRQHandler,
    TEMP_IRQHandler,
    RNG_IRQHandler,
    ECB_IRQHandler,
    CCM_AAR_IRQHandler,
    WDT_IRQHandler,
    RTC1_IRQHandler,
    QDEC_IRQHandler,
	COMP_LPCOMP_IRQHandler,
	SWI0_EGU0_IRQHandler,
	SWI1_EGU1_IRQHandler,
	SWI2_EGU2_IRQHandler,
	SWI3_EGU3_IRQHandler,
	SWI4_EGU4_IRQHandler,
	SWI5_EGU5_IRQHandler,
	TIMER3_IRQHandler,
	TIMER4_IRQHandler,
	PWM0_IRQHandler,
	PDM_IRQHandler,
    0,
    0,
	MWU_IRQHandler,
	PWM1_IRQHandler,
	PWM2_IRQHandler,
	SPIM2_SPIS2_SPI2_IRQHandler,
	RTC2_IRQHandler,
	I2S_IRQHandler,
	FPU_IRQHandler,
    USBD_IRQHandler,
    UARTE1_IRQHandler,
    QSPI_IRQHandler,
    CRYPTOCELL_IRQHandler,
	0,
	0,
    PWM3_IRQHandler,
    0,
    SPIM3_IRQHandler,
	0,
	0,
	0,
};

#if (__FPU_USED == 1)
// Function handles and clears exception flags in FPSCR register and at the stack.
// During interrupt, handler execution FPU registers might be copied to the stack
// (see lazy stacking option) and it is necessary to clear data at the stack
// which will be recovered in the return from interrupt handling.
__WEAK void FPU_IRQHandler(void)
{
    // Prepare pointer to stack address with pushed FPSCR register (0x40 is FPSCR register offset in stacked data)
    uint32_t * fpscr = (uint32_t * )(FPU->FPCAR + 0x40);
    // Execute FPU instruction to activate lazy stacking
    (void)__get_FPSCR();
    // Clear flags in stacked FPSCR register. To clear IDC, IXC, UFC, OFC, DZC and IOC flags, use 0x0000009F mask.
    *fpscr = *fpscr & ~(0x0000009F);
}
#endif

