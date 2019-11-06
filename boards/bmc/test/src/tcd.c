/**
 * \file
 *
 * \brief TCD related functionality implementation.
 *
 (c) 2018 Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms,you may use this software and
    any derivatives exclusively with Microchip products.It is your responsibility
    to comply with third party license terms applicable to your use of third party
    software (including open source software) that may accompany Microchip software.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 */

/**
 * \addtogroup doc_driver_tcd
 *
 * \section doc_driver_tcd_rev Revision History
 * - v0.0.0.1 Initial Commit
 *
 *@{
 */
#include "tcd.h"
#include "ccp.h"

/**
 * \brief Initialize TCD interface
 *
 * \return Initialization status.
 */
int8_t TIMER_0_init()
{

	// Enable Protected register, peripheral must be disabled (ENABLE=0, in TCD.CTRLA).

	TCD0.CMPASET = 100; /* Compare A Set: 100 */

	TCD0.CMPACLR = 110; /* Compare A Clear: 110 */

	TCD0.CMPBSET = 120; /* Compare B Set: 120 */

	TCD0.CMPBCLR = 800; /* Compare B Clear: 200 */

	// TCD0.CTRLB = TCD_WGMODE_ONERAMP_gc; /* One ramp mode */

	// TCD0.CTRLC = 0 << TCD_AUPDATE_bp /* Auto update: disabled */
	//		 | TCD_CMPCSEL_PWMA_gc /* PWM A output */
	//		 | TCD_CMPDSEL_PWMA_gc /* PWM A output */
	//		 | 0 << TCD_CMPOVR_bp /* Compare output value override: disabled */
	//		 | 0 << TCD_FIFTY_bp; /* Fifty percent waveform: disabled */

	// TCD0.CTRLD = 0 << TCD_CMPAVAL_gp /* Compare A value: 0 */
	//		 | 0 << TCD_CMPBVAL_gp; /* Compare B value: 0 */

	// TCD0.DBGCTRL = 0 << TCD_DBGRUN_bp /* Debug run: disabled */
	//		 | 0 << TCD_FAULTDET_bp; /* Fault detection: disabled */

	// TCD0.DITCTRL = TCD_DITHERSEL_ONTIMEB_gc; /* On-time ramp B */

	// TCD0.DITVAL = 0; /* Dither value: 0 */

	// TCD0.DLYCTRL = TCD_DLYPRESC_DIV1_gc /* No prescaling */
	//		 | TCD_DLYSEL_OFF_gc /* No delay */
	//		 | TCD_DLYTRIG_CMPASET_gc; /* Compare A set */

	// TCD0.DLYVAL = 0 << TCD_DLYVAL_gp; /* Delay value: 0 */

	// TCD0.EVCTRLA = TCD_CFG_NEITHER_gc /* Neither Filter nor Asynchronous Event is enabled */
	//		 | TCD_ACTION_FAULT_gc /* Event trigger a fault */
	//		 | TCD_EDGE_FALL_LOW_gc /* The falling edge or low level of event generates retrigger or fault action */
	//		 | 0 << TCD_TRIGEI_bp; /* Trigger event enable: disabled */

	// TCD0.EVCTRLB = TCD_CFG_NEITHER_gc /* Neither Filter nor Asynchronous Event is enabled */
	//		 | TCD_ACTION_FAULT_gc /* Event trigger a fault */
	//		 | TCD_EDGE_FALL_LOW_gc /* The falling edge or low level of event generates retrigger or fault action */
	//		 | 0 << TCD_TRIGEI_bp; /* Trigger event enable: disabled */

	ccp_write_io((void *)&(TCD0.FAULTCTRL),
	             1 << TCD_CMPAEN_bp       /* Compare A enable: enabled */
	                 | 0 << TCD_CMPA_bp   /* Compare A value: disabled */
	                 | 0 << TCD_CMPB_bp   /* Compare B value: disabled */
	                 | 1 << TCD_CMPBEN_bp /* Compare B enable: enabled */
	                 | 0 << TCD_CMPC_bp   /* Compare C value: disabled */
	                 | 0 << TCD_CMPCEN_bp /* Compare C enable: disabled */
	                 | 0 << TCD_CMPD_bp   /* Compare D vaule: disabled */
	                 | 0 << TCD_CMPDEN_bp /* Compare D enable: disabled */);

	// TCD0.INPUTCTRLA = TCD_INPUTMODE_NONE_gc << TCD_INPUTMODE_gp; /* Input has no actions */

	// TCD0.INPUTCTRLB = TCD_INPUTMODE_NONE_gc << TCD_INPUTMODE_gp; /* Input has no actions */

	// TCD0.INTCTRL = 0 << TCD_OVF_bp /* Overflow interrupt enable: disabled */
	//		 | 0 << TCD_TRIGA_bp /* Trigger A interrupt enable: disabled */
	//		 | 0 << TCD_TRIGB_bp; /* Trigger B interrupt enable: disabled */

	while ((TCD0.STATUS & TCD_ENRDY_bm) == 0)
		; // Wait for Enable Ready to be high.

	TCD0.CTRLA = 1 << TCD_ENABLE_bp      /* Enable: enabled */
	             | TCD_CLKSEL_20MHZ_gc   /*  */
	             | TCD_CNTPRES_DIV1_gc   /* Sync clock divided by 1 */
                      ;
//	             | TCD_SYNCPRES_DIV1_gc; /*  */

	return 0;
}
