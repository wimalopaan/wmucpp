#include <avr/io.h>

#define SIGNAL_PERIOD_EXAMPLE_VALUE            (0xC8)
#define SIGNAL_DUTY_CYCLE_EXAMPLE_VALUE        (0x64)

void TCD0_init(void)
{    
    /* set the waveform mode */
//    TCD0.CTRLB = TCD_WGMODE_DS_gc;
    
    /* set the signal period */
    TCD0.CMPBCLR = SIGNAL_PERIOD_EXAMPLE_VALUE;    
    
    /* enable write protected register */
    CPU_CCP = CCP_IOREG_gc;    
    
    TCD0.FAULTCTRL = TCD_CMPAEN_bm            /* enable channel A */
                   | TCD_CMPBEN_bm;            /* enable channel B */

    /* the signals are alternatively active and a small
       symmetric dead time is needed */
    TCD0.CMPBSET = SIGNAL_DUTY_CYCLE_EXAMPLE_VALUE + 1;                                        
    TCD0.CMPASET = SIGNAL_DUTY_CYCLE_EXAMPLE_VALUE - 1;    
    
    /* ensure ENRDY bit is set */
    while(!(TCD0.STATUS & TCD_ENRDY_bm))
    {
        ;
    }
    
//    TCD0.CTRLA = TCD_CLKSEL_20MHZ_gc        /* choose the timer's clock */
//               | TCD_CNTPRES_DIV1_gc        /* choose the prescaler */
//               | TCD_ENABLE_bm                /* enable the timer */
//                 | TCD_SYNCPRES_DIV1_bm; /*  */
                 

    TCD0.CTRLA = 1 << TCD_ENABLE_bp      /* Enable: enabled */
	             | TCD_CLKSEL_20MHZ_gc   /*  */
	             | TCD_CNTPRES_DIV1_gc;   /* Sync clock divided by 1 */
//	             | TCD_SYNCPRES_DIV1_gc; /*  */
}


void PORT_init(void)
{
    PORTA.DIRSET = PIN6_bm            /* set pin 4 as output */
                    | PIN7_bm        /* set pin 5 as output */
                    | PIN3_bm;
}


int main() {
    PORT_init();
    TCD0_init();

    while(true) {
        PORTA.OUTTGL = PIN3_bm;
    }
    
}

