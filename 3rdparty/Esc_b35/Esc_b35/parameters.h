

//*********************
// Options
//*********************
#define F_CPU	16000000L
#define VERSION	(0x27)
//#define   SKIP_BEEP
#define   INPUT_RANGE     (0xFF)
#define   PWM_RANGE       200 // number of  counts for a full PWM cycle
#define   STARTUP_POWER   (20) // Start up Power (0-255)

#define COMPERTURN      (42) // 7 pole motor with 3 phase stator (poles*phases*2)
#define T1MHZFREQ       (F_CPU/8/1000000L) // T1 frequency in MegaHertz
#define TIMING_N_MEAS   (4) // number of comutations in "timing" variable
#define COM_DELAY_FRAC  (4) //  (1 / Fraction of cycle to delay after zero cross) 

#define RPM_PER_STEP    (30) //Used in governor mode
#define MAX_GOV_PWM     (0x80)

#define BAUD            38400

#define timeoutSTART	65000
#define timeoutMIN	48000

#define RPM_MIN         (200)
#define RPM_RANGE	4800	// ( RPM )
#define ENOUGH_GOODIES	12 // number of good startup cycles before going to Run mode

#define UART_TOT_MS      2000// milliseconds for UART, no-message timeout




//*********************
// Configuration
//********************* 
#define   MOTOR_ID	  1	


//------------- PORT DEFINITION -------------------
//
//  ***** IMPORTANT !!! ****
//   Pin asignment changes depending on ESC hardware design
//   Wrong asignment of pins might shor-circuit the P and N FETS
//   with catastrofic results, so pay ATTENTION to this definitions 
//   before burning the firmware.

//*********************
// PORT B definitions *
//*********************
#define	AnFET_port	PORTB

#define	AnFET_pin       0


//*********************
// PORT C definitions *
//*********************
#define	ApFET_port      PORTC
#define	BnFET_port      PORTC
#define	BpFET_port      PORTC

#define	ApFET_pin	3
#define	BnFET_pin	4
#define	BpFET_pin	5


//*********************
// PORT D 
//*********************  
#define	CpFET_port      PORTD
#define	CnFET_port      PORTD

#define	CpFET_pin	4
#define	CnFET_pin	5


//*********************
// ANALOGUE
//*********************  

#define ACCU_MUX  2 	//ADC7 voltage control input
#define	c_comp	  6 // common comparator input (AIN0)

#define mux_a	  0	// ADC2 phase input
#define mux_b     7     // ADC3 phase input
#define mux_c	  6	// ADC1 phase input
