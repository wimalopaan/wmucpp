#define NDEBUG

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/pwm.h>
#include <mcu/internals/event.h>
#include <mcu/internals/ccl.h>
#include <mcu/internals/pit.h>
//#include <mcu/internals/adc.h>

#include <external/hal/alarmtimer.h>
//#include <external/hal/adccontroller.h>
#include <external/hott/sumdprotocolladapter.h>
#include <external/solutions/series01/rpm.h>
#include <external/solutions/series01/sppm_in.h>

#include <external/hott/experimental/sensor.h>
#include <external/hott/experimental/adapter.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

using PortA = Port<A>;
using PortB = Port<B>;
using PortD = Port<D>;
using PortF = Port<F>;
using PortE = Port<E>;

using led = Pin<PortF, 5>; 
using pf2 = Pin<PortF, 2>; 
using pb1 = Pin<PortB, 1>; 
//using pa2 = Pin<PortA, 2>; 
using pe3 = Pin<PortE, 3>; // ppm (s.u.)
using rpmPin = Pin<PortE, 0>; 

using ccp = Cpu::Ccp<>;
using clock = Clock<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Alt1>;
using usart1Position = Portmux::Position<Component::Usart<1>, Portmux::Default>;
using usart3Position = Portmux::Position<Component::Usart<3>, Portmux::Default>;

using terminalDevice = Usart<usart0Position, External::Hal::NullProtocollAdapter, UseInterrupts<false>>;
using terminal = etl::basic_ostream<terminalDevice>;

using sumd = Hott::SumDProtocollAdapter<0, AVR::UseInterrupts<false>>;
using rcUsart = AVR::Usart<usart1Position, sumd, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::AltD>;
using pwm = PWM::DynamicPwm<tcaPosition>;

using tcbPosition = Portmux::Position<Component::Tcb<0>, Portmux::Default>;

using ccl0Position = Portmux::Position<Component::Ccl<0>, Portmux::Default>;
using ccl1Position = Portmux::Position<Component::Ccl<1>, Portmux::Default>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, usart1Position, usart3Position, tcaPosition, tcbPosition, ccl0Position, ccl1Position>>;
//using portmux = Portmux::StaticMapper<Meta::List<usart0Position, usart1Position, usart3Position>>;
//using portmux = Portmux::StaticMapper<Meta::List<usart0Position>>;

using evch0 = Event::Channel<0, Event::Generators::PitDiv<1024>>;
using evch1 = Event::Channel<4, Event::Generators::Pin<pe3>>; 
using evuser0 = Event::Route<evch0, Event::Users::Lut<0, A>>;
using evuser1 = Event::Route<evch0, Event::Users::Lut<1, A>>;
using evuser2 = Event::Route<evch1, Event::Users::Tcb<0>>;
using evrouter = Event::Router<Event::Channels<evch0, evch1>, Event::Routes<evuser0, evuser1, evuser2>>;

using lut0 = Ccl::SimpleLut<0, Ccl::Input::Tca0<0>, Ccl::Input::Event<A>, Ccl::Input::Mask>;
using lut1 = Ccl::SimpleLut<1, Ccl::Input::Tca0<0>, Ccl::Input::Event<A>, Ccl::Input::Mask>;

using pit = Rtc::Pit<>;

using ppm = External::Ppm::SinglePpmIn<Component::Tcb<0>>; // in andere header Datei verschieben

namespace  {
//    constexpr auto dt = 2_ms;
    constexpr auto dt = 2000_us;
    constexpr auto fRtc = 500_Hz;
    constexpr auto fPwm = 1000_Hz;
}

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
//using systemTimer = SystemTimer<Component::Timer<0, A>, dt>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer>;

using sensor = Hott::Experimental::Sensor<usart3Position, AVR::Usart, AVR::BaudRate<19200>, Hott::GamMsg, systemTimer>;

using rpm = External::Rpm::RpmGpio<rpmPin, systemTimer>;

using isrRegistrar = IsrRegistrar<rpm::ImpulsIsr>;

//using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;
//using adcController = External::Hal::AdcController<adc, 4, 5>;
//using pd4 = Pin<PortD, 4>; 

typedef void (*adc_irq_cb_t)(void);

/** Datatype for the result of the ADC conversion */
typedef uint16_t adc_result_t;

//* Analog channel selection */
typedef ADC_MUXPOS_t adc_0_channel_t;


enum port_pull_mode {
	PORT_PULL_OFF,
	PORT_PULL_UP,
};

enum port_dir {
	PORT_DIR_IN,
	PORT_DIR_OUT,
	PORT_DIR_OFF,
};


static inline void PORTD_set_pin_pull_mode(const uint8_t pin, const enum port_pull_mode pull_mode)
{
	volatile uint8_t *port_pin_ctrl = ((uint8_t *)&PORTD + 0x10 + pin);

	if (pull_mode == PORT_PULL_UP) {
		*port_pin_ctrl |= PORT_PULLUPEN_bm;
	} else if (pull_mode == PORT_PULL_OFF) {
		*port_pin_ctrl &= ~PORT_PULLUPEN_bm;
	}
}

static inline void PORTD_pin_set_inverted(const uint8_t pin, const bool inverted)
{
	volatile uint8_t *port_pin_ctrl = ((uint8_t *)&PORTD + 0x10 + pin);

	if (inverted) {
		*port_pin_ctrl |= PORT_INVEN_bm;
	} else {
		*port_pin_ctrl &= ~PORT_INVEN_bm;
	}
}

static inline void PORTD_pin_set_isc(const uint8_t pin, const PORT_ISC_t isc)
{
	volatile uint8_t *port_pin_ctrl = ((uint8_t *)&PORTD + 0x10 + pin);

	*port_pin_ctrl = (*port_pin_ctrl & ~PORT_ISC_gm) | isc;
}

static inline void PORTD_set_port_dir(const uint8_t mask, const enum port_dir dir)
{
	switch (dir) {
	case PORT_DIR_IN:
		VPORTD.DIR &= ~mask;
		break;
	case PORT_DIR_OUT:
		VPORTD.DIR |= mask;
		break;
	case PORT_DIR_OFF:
		/*/ should activate the pullup for power saving
		  but a bit costly to do it here */
		{
			for (uint8_t i = 0; i < 8; i++) {
				if (mask & 1 << i) {
					*((uint8_t *)&PORTD + 0x10 + i) |= 1 << PORT_PULLUPEN_bp;
				}
			}
		}
		break;
	default:
		break;
	}
}

static inline void PORTD_set_pin_dir(const uint8_t pin, const enum port_dir dir)
{
	switch (dir) {
	case PORT_DIR_IN:
		VPORTD.DIR &= ~(1 << pin);
		break;
	case PORT_DIR_OUT:
		VPORTD.DIR |= (1 << pin);
		break;
	case PORT_DIR_OFF:
		*((uint8_t *)&PORTD + 0x10 + pin) |= 1 << PORT_PULLUPEN_bp;
		break;
	default:
		break;
	}
}

static inline void PORTD_set_port_level(const uint8_t mask, const bool level)
{
	if (level == true) {
		VPORTD.OUT |= mask;
	} else {
		VPORTD.OUT &= ~mask;
	}
}

static inline void PORTD_set_pin_level(const uint8_t pin, const bool level)
{
	if (level == true) {
		VPORTD.OUT |= (1 << pin);
	} else {
		VPORTD.OUT &= ~(1 << pin);
	}
}

static inline void PORTD_toggle_port_level(const uint8_t mask)
{
	PORTD.OUTTGL = mask;
}

static inline void PORTD_toggle_pin_level(const uint8_t pin)
{
	VPORTD.IN |= 1 << pin;
}

static inline uint8_t PORTD_get_port_level()
{
	return VPORTD.IN;
}

static inline bool PORTD_get_pin_level(const uint8_t pin)
{
	return VPORTD.IN & (1 << pin);
}

static inline void PORTD_write_port(const uint8_t value)
{
	VPORTD.OUT = value;
}


static inline void PD0_set_pull_mode(const enum port_pull_mode pull_mode)
{
	PORTD_set_pin_pull_mode(0, pull_mode);
}

static inline void PD0_set_dir(const enum port_dir dir)
{
	PORTD_set_pin_dir(0, dir);
}

static inline void PD0_set_isc(const PORT_ISC_t isc)
{
	PORTD_pin_set_isc(0, isc);
}

static inline void PD0_set_inverted(const bool inverted)
{
	PORTD_pin_set_inverted(0, inverted);
}

static inline void PD0_set_level(const bool level)
{
	PORTD_set_pin_level(0, level);
}

static inline void PD0_toggle_level()
{
	PORTD_toggle_pin_level(0);
}

static inline bool PD0_get_level()
{
	return PORTD_get_pin_level(0);
}

static inline void PD1_set_pull_mode(const enum port_pull_mode pull_mode)
{
	PORTD_set_pin_pull_mode(1, pull_mode);
}

static inline void PD1_set_dir(const enum port_dir dir)
{
	PORTD_set_pin_dir(1, dir);
}

static inline void PD1_set_isc(const PORT_ISC_t isc)
{
	PORTD_pin_set_isc(1, isc);
}

static inline void PD1_set_inverted(const bool inverted)
{
	PORTD_pin_set_inverted(1, inverted);
}

static inline void PD1_set_level(const bool level)
{
	PORTD_set_pin_level(1, level);
}

static inline void PD1_toggle_level()
{
	PORTD_toggle_pin_level(1);
}

static inline bool PD1_get_level()
{
	return PORTD_get_pin_level(1);
}

int8_t ADC_0_init()
{

	// ADC0.CALIB = ADC_DUTYCYC_DUTY50_gc; /* 50% Duty cycle */

//	 ADC0.CTRLB = ADC_SAMPNUM_ACC1_gc; /* 1 ADC sample */

	 ADC0.CTRLC = ADC_PRESC_DIV16_gc /* CLK_PER divided by 2 */
			 | ADC_REFSEL_INTREF_gc /* Internal reference */
//                  | ADC_REFSEL_VDDREF_gc /* Internal reference */
			 | 0 << ADC_SAMPCAP_bp; /* Sample Capacitance Selection: disabled */

	// ADC0.CTRLD = 0 << ADC_ASDV_bp /* Automatic Sampling Delay Variation: disabled */
	//		 | 0x0 << ADC_SAMPDLY_gp /* Sampling Delay Selection: 0x0 */
	//		 | ADC_INITDLY_DLY0_gc; /* Delay 0 CLK_ADC cycles */

	// ADC0.CTRLE = ADC_WINCM_NONE_gc; /* No Window Comparison */

	// ADC0.DBGCTRL = 0 << ADC_DBGRUN_bp; /* Debug run: disabled */

	// ADC0.EVCTRL = 0 << ADC_STARTEI_bp; /* Start Event Input Enable: disabled */

	// ADC0.INTCTRL = 0 << ADC_RESRDY_bp /* Result Ready Interrupt Enable: disabled */
	//		 | 0 << ADC_WCMP_bp; /* Window Comparator Interrupt Enable: disabled */

	// ADC0.MUXPOS = ADC_MUXPOS_AIN0_gc; /* ADC input pin 0 */

	// ADC0.SAMPCTRL = 0x0 << ADC_SAMPLEN_gp; /* Sample length: 0x0 */

	// ADC0.WINHT = 0x0; /* Window Comparator High Threshold: 0x0 */

	// ADC0.WINLT = 0x0; /* Window Comparator Low Threshold: 0x0 */

	ADC0.CTRLA = 1 << ADC_ENABLE_bp     /* ADC Enable: enabled */
	             | 0 << ADC_FREERUN_bp  /* ADC Freerun mode: disabled */
	             | ADC_RESSEL_10BIT_gc  /* 10-bit mode */
	             | 0 << ADC_RUNSTBY_bp; /* Run standby mode: disabled */

	return 0;
}

/**
 * \brief Enable ADC_0
 * 1. If supported by the clock system, enables the clock to the ADC
 * 2. Enables the ADC module by setting the enable-bit in the ADC control register
 *
 * \return Nothing
 */
void ADC_0_enable()
{
	ADC0.CTRLA |= ADC_ENABLE_bm;
}
/**
 * \brief Disable ADC_0
 * 1. Disables the ADC module by clearing the enable-bit in the ADC control register
 * 2. If supported by the clock system, disables the clock to the ADC
 *
 * \return Nothing
 */
void ADC_0_disable()
{
	ADC0.CTRLA &= ~ADC_ENABLE_bm;
}

/**
 * \brief Start a conversion on ADC_0
 *
 * \param[in] channel The ADC channel to start conversion on
 *
 * \return Nothing
 */
void ADC_0_start_conversion(adc_0_channel_t channel)
{
	ADC0.MUXPOS  = channel;
	ADC0.COMMAND = ADC_STCONV_bm;
}

/**
 * \brief Check if the ADC conversion is done
 *
 * \return The status of ADC converison done check
 * \retval true The ADC conversion is done
 * \retval false The ADC converison is not done
 */
bool ADC_0_is_conversion_done()
{
	return (ADC0.INTFLAGS & ADC_RESRDY_bm);
}

/**
 * \brief Read a conversion result from ADC_0
 *
 * \return Conversion result read from the ADC_0 ADC module
 */
adc_result_t ADC_0_get_conversion_result(void)
{
	return (ADC0.RES);
}

/**
 * \brief Start a conversion, wait until ready, and return the conversion result
 *
 * \return Conversion result read from the ADC_0 ADC module
 */
adc_result_t ADC_0_get_conversion(adc_0_channel_t channel)
{
	adc_result_t res;

	ADC_0_start_conversion(channel);
	while (!ADC_0_is_conversion_done())
		;
	res = ADC_0_get_conversion_result();
	ADC0.INTFLAGS |= ADC_RESRDY_bm;
	return res;
}

/**
 * \brief Return the number of bits in the ADC conversion result
 *
 * \return The number of bits in the ADC conversion result
 */
uint8_t ADC_0_get_resolution()
{
	return (ADC0.CTRLA & ADC_RESSEL_bm) ? 8 : 10;
}

int8_t VREF_0_init()
{

	VREF_CTRLA = VREF_AC0REFSEL_0V55_gc     /* Voltage reference at 0.55V */
	             | VREF_ADC0REFSEL_4V34_gc; /* Voltage reference at 4.34V */

	VREF_CTRLB = 1 << VREF_ADC0REFEN_bp   /* ADC0 reference enable: enabled */
	             | 0 << VREF_AC0REFEN_bp; /* AC0 DACREF reference enable: disabled */

	return 0;
}

/* Configure pins and initialize registers */
void ADC_0_initialization(void)
{

	// Disable digital input buffer
	PD0_set_isc(PORT_ISC_INPUT_DISABLE_gc);
	// Disable pull-up resistor
	PD0_set_pull_mode(PORT_PULL_OFF);

	// Disable digital input buffer
	PD1_set_isc(PORT_ISC_INPUT_DISABLE_gc);
	// Disable pull-up resistor
	PD1_set_pull_mode(PORT_PULL_OFF);

	ADC_0_init();
}


int main() {
    uint8_t counter = 0;
    
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
    
    evrouter::init();
   
    lut0::init(std::byte{0x08});
    lut1::init(std::byte{0x02});

    pit::init();
    
    terminalDevice::init<BaudRate<9600>>();
    rcUsart::init<BaudRate<115200>>();
    sensor::init();
    
    systemTimer::init();
    
    led::template dir<Output>();     
//    pf2::template dir<Output>();     
    pb1::template dir<Output>();     
//    pa2::template dir<Output>();     
    
//    pe3::template dir<Input>();     

//    pa2::on();
    pb1::on();
    
    pwm::init();
    pwm::frequency(fPwm);
//    pwm::on<PWM::WO<0>, PWM::WO<1>, PWM::WO<2>>();
////    pwm::off<PWM::WO<2>>();

    pwm::duty<PWM::WO<0>>(1000);

    ppm::init();
    
    rpm::init();

//    pd4::template attributes<AVR::Attributes::DigitalDisable<>>();    
//    pd4::template pullup<false>();    

    //    adcController::init();
    
    
    const adc_0_channel_t ch = ADC_MUXPOS_AIN0_gc;
    
    VREF_0_init();
	ADC_0_initialization();

    ADC_0_start_conversion(ch);
    
    {
        etl::Scoped<etl::EnableInterrupt<>> ei;
        
        const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);
    
        while(true) {
            terminalDevice::periodic();
            rcUsart::periodic();
            sensor::periodic();
//            adcController::periodic();
            systemTimer::periodic([&]{
                pf2::toggle(); // 5,8us - 17us
                sensor::ratePeriodic();

                alarmTimer::periodic([&](const auto& t){
                    if (periodicTimer == t) {
                        led::toggle();
//                        etl::outl<terminal>("test12: "_pgm, ++counter);
                        etl::outl<terminal>("test12: "_pgm, ++counter, " ch0: "_pgm, sumd::value(0).toInt());
//                        etl::outl<terminal>("co: "_pgm, sensor::collisions(), " ar: "_pgm, sensor::asciiPackages(), " br: "_pgm, sensor::binaryPackages());
//                        etl::outl<terminal>("ppm: "_pgm, ppm::value());
//                        etl::outl<terminal>("rpm: "_pgm, rpm::diff());
                        if (auto c = terminalDevice::get()) {
                            etl::outl<terminal>("c: "_pgm, *c);
                        }
                        if (ADC_0_is_conversion_done()) {
                            etl::outl<terminal>("adc0: "_pgm, ADC_0_get_conversion_result());
                            ADC_0_start_conversion(ch);
                        }
                    }
                });
            });
        }
    }
}

ISR(PORTE_PORT_vect) {
    isrRegistrar::isr<AVR::ISR::Port<E>>();
}
