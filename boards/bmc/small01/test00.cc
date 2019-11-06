#define NDEBUG

#include <mcu/avr.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/eeprom.h>
#include <mcu/internals/pwm.h>
#include <mcu/internals/ccl.h>

#include <external/hal/alarmtimer.h>
#include <external/hal/adccontroller.h>
//#include <external/hott/sumdprotocolladapter.h>
//#include <external/hott/experimental/sensor.h>
//#include <external/hott/hott.h>
//#include <external/hott/menu.h>

#include <std/chrono>
#include <etl/output.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

struct Storage final {
    Storage() = delete;
    enum class AVKey : uint8_t {TimeOut = 0, SoftStart, SensorType,
                                _Number};
    
    struct ApplData final : public EEProm::DataBase<ApplData> {
        etl::uint_NaN<uint8_t>& operator[](AVKey key) {
            return AValues[static_cast<uint8_t>(key)];
        }
    private:
        std::array<etl::uint_NaN<uint8_t>, static_cast<uint8_t>(AVKey::_Number)> AValues;
    };
};

using eeprom = EEProm::Controller<Storage::ApplData>;
auto& appData = eeprom::data();

using PortA = Port<A>;
using eflag = Pin<PortA, 2>; 

using in1 = Pin<PortA, 6>; 
using in2 = Pin<PortA, 7>; 

using ccp = Cpu::Ccp<>;
using clock = Clock<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Alt1>;
using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;

using tcdPosition = Portmux::Position<Component::Tcd<0>, Portmux::Default>;
using pwm = PWM::DynamicPwm<tcdPosition>;

namespace Parameter {
    constexpr uint8_t menuLines = 8;
    constexpr auto fRtc = 500_Hz;
}

using systemTimer = SystemTimer<Component::Rtc<0>, Parameter::fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer, 2>;

#ifdef USE_HOTT
#else
using terminalDevice = AVR::Usart<usart0Position, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>>;
using terminal = etl::basic_ostream<terminalDevice>;
#endif

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;
using adcController = External::Hal::AdcController<adc, Meta::NList<0>>;

using lut0 = Ccl::SimpleLut<0, Ccl::Input::Tcd<0>, Ccl::Input::Mask, Ccl::Input::Mask>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, tcaPosition, tcdPosition>>;

void TCD0_init(void) {    
//    TCD0.CMPASET = 0;    
//    TCD0.CMPACLR = 100;    
//    TCD0.CMPBSET = 300;                                        
//    TCD0.CMPBCLR = 900;    
    
    CPU_CCP = CCP_IOREG_gc;    
    TCD0.FAULTCTRL = TCD_CMPAEN_bm            /* enable channel A */
                   | TCD_CMPBEN_bm;            /* enable channel B */
    
    /* ensure ENRDY bit is set */
    while(!(TCD0.STATUS & TCD_ENRDY_bm));
    
    TCD0.CTRLA = 1 << TCD_ENABLE_bp;


    while(!(TCD0.STATUS & TCD_CMDRDY_bm));
    
    TCD0.CMPASET = 0;    
    TCD0.CMPACLR = 100;    
    TCD0.CMPBSET = 300;                                        
    TCD0.CMPBCLR = 900;    
    
    TCD0.CTRLE = 0x01;
}


void PORT_init(void)
{
    PORTA.DIRSET = PIN6_bm            /* set pin 4 as output */
                    | PIN7_bm        /* set pin 5 as output */
                    | PIN3_bm;
}


int main() {
//    ccp::unlock([]{
//        clock::prescale<1>();
//    });
    
//    eflag::template dir<Input>();
//    eflag::template pullup<true>();
    
//    in1::template dir<Output>();
//    in2::template dir<Output>();

//    portmux::init();
//    eeprom::init();
//    systemTimer::init();
//    terminalDevice::init<AVR::BaudRate<9600>>();

    PORT_init();

//    TCD0_init();

//    while(true) {
//        PORTA.OUTTGL = PIN3_bm;
//    }
    
    pwm::init();
    pwm::frequency(5000_Hz);
    pwm::template duty<PWM::WO<0>>(pwm::max() / 10);
    pwm::template on<Meta::List<PWM::WO<0>>>();

//    lut0::init(std::byte{0x01});
//    lut0::enable();
    
    const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);

    uint8_t counter = 0;
    
    bool eepSave = false;
    while(true) {
        eepSave |= eeprom::saveIfNeeded();
#ifdef USE_HOTT
#else
        terminalDevice::periodic();
#endif
        systemTimer::periodic([&]{
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    ++counter;
                    if ((counter % 2) == 0) {
#ifndef USE_HOTT
                        etl::outl<terminal>("test00"_pgm);
#endif
                    }
                    else {
                    }
                }
            });
            appData.expire();
        });
    }
}

