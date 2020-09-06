#define NDEBUG

#define USE_IBUS
#define USE_DAISY

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/pwm.h>

#include <external/hal/adccontroller.h>
#include <external/hal/alarmtimer.h>
#include <external/bluetooth/roboremo.h>
#include <external/bluetooth/qtrobo.h>
#include <external/sbus/sbus.h>
#include <external/ibus/ibus.h>

#include <external/solutions/button.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

namespace  {
#ifdef USE_HOTT
    constexpr auto fRtc = 500_Hz;
#endif
#ifdef USE_SBUS
    constexpr auto fRtc = 1000_Hz;
#endif
#ifdef USE_IBUS
    constexpr auto fRtc = 2000_Hz;
#endif
#ifdef USE_PPM
    constexpr auto fRtc = 128_Hz;
#endif
}

template<uint8_t N, typename PWM, typename ADC, typename InA, typename InB, typename PA>
struct ChannelFsm {
    using adc_index_t = typename ADC::index_type;
    static inline constexpr adc_index_t adci{N};
    
    using pa_value_t = typename PA::value_type;
    using channel_t = typename PA::channel_t;
    
    static inline constexpr channel_t channel{N};
    
    using pwm_value_t = typename PWM::value_type;
    
    enum class State : uint8_t {Init, Off, Forward, Backward};
    
    inline static void init() {
        InA::template dir<Output>();
        InA::off();
        InB::template dir<Output>();
        InB::off();
    }
    inline static void periodic() {
//        switch(mState) {
        
//        }
    }
    inline static void ratePeriodic() {
        auto cv = ADC::value(adci);                
        
        pa_value_t pv = PA::value(channel);
//        pa_value_t::_;
        
        constexpr uint16_t pa_mid = (pa_value_t::Upper + pa_value_t::Lower) / 2;
        constexpr uint16_t pa_half = (pa_value_t::Upper - pa_value_t::Lower) / 2;
        
        pwm_value_t pvs{0};
        
        if (pv.toInt() > (pa_mid + 10)) {
            pvs = ((uint32_t)pv.toInt() - pa_mid) * PWM::pwmMax / pa_half; 
            forward();
        }
        else if (pv.toInt() < (pa_mid - 10)) {
            pvs = (pa_mid - (uint32_t)pv.toInt()) * PWM::pwmMax / pa_half; 
            backward();
        }
        else {
            pvs = 0;
            off();
        }
        v0 = pvs;
        PWM::template pwm<N>(pvs);
    }
//private:
    inline static pwm_value_t v0;
    inline static void forward() {
        InA::on();
        InB::off();
    }
    inline static void backward() {
        InA::on();
        InB::on();
    }
    inline static void off() {
        InA::off();
        InB::off();
    }
    
    
    inline static State mState{State::Init};
};

template<typename PWM, typename ChList, typename Led>
struct GlobalFsm;

template<typename PWM, typename Led, typename... Chs>
struct GlobalFsm<PWM, Meta::List<Chs...>, Led> {
    using channel_list = Meta::List<Chs...>;
    static_assert(Meta::size_v<channel_list> <= 4, "too much channels");
    static_assert(Meta::is_set_v<channel_list>, "channels must be different");
    
    enum class State : uint8_t {Init};
    
    inline static void init() {
        Led::template dir<Output>();
        
        PWM::init();    
        (Chs::init(), ...);
    }
    inline static void periodic() {
        (Chs::periodic(), ...);
        switch(mState) {
        
        }
    }
    inline static void ratePeriodic() {
        (Chs::ratePeriodic(), ...);
    }
    
private:
    static inline State mState{State::Init};
    
};

using ccp = Cpu::Ccp<>;
using clock = Clock<>;

using usart1Position = Portmux::Position<Component::Usart<1>, Portmux::Default>; // Servo / DBG
using usart2Position = Portmux::Position<Component::Usart<2>, Portmux::Default>; // Sensor

using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;

using pwm = PWM::DynamicPwm8Bit<tcaPosition>;

using portmux = Portmux::StaticMapper<Meta::List<usart1Position, usart2Position>>;

using servo_pa = IBus::Servo::ProtocollAdapter<0>;
using servo = Usart<usart1Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
using terminal = etl::basic_ostream<servo>;

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer>;

using ledPin  = Pin<Port<D>, 2>; 

using pwmPin1 = Pin<Port<A>, 0>; 
using pwmPin2 = Pin<Port<A>, 1>; 
using pwmPin3 = Pin<Port<A>, 2>; 
using pwmPin4 = Pin<Port<A>, 3>; 

using end1Pin = Pin<Port<F>, 5>; 
using end2Pin = Pin<Port<D>, 1>; 
using end3Pin = Pin<Port<A>, 4>; 
using end4Pin = Pin<Port<A>, 5>; 

using inA1Pin = Pin<Port<F>, 3>; 
using inB1Pin = Pin<Port<F>, 1>; 
using inA2Pin = Pin<Port<D>, 3>; 
using inB2Pin = Pin<Port<D>, 6>; 
using inA3Pin = Pin<Port<C>, 3>; 
using inB3Pin = Pin<Port<C>, 2>; 
using inA4Pin = Pin<Port<A>, 6>; 
using inB4Pin = Pin<Port<A>, 7>; 

using dgbPin = Pin<Port<C>, 0>; // tx 
using ibusPin = Pin<Port<C>, 1>; // rx 

using csf1Pin = Pin<Port<F>, 2>; // ADC 12
using csf2Pin = Pin<Port<D>, 7>; // ADC 7
using csf3Pin = Pin<Port<D>, 5>; // ADC 5
using csf4Pin = Pin<Port<D>, 0>; // ADC 0 

using sensorUartPin = Pin<Port<F>, 0>; // tx 

using daisyChain= Pin<Port<F>, 4>; 

#ifdef USE_DAISY
struct IBusThrough {
    inline static void init() {
        daisyChain::template dir<Output>();
    }
    inline static void on() {
        daisyChain::on();
    }
    inline static void off() {
        daisyChain::off();
    }
};
using ibt = IBusThrough;
#else
using ibt = void;
#endif

using ibus_sensor = IBus::Sensor<usart2Position, AVR::Usart, AVR::BaudRate<115200>, 
                          Meta::List<>, systemTimer, ibt
//                          , etl::NamedFlag<true>
//                           , etl::NamedFlag<true>
                          >;

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;
using adcController = External::Hal::AdcController<adc, Meta::NList<12, 7, 5, 0, 0x1e>>; // 1e = temp
using adi_t = adcController::index_type;

using ch0 = ChannelFsm<0, pwm, adcController, inA1Pin, inB1Pin, servo_pa>;
using ch1 = ChannelFsm<1, pwm, adcController, inA2Pin, inB2Pin, servo_pa>;
using ch2 = ChannelFsm<2, pwm, adcController, inA3Pin, inB3Pin, servo_pa>;
using ch3 = ChannelFsm<3, pwm, adcController, inA4Pin, inB4Pin, servo_pa>;

using gfsm = GlobalFsm<pwm, Meta::List<ch0, ch1, ch2, ch3>, ledPin>;

int main() {
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
   
    ledPin::dir<Output>();
    
    servo::init<BaudRate<115200>>();
    ibus_sensor::init();
    systemTimer::init();
    adcController::init();

    gfsm::init();
    
    const auto periodicTimer = alarmTimer::create(100_ms, External::Hal::AlarmFlags::Periodic);
    const auto aTimer = alarmTimer::create(1000_ms, External::Hal::AlarmFlags::Periodic);
    etl::outl<terminal>("rcquadc 01"_pgm);

    while(true) {
       
        adcController::periodic();
        servo::periodic();
        ibus_sensor::periodic();
        
        gfsm::periodic();
        
        systemTimer::periodic([&]{
            ibus_sensor::ratePeriodic();
            
            gfsm::ratePeriodic();
            
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    ledPin::toggle();
                    etl::outl<terminal>("c1: "_pgm, adcController::value(adi_t{0}).toInt(), " v0: "_pgm, ch0::v0);
                }
                if (aTimer == t) {
                }
            });
        });
    }
}

