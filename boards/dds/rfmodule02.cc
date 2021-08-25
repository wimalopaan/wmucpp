#define NDEBUG

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/pwm.h>
#include <mcu/internals/eeprom.h>
#include <mcu/internals/spi.h>
#include <mcu/internals/ccl.h>
#include <mcu/internals/sigrow.h>
#include <mcu/internals/event.h>
#include <mcu/internals/syscfg.h>

#include <external/hal/adccontroller.h>
#include <external/hal/alarmtimer.h>
#include <external/bluetooth/roboremo.h>
#include <external/bluetooth/qtrobo.h>
#include <external/sbus/sbus.h>
#include <external/sbus/sport.h>
#include <external/ibus/ibus2.h>
#include <external/hott/sumdprotocolladapter.h>
#include <external/hott/experimental/sensor.h>
#include <external/hott/experimental/adapter.h>
#include <external/hott/menu.h>

#include <external/solutions/button.h>
#include <external/solutions/blinker.h>
#include <external/solutions/analogsensor.h>
#include <external/solutions/series01/swuart.h>
#include <external/solutions/series01/rpm.h>
#include <external/solutions/apa102.h>
#include <external/solutions/series01/sppm_in.h>
#include <external/solutions/rc/busscan.h>
#include <external/solutions/series01/cppm_out.h>
#include <external/solutions/rc/rf.h>

#include <mcu/pgm/pgmarray.h>

#include <std/chrono>
#include <std/bit>

#include <etl/output.h>
#include <etl/meta.h>

#include "mpm.h"
#include "dds.h"

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

#ifndef NDEBUG
namespace xassert {
    etl::StringBuffer<160> ab;
    etl::StringBuffer<10> aline;
    bool on{false};
}
#endif

namespace  {
    constexpr auto fRtc = 1000_Hz;
}

template<typename L>
struct ColorLedAdapter {
    using index_t = L::index_type;
    static inline void init() {
        L::init();
    }
    static inline void activate() {
        L::set(index_t{0}, mColor);
        L::out();
    }
    static inline void inactivate() {
        L::set(index_t{0}, External::cRGB{});
        L::out();        
    }
    static inline void periodic() {
        L::periodic();
    }    
    static inline void color(const External::Crgb& c) {
        mColor = c;
    }
private:
    static constexpr External::Crgb red{External::Red{128}, External::Green{0}, External::Blue{0}};
    inline static External::Crgb mColor{red};
};


using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using sigrow = SigRow<>;

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;

//using dbgPin = Pin<Port<A>, 6>;

//using a0Pin = Pin<Port<A>, 0>; // usart0
//using a1Pin = Pin<Port<A>, 1>;
//using a2Pin = Pin<Port<A>, 2>;

using a5Pin = Pin<Port<A>, 5>;

using dataPin = Pin<Port<C>, 0>;
using dataAct = ActiveHigh<dataPin, Output>;

using clkPin = Pin<Port<C>, 2>;
using clkAct = ActiveHigh<clkPin, Output>;
using clkSig = SinglePulse<clkAct>;

using fupPin = Pin<Port<D>, 0>;
using fupAct = ActiveHigh<fupPin, Output>;
using fupSig = SinglePulse<fupAct>;

using rstPin = Pin<Port<D>, 1>;
using rstAct = ActiveHigh<rstPin, Output>;
using rstSig = SinglePulse<rstAct>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>; 
using tdev = Usart<usart0Position, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<16>, AVR::SendQueueLength<256>>;
using terminal = etl::basic_ostream<tdev>;

using tca0Position = AVR::Portmux::Position<AVR::Component::Tca<0>, Portmux::Default>;
using cppm = External::Ppm::Cppm<tca0Position, std::integral_constant<uint8_t, 16>, AVR::UseInterrupts<true>>;

using usart2Position = Portmux::Position<Component::Usart<2>, Portmux::Default>; 
using mpm_pa = External::Multiprotocol::ProtocollAdapter<0, systemTimer>;
using mpm = Usart<usart2Position, mpm_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
using mpm_out = External::Multiprotocol::Generator<mpm, systemTimer, External::RC::NumberOfBands>;

using lut1Position = AVR::Portmux::Position<AVR::Component::Ccl<1>, Portmux::Default>;
using lut1 = Ccl::SimpleLut<1, Ccl::Input::Mask, Ccl::Input::Mask, Ccl::Input::Usart<2>>;    

using spiPosition = Portmux::Position<Component::Spi<0>, Portmux::Default>;
using spi = AVR::Spi<spiPosition, AVR::QueueLength<16>,  AVR::UseInterrupts<false>>;
using ledStripe = External::LedStripe<spi, External::APA102, 1>;
using led = ColorLedAdapter<ledStripe>;

using blinkLed = External::Blinker2<led, systemTimer, 100_ms, 2500_ms>;
using bc_t = blinkLed::count_type;

using ad9851 = External::DDS::AD9851<systemTimer, dataAct, clkSig, fupSig, rstSig>;

using portmux = Portmux::StaticMapper<Meta::List<tca0Position, lut1Position, spiPosition>>;

template<typename MCU = DefaultMcuType>
struct Fsm {
    using ch_t = cppm::channel_t;
    using rv_t = cppm::ranged_type;
    
    enum class State : uint8_t {Undefined, Init, Search, Run, RunRF};
    
    static inline constexpr External::Tick<systemTimer> mInitTicks{500_ms};
    static inline constexpr External::Tick<systemTimer> mChangeTicks{100_ms};
    static inline constexpr External::Tick<systemTimer> mDebugTicks{500_ms};
    
    using Crgb = External::Crgb;
    static constexpr Crgb white{External::Red{64}, External::Green{64}, External::Blue{64}};
    static constexpr Crgb red{External::Red{128}, External::Green{0}, External::Blue{0}};
    static constexpr Crgb green{External::Red{0}, External::Green{128}, External::Blue{0}};
    static constexpr Crgb blue{External::Red{0}, External::Green{0}, External::Blue{128}};
    static constexpr Crgb yellow{External::Red{128}, External::Green{128}, External::Blue{0}};
    static constexpr Crgb magenta{External::Red{128}, External::Green{0}, External::Blue{128}};
    static constexpr Crgb cyan{External::Red{0}, External::Green{128}, External::Blue{128}};
    
    
    static inline void init() {
//        dbgPin::dir<Output>();
        blinkLed::init();
        
        a5Pin::dir<Output>();
        ad9851::init();
        tdev::init<BaudRate<9600>>();
        mpm::template init<AVR::BaudRate<100000>, AVR::FullDuplex, true, 1>();
        mpm_out::init();
        mpm::txPinDisable();
        lut1::init(0x0f_B); // route TXD (inverted) to lut0-out 
        
        cppm::set(ch_t{0}, rv_t{cppm::ocMedium});
        cppm::set(ch_t{1}, rv_t{cppm::ocMedium});
        cppm::set(ch_t{2}, rv_t{cppm::ocMedium});
        cppm::set(ch_t{3}, rv_t{cppm::ocMedium});
        cppm::set(ch_t{4}, rv_t{cppm::ocMedium});
        cppm::set(ch_t{5}, rv_t{cppm::ocMedium});
        cppm::set(ch_t{6}, rv_t{cppm::ocMedium});
        cppm::set(ch_t{7}, rv_t{cppm::ocMedium});
        etl::outl<terminal>("rfmodule 02"_pgm);
    }
    
    static inline void periodic() {
//        dbgPin::toggle();
        led::periodic();
        tdev::periodic();
        ad9851::periodic();
        mpm::periodic();
        mpm_out::periodic();
    }

    static inline void update() {
        const auto ch = mpm_pa::channel();
        const auto subProto = mpm_pa::subProto();
        if (subProto <= uint8_t(External::RC::Band::_72MHz)) {
            External::RC::Band band{subProto.toInt()};
            if (mState == State::Run) {
                if (const auto rfCh = ad9851::channel(band, ch)) {
                    mpm_out::band(band);
                    mpm_out::channel(rfCh.toRanged());
                }
            }
        }
    }
    
    static inline void ratePeriodic() {
        blinkLed::ratePeriodic();
        ad9851::ratePeriodic();
        mpm_pa::ratePeriodic();
        mpm_out::ratePeriodic();
        const auto oldState = mState;
        ++mStateTick;
        ++mChangeTick;
        (++mDebugTick).on(mDebugTicks, []{
            etl::outl<terminal>("rf: "_pgm, mpm_pa::channel(), " P: "_pgm, mpm_pa::proto(), " S:"_pgm, mpm_pa::subProto(), " bi: "_pgm, (uint8_t)mpm_pa::bind());
        });
        
        update();
        
        switch(mState) {
        case State::Undefined:
            mStateTick.on(mInitTicks, []{
               mState = State::Init; 
            });
            break;
        case State::Init:
            if (ad9851::ready()) {
                mState = State::Run;
            }
            break;
        case State::Search:
            if (mpm_pa::proto() == 88) {
                mState = State::Run;
            }
            break;
        case State::Run:
            if (mpm_pa::proto() != 88) {
                mState = State::Search;
            }
            if (mpm_pa::bind()) {
                mState = State::RunRF;
            }
            break;
        case State::RunRF:
            if (!mpm_pa::bind()) {
                mState = State::Run;
            }
            for(uint8_t i{0}; i < 8; ++i) {
                if (const auto v = mpm_pa::value(i)) {
                    const auto vr = v.toRanged();
                    const auto cv = AVR::Pgm::scaleTo<cppm::ranged_type>(vr);
                    cppm::set(ch_t{i}, cv);
                }
            }
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                cppm::init();
                led::color(red);
                blinkLed::blink(bc_t{1});
                etl::outl<terminal>("S I"_pgm);
                break;
            case State::Search:
                led::color(red);
                blinkLed::blink(bc_t{2});
                etl::outl<terminal>("S Sea"_pgm);
                mpm_out::valid(false);
                break;
            case State::Run:
                etl::outl<terminal>("S Run"_pgm);
                led::color(blue);
                blinkLed::blink(bc_t{1});
                mpm_out::valid(true);
                ad9851::rfOn(false);
                break;
            case State::RunRF:
                etl::outl<terminal>("S RunRf"_pgm);
                led::color(green);
                blinkLed::blink(bc_t{2});
                ad9851::rfOn(true);
                break;
            }
        } 
    }
private:
    static inline rv_t vv{cppm::ocMedium};    
    static inline State mState{State::Undefined};
    static inline External::Tick<systemTimer> mStateTick;
    static inline External::Tick<systemTimer> mDebugTick;
    static inline External::Tick<systemTimer> mChangeTick;
};

using fsm = Fsm<>;

using isrRegistrar = IsrRegistrar<cppm::CmpHandler<ad9851::SetBaseFrequency>, cppm::OvfHandler<ad9851::SetUpperFrequency>>;

int main() {
    portmux::init();
    ccp::unlock([]{
         clock::template init<Project::Config::fMcuMhz>();
    });
    systemTimer::init();
    
    fsm::init();
    {
        etl::Scoped<etl::EnableInterrupt<>> ei;
        while(true) {
            fsm::periodic();                        
            systemTimer::periodic([&]{
                fsm::ratePeriodic();                        
            });
        }
    }    
}
 
ISR(TCA0_OVF_vect) {
    a5Pin::on();
    isrRegistrar::isr<AVR::ISR::Tca<0>::Ovf>();
}
ISR(TCA0_CMP1_vect) {
    a5Pin::off();
    isrRegistrar::isr<AVR::ISR::Tca<0>::Cmp<1>>();
}
