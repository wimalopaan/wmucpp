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

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

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

using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using sigrow = SigRow<>;

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;

using dbgPin = Pin<Port<A>, 6>;

//using a0Pin = Pin<Port<A>, 0>; // usart0
//using a1Pin = Pin<Port<A>, 1>;

//using a2Pin = Pin<Port<A>, 2>;
using a5Pin = Pin<Port<A>, 5>;

using ad_data = Port<D>;

template<AVR::Concepts::ActivatableOut A>
struct SinglePulse {
    static inline void init() {
        A::init();
    }
    static inline void pulse() {
        A::activate();
        A::inactivate();
    }
};

using clkPin = Pin<Port<C>, 0>;
using clkAct = ActiveHigh<clkPin, Output>;
using clkSig = SinglePulse<clkAct>;

using fupPin = Pin<Port<C>, 1>;
using fupAct = ActiveHigh<fupPin, Output>;
using fupSig = SinglePulse<fupAct>;

using rstPin = Pin<Port<C>, 2>;
using rstAct = ActiveHigh<rstPin, Output>;
using rstSig = SinglePulse<rstAct>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>; // Sensor
using tdev = Usart<usart0Position, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<16>, AVR::SendQueueLength<256>>;
using terminal = etl::basic_ostream<tdev>;

using tca0Position = AVR::Portmux::Position<AVR::Component::Tca<0>, Portmux::Default>;
using cppm = External::Ppm::Cppm<tca0Position, std::integral_constant<uint8_t, 16>, AVR::UseInterrupts<true>>;
using lut0 = Ccl::SimpleLut<0, Ccl::Input::Mask, Ccl::Input::Tca0<1>, Ccl::Input::Mask>;    

using portmux = Portmux::StaticMapper<Meta::List<tca0Position>>;

template<typename Data, typename WClkSig, typename FUpdSig, typename ResetSig>
struct AD9851Serial {
    static inline constexpr uint64_t ad_f87  = 40'915'000;
    static inline constexpr uint64_t ad_f87u = 40'917'000; // besser 2KHz?
    static inline constexpr uint64_t ad_c = uint64_t{1} << 32;
    static inline constexpr uint64_t ad_sysclk = 30'000'000 * 6; // 180MHz bei 30MHz Quarz
    static inline constexpr uint32_t ad_v87  = (ad_f87 * ad_c) / ad_sysclk;
    static inline constexpr uint32_t ad_v87u = (ad_f87u * ad_c) / ad_sysclk;
//    std::integral_constant<uint64_t, ad_v87>::_;
//    std::integral_constant<uint64_t, ad_v87u>::_;  
    
    using d7Pin = AVR::Pin<Data, 7>;
    
    static inline constexpr auto ff = []{
        std::array<std::byte, 4> data{};
        
        data[0] = std::byte((ad_v87 >> 24) & 0xff);
        data[1] = std::byte((ad_v87 >> 16) & 0xff);
        data[2] = std::byte((ad_v87 >>  8) & 0xff);
        data[3] = std::byte((ad_v87 >>  0) & 0xff);
        
        return data;
    }();
    static inline constexpr auto ffu = []{
        std::array<std::byte, 4> data{};
        
        data[0] = std::byte((ad_v87u >> 24) & 0xff);
        data[1] = std::byte((ad_v87u >> 16) & 0xff);
        data[2] = std::byte((ad_v87u >>  8) & 0xff);
        data[3] = std::byte((ad_v87u >>  0) & 0xff);
        
        return data;
    }();
    
    enum class Freq : uint8_t {Ch87 = 0};
    
    enum class State : uint8_t {Undefined, Init, Ready, Transfer};
    
    static inline bool ready() {
        return mState == State::Ready;
    }

    static inline void put(std::byte b) {
        for(uint8_t i{}; i < 8; ++i) {
            if (std::any(b & 0x01_B)) {
                d7Pin::on();
            }
            else {
                d7Pin::off();
            }
            b >>= 1;
            WClkSig::pulse();
        }
    }
    
    // Reihenfolge gegenüber parallel Mode vertauscht.
    static inline void put(const std::array<std::byte, 4>& a) {
        put(a[3]);
        put(a[2]);
        put(a[1]);
        put(a[0]);
        put(0x01_B); // refclock 6x
    }
    
    static inline void set(const Freq) {
        put(ff);
        FUpdSig::pulse();
    }
    
    static inline void setu() {
        put(ffu);
        FUpdSig::pulse();
    }

    static inline void setSerial() {
        Data::get() = 0x03_B;        
        WClkSig::pulse();
        FUpdSig::pulse();
    }
    
    static inline void init() {
        Data::dir() = 0xff_B;        
        Data::get() = 0x00_B;        
        WClkSig::init();
        FUpdSig::init();
        ResetSig::init();
    }  
    
    static inline void periodic() {
        const auto oldState = mState;
        switch (mState) {
        case State::Undefined:
            mState = State::Init;
            break;
        case State::Init:
            mState = State::Ready;
            break;
        case State::Ready:
            break;
        case State::Transfer:
            break;
        }
        if (oldState != mState) {
            switch (mState) {
            case State::Undefined:
                break;
            case State::Init:
                ResetSig::pulse();
                setSerial();
                break;
            case State::Ready:
                break;
            case State::Transfer:
                break;
            }
        }
    }
private:
    static inline State mState{State::Undefined};
};

using ad9851 = AD9851Serial<ad_data, clkSig, fupSig, rstSig>;

template<typename MCU = DefaultMcuType>
struct Fsm {
    using ch_t = cppm::channel_t;
    using rv_t = cppm::ranged_type;
    
    enum class State : uint8_t {Undefined, Init, Set, Run};
    
    static inline constexpr External::Tick<systemTimer> mInitTicks{500_ms};
    static inline constexpr External::Tick<systemTimer> mChangeTicks{100_ms};
    static inline constexpr External::Tick<systemTimer> mDebugTicks{500_ms};
    
    static inline void init() {
        dbgPin::dir<Output>();
//        a0Pin::dir<Output>();
//        a1Pin::dir<Output>();
//        a2Pin::dir<Output>();
        a5Pin::dir<Output>();
        ad9851::init();
        tdev::init<BaudRate<9600>>();
        
        cppm::set(ch_t{0}, rv_t{cppm::ocMedium});
        cppm::set(ch_t{1}, rv_t{cppm::ocMedium});
        cppm::set(ch_t{2}, rv_t{cppm::ocMedium});
        cppm::set(ch_t{3}, rv_t{cppm::ocMedium});
        cppm::set(ch_t{4}, rv_t{cppm::ocMedium});
        cppm::set(ch_t{5}, rv_t{cppm::ocMedium});
        cppm::set(ch_t{6}, rv_t{cppm::ocMedium});
        cppm::set(ch_t{7}, rv_t{cppm::ocMedium});
        
        etl::outl<terminal>("dds10"_pgm);
    }
    static inline void periodic() {
        dbgPin::toggle();
        tdev::periodic();
        ad9851::periodic();
    }
    static inline void ratePeriodic() {
        const auto oldState = mState;
        ++mStateTick;
        ++mChangeTick;
        (++mDebugTick).on(mDebugTicks, []{
            etl::outl<terminal>("v: "_pgm, vv.toInt());
        });
        switch(mState) {
        case State::Undefined:
            mStateTick.on(mInitTicks, []{
               mState = State::Init; 
            });
            break;
        case State::Init:
            mStateTick.on(mInitTicks, []{
               mState = State::Set; 
            });
            break;
        case State::Set:
            if (ad9851::ready()) {
                mState = State::Run;
            }
            break;
        case State::Run:
            mChangeTick.on(mChangeTicks, []{
                cppm::set(ch_t{0}, vv);
                vv += 100;
                if (vv.isTop()) {
                    vv.setToBottom();
                }
            });
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                break;
            case State::Set:
                ad9851::set(ad9851::Freq::Ch87);
                break;
            case State::Run:
                cppm::init();
                lut0::init(0x33_B); // invert
        //        lut0::enable(); // nicht notwendig
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

struct Setter {
    inline static void once() {
        ad9851::setu();
        a5Pin::off();        
    }
};
struct ReSetter {
    inline static void once() {
        ad9851::set(ad9851::Freq::Ch87);
        a5Pin::on();        
    }
};

using isrRegistrar = IsrRegistrar<cppm::CmpHandler<ReSetter>, cppm::OvfHandler<Setter>>;

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
    isrRegistrar::isr<AVR::ISR::Tca<0>::Ovf>();
}
ISR(TCA0_CMP1_vect) {
    isrRegistrar::isr<AVR::ISR::Tca<0>::Cmp<1>>();
}
