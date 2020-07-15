#define NDEBUG

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/eeprom.h>

#include <external/hal/alarmtimer.h>
#include <external/sbus/sbus.h>
#include <external/ibus/ibus.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

using PortA = Port<A>;
using PortF = Port<F>;

using led = Pin<PortF, 5>; 
using pa0 = Pin<PortF, 2>; 

using ccp = Cpu::Ccp<>;
using clock = Clock<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>;
using usart1Position = Portmux::Position<Component::Usart<1>, Portmux::Default>;
using usart2Position = Portmux::Position<Component::Usart<2>, Portmux::Alt1>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, usart1Position, usart2Position>>;

namespace  {
    constexpr auto fRtc = 1000_Hz; // 1ms
}

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer>;

//using servo_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer>;
using servo_pa = IBus::Servo::ProtocollAdapter<0>;
using servo = AVR::Usart<usart1Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>>;

using terminalDevice = Usart<usart2Position, External::Hal::NullProtocollAdapter, UseInterrupts<false>>;
//using terminalDevice = servo;
using terminal = etl::basic_ostream<terminalDevice>;

template<auto N = 8>
struct SwitchStates {
    enum class SwState : uint8_t {Off, Steady, Blink1, Blink2};
    
    using addr_t  = etl::uint_ranged<uint8_t, 0, 3>;
    inline static constexpr addr_t address{0};
    
    static constexpr void init() {
    }
    static constexpr uint8_t size() {
        return N;
    }
    static inline auto& switches() {
        return swStates;
    }
private:
    static inline std::array<SwState, N> swStates{};
};

using sw = SwitchStates<>;

namespace Storage {
//    using tick_type = External::Tick<SoftTimer, uint8_t>;
//    struct Blink {
//        tick_type duration{};
//        tick_type intervall{};
//    };

    struct SwitchConfig {
//        using pwm_type = etl::uint_ranged_NaN<uint8_t, 0, pwm::pwmMax>;
        using channel_type = etl::uint_ranged_NaN<uint8_t, 0, 15>;
//        using tick_t = tick_type;
        
//        const auto& pwmValue() const {
//            return mPwm;
//        }
//        auto& pwmValue() {
//            return mPwm;
//        }
//        void pwmValue(const pwm_type v){
//            mPwm = v;
//        }
//        auto& blinks() {
//            return mBlinks;
//        }
        auto& passThru() {
            return mPassThruChannel;
        }
    private:
        channel_type mPassThruChannel{};
//        pwm_type mPwm{};
//        std::array<Blink, 4> mBlinks;
    };
    
    enum class AVKey : uint8_t {Magic, 
                                Ch0, Ch1, Ch2, Ch3, Ch4, Ch5, Ch6, Ch7,
                                Undefined, 
                                _Number};
    
    struct ApplData final : public EEProm::DataBase<ApplData> {
        using value_type = SwitchConfig;
        value_type& operator[](const AVKey key) {
            if (key == AVKey::Undefined) {
                AValues[static_cast<uint8_t>(AVKey::Undefined)] = SwitchConfig{};
            }
            return {AValues[static_cast<uint8_t>(key)]};
        }
        template<typename T>
        value_type& operator[](const T i) {
            return operator[](mapToKey(i));
        }
        
        inline void mpxMode(uint8_t, auto) {
            
        }
    private:
        template<typename T>
        inline Storage::AVKey mapToKey(const T i) {
            switch(i) {
            case 0:
                return Storage::AVKey::Ch0;
            case 1:
                return Storage::AVKey::Ch1;
            case 2:
                return Storage::AVKey::Ch2;
            case 3:
                return Storage::AVKey::Ch3;
            case 4:
                return Storage::AVKey::Ch4;
            case 5:
                return Storage::AVKey::Ch5;
            case 6:
                return Storage::AVKey::Ch6;
            case 7:
                return Storage::AVKey::Ch7;
            }
            return Storage::AVKey::Undefined;
        }
        std::array<value_type, static_cast<uint8_t>(AVKey::_Number)> AValues;
    };
}

using eeprom = EEProm::Controller<Storage::ApplData>;

using ibus_switch = IBus::Switch::MultiAdapter<servo_pa, Meta::List<sw>, eeprom>;

int main() {
    uint8_t counter = 0;
    
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
   
//    servo::init<AVR::BaudRate<100000>, FullDuplex, true, 1>(); // 8E2
    servo::init<AVR::BaudRate<115200>>(); 

    terminalDevice::init<AVR::BaudRate<9600>>();
    
    systemTimer::init();

    ibus_switch::init();
    
    led::template dir<Output>();     
    pa0::template dir<Output>();     

    const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);

    using ch_t = servo_pa::channel_t;
    
    while(true) {
        terminalDevice::periodic();
        servo::periodic();        
        ibus_switch::periodic();
        
        systemTimer::periodic([&]{
            pa0::toggle();
            
//            servo_pa::ratePeriodic();
            
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    led::toggle();
//                    etl::out<terminal>("sw: "_pgm);
//                    for(const auto& s: sw::switches()) {
//                        etl::out<terminal>((uint8_t) s);
//                    }
//                    etl::outl<terminal>(" cnt: "_pgm, servo_pa::c, " ch0: "_pgm, servo_pa::value(ch_t{9}).toInt(), " s: "_pgm, (uint8_t)servo_pa::mState);
//                    etl::outl<terminal>(" ch0: "_pgm, servo_pa::value(ch_t{9}).toInt(), " pt0: "_pgm, eeprom::data()[0].passThru().toInt());
                    etl::outl<terminal>(" ch14: "_pgm, servo_pa::value(ch_t{13}).toInt());
                    etl::outl<terminal>(" ch15: "_pgm, servo_pa::value(ch_t{14}).toInt());
                    etl::outl<terminal>(" ch16: "_pgm, servo_pa::value(ch_t{15}).toInt());
                    etl::outl<terminal>(" ch17: "_pgm, servo_pa::value(ch_t{16}).toInt());
                    etl::outl<terminal>(" ch18: "_pgm, servo_pa::value(ch_t{17}).toInt());
                    
                    const auto cv = servo_pa::value(ch_t{9});
                    
//                    if (IBus::Switch::Protocol1::isControlMessage(cv)) {
//                        const auto p = IBus::Switch::Protocol1::toParameter(cv);
//                        const auto v = IBus::Switch::Protocol1::toParameterValue(cv);
//                        etl::outl<terminal>(" p: "_pgm, p.toInt(), " v: "_pgm, v.toInt());
//                    }
                    
                }
            });
        });
    }
}

