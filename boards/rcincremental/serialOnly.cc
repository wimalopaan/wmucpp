#define NDEBUG

// Nur serial

#include <mcu/avr.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/sigrow.h>
#include <mcu/internals/sleep.h>
#include <mcu/internals/dac.h>
#include <mcu/internals/ccl.h>
#include <mcu/internals/eeprom.h>
#include <mcu/internals/watchdog.h>

#include <external/solutions/cells.h>
#include <external/solutions/blinker.h>
#include <external/ibus/ibus2.h>
#include <external/hott/hott.h>
#include <external/hott/experimental/sensor.h>
#include <external/hott/experimental/adapter.h>
#include <external/hal/alarmtimer.h>
#include <external/hal/adccontroller.h>
#include <external/solutions/series01/swuart.h>
#include <external/solutions/analogsensor.h>
#include <external/solutions/tick.h>
#include <external/solutions/button.h>
#include <external/solutions/rotaryencoder.h>
#include <external/solutions/series01/sppm_out.h>
#include <external/solutions/series01/cppm_out.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

inline static constexpr auto fRtc = 2000_Hz;
inline static constexpr auto longPress = 3000_ms;

template<typename Out, typename In>
static inline Out enum_cast(const In in) {
    return Out{(uint8_t)in};    
}

namespace SimpleProtocol {
    struct CheckSum final {
        inline void reset() {
            mSum = 0;
        }
        inline const std::byte& operator+=(const std::byte& b) {
            mSum += static_cast<uint8_t>(b);
            return b;
        }
        inline operator std::byte() const {
            return std::byte{mSum};
        }
    private:
        uint8_t mSum{};
    };
    
    template<uint8_t Size>
    struct Sender final {
        using values_type = std::array<std::byte, Size>;
        
        static inline constexpr uint8_t size() {
            return values_type::size();
        }
        static inline constexpr uint8_t length() {
            return values_type::size() + 2;
        }
        template<uint8_t N>
        static inline void set(const std::byte v) {
            static_assert(N < Size);
            mValues[N] = v;
        }
        template<typename Dev>
        static inline void send() {
            CheckSum cs;
            cs += startByte;
            Dev::put(startByte);
            
            for(const auto& v: mValues) {
                cs += v;
                Dev::put(v);
            }
            Dev::put(cs);
        }
    private:
        inline static constexpr std::byte startByte{0xa5};
        inline static values_type mValues{};
    };
    
    template<auto Size, typename MCU = DefaultMcuType>
    struct Adapter {
        using values_type = std::array<std::byte, Size>;
        
        inline static constexpr std::byte startByte{0xa5};
        
        enum class State : uint8_t {Init, AwaitData, AwaitCS};
        
        inline static bool process(const std::byte b) {
            static CheckSum cs;
            switch(mState) {
            case State::Init:
                cs.reset();
                if (b == startByte) {
                    cs += b;
                    mReceivesBytes.setToBottom();
                    mState = State::AwaitData;
                }
                break;
            case State::AwaitData:
                mBuffer[mReceivesBytes] = b;
                cs += b;
                if (mReceivesBytes.isTop()) {
                    mState = State::AwaitCS;
                }
                ++mReceivesBytes;
                break;
            case State::AwaitCS:
                if (b == cs) {
                    etl::copy(mValues, mBuffer);
                }
                mState = State::Init;
                break;
            }
            return true;
        }
        inline static const values_type& data() {
            return mValues;
        }
        inline static void ratePeriodic() {}
    private:
        inline static etl::index_type_t<values_type> mReceivesBytes;
        inline static values_type mBuffer{};
        inline static values_type mValues{};
        inline static State mState{State::Init};
    };
}

namespace Storage {
    enum class Rot : uint8_t {Absolute = 1, Bounded, Speed};
    
    struct ApplData final : public EEProm::DataBase<ApplData> {
        
        uint8_t magic() {
            return mMagic;
        }
        void clear() {
            mMagic = 42;
            mRot = Rot::Absolute;
            change();
        }
        Rot& rotation() {
            return mRot;
        }
    private:
        Rot mRot{Rot::Absolute};
        uint8_t mMagic{};
    };
}

using sender = SimpleProtocol::Sender<4>;
using recv_pa = SimpleProtocol::Adapter<sender::size()>;

template<typename HWRev = void>
struct Devices {
    using ccp = Cpu::Ccp<>;
    using clock = Clock<>;
    using sigrow = SigRow<>;
    using sleep = Sleep<>;
    using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
    
    using tca0Position = AVR::Portmux::Position<AVR::Component::Tca<0>, Portmux::Default>;
    using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Alt1>;
    
    using uart = Usart<usart0Position, recv_pa, UseInterrupts<false>, ReceiveQueueLength<0>, SendQueueLength<sender::length() + 10>>;

    using outPin = Pin<Port<A>, 1>;
    
    using portmux = Portmux::StaticMapper<Meta::List<tca0Position, usart0Position>>;
    
    using pinA = Pin<Port<A>, 3>;
    using pinB = Pin<Port<A>, 2>;
    
    using rot1_t = uint8_t; // wrap around
    using rot2_t = etl::uint_ranged<uint8_t, 0, 255>; // saturating
    using rotary = External::RotaryEncoder<pinA, pinB, std::variant<rot1_t, rot2_t>>;
    
    using pinT = Pin<Port<A>, 7>;
    using b = ActiveLow<pinT, Input>;
    using button = External::Button2<b, systemTimer, External::Tick<systemTimer>(50_ms), External::Tick<systemTimer>(longPress)>;
    
    using eeprom = EEProm::Controller<Storage::ApplData>;
    
    using wdt   = WatchDog<systemTimer::intervall>;
};

template<typename Timer, typename Rot, typename But,
         typename Uart, typename EEProm>
struct GlobalFSM {
    enum class State : uint8_t {RunAbsolute = (uint8_t)Storage::Rot::Absolute, 
                                RunBounded = (uint8_t)Storage::Rot::Bounded,
                                RunSpeed = (uint8_t)Storage::Rot::Speed, 
                                Undefined, Init};
    
    static inline constexpr External::Tick<Timer> initTicks{100_ms};
    static inline constexpr External::Tick<Timer> speedTicks{200_ms};
    static inline constexpr External::Tick<Timer> eepromTicks{300_ms};
    
    static inline constexpr External::Tick<Timer> sendTicks{100_ms};
    
    using rot1_t = Meta::nth_element<0, typename Rot::type_list>;
    using rot2_t = Meta::nth_element<1, typename Rot::type_list>;
    
    using eeprom = EEProm;
    
    inline static void init() {
        Uart::template init<BaudRate<9600>>(); 
    }  
    inline static void periodic() {
        Uart::periodic(); 
        eeprom::saveIfNeeded();
    }  
    inline static void setOutputs(const uint8_t rot) {
        sender::set<0>(std::byte{rot});
        sender::set<1>(std::byte(But::pressed() ? 255 : 0));
        sender::set<2>(recv_pa::data()[0]);
        sender::set<3>(recv_pa::data()[1]);
        mSendTicks.on(sendTicks, [&]{
            sender::send<Uart>();            
        });
    }
    inline static void ratePeriodic() {
        const auto oldState = mState;
        ++mStateTicks;
        ++mSpeedTicks;
        ++mSendTicks;
        
        (++mEEpromTicks).on(eepromTicks, []{
            eeprom::data().expire(); 
        });
        
        Rot::rateProcess();
        But::ratePeriodic();
        
        const auto be = But::event();
        
        switch(mState) {
        case State::Undefined:
            mStateTicks.on(initTicks, []{
                mState = State::Init;
            });
            break;
        case State::Init:
            mStateTicks.on(initTicks, []{
                mState = enum_cast<State>(eeprom::data().rotation());
            });
            break;
        case State::RunAbsolute:
            if (be == But::Press::Long) {
                mState = State::RunBounded;
            }
            else {
                Rot::value().visit([&]<typename T>(const T vv){
                                       setOutputs(vv);
                                   });
            }
            break; 
        case State::RunBounded:
            if (be == But::Press::Long) {
                mState = State::RunSpeed;
            }
            else {
                Rot::value().visit([&]<typename T>(const T vv){
                                       setOutputs(vv);
                                   });
            }
            break; 
        case State::RunSpeed:
            if (be == But::Press::Long) {
                mState = State::RunAbsolute;
            }
            else {
                static uint8_t last{127}; 
                mSpeedTicks.on(speedTicks, [&]{
                    Rot::value().visit([&]<typename T>(const T vv){
                                           const uint8_t diff = 4 * (vv - last) + 127;
                                           last = vv;
                                           setOutputs(diff);
                                       });
                });
            }
            break;
        }
        if (oldState != mState) {
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                break;
            case State::RunAbsolute:
                Rot::set(rot1_t{127});
                eeprom::data().rotation() = enum_cast<Storage::Rot>(State::RunAbsolute);
                eeprom::data().change();
                break;
            case State::RunSpeed:
                Rot::set(rot1_t{127});
                eeprom::data().rotation() = enum_cast<Storage::Rot>(State::RunSpeed);
                eeprom::data().change();
                break;
            case State::RunBounded:
                Rot::set(rot2_t{127}); // activate other variante type
                eeprom::data().rotation() = enum_cast<Storage::Rot>(State::RunBounded);
                eeprom::data().change();
                break;
            }
        }
    }  
private:
    static inline External::Tick<Timer> mStateTicks;
    static inline External::Tick<Timer> mSpeedTicks;
    static inline External::Tick<Timer> mSendTicks;
    static inline External::Tick<Timer> mEEpromTicks;
    inline static State mState{State::Undefined};
    inline static bool mButton{};
};

template<typename Devs>
struct Application {
    using uart = Devs::uart;
    
    using gfsm = GlobalFSM<typename Devs::systemTimer, typename Devs::rotary, typename Devs::button, uart, typename Devs::eeprom>;
    
    inline static void init() {
        gfsm::init();
    }  
    
    [[noreturn]] inline static void run() {
        while(true) {
            gfsm::periodic();
            Devs::systemTimer::periodic([&]{
                gfsm::ratePeriodic();
            });
        }    
    }
};

using devices = Devices<>;
using app = Application<devices>;

int main() {
    app::init();
    app::run();
}

#ifndef NDEBUG
[[noreturn]] inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], const unsigned int line [[maybe_unused]]) noexcept {
    while(true) {
    }
}
template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, const unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif

