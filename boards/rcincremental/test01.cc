#define NDEBUG

// Drei Modi: 
// 1) Poti (dac / taster)
// 2) Cppm-Master (cppm / uart-rx)
// 3) Cppm-Slave (dac / uart-tx)
// todo
// 4) Cppm-Master (cppm inverted / uart-rx)
//
// Beim Einschalten Taster 1s gedrückt halten, mit LED an Out-Pin kontrollierem
// Blink-Muster

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
//#include <external/ibus/ibus.h>
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
    enum class Mode : uint8_t {Poti = 1, Master, Slave};

    enum class Rot : uint8_t {Absolute = 1, Bounded, Speed};
    
    struct Poti : std::integral_constant<Mode, Mode::Poti>{};
    namespace Cppm {
        struct Master: std::integral_constant<Mode, Mode::Master>{};
        struct Slave: std::integral_constant<Mode, Mode::Slave>{};
    }

    struct ApplData final : public EEProm::DataBase<ApplData> {
        
        uint8_t magic() {
            return mMagic;
        }
        void clear() {
            mMagic = 42;
            mMode = Mode::Poti;
            mRot = Rot::Absolute;
            change();
        }
        Mode& mode() {
            return mMode;
        }
        Rot& rotation() {
            return mRot;
        }
    private:
        Rot mRot{Rot::Absolute};
        Mode mMode{Mode::Poti};
        uint8_t mMagic{};
    };
}

using sender = SimpleProtocol::Sender<2>;
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

    // Dac / lut(cppm) Ausgang
#ifndef NDEBUG
    using cppm = void;
    using lut0 = void;
    using dac = void;
#else
    using cppm = External::Ppm::Cppm<tca0Position>;
    using lut0 = Ccl::SimpleLut<0, Ccl::Input::Mask, Ccl::Input::Tca0<1>, Ccl::Input::Mask>;    
    using dac = DAC<0>;
#endif
    
    // Tastausgang / led / uart
#ifndef NDEBUG
    using uart = Usart<usart0Position, recv_pa, UseInterrupts<false>, ReceiveQueueLength<0>>;
#else
    using uart = Usart<usart0Position, recv_pa, UseInterrupts<false>, ReceiveQueueLength<0>, 
    SendQueueLength<sender::length() + 10>>;
#endif
#ifndef NDEBUG
    using outPin = Pin<Port<A>, 6>; // Test
#else
    using outPin = Pin<Port<A>, 1>; // Tastausgang
#endif
    
    using portmux = Portmux::StaticMapper<Meta::List<tca0Position, usart0Position>>;
    
    using pinA = Pin<Port<A>, 3>;
    using pinB = Pin<Port<A>, 2>;

    using rot1_t = uint8_t;
    using rot2_t = etl::uint_ranged<uint8_t, 0, 255>;
    using rotary = External::RotaryEncoder<pinA, pinB, std::variant<rot1_t, rot2_t>>;
    
    using pinT = Pin<Port<A>, 7>;
    using b = ActiveLow<pinT, Input>;
    using button = External::Button2<b, systemTimer, External::Tick<systemTimer>(50_ms), External::Tick<systemTimer>(longPress)>;

    using eeprom = EEProm::Controller<Storage::ApplData>;

    using wdt   = WatchDog<systemTimer::intervall>;
};

template<typename Timer, typename Rot, typename But, typename Dac, typename OutPin, typename CPpm, typename Lut,
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
        if constexpr(!std::is_same_v<CPpm, void>) {
            CPpm::init();        
            if constexpr(!std::is_same_v<Lut, void>) {
                Lut::init(std::byte{0x33}); // invert
//                Lut::init(std::byte{0xcc}); // no invert
            }
        }
        if constexpr(!std::is_same_v<Dac, void>) {
            Dac::init();        
        }
        if constexpr(!std::is_same_v<OutPin, void>) {
            OutPin::low();
            OutPin::template dir<Output>();
        }
        if constexpr(!std::is_same_v<Uart, void>) {
            if constexpr(!std::is_same_v<CPpm, void>) {
                Uart::template init<BaudRate<9600>, AVR::HalfDuplex>(); 
            }
            else {
                Uart::template init<BaudRate<9600>>(); 
            }
        }
    }  
    inline static void periodic() {
        if constexpr(!std::is_same_v<Uart, void>) {
            Uart::periodic(); 
        }
        if constexpr(!std::is_same_v<CPpm, void>) {
            CPpm::periodic();        
        }
        eeprom::saveIfNeeded();
    }  
    inline static void setOutputs(const uint8_t rot) {
        if constexpr(!std::is_same_v<Dac, void>) { // poti
            Dac::put(rot);
            if constexpr(!std::is_same_v<OutPin, void>) {
                if (But::pressed()) {
                    OutPin::high();
                }
                else {
                    OutPin::low();
                }
            }                                      
        }
        if constexpr(!std::is_same_v<CPpm, void>) { // master
            using channel_t = CPpm::channel_t;
            using ch_value_t = typename CPpm::ranged_type;
            const auto rv = etl::uint_ranged<uint8_t, 0, 255>{rot};
            const ch_value_t v = etl::scaleTo<ch_value_t>(rv);
            CPpm::set(channel_t{0}, v);
            CPpm::set(channel_t{1}, (But::pressed() ? ch_value_t{ch_value_t::Upper} : ch_value_t{ch_value_t::Lower}));
            if constexpr(!std::is_same_v<Uart, void>) {
                using channel_t = CPpm::channel_t;
                using ch_value_t = typename CPpm::ranged_type;
                const auto x1 = recv_pa::data()[0];
                const auto rv1 = etl::uint_ranged<uint8_t, 0, 255>{(uint8_t)x1};
                const ch_value_t v1 = etl::scaleTo<ch_value_t>(rv1);
                CPpm::set(channel_t{2}, v1);
                const auto x2 = recv_pa::data()[1];
                const auto rv2 = etl::uint_ranged<uint8_t, 0, 255>{(uint8_t)x2};
                const ch_value_t v2 = etl::scaleTo<ch_value_t>(rv2);
                CPpm::set(channel_t{3}, v2);
            }
        }
        else { 
            if constexpr(!std::is_same_v<Uart, void>) { // slave
                sender::set<0>(std::byte{rot});
                sender::set<1>(std::byte(But::pressed() ? 255 : 0));
                mSendTicks.on(sendTicks, [&]{
                    sender::send<Uart>();            
                });
            }
        }
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
        {
            if (be == But::Press::Long) {
                mState = State::RunBounded;
            }
            uint8_t rvalue;
            Rot::value().visit([&]<typename T>(const T vv){
                                   rvalue = vv;
                               });
            setOutputs(rvalue);
        }  
            break; 
        case State::RunBounded:
        {
            if (be == But::Press::Long) {
                mState = State::RunSpeed;
            }
            uint8_t rvalue;
            Rot::value().visit([&]<typename T>(const T vv){
                                   rvalue = vv;
                               });
            setOutputs(rvalue);
        }
            break; 
        case State::RunSpeed:
        {
            if (be == But::Press::Long) {
                mState = State::RunAbsolute;
            }
            mSpeedTicks.on(speedTicks, [&]{
                static uint8_t last = 127; // why warning?
                uint8_t rvalue;
                Rot::value().visit([&]<typename T>(const T vv){
                                       rvalue = vv;
                                   });
                const uint8_t diff = 4 * (rvalue - last) + 127;
                last = rvalue;
                setOutputs(diff);
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

template<typename Devs, typename Config>
struct Application {
    using dac = std::conditional_t<(std::is_same_v<Config, Storage::Poti> || std::is_same_v<Config, Storage::Cppm::Slave>), typename Devs::dac, void>;
    using outPin = std::conditional_t<std::is_same_v<Config, Storage::Poti>, typename Devs::outPin, void>;
    using cppm = std::conditional_t<std::is_same_v<Config, Storage::Cppm::Master>, typename Devs::cppm, void>;
    using lut = std::conditional_t<std::is_same_v<Config, Storage::Cppm::Master>, typename Devs::lut0, void>;
    using uart = std::conditional_t<(std::is_same_v<Config, Storage::Cppm::Master>) || (std::is_same_v<Config, Storage::Cppm::Slave>), typename Devs::uart, void>;
     
    using gfsm = GlobalFSM<typename Devs::systemTimer, typename Devs::rotary, typename Devs::button, dac, outPin, cppm, lut, uart, typename Devs::eeprom>;
    
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

template<typename Devs, template<typename, typename> typename App>
struct Startup {
    using timer = Devs::systemTimer;
    using button = Devs::button;
    using outPin = Devs::outPin;
    using b_ev_t = button::Press;
    using eeprom = Devs::eeprom;
    using rotary = Devs::rotary;
        
    enum class State : uint8_t {Undefined = 100, Wait, Config, EEProm, 
                                Debug,
                                ModePoti = (uint8_t)Storage::Poti::value, 
                                ModeCppmMaster = (uint8_t)Storage::Cppm::Master::value, 
                                ModeCppmSlave = (uint8_t)Storage::Cppm::Slave::value}; 

    static inline constexpr External::Tick<timer> initTicks{200_ms};
    static inline constexpr External::Tick<timer> eepromTicks{200_ms};
    static inline constexpr External::Tick<timer> waitTimeoutTicks{longPress + 1000_ms};
    static inline constexpr External::Tick<timer> debugTicks{500_ms};
  
    using led = ActiveHigh<outPin, Output>;
    using blinker = External::Blinker2<led, timer, 100_ms, 2000_ms, 3>;
    using bl_count_t = blinker::count_type;
    
#ifndef NDEBUG
    using term = etl::basic_ostream<typename Devs::uart>;
#endif
    
    inline static void init() {
//        Devs::wdt::template init<typename Devs::ccp>();
        Devs::portmux::init();
        Devs::ccp::unlock([]{
            Devs::clock::template prescale<2>(); 
        });
        timer::init();
        button::init();
        eeprom::init();
        if (eeprom::data().magic() != 42) {
            eeprom::data().clear();
        }
        rotary::init(127);
        blinker::init();
#ifndef NDEBUG
        Devs::uart::template init<BaudRate<9600>>();
#endif
    }
    
    [[noreturn]]inline static void run() {
#ifndef NDEBUG
        etl::outl<term>("test01"_pgm);
#endif
        while(true) {
#ifndef NDEBUG
            Devs::uart::periodic();
#endif
            eeprom::saveIfNeeded();
            
            Devs::systemTimer::periodic([]{
                rotary::rateProcess();
                blinker::ratePeriodic();
                button::ratePeriodic();

                const auto e = button::event();
                
                (++mEEpromTicks).on(eepromTicks, []{
                    eeprom::data().expire(); 
                });
                
                const auto oldState = mState;
                ++mStateTicks;
            
                switch(mState) {
                case State::Undefined:
                    if (e == b_ev_t::Short) {
                        mState = State::Wait;
                    }
                    mStateTicks.on(initTicks,[]{
                        mState = State::EEProm;
                    });
                    break;
                case State::Wait:
                    if (e == b_ev_t::Long) {
                        mState = State::Config;
                    }
                    if (e == b_ev_t::Release) {
                        mState = State::EEProm;
                    }
                    mStateTicks.on(waitTimeoutTicks,[]{
                        mState = State::EEProm;
                    });
                    break;
                case State::Config:
                    if (e == b_ev_t::Long) {
                        mState = State::EEProm;
                        eeprom::data().mode() = Storage::Mode{blinker::blinkCount().toInt()};
                        eeprom::data().change();
                    }
                    else {
                        rotary::value().visit([]<typename T>(const T& vv){
                                                  if constexpr(std::is_same_v<T, uint8_t>) {
                                                      static uint8_t last{127};
                                                      if (vv > last) {
                                                          auto bc = blinker::blinkCount();
                                                          blinker::blink(++bc);
                                                      }
                                                      if (vv < last) {
                                                          auto bc = blinker::blinkCount();
                                                          blinker::blink(--bc);                                                      
                                                      }
                                                      last = vv;
                                                  }
                                              }); 
                    }
                    break;
                case State::EEProm:
//                    mState = State::Debug;
                    mState = enum_cast<State>(eeprom::data().mode());
                    break;
                case State::ModePoti:
                    App<Devs, Storage::Poti>::run();
                    break;
                case State::ModeCppmMaster:
                    App<Devs, Storage::Cppm::Master>::run();
                    break;
                case State::ModeCppmSlave:
                    App<Devs, Storage::Cppm::Slave>::run();
                    break;
                case State::Debug:
                    mStateTicks.on(debugTicks, []{
//                        static uint8_t c= 0;
//                        etl::outl<term>("d: "_pgm, c++);
                    });
                    break;
                }
                if (oldState != mState) {
                    mStateTicks.reset();
                    switch(mState) {
                    case State::Undefined:
                        break;
                    case State::Config:
//                        etl::outl<term>("s c"_pgm);
                        blinker::init();
                        blinker::blink(bl_count_t{(uint8_t)eeprom::data().mode()});
//                        blinker::blink(bl_count_t{2});
                        break;
                    case State::EEProm:
                        blinker::off();
                        break;
                    case State::Wait:
                        break;
                    case State::ModePoti:
                        App<Devs, Storage::Poti>::init();
                        break;
                    case State::ModeCppmMaster:
                        App<Devs, Storage::Cppm::Master>::init();
                        break;
                    case State::ModeCppmSlave:
                        App<Devs, Storage::Cppm::Slave>::init();
                        break;
                    case State::Debug:
                        break;
                    }            
                }
            });
        }
    }
private:
    static inline External::Tick<timer> mEEpromTicks;
    static inline External::Tick<timer> mStateTicks;
    static inline State mState{State::Undefined};
};

using devices = Devices<>;
using startup = Startup<devices, Application>;

int main() {
    startup::init();
    startup::run();
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

