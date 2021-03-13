//#define NDEBUG
 
#define AUTO_BUS
#define LEARN_DOWN // start at highest channel number downwards

#include "board.h"
#include "swout.h"
#include "switch.h"

//template<typename PA, typename SW, typename OUT, typename NVM, typename Timer, typename Term = void>
template<typename BusDevs>
struct FSM {
    using devs = BusDevs::devs;
    using addr_t = RCSwitch::addr_t;
    
    using protocol_t = BusDevs::BusParam::proto_type;
    
    using Timer = devs::systemTimer;
    using Term = BusDevs::terminal;
    using NVM = BusDevs::eeprom;
    using SW = BusDevs::ibus_switch;
    using PA = BusDevs::servo_pa;
    using OUT = BusDevs::out;
    
    enum class State : uint8_t {Undefined, StartWait, SearchChannel, Run, ShowAddress, ShowAddressWait, LearnTimeout};
    
    static constexpr auto intervall = Timer::intervall;
    static constexpr External::Tick<Timer> learnTimeoutTicks{3000_ms};
    static constexpr External::Tick<Timer> waitTimeoutTicks{1000_ms};
    static constexpr External::Tick<Timer> showAdressTimeoutTicks{2000_ms};
    
    static inline void init(const bool inverted) {
        // todo:
    }
    
    static inline void periodic() {
        //todo: 
    }
    
    static inline void ratePeriodic() {
        const auto oldstate = mState;
#ifdef USE_SBUS
        PA::ratePeriodic();
#endif
        ++stateTicks;
        switch(mState) {
        case State::Undefined:
            mState = State::StartWait;
            break;
        case State::StartWait:
            stateTicks.on(waitTimeoutTicks, []{
                mState = State::SearchChannel;
            });
            break;
        case State::SearchChannel:
            stateTicks.on(learnTimeoutTicks, []{
                mState = State::LearnTimeout;
            });
            if (search()) {
                mState = State::ShowAddress;
            }
            break;
        case State::LearnTimeout:
            etl::outl<Term>("timeout ch: "_pgm, NVM::data().channel().toInt(), " adr: "_pgm, NVM::data().address().toInt());
            
            if (NVM::data().channel() && NVM::data().address()) {
                etl::outl<Term>("load eep"_pgm);
                SW::channel(NVM::data().channel());
                SW::address(typename SW::addr_t{NVM::data().address().toInt()});
            }
            etl::outl<Term>("using ch: "_pgm, SW::mChannel.toInt(), " adr: "_pgm, SW::mAddr.toInt());
            mState = State::Run;
            break;
        case State::ShowAddress:
            etl::outl<Term>("learned ch: "_pgm, NVM::data().channel().toInt(), " adr: "_pgm, NVM::data().address().toInt());
            if (auto const a = NVM::data().address()) {
                using index_t = typename OUT::index_t;
                OUT::setSwitchOn(index_t{a.toInt()});
                mState = State::ShowAddressWait;
            }
            else {
                mState = State::Run;
            }
            break;
        case State::ShowAddressWait:
            stateTicks.on(showAdressTimeoutTicks, []{
                mState = State::Run;
                OUT::allOff();
            });
            break;
        case State::Run:
            SW::periodic();
            OUT::setSwitches();
            break;
        }
        if (oldstate != mState) {
            stateTicks.reset();
        }
    }
#ifdef NDEBUG    
private:
#endif
    static inline bool search() {
        etl::outl<Term>("c: "_pgm, learnChannel.toInt(), " v: "_pgm, protocol_t::toParameterValue(PA::valueMapped(learnChannel.toRangedNaN())).toInt(), " r: "_pgm, PA::valueMapped(learnChannel.toRangedNaN()).toInt());
        if (const auto lc = PA::valueMapped(learnChannel.toRangedNaN()); lc && SW::isLearnCode(lc)) {
            if (const auto pv = protocol_t::toParameterValue(lc).toInt(); (pv >= 1) && ((pv - 1) <= RCSwitch::addr_t::Upper)) {
                const uint8_t addr = pv - 1;
                SW::channel(learnChannel.toRangedNaN());
                SW::address(addr_t(addr));
                NVM::data().channel() = learnChannel;
                NVM::data().address() = addr;
                NVM::data().change();
                return true;
            }
        }   
#ifdef LEARN_DOWN
        --learnChannel;
#else
        ++learnChannel;
#endif
        return false;
    }
    using ch_t = PA::channel_t;

#ifdef LEARN_DOWN
    static inline etl::uint_ranged_circular<uint8_t, ch_t::Lower, ch_t::Upper> learnChannel{ch_t::Upper};
#else
    static inline etl::uint_ranged_circular<uint8_t, ch_t::Lower, ch_t::Upper> learnChannel{0};
#endif
    static inline State mState{State::Undefined};
    inline static External::Tick<Timer> stateTicks;
};

template<auto N = 8>
struct SwitchStates {
    enum class SwState : uint8_t {Off, Blink1, Steady, Blink2, PassThru};
    
    static constexpr void init() {
    }
    static constexpr uint8_t size() {
        return N;
    }
    static inline auto& switches() {
        return swStates;
    }
    static inline void testMode(const auto& v) {
//        decltype(v)::_;
        if (const auto vv = v.toInt(); vv == 0) {
            std::fill(std::begin(swStates), std::end(swStates), SwState::Off);
        }
        else if ((vv >= 1) && (vv <= 8)) {
            std::fill(std::begin(swStates), std::end(swStates), SwState::Off);
            swStates[vv - 1] = SwState::Steady;
        }
        else {
            std::fill(std::begin(swStates), std::end(swStates), SwState::Steady);
        }
        
    }
private:
    static inline std::array<SwState, N> swStates{};
};

template<typename HWRev = void, typename MCU = DefaultMcuType>
struct Devices {
    using ccp = Cpu::Ccp<>;
    using clock = Clock<>;
    using sigrow = SigRow<>;

    using systemTimer = SystemTimer<Component::Rtc<0>, External::Bus::fRtc>;

    using scanDevPosition = usart0Position;

    using ppmDevPosition = void;  
    
    using evrouter = ::evrouter;

    using scan_term_dev = void;

    using scanLedPin = void;
    
    static inline void init() {
        portmux::init();
        ccp::unlock([]{
            clock::template prescale<1>();
        });
        systemTimer::init(); 
    }
    static inline void periodic() {}
    
};

template<typename Bus>
struct BusDevs;

template<typename Devs>
struct BusDevs<External::Bus::IBusIBus<Devs>> {
    using bus_type = External::Bus::IBusIBus<Devs>;
    
    struct BusParam {
        using bus_t = bus_type;
        using proto_type = RCSwitch::Protocol2<RCSwitch::High>;
    };
    
    using devs = Devs;
    
    using servo_pa = IBus2::Servo::ProtocollAdapter<0>;
    using servo = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

    using terminal = etl::basic_ostream<servo>;
    
    using eeprom = EEProm::Controller<Storage::ApplData<typename servo_pa::channel_t, RCSwitch::addr_t>>;

    using sw = SwitchStates<>;
    
    using out = External::Output<ledList, pwm, sw, eeprom>;
    
    using ibus_switch = Digital<BusDevs>;
};

template<typename Devs>
struct BusDevs<External::Bus::SBusSPort<Devs>> {
    using bus_type = External::Bus::SBusSPort<Devs>;
    
    struct BusParam {
        using bus_t = bus_type;
        using proto_type = RCSwitch::Protocol2<RCSwitch::Low>;
    };
    
    using devs = Devs;
    
    using servo_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer>;
    using servo = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

    using terminal = etl::basic_ostream<servo>;
    
    using eeprom = EEProm::Controller<Storage::ApplData<typename servo_pa::channel_t, RCSwitch::addr_t>>;

    using sw = SwitchStates<>;
    
    using out = External::Output<ledList, pwm, sw, eeprom>;
    
    using ibus_switch = Digital<BusDevs>;
    
};
template<typename BusSystem, typename MCU = DefaultMcuType>
struct Application {
    using busdevs = BusDevs<BusSystem>;
    
    inline static void run(const bool inverted = false) {
        if constexpr(External::Bus::isIBus<BusSystem>::value || External::Bus::isSBus<BusSystem>::value) {
            using devs = busdevs::devs;
            using systemTimer = devs::systemTimer;
            using gfsm = FSM<busdevs>;
            
            gfsm::init(inverted);
            while(true) {
                gfsm::periodic(); 
                systemTimer::periodic([&]{
                    gfsm::ratePeriodic();
                });
            }
        }
    }
};

using devices = Devices<>;

#ifndef NDEBUG
using scanner = External::Scanner<devices, Application>;
#else
using scanner = External::Scanner<devices, Application, AVR::HalfDuplex>;
#endif

int main() {
    scanner::run();
}

#if 0

int main() {
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
    
    fsm::init();
    
#ifdef USE_SBUS
    servo::init<AVR::BaudRate<100000>, FullDuplex, true, 1>(); // 8E2
#endif
#ifdef USE_IBUS
    servo::init<BaudRate<115200>>();
#endif
    systemTimer::init();
    out::init();

#ifdef USE_SBUS
    etl::outl<terminal>("multi8d sbus"_pgm);
#endif    
#ifdef USE_IBUS
    etl::outl<terminal>("multi8d ibus"_pgm);
#endif    
    
    eeprom::init();
    if (!((appData.magic() == 42))) {
        appData.clear();
        appData.magic() = 42;
        appData.change();
#ifndef NDEBUG
        etl::outl<terminal>("eep init"_pgm);
#endif
    }
    else {
#ifndef NDEBUG
        etl::outl<terminal>("eep ch: "_pgm, appData.channel().toInt(), " adr: "_pgm, appData.address().toInt());        
#endif
    }
    
    const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);
    const auto eepromTimer = alarmTimer::create(3000_ms, External::Hal::AlarmFlags::Periodic);
    
//    uint16_t counter{};
    
    while(true) {
        eeprom::saveIfNeeded([&]{
            etl::outl<terminal>("save eep"_pgm);
        });
        servo::periodic();
        systemTimer::periodic([&]{
            fsm::ratePeriodic();
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
#ifndef NDEBUG
//                    etl::outl<terminal>("c: "_pgm, counter++, " mpx0: "_pgm, (uint8_t)appData.mMpxModes[0]);
//                    etl::outl<terminal>("c9: "_pgm, servo_pa::valueMapped(9).toInt(), " r: "_pgm, servo_pa::value(9).toInt());
//                    etl::outl<terminal>("c9: "_pgm, servo_pa::valueMapped(9).toInt(), " r: "_pgm, servo_pa::value(9).toInt());
//                    etl::outl<terminal>("c0: "_pgm, servo_pa::mChannels[9]);
//                    etl::outl<terminal>("lo: "_pgm, ibus_switch::lastOnIndex.toInt());
//                    etl::outl<terminal>("sw0: "_pgm, (uint8_t)sw::switches()[0], "sw1: "_pgm, (uint8_t)sw::switches()[1]);
                    etl::out<terminal>("sw: [ "_pgm);
                    for(const auto& l : sw::switches()) {
                        etl::out<terminal>(uint8_t(l), " "_pgm);
                    }
                    etl::outl<terminal>(" ]"_pgm);
//                    etl::outl<terminal>("lpp: "_pgm, (uint8_t)ibus_switch::lpp, " lpv: "_pgm, (uint8_t)ibus_switch::lpv, " lmv: "_pgm, (uint8_t)ibus_switch::lmv, " lo: "_pgm, (uint8_t)ibus_switch::lastOnIndex.toInt());
//                    etl::outl<terminal>("b0: "_pgm, (uint8_t)appData[0].blinks()[0].intervall.value, "b1: "_pgm, (uint8_t)appData[0].blinks()[1].intervall.value);
                    etl::outl<terminal>("state:  "_pgm, (uint8_t)fsm::mState);
                    
#endif
                }
                else if (eepromTimer == t) {
                    appData.expire();
                }
            });
        });
    }
}
#endif

#ifndef NDEBUG
/*[[noreturn]] */inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], unsigned int line [[maybe_unused]]) noexcept {
    while(true) {
////        dbg1::toggle();
    }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
