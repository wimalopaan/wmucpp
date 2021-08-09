#define NDEBUG
 
#define AUTO_BUS
#define LEARN_DOWN // start at highest channel number downwards
 
#include "board.h"
#include "swout.h"
#include "switch.h"

template<typename BusDevs>
struct FSM {
    using devs = BusDevs::devs;
    using bus_type = BusDevs::bus_type;
    using addr_t = RCSwitch::addr_t;
    
    using protocol_t = BusDevs::BusParam::proto_type;
    
    using Timer = devs::systemTimer;
    using Term = BusDevs::terminal;
    using NVM = BusDevs::eeprom;
    using SW = BusDevs::ibus_switch;
    using Servo = BusDevs::servo;
    using PA = BusDevs::servo_pa;
    using OUT = BusDevs::out;
    
    using jp = devs::jpPin;
    
    enum class State : uint8_t {Undefined,
                                CheckJp, ShowBus, ShowBusWait,
                                StartWait, SearchChannel, Run, ShowAddress, ShowAddressWait, LearnTimeout};
    
    static constexpr auto intervall = Timer::intervall;
    static constexpr External::Tick<Timer> learnTimeoutTicks{3000_ms};
    static constexpr External::Tick<Timer> waitTimeoutTicks{1000_ms};
    static constexpr External::Tick<Timer> showAdressTimeoutTicks{2000_ms};
    static constexpr External::Tick<Timer> eepromTimeoutTicks{1000_ms};
    
    inline static auto& data() {
        return NVM::data();
    }
    
    static inline void init(const bool inverted) {
        jp::template dir<Input>();
        jp::template pullup<true>();
        if constexpr(External::Bus::isIBus<bus_type>::value) {
            Servo::template init<BaudRate<115200>>();
            etl::outl<Term>("multi8d ibus"_pgm);
        }
        else if constexpr(External::Bus::isSBus<bus_type>::value) {
            Servo::template init<AVR::BaudRate<100000>, FullDuplex, true, 1>(); // 8E2
            if (inverted) {
                Servo::rxInvert(true);
            }
            etl::outl<Term>("multi8d sbus"_pgm);
        }
        else {
            static_assert(std::false_v<bus_type>, "bus type not supported");
        }

        systemTimer::init();
        OUT::init();
        
        NVM::init();
        if (!((data().magic() == 42))) {
            data().clear();
            data().magic() = 42;
            data().change();
#ifndef NDEBUG
            etl::outl<Term>("eep init"_pgm);
#endif
        }
        else {
#ifndef NDEBUG
            etl::outl<Term>("eep ch: "_pgm, data().channel().toInt(), " adr: "_pgm, data().address().toInt());        
#endif
        }
    }
    
    static inline void periodic() {
        NVM::saveIfNeeded([&]{
            etl::outl<Term>("save eep"_pgm);
        });
        Servo::periodic();
    }
    
    using index_t = typename OUT::index_t;
    
    static inline void ratePeriodic() {
        const auto oldstate = mState;
        PA::ratePeriodic();
        ++stateTicks;
        switch(mState) {
        case State::Undefined:
            mState = State::CheckJp;
            break;
        case State::CheckJp:
            if (!jp::read()) {
                mShow = true;
                mState = State::ShowBus;
            }
            else {
                mState = State::StartWait;
            }
            break;
        case State::ShowBus:
            if constexpr(External::Bus::isIBus<bus_type>::value) {
                OUT::setSwitchOn(index_t{5});                    
                OUT::setSwitchOn(index_t{6});                    
                OUT::setSwitchOn(index_t{7});                    
            }
            if constexpr(External::Bus::isSBus<bus_type>::value) {
                OUT::setSwitchOn(index_t{6});                    
                OUT::setSwitchOn(index_t{7});                    
            }
            mState = State::ShowBusWait;
            break;
        case State::ShowBusWait:
            stateTicks.on(waitTimeoutTicks, []{
                mState = State::StartWait;
                OUT::allOff();
            });
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
            stateTicks.on(eepromTimeoutTicks, []{
                data().expire();
            });
            SW::periodic();
            OUT::setSwitches();
            break;
        }
        if (oldstate != mState) {
            stateTicks.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::CheckJp:
                break;
            case State::ShowBus:
                if constexpr(External::Bus::isIBus<bus_type>::value) {
                    if (mShow) {
                        OUT::setSwitchOn(index_t{5});                    
                        OUT::setSwitchOn(index_t{6});                    
                        OUT::setSwitchOn(index_t{7});                    
                    }
                }
                if constexpr(External::Bus::isSBus<bus_type>::value) {
                    if (mShow) {
                        OUT::setSwitchOn(index_t{6});                    
                        OUT::setSwitchOn(index_t{7});                    
                    }
                }
                break;
            case State::ShowBusWait:
                break;
            case State::StartWait:
                if (mShow) {
                    OUT::allOff();
                    OUT::setSwitchOn(index_t{7});    
                }
                break;
            case State::SearchChannel:
                if (mShow) {
                    OUT::allOff();
                    OUT::setSwitchOn(index_t{6});    
                }
                break;
            case State::LearnTimeout:
                if (mShow) {
                    OUT::allOff();
                    OUT::setSwitchOn(index_t{5});    
                }
                break;
            case State::ShowAddress:
                if (mShow) {
                    OUT::allOff();
                    OUT::setSwitchOn(index_t{4});    
                }
                break;
            case State::ShowAddressWait:
                if (mShow) {
                    OUT::allOff();
                    OUT::setSwitchOn(index_t{3});    
                }
                break;
            case State::Run:
                OUT::allOff();
                break;
            }
        }
    }
#ifdef NDEBUG    
private:
#endif
    static inline bool mShow{false};
    
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
    
    using jpPin = lvPin;
    
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

//#ifndef NDEBUG
//using scanner = External::Scanner<devices, Application>;
//#else
//using scanner = External::Scanner<devices, Application, AVR::HalfDuplex>;
//#endif

using scanner = External::Scanner<devices, Application>;

int main() {
    scanner::run();
}

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
