#define NDEBUG

//#define DEBUG2 // RX/TX change -> full duplex (man muss dann Ibus-Input und Telemetrie tauschen)

#define INV_LED // onboad LED inverted

#define AUTO_BUS // SCAN für SBus / IBus
#define SBUS_IBUS_NO_WARN

#define LEARN_DOWN // start at highest channel number downwards

#include "board.h"

template<typename BusDevs>
struct GFSM {
    using devs = BusDevs::devs;
    using Timer = devs::systemTimer;
    using configPin = devs::configPin;
    
    using bus_type = BusDevs::bus_type;
    using servo = BusDevs::servo;
    using PA = BusDevs::servo_pa;
    using terminal = BusDevs::terminal;
    
    using channel_t = PA::channel_t;
    using search_t = etl::uint_ranged_circular<uint8_t, channel_t::Lower, 15>;
    using value_t = PA::value_type;
    
    using ppmA = devs::ppmA;
    using ppmB = devs::ppmB;
    using ppmC = devs::ppmC;
    using index_t = ppmA::index_type;
    using evrouter = devs::evrouter;
    
    static inline constexpr value_t chThreshH = value_t{value_t::Mid + (value_t::Upper - value_t::Lower) / 4};
    static inline constexpr value_t chThreshL = value_t{value_t::Mid + (value_t::Upper - value_t::Lower) / 6};

    using NVM = BusDevs::eeprom;
    
    inline static auto& data() {
        return NVM::data();
    }
    
    enum class State : uint8_t {Undefined, 
                                ShowBus,
                                CheckConfig, SearchChannel, ShowChannel, ShowChannel2,
                                Init, InitPpm, Run};

    static constexpr auto showTimeout = 1000_ms;
    
    static constexpr External::Tick<Timer> ppmTimeoutTicks{20_ms};
    static constexpr External::Tick<Timer> initTimeoutTicks{100_ms};
//    static constexpr External::Tick<Timer> debugTimeoutTicks{500_ms};
    static constexpr External::Tick<Timer> eepromTimeoutTicks{1000_ms};
    static constexpr External::Tick<Timer> showTimeoutTicks{showTimeout};
    
    using blinker = External::Blinker2<typename devs::led, Timer, 100_ms, showTimeout>;
    using bcount_t = blinker::count_type;
    
    static inline void init(const bool inverted) {
        NVM::init();
        if (data().magic() != 42) {
            data().magic() = 42;
            data().clear();
            data().change();
        }
        if (const auto ch = data().channel(); ch) {
            searchCh = search_t{ch.toInt()};
        }
        
        wdt::init<ccp>();        
        reset::noWatchDog([]{
            uninitialzed::reset();
        });
        reset::onWatchDog([]{
            uninitialzed::counter = uninitialzed::counter + 1;        
        });
        blinker::init();

        if constexpr(External::Bus::isIBus<bus_type>::value) {
            servo::template init<BaudRate<115200>, HalfDuplex>();
            servo::template txEnable<false>();
        }
        else if constexpr(External::Bus::isSBus<bus_type>::value) {
            servo::template init<AVR::BaudRate<100000>, HalfDuplex, true, 1>(); // 8E2
            servo::template txEnable<false>();
            if (inverted) {
                servo::txInvert(true);
            }
        }
        else {
            static_assert(std::false_v<bus_type>, "bus type not supported");
        }
        configPin::init();
    }

    static inline void periodic() {
        servo::periodic();
        NVM::saveIfNeeded([&]{
            etl::outl<terminal>("ep s"_pgm);
        });
        if (mState == State::Run) {
            auto cv = PA::value(channel_t(searchCh + index));
            
            if (index == 0) { // SO1
                ppmA::ppm(index_t{1}, cv);
            }
            else if (index == 1) { // SO2
                ppmA::ppm(index_t{0}, cv);
            }
            else if (index == 2) { // SO3
                ppmB::ppm_async(cv);
            }
            else if (index == 3) { // SO4
                ppmC::ppm_async(cv);
            }
            else if (index == 4) { // Q0
                ppmA::ppm(index_t{2}, cv);
            }
            ++index;
        }
    }

    static inline void ratePeriodic() {
        wdt::reset();
        PA::ratePeriodic();
        blinker::ratePeriodic();
            
//        (++debugTicks).on(debugTimeoutTicks, []{
////            etl::outl<terminal>("e: "_pgm, (uint8_t)esc::mState, " "_pgm, esc::mTarget.toInt(), " "_pgm, esc::mActual.toInt());
////            etl::outl<terminal>("e: "_pgm, (uint8_t)esc::mState, " "_pgm, esc::mTarget.toInt(), " "_pgm, esc::mActual.toInt(), " "_pgm, out360::adiff());
//        });
        (++mEepromTicks).on(eepromTimeoutTicks, []{
            NVM::data().expire();
        });
        
        const auto oldState = mState;
        ++stateTicks;        
        switch(mState) {
        case State::Undefined:
            mState = State::Init;
            break;
        case State::Init:
            stateTicks.on(initTimeoutTicks, []{
                mState = State::ShowBus;
            });
            break;
        case State::ShowBus:
            stateTicks.on(showTimeoutTicks, []{
                mState = State::CheckConfig;
            });
            break;
        case State::CheckConfig:
            if (configPin::isActive()) {
                mState = State::SearchChannel;
            }
            else {
                mState = State::InitPpm;
            }
            break;
        case State::SearchChannel:
            if (const auto v = PA::value(searchCh.toInt()); v) {
                if (v.toInt() > chThreshH.toInt()) {
                    mState = State::ShowChannel;
                }
                else {
                    ++searchCh;
                }
            }
            break;
        case State::ShowChannel:
            if (const auto v = PA::value(searchCh.toInt()); v) {
                if (v.toInt() < chThreshL.toInt()) {
                    mState = State::ShowChannel2;
                }
            }
            break;
        case State::ShowChannel2:
            stateTicks.on(showTimeoutTicks, []{
                mState = State::InitPpm;
            });
            break;
        case State::InitPpm:
            mState = State::Run;
            break;
        case State::Run:
            stateTicks.on(ppmTimeoutTicks, []{
                ppmB::onReload([]{
                    evrouter::template strobe<0>();
                });
                ppmC::onReload([]{
                    evrouter::template strobe<1>();
                });
            });
            break;
        }
        if (oldState != mState) {
            stateTicks.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                etl::outl<terminal>("S Ini"_pgm);
                break;
            case State::ShowBus:
                etl::outl<terminal>("S Sbu"_pgm);
                if constexpr(External::Bus::isIBus<bus_type>::value) {
                    blinker::onePeriod(bcount_t{1});
                }
                else if constexpr(External::Bus::isSBus<bus_type>::value) {
                    blinker::onePeriod(bcount_t{2});
                }
                break;
            case State::CheckConfig:
                etl::outl<terminal>("S CkC"_pgm);
                break;
            case State::SearchChannel:
                blinker::blink(bcount_t{5});
                etl::outl<terminal>("S Sea"_pgm);
                break;
            case State::ShowChannel:
                etl::outl<terminal>("S Sch "_pgm, searchCh.toInt());
                blinker::off();
                data().channel() = channel_t{searchCh.toInt()};
                data().change();
                break;
            case State::ShowChannel2:
                blinker::steady();
                etl::outl<terminal>("S Sch2 "_pgm, searchCh.toInt());
                break;
            case State::InitPpm:
                ppmA::init();
                ppmB::init();
                ppmC::init();
                break;
            case State::Run:
                blinker::off();
                etl::outl<terminal>("S Run"_pgm);
                break;
            }
        }
    }
private:    
    inline static etl::uint_ranged_circular<uint8_t, 0, 4> index;
    inline static search_t searchCh{11}; // ch12
//    inline static External::Tick<Timer> debugTicks;
    inline static External::Tick<Timer> stateTicks;
    inline static External::Tick<Timer> mEepromTicks;
    static inline State mState{State::Undefined};
};

template<typename HWRev = void, typename MCU = DefaultMcuType>
struct Devices {
    using ccp = Cpu::Ccp<>;
    using clock = Clock<>;
    using sigrow = SigRow<>;
    
    using systemTimer = SystemTimer<Component::Rtc<0>, External::Bus::fRtc>;
    
    using servoPosition = usart0Position;
    using scanDevPosition = usart0Position;

    using scanTermPosition = void;
    using ppmDevPosition = void;  
    
    using scan_term_dev = void;
    
    using scanLedPin = AVR::ActiveLow<daisyChain, Output>;
    using led = scanLedPin;
    
    using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;
    using ppmA = External::Ppm::PpmOut<tcaPosition, Meta::List<PWM::WO<0>, PWM::WO<1>, PWM::WO<2>>>;
    
    using tcb0Position = Portmux::Position<Component::Tcb<0>, Portmux::Default>;
    using ppmB = External::Ppm::PpmOut<tcb0Position>;
    
    using tcb1Position = Portmux::Position<Component::Tcb<1>, Portmux::Default>;
    using ppmC = External::Ppm::PpmOut<tcb1Position>;
    
    using portmux = Portmux::StaticMapper<Meta::List<usart0Position, tcaPosition, tcb0Position, tcb1Position>>;
    
    using evch0 = Event::Channel<0, void>;
    using evch1 = Event::Channel<1, void>;
    using evuser0 = Event::Route<evch0, Event::Users::Tcb<0>>;
    using evuser1 = Event::Route<evch1, Event::Users::Tcb<1>>;
    using evrouter = Event::Router<Event::Channels<evch0, evch1>, Event::Routes<evuser0, evuser1>>;
    
    using configPin = AVR::ActiveLow<so4Pin, Input>;

    static inline void init() {
        evrouter::init();
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
    
    using systemTimer = Devs::systemTimer;
    using led = Devs::led;
    
    using servo_pa = IBus2::Servo::ProtocollAdapter<0>;
    using servo = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<16>>;
    
    using eeprom = EEProm::Controller<Storage::ApplDataSchottel<typename servo_pa::channel_t, bus_type>>;
    
    using evrouter = Devs::evrouter;
    
    using terminal = etl::basic_ostream<void>;
};

template<typename Devs>
struct BusDevs<External::Bus::SBusSPort<Devs>> {
    using bus_type = External::Bus::SBusSPort<Devs>;
    
    struct BusParam {
        using bus_t = bus_type;
        using proto_type = RCSwitch::Protocol2<RCSwitch::Low>;
    };

    using devs = Devs;
    
    using systemTimer = Devs::systemTimer;
    using led = Devs::led;
    
    using servo_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer>;
    using servo = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<16>>;
    
    using eeprom = EEProm::Controller<Storage::ApplDataSchottel<typename servo_pa::channel_t, bus_type>>;
    
    using evrouter = Devs::evrouter;

    using terminal = etl::basic_ostream<void>;
};

template<typename BusSystem, typename MCU = DefaultMcuType>
struct Application {
    using devs = BusDevs<BusSystem>;
    
    inline static void run(const bool inverted = false) {
        if constexpr(External::Bus::isIBus<BusSystem>::value || External::Bus::isSBus<BusSystem>::value) {
            using terminal = devs::terminal;
            using systemTimer = devs::systemTimer;
            using gfsm = GFSM<devs>;
            
            gfsm::init(inverted);
            etl::outl<terminal>("autob1c5"_pgm);
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
using scanner = External::Scanner<devices, Application, AVR::HalfDuplex>;

int main() {
    scanner::run();
}

#ifndef NDEBUG
[[noreturn]] inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], const unsigned int line [[maybe_unused]]) noexcept {
    //    etl::outl<terminal>("Assertion failed: "_pgm, expr, etl::Char{','}, file, etl::Char{','}, line);
    while(true) {
        //        assertPin::toggle();
    }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, const unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
