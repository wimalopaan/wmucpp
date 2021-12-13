#define NDEBUG

//#define DEBUG2 // RX/TX change -> full duplex (man muss dann Ibus-Input und Telemetrie tauschen) 

#define INV_LED // onboad LED inverted
 
#define AUTO_BUS // SCAN für SBus / IBus
#define SBUS_IBUS_NO_WARN

#define LEARN_DOWN // start at highest channel number downwards

#define START_AT_CHANNEL_16

#include "board.h"

template<typename Devs, typename FsmList>
struct GFSM;

template<typename Devs, typename... Fsms>
struct GFSM<Devs, Meta::List<Fsms...>> {
    using bus_type = Devs::bus_type;
    using devs = bus_type::devs;
    
    using Servo = Devs::servo;
    using servo_pa = Devs::servo_pa;
    
    using ch_t = servo_pa::channel_t;
    
    enum class State : uint8_t {Undefined, StartShowBus, ShowBus, StartWait, SearchChannel, AfterSearch, InitRun, Run, 
                                ShowAddress, ShowAddressWait, LearnTimeout,
                                Test1};
    
    using Timer = Devs::systemTimer;
    using SW = Devs::bus_switch;
    using Term = Devs::terminal;
    using NVM = Devs::eeprom;
    using RELD = Devs::reloader;
//    using evrouter = Devs::evrouter;
    
    static constexpr auto intervall = Timer::intervall;
    
    inline static constexpr auto learnTimeout = 4000_ms;
    inline static constexpr auto scanTimeout = 50_ms;
    
    static_assert(learnTimeout > (scanTimeout * (ch_t::Upper + 1) * 2), "");
    
    static constexpr External::Tick<Timer> learnTimeoutTicks{learnTimeout};
    static constexpr External::Tick<Timer> scanTimeoutTicks{scanTimeout};
    static constexpr External::Tick<Timer> waitTimeoutTicks{3000_ms};
    static constexpr External::Tick<Timer> reloadTimeoutTicks{20_ms};
    static constexpr External::Tick<Timer> signalTimeoutTicks{500_ms};
    static constexpr External::Tick<Timer> eepromTimeout{1000_ms};
    
    using blinker = External::SimpleBlinker<typename Devs::led, Timer, 300_ms>;
    
    inline static auto& data() {
        return NVM::data();
    }
    
    static inline void init(const bool inverted) {
        SW::init();
        NVM::init();
        if (data().magic() != 42) {
            data().magic() = 42;
            data().clear();
            data().change();
            etl::outl<Term>("e init"_pgm);
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
            Servo::template init<BaudRate<115200>, HalfDuplex>();
            Servo::template txEnable<false>();
        }
        else if constexpr(External::Bus::isSBus<bus_type>::value) {
            Servo::template init<AVR::BaudRate<100000>, HalfDuplex, true, 1>(); // 8E2
            Servo::template txEnable<false>();
            if (inverted) {
                Servo::txInvert(true);
            }
        }
        else {
            static_assert(std::false_v<bus_type>, "bus type not supported");
        }
    }
    
    static inline void periodic() {
        Servo::periodic();
        NVM::saveIfNeeded([&]{
            etl::outl<Term>("ep s"_pgm);
        });
        switch(mState) {
        case State::Run:
            (Fsms::periodic(), ...);
            break;
        case State::Undefined:
        case State::StartShowBus:
        case State::ShowBus:
        case State::StartWait:
        case State::SearchChannel:
        case State::AfterSearch:
        case State::InitRun:
        case State::ShowAddress:
        case State::ShowAddressWait:
        case State::LearnTimeout:
        case State::Test1:
        default:
            break;
        }
    }
    
    static inline void test(const auto) {}
    
    static inline void ratePeriodic() {
        servo_pa::ratePeriodic();
        wdt::reset();
        const auto oldState = mState;
        blinker::ratePeriodic();
        ++stateTicks;
        ++mEepromTicks;

        switch(mState) {
        case State::Undefined:
            mState = State::StartShowBus;
            break;
        case State::StartShowBus:
            stateTicks.on(signalTimeoutTicks, []{
                mState = State::ShowBus;
            });
            break;
        case State::ShowBus:
            stateTicks.on(waitTimeoutTicks, []{
                mState = State::StartWait;
            });
            break;
        case State::StartWait:
            stateTicks.on(waitTimeoutTicks, []{
                mState = State::SearchChannel;
            });
            break;
        case State::SearchChannel:
            if (search()) {
                mState = State::AfterSearch;
            }
            stateTicks.on(learnTimeoutTicks, []{
                mState = State::LearnTimeout;
            });
            break;
        case State::AfterSearch:
            stateTicks.on(signalTimeoutTicks, []{
                mState = State::ShowAddress;
            });
            break;
        case State::LearnTimeout:
            etl::outl<Term>("timeout ch: "_pgm, NVM::data().channel(), " adr: "_pgm, NVM::data().address().toInt());
            if (NVM::data().channel() && NVM::data().address()) {
                SW::channel(NVM::data().channel());
                SW::address(typename RCSwitch::addr_t{NVM::data().address().toInt()});
            }
            mState = State::InitRun;
            break;
        case State::ShowAddress:
            etl::outl<Term>("learned ch: "_pgm, NVM::data().channel(), " adr: "_pgm, NVM::data().address().toInt());
            blinker::blink(NVM::data().address().toInt() + 1);
            mState = State::ShowAddressWait;
            break;
        case State::ShowAddressWait:
            if (!blinker::isActive()) {
                mState = State::InitRun;
            }
            break;
        case State::Run:
            SW::ratePeriodic();
            stateTicks.on(reloadTimeoutTicks, []{
                RELD::reload();
            });
            if (SW::receivedControl()) {
                blinker::steady();                
            }
            else {
                blinker::off();                
            }
            mEepromTicks.on(eepromTimeout, []{
                NVM::data().expire();
            });
            break;
        case State::InitRun:
            (Fsms::init(), ...);
            RELD::init();
            mState = State::Run;
            break;
        case State::Test1:
            if (!SW::receivedControl()) {
                mState = State::Run;
            }
            break;
        }
        if (oldState != mState) {
            stateTicks.reset();
            switch(mState) {
            case State::Run:
                break;
            case State::Undefined:
                break;
            case State::StartShowBus:
                blinker::off();
                break;
            case State::ShowBus:
                if constexpr(External::Bus::isIBus<bus_type>::value) {
                    blinker::blink(1);
                }
                else if constexpr(External::Bus::isSBus<bus_type>::value) {
                    blinker::blink(2);
                }
                break;
            case State::StartWait:
                blinker::steady();
                etl::outl<Term>("swait"_pgm);
                break;
            case State::SearchChannel:
                blinker::off();
                etl::outl<Term>("search"_pgm);
                break;
            case State::AfterSearch:
            case State::InitRun:
            case State::ShowAddress:
            case State::ShowAddressWait:
            case State::LearnTimeout:
            case State::Test1:
            default:
                break;
            }
        }
    }
private:
    using protocol_t = Devs::BusParam::proto_type;
    using addr_t = RCSwitch::addr_t;
    
    static inline bool search() {
        if (const auto lc = servo_pa::valueMapped(learnChannel.toRangedNaN()); lc && SW::isLearnCode(lc)) {
            if (const auto pv = protocol_t::toParameterValue(lc).toInt(); (pv >= 1) && ((pv - 1) <= addr_t::Upper)) {
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
#ifdef LEARN_DOWN
# ifdef START_AT_CHANNEL_16
    static inline etl::uint_ranged_circular<uint8_t, ch_t::Lower, ch_t::Upper> learnChannel{16};
# else
    static inline etl::uint_ranged_circular<uint8_t, ch_t::Lower, ch_t::Upper> learnChannel{ch_t::Upper};
# endif
#else
    static inline etl::uint_ranged_circular<uint8_t, ch_t::Lower, ch_t::Upper> learnChannel{0};
#endif
    static inline State mState{State::Undefined};
    inline static External::Tick<Timer> stateTicks;
    inline static External::Tick<Timer> mEepromTicks;
};

template<typename PPM, uint8_t Channel = 0>
struct Adapter {
    using ppm_index_t = PPM::index_type;
    
    inline static constexpr auto ocMax =  PPM::ocMax;
    inline static constexpr auto ocMin =  PPM::ocMin;
    inline static constexpr auto ocMedium =  PPM::ocMedium;
    
    inline static void init() {
        PPM::init();
    }
    using ranged_type = PPM::ranged_type;    
    inline static void onReload(auto f) {
        PPM::template onCompareMatch<PWM::WO<Channel>>([&]{
            f();
        });
    }
    
    inline static void ppmRaw(const auto v) {
        PPM::ppmRaw(ppm_index_t{Channel}, v);
    }
    
    inline static void ppm_async(const auto v) {
        PPM::ppm(ppm_index_t{Channel}, v);
    }
};

template<typename PPM, typename PA, typename NVM, uint8_t Address = 0, uint8_t Size = 8>
struct FSM {
    inline static constexpr uint8_t address = Address;
    
    enum class SwState : uint8_t {Off, Steady, Blink1, Blink2};
    
    using ppm_value_t = PPM::ranged_type;
    
    using cycle10_t = etl::uint_ranged_circular<uint8_t, 0, 9>; // Robbe / CP mode use only 9 cycles, so we have to increment twice
    using cycle9_t = etl::uint_ranged_circular<uint8_t, 0, 8>; // Robbe / CP mode use only 9 cycles, so we have to increment twice
    using cycle6_t = etl::uint_ranged_circular<uint8_t, 0, 5>; // Robbe / CP mode use only 9 cycles, so we have to increment twice
    using cycle17_t = etl::uint_ranged_circular<uint8_t, 0, 16>; // CP16 mode use 17 cycles
    
    static inline void init() {
        PPM::init();    
    }
    static inline void periodic() {
        PPM::onReload([]{
            update();
        });
    }
    static inline auto& switches() {
        return swStates;
    }
    inline static void update() {
        const uint16_t impulsOffset = NVM::data().mpxOffset(Address);
        const auto mode = NVM::data().mpxMode(Address);
        
        if (mode == Storage::Mode::Graupner8K) {
            if (cycle10 < 2) {
                PPM::ppmRaw(PPM::ocMax + impulsOffset);
            }
            else {
                uint8_t i = cycle10 - 2;
                pulse8(i);
            }
            ++cycle10;
        }
        else if (mode == Storage::Mode::Graupner4K) {
            if (cycle6 < 2) {
                PPM::ppmRaw(PPM::ocMax + impulsOffset);
            }
            else {
                uint8_t i = cycle6 - 2;
                pulse8(i);
            }
            ++cycle6;
        }
        else if (mode == Storage::Mode::Robbe) {
            if (cycle9 < 1) {
                PPM::ppmRaw(PPM::ocMin - impulsOffset);
            }
            else {
                uint8_t i = Size - 1 - (cycle9 - 1); // Robbe counts channels in reverse order
                pulse8(i);
            }
            ++cycle9;
        }
        else if (mode == Storage::Mode::CP8) {
            if (cycle9 < 1) {
                PPM::ppmRaw(PPM::ocMax + impulsOffset);
            }
            else {
                uint8_t i = cycle9 - 1;
                pulse8ShortOnly(i);
            }
            ++cycle9;
        }
        else if (mode == Storage::Mode::CP16) {
            if (cycle17 < 1) {
                PPM::ppmRaw(PPM::ocMax + impulsOffset);
            }
            else {
                uint8_t i = cycle17 - 1;
                pulse16ShortOnly(i);
            }
            ++cycle17;
        }
        else if (mode == Storage::Mode::XXX) {
            if (cycle10 < 2) {
                PPM::ppmRaw(PPM::ocMin - impulsOffset);
            }            
            else {
                uint8_t i = cycle10 - 2;
                pulse8(i);
            }
            ++cycle10;
        }
        else {
            if (cycle10 < 2) {
                PPM::ppmRaw(PPM::ocMax + impulsOffset);
            }
            else {
                uint8_t i = cycle10 - 2;
                pulse8(i);
            }
            ++cycle10;
        }
    }
private:
    static inline void pulse8ShortOnly(uint8_t i) {
        const uint16_t pulseOffset = NVM::data().pulseOffset(Address);
        
        if (swStates[i] == SwState::Off) {
            PPM::ppmRaw(PPM::ocMedium);
        }
        else if (swStates[i] == SwState::Steady) {
            PPM::ppmRaw(PPM::ocMin + pulseOffset);
        }
        else if (swStates[i] == SwState::Blink1) {
            PPM::ppmRaw(PPM::ocMin + pulseOffset);
        }
        else {
            PPM::ppmRaw(PPM::ocMedium);
        }
    }
    static inline void pulse16ShortOnly(uint8_t i) {
        const uint16_t pulseOffset = NVM::data().pulseOffset(Address);
        
        if (i < 8) {
            if (swStates[i] == SwState::Off) {
                PPM::ppmRaw(PPM::ocMedium);
            }
            else if (swStates[i] == SwState::Steady) {
                PPM::ppmRaw(PPM::ocMin + pulseOffset);
            }
            else {
                PPM::ppmRaw(PPM::ocMedium);
            }
        }
        else {
            const uint8_t k = i - 8;
            if (swStates[k] == SwState::Off) {
                PPM::ppmRaw(PPM::ocMedium);
            }
            else if (swStates[k] == SwState::Blink1) {
                PPM::ppmRaw(PPM::ocMin + pulseOffset);
            }
            else {
                PPM::ppmRaw(PPM::ocMedium);
            }            
        }
    }
    static inline void pulse8(uint8_t i) {
        constexpr Storage::ChannelIndex::addr_type adr{Address};
        const Storage::ChannelIndex chi{adr, Storage::ChannelIndex::channel_type{i}};
        
        const uint16_t pulseOffset = NVM::data().pulseOffset(Address);
        
        if (const auto pch = NVM::data().passThru(chi)) {
            auto v = PA::value(pch);
            PPM::ppm_async(v);
        }
        else if (swStates[i] == SwState::Off) {
            PPM::ppmRaw(PPM::ocMedium);
        }
        else if (swStates[i] == SwState::Steady) {
            PPM::ppmRaw(PPM::ocMax - pulseOffset);
        }
        else if (swStates[i] == SwState::Blink1) {
            PPM::ppmRaw(PPM::ocMin + pulseOffset);
        }
        else {
            PPM::ppmRaw(PPM::ocMedium);
        }
    }
    static inline cycle17_t cycle17;
    static inline cycle10_t cycle10;
    static inline cycle9_t cycle9;
    static inline cycle6_t cycle6;
    static inline std::array<SwState, Size> swStates;
};

template<typename HWRev = void, typename MCU = DefaultMcuType>
struct Devices {
    using ccp = Cpu::Ccp<>;
    using clock = Clock<>;
    using sigrow = SigRow<>;
    
    using systemTimer = SystemTimer<Component::Rtc<0>, External::Bus::fRtc>;
    
    using servoPosition = usart0Position;

    using scanTermPosition = void;
    
    using scanDevPosition = servoPosition;
    
#ifdef NDEBUG
    using scan_term_dev = void;
#else
    using scan_term_dev = Usart<scanTermPosition, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<1>, AVR::SendQueueLength<256>>;
#endif
    
    using ppmDevPosition = void;  
    //    using evrouter = void;
    
    using scanLedPin = AVR::ActiveLow<daisyChain, Output>;
    
    using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;
    using ppmA = External::Ppm::PpmOut<tcaPosition, Meta::List<PWM::WO<0>, PWM::WO<1>, PWM::WO<2>>>;
    
    using tcb0Position = Portmux::Position<Component::Tcb<0>, Portmux::Default>;
    using ppmB = External::Ppm::PpmOut<tcb0Position>;
    
    using tcb1Position = Portmux::Position<Component::Tcb<1>, Portmux::Default>;
    using ppmC = External::Ppm::PpmOut<tcb1Position>;
    
    using portmux = Portmux::StaticMapper<Meta::List<usart0Position, tcaPosition, tcb0Position, tcb1Position>>;
    
    using ppmCh1 = Adapter<ppmA, 0>;
    using ppmCh2 = Adapter<ppmA, 1>;
    using ppmCh3 = Adapter<ppmA, 2>;
    
    using evch0 = Event::Channel<0, void>;
    using evch1 = Event::Channel<1, void>;
    using evuser0 = Event::Route<evch0, Event::Users::Tcb<0>>;
    using evuser1 = Event::Route<evch1, Event::Users::Tcb<1>>;
    using evrouter = Event::Router<Event::Channels<evch0, evch1>, Event::Routes<evuser0, evuser1>>;
    
#ifndef DEBUG2
# ifdef INV_LED
    using led = AVR::ActiveLow<daisyChain, Output>;
# else
    using led = AVR::ActiveHigh<daisyChain, Output>;
# endif
#else
    using led = AVR::ActiveLow<AVR::NoPin, Output>;
#endif
    
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
    
    using systemTimer = Devs::systemTimer;
    using led = Devs::led;
//    using eeprom = Devs::eeprom;    
    
    using servo_pa = IBus2::Servo::ProtocollAdapter<0>;
    using servo = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<16>>;
    
    using eeprom = EEProm::Controller<Storage::ApplDataBus<typename servo_pa::channel_t, RCSwitch::addr_t, bus_type>>;
    
    using evrouter = Devs::evrouter;
    
#ifndef NDEBUG
    using terminal = etl::basic_ostream<servo>;
#else
    using terminal = etl::basic_ostream<void>;
#endif
    
    using fsm1 = FSM<typename Devs::ppmCh1, servo_pa, eeprom, 0>;
    using fsm2 = FSM<typename Devs::ppmCh2, servo_pa, eeprom, 1>;
    using fsm3 = FSM<typename Devs::ppmCh3, servo_pa, eeprom, 2>;
    using fsm4 = FSM<typename Devs::ppmB, servo_pa, eeprom, 3>;
    using fsm5 = FSM<typename Devs::ppmC, servo_pa, eeprom, 4>;
    
    using fsms = Meta::List<fsm1, fsm2, fsm3>;
    using fsms2 = Meta::List<fsm1, fsm2, fsm3, fsm4, fsm5>;
    
    using bus_switch = RCSwitch::MultiAdapter<BusParam, servo_pa, fsms2, eeprom>;

    struct Reloader {
        static inline void init() {
            fsm4::init();
            fsm5::init();
        }
        static inline void reload() {
            Devs::ppmB::onReload([]{
                fsm4::update();
                evrouter::template strobe<0>();
            });
            Devs::ppmC::onReload([]{
                fsm5::update();
                evrouter::template strobe<1>();
            });
        }
    };
    using reloader = Reloader;
    
};

template<typename Devs>
struct BusDevs<External::Bus::SBusSPort<Devs>> {
    using bus_type = External::Bus::SBusSPort<Devs>;
    
    struct BusParam {
        using bus_t = bus_type;
        using proto_type = RCSwitch::Protocol2<RCSwitch::Low>;
    };
    
    using systemTimer = Devs::systemTimer;
    using led = Devs::led;
//    using eeprom = Devs::eeprom;    
    
    using servo_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer>;
    using servo = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<16>>;
    
    using eeprom = EEProm::Controller<Storage::ApplDataBus<typename servo_pa::channel_t, RCSwitch::addr_t, bus_type>>;
    
    using evrouter = Devs::evrouter;

#ifndef NDEBUG
    using terminal = etl::basic_ostream<servo>;
#else
    using terminal = etl::basic_ostream<void>;
#endif
    
    using fsm1 = FSM<typename Devs::ppmCh1, servo_pa, eeprom, 0>;
    using fsm2 = FSM<typename Devs::ppmCh2, servo_pa, eeprom, 1>;
    using fsm3 = FSM<typename Devs::ppmCh3, servo_pa, eeprom, 2>;
    using fsm4 = FSM<typename Devs::ppmB, servo_pa, eeprom, 3>;
    using fsm5 = FSM<typename Devs::ppmC, servo_pa, eeprom, 4>;
    
    using fsms = Meta::List<fsm1, fsm2, fsm3>;
    using fsms2 = Meta::List<fsm1, fsm2, fsm3, fsm4, fsm5>;
    
    using bus_switch = RCSwitch::MultiAdapter<BusParam, servo_pa, fsms2, eeprom>;

    struct Reloader {
        static inline void init() {
            fsm4::init();
            fsm5::init();
        }
        static inline void reload() {
            Devs::ppmB::onReload([]{
                fsm4::update();
                evrouter::template strobe<0>();
            });
            Devs::ppmC::onReload([]{
                fsm5::update();
                evrouter::template strobe<1>();
            });
        }
    };
    using reloader = Reloader;
};

template<typename BusSystem, typename MCU = DefaultMcuType>
struct Application {
    using devs = BusDevs<BusSystem>;
    
    inline static void run(const bool inverted = false) {
        if constexpr(External::Bus::isIBus<BusSystem>::value || External::Bus::isSBus<BusSystem>::value) {
            using terminal = devs::terminal;
            using systemTimer = devs::systemTimer;
            using gfsm = GFSM<devs, typename devs::fsms>;
            
            gfsm::init(inverted);
            etl::outl<terminal>("auto2multi"_pgm);
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
