#define USE_SBUS
//#define USE_IBUS

#include "board.h"

template<typename PA, typename SW, typename OUT, typename NVM, typename Timer, typename Term = void>
struct GFSM {
    enum class State : uint8_t {Undefined, SearchChannel, Run, ShowAddress, LearnTimeout};
    
    static constexpr auto intervall = Timer::intervall;
    static constexpr External::Tick<Timer> learnTimeoutTicks{5000_ms};
    
    static inline void init() {}
    
    static inline void ratePeriodic() {
        ++stateTicks;
        switch(mState) {
        case State::Undefined:
            mState = State::SearchChannel;
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
            //            etl::outl<Term>("timeout ch: "_pgm, NVM::data().channel().toInt(), " adr: "_pgm, NVM::data().address().toInt());
            if (NVM::data().channel() && NVM::data().address()) {
                SW::channel(NVM::data().channel());
                SW::address(NVM::data().address());
            }
            mState = State::Run;
            break;
        case State::ShowAddress:
            //            etl::outl<Term>("learned ch: "_pgm, NVM::data().channel().toInt(), " adr: "_pgm, NVM::data().address().toInt());
            mState = State::Run;
            break;
        case State::Run:
            SW::periodic();
            //            OUT::setSwitches();
            break;
        }
    }
private:
    using protocol_t = typename SW::protocol_t;
    using addr_t = typename protocol_t::addr_t;
    
    static inline bool search() {
        //        etl::outl<Term>("test: "_pgm, learnChannel.toInt());
        if (const auto lc = PA::valueMapped(learnChannel.toRangedNaN()); lc && SW::isLearnCode(lc)) {
            if (const auto pv = protocol_t::toParameterValue(lc).toInt(); (pv >= 1) && ((pv - 1) <= SW::protocol_t::addr_t::Upper)) {
                const uint8_t addr = pv - 1;
                SW::channel(learnChannel.toRangedNaN());
                SW::address(addr_t(addr));
                NVM::data().channel() = learnChannel;
                NVM::data().address().set(addr);
                NVM::data().change();
                return true;
            }
        }   
        ++learnChannel;
        return false;
    }
    using ch_t = PA::channel_t;
    
    static inline etl::uint_ranged_circular<uint8_t, ch_t::Lower, ch_t::Upper> learnChannel{0};
    static inline State mState{State::Undefined};
    inline static External::Tick<Timer> stateTicks;
};

using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;
using ppmA = External::Ppm::PpmOut<tcaPosition, Meta::List<PWM::WO<0>, PWM::WO<1>, PWM::WO<2>>>;

using tcb0Position = Portmux::Position<Component::Tcb<0>, Portmux::Default>;
using ppmB = External::Ppm::PpmOut<tcb0Position>;

using tcb1Position = Portmux::Position<Component::Tcb<1>, Portmux::Default>;
using ppmC = External::Ppm::PpmOut<tcb1Position>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, tcaPosition, tcb0Position, tcb1Position>>;

#ifdef USE_SBUS
using servo_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer>;
#endif
#ifdef USE_IBUS
using servo_pa = IBus::Servo::ProtocollAdapter<0>;
#endif
using servo = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>>;

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
    
    static inline constexpr uint8_t offset{10};
    
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
    //private:
    inline static void update() {
        const auto mode = NVM::data().mpxMode(Address);
        
        if (mode == Storage::Mode::Graupner8K) {
            if (cycle10 < 2) {
                PPM::ppmRaw(PPM::ocMax + 100);
//                PPM::ppmRaw(PPM::ocMax + 200);
            }
            else {
                uint8_t i = cycle10 - 2;
                pulse(i);
            }
            ++cycle10;
        }
        else if (mode == Storage::Mode::Graupner4K) {
            if (cycle6 < 2) {
                PPM::ppmRaw(PPM::ocMax + 100);
            }
            else {
                uint8_t i = cycle6 - 2;
                pulse(i);
            }
            ++cycle6;
        }
        else if (mode == Storage::Mode::Robbe) {
            if (cycle9 < 1) {
                PPM::ppmRaw(PPM::ocMin - 100);
            }
            else {
                uint8_t i = Size - 1 - (cycle9 - 1); // Robbe counts channels in reverse order
                pulse(i);
            }
            ++cycle9;
        }
        else if (mode == Storage::Mode::CP) {
            if (cycle9 < 1) {
                PPM::ppmRaw(PPM::ocMax + 100);
            }
            else {
                uint8_t i = cycle9 - 1;
                pulse(i);
            }
            ++cycle9;
        }
        else if (mode == Storage::Mode::XXX) {
            if (cycle10 < 2) {
                PPM::ppmRaw(PPM::ocMin - 100);
            }            
            else {
                uint8_t i = cycle10 - 2;
                pulse(i);
            }
            ++cycle10;
        }
        else {
            if (cycle10 < 2) {
                PPM::ppmRaw(PPM::ocMax + 100);
            }
            else {
                uint8_t i = cycle10 - 2;
                pulse(i);
            }
            ++cycle10;
        }
    }
    
    static inline void pulse(uint8_t i) {
        if (const auto pch = NVM::data()[i].passThru()) {
            auto v = PA::value(pch);
            PPM::ppm_async(v);
        }
        else if (swStates[i] == SwState::Off) {
            PPM::ppmRaw(PPM::ocMedium);
        }
        else if (swStates[i] == SwState::Steady) {
            PPM::ppmRaw(PPM::ocMax - 200);
        }
        else if (swStates[i] == SwState::Blink1) {
            PPM::ppmRaw(PPM::ocMin + 200);
        }
        else {
            PPM::ppmRaw(PPM::ocMedium);
        }
        
    }
    static inline cycle10_t cycle10;
    static inline cycle9_t cycle9;
    static inline cycle6_t cycle6;
    static inline std::array<SwState, Size> swStates;
};

using eeprom = EEProm::Controller<Storage::ApplData<servo_pa::channel_t, IBus::Switch::Protocol1::addr_t>>;

using ppmCh1 = Adapter<ppmA, 0>;
using ppmCh2 = Adapter<ppmA, 1>;
using ppmCh3 = Adapter<ppmA, 2>;
using fsm1 = FSM<ppmCh1, servo_pa, eeprom, 0>;
using fsm2 = FSM<ppmCh2, servo_pa, eeprom, 1>;
using fsm3 = FSM<ppmCh3, servo_pa, eeprom, 2>;
using fsm4 = FSM<ppmB, servo_pa, eeprom, 3>;
using fsm5 = FSM<ppmC, servo_pa, eeprom, 4>;

using ibus_switch = IBus::Switch::MultiAdapter<servo_pa, Meta::List<fsm1, fsm2, fsm3, fsm4, fsm5>, eeprom>;

using evch0 = Event::Channel<0, void>;
using evch1 = Event::Channel<1, void>;
using evuser0 = Event::Route<evch0, Event::Users::Tcb<0>>;
using evuser1 = Event::Route<evch1, Event::Users::Tcb<1>>;
using evrouter = Event::Router<Event::Channels<evch0, evch1>, Event::Routes<evuser0, evuser1>>;

using gfsm = GFSM<servo_pa, ibus_switch, void, eeprom, systemTimer>;

auto& appData = eeprom::data();

int main() {
    wdt::init<ccp>();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
    
    reset::noWatchDog([]{
        uninitialzed::reset();
    });
    
    reset::onWatchDog([]{
        uninitialzed::counter = uninitialzed::counter + 1;        
    });
    
    evrouter::init();
    portmux::init();
    systemTimer::init();
    
#ifdef USE_IBUS
    servo::init<AVR::BaudRate<115200>, HalfDuplex>();
    servo::txEnable<false>();
#endif
#ifdef USE_SBUS
    servo::init<AVR::BaudRate<100000>, HalfDuplex, true, 1>(); // 8E2
    servo::txEnable<false>();
#endif
    fsm1::init();
    fsm2::init();
    fsm3::init();
    fsm4::init();
    fsm5::init();
    ibus_switch::init();
    
    eeprom::init();
    {
        if (!((appData.magic() == 42))) {
            appData.magic() = 42;
            appData.clear();
            appData.change();
        }
    }
    
    const auto periodicTimer = alarmTimer::create(20_ms, External::Hal::AlarmFlags::Periodic);
    const auto eepromTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);
    
    while(true) {
        eeprom::saveIfNeeded([&]{
            //            fsm::load();                
        });
        servo::periodic();
        
        fsm1::periodic();
        fsm2::periodic();
        fsm3::periodic();
        
        systemTimer::periodic([&]{
            wdt::reset();
            gfsm::ratePeriodic();
            
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    ppmB::onReload([]{
                        fsm4::update();
                        evrouter::strobe<0>();
                    });
                    ppmC::onReload([]{
                        fsm5::update();
                        evrouter::strobe<1>();
                    });
                }
                else if (eepromTimer == t) {
                    appData.expire();
                }
            });
        });
    }
}

#ifndef NDEBUG
[[noreturn]] inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], unsigned int line [[maybe_unused]]) noexcept {
#if !(defined(USE_IBUS) || defined(USE_HOTT))
    etl::outl<terminal>("Assertion failed: "_pgm, expr, etl::Char{','}, file, etl::Char{','}, line);
#endif
    while(true) {
        dbg1::toggle();
    }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
