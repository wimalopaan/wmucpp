#define NDEBUG

//#define USE_IBUS
#define USE_SBUS

#include "board.h"
#include "swout.h"

template<typename PA, typename SW, typename OUT, typename NVM, typename Timer, typename Term = void>
struct FSM {
    enum class State : uint8_t {Undefined, SearchChannel, Run, ShowAddress, LearnTimeout};
    
    static constexpr auto intervall = Timer::intervall;
    static constexpr External::Tick<Timer> learnTimeoutTicks{5000_ms};
    
    static inline void init() {}
    
    static inline void ratePeriodic() {
#ifdef USE_SBUS
        PA::ratePeriodic();
#endif
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
            etl::outl<Term>("timeout ch: "_pgm, NVM::data().channel().toInt(), " adr: "_pgm, NVM::data().address().toInt());
            if (NVM::data().channel() && NVM::data().address()) {
                SW::channel(NVM::data().channel());
                SW::address(NVM::data().address());
            }
            etl::outl<Term>("using ch: "_pgm, SW::mChannel.toInt(), " adr: "_pgm, SW::mAddr.toInt());
            mState = State::Run;
            break;
        case State::ShowAddress:
            etl::outl<Term>("learned ch: "_pgm, NVM::data().channel().toInt(), " adr: "_pgm, NVM::data().address().toInt());
            mState = State::Run;
            break;
        case State::Run:
            SW::periodic();
            OUT::setSwitches();
            break;
        }
    }
    
private:
    using protocol_t = typename SW::protocol_t;
    using addr_t = typename protocol_t::addr_t;
    
    static inline bool search() {
//        if (const auto lc = PA::value(learnChannel.toRangedNaN())) {
//            etl::outl<Term>("c: "_pgm, learnChannel.toInt(), " v: "_pgm, lc.toInt());
//        }
//        else {
//            etl::outl<Term>("c: "_pgm, learnChannel.toInt());
//        }
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

#ifdef USE_IBUS
using servo_pa = IBus::Servo::ProtocollAdapter<0>;
#endif
#ifdef USE_SBUS
using servo_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer>;
#endif

using eeprom = EEProm::Controller<Storage::ApplData<servo_pa::channel_t, IBus::Switch::Protocol1::addr_t>>;

using servo = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
using terminalDevice = servo;
using terminal = etl::basic_ostream<servo>;

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
private:
    static inline std::array<SwState, N> swStates{};
};

using sw = SwitchStates<>;

using out = External::Output<ledList, pwm, sw, eeprom>;

using ibus_switch = IBus::Switch::Digital<servo_pa, sw, out>;

auto& appData = eeprom::data();

using fsm = FSM<servo_pa, ibus_switch, out, eeprom, systemTimer, terminal>;

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
        appData.magic() = 42;
        appData.clear();
        appData.change();
        etl::outl<terminal>("eep init"_pgm);
    }
    else {
        etl::outl<terminal>("eep ch: "_pgm, appData.channel().toInt(), " adr: "_pgm, appData.address().toInt());        
    }
    
    const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);
    const auto eepromTimer = alarmTimer::create(3000_ms, External::Hal::AlarmFlags::Periodic);
    
    uint16_t counter{};
    
    while(true) {
        eeprom::saveIfNeeded([&]{
            etl::outl<terminal>("save eep"_pgm);
        });
        servo::periodic();
        systemTimer::periodic([&]{
            fsm::ratePeriodic();
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
//                    etl::outl<terminal>("c: "_pgm, counter++, " mpx0: "_pgm, (uint8_t)appData.mMpxModes[0]);
//                    etl::outl<terminal>("c9: "_pgm, servo_pa::valueMapped(9).toInt(), " r: "_pgm, servo_pa::value(9).toInt());
//                    etl::outl<terminal>("c9: "_pgm, servo_pa::valueMapped(9).toInt(), " r: "_pgm, servo_pa::value(9).toInt());
//                    etl::outl<terminal>("c0: "_pgm, servo_pa::mChannels[0]);
//                    etl::outl<terminal>("lo: "_pgm, ibus_switch::lastOnIndex.toInt());
                    etl::outl<terminal>("sw0: "_pgm, (uint8_t)sw::switches()[0], "bm: "_pgm, out::mode().toInt(), " bi0_1: "_pgm, appData[0].blinks()[0].intervall.value.toInt(), " bi0_2: "_pgm, appData[0].blinks()[1].intervall.value.toInt());
                }
                else if (eepromTimer == t) {
                    appData.expire();
                }
            });
        });
    }
}

