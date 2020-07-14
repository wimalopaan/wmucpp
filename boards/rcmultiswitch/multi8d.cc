#define NDEBUG

#define USE_IBUS

#include "board.h"
#include "swout.h"

template<typename PA, typename SW, typename OUT, typename NVM, typename Timer, typename Term = void>
struct FSM {
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
            etl::outl<Term>("timeout ch: "_pgm, NVM::data().channel().toInt(), " adr: "_pgm, NVM::data().address().toInt());
            if (NVM::data().channel() && NVM::data().address()) {
                SW::channel(NVM::data().channel());
                SW::address(NVM::data().address());
            }
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
        etl::outl<Term>("test: "_pgm, learnChannel.toInt());
        if (const auto lc = PA::value(learnChannel.toRangedNaN()); lc && SW::isLearnCode(lc)) {
            const auto addr = protocol_t::toParameter(lc).toInt() - 1;
            if ((addr <= SW::protocol_t::addr_t::Upper)) {
                SW::channel(learnChannel.toRangedNaN());
                SW::address(addr_t(addr));
                NVM::data().channel() = learnChannel;
                NVM::data().address() = addr;
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

using servo_pa = IBus::Servo::ProtocollAdapter<0>;

using eeprom = EEProm::Controller<Storage::ApplData<servo_pa::channel_t, IBus::Switch::Protocol1::addr_t>>;

using servo = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>>;
using terminalDevice = servo;
using terminal = etl::basic_ostream<servo>;

template<auto N = 8>
struct SwitchStates {
    enum class SwState : uint8_t {Off, On, Blink1 = On, Steady, Blink2, PassThru};
    
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
    
    servo::init<BaudRate<115200>>();
    systemTimer::init();
    out::init();
    
    etl::outl<terminal>("multi8d"_pgm);
    
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
                    etl::outl<terminal>("c: "_pgm, counter++);
                }
                else if (eepromTimer == t) {
                    appData.expire();
                }
            });
        });
    }
}

