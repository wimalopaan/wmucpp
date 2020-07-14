#define NDEBUG

#define USE_IBUS

#include "board.h"
#include "swout.h"

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

int main() {
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
    
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
    
    ibus_switch::address(ibus_switch::addr_t{1});
    
    const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);
    const auto learnTimer = alarmTimer::create(5000_ms, External::Hal::AlarmFlags::Periodic);
    const auto eepromTimer = alarmTimer::create(3000_ms, External::Hal::AlarmFlags::Periodic);
    
    uint16_t counter{};
    
    using ch_t = servo_pa::channel_t;
    
    bool learn{true};
    etl::uint_ranged_circular<uint8_t, ch_t::Lower, ch_t::Upper> learnChannel{0};
    
    while(true) {
        eeprom::saveIfNeeded([&]{
            etl::outl<terminal>("save eep"_pgm);
        });
        servo::periodic();
        systemTimer::periodic([&]{
            if (learn) {
                etl::outl<terminal>("test: "_pgm, learnChannel.toInt());
                if (const auto lc = servo_pa::value(learnChannel.toRangedNaN()); lc && ibus_switch::isLearnCode(lc)) {
                    const auto addr = ibus_switch::protocol_t::toParameter(lc).toInt() - 1;
                    if ((addr <= ibus_switch::protocol_t::addr_t::Upper)) {
                        ibus_switch::channel(learnChannel.toRangedNaN());
                        ibus_switch::address(ibus_switch::protocol_t::addr_t(addr));
                        appData.channel() = learnChannel;
                        appData.address() = addr;
                        appData.change();
                        learn = false;
                        etl::outl<terminal>("learned ch: "_pgm, learnChannel.toInt(), " adr: "_pgm, addr);
                    }
                }   
                ++learnChannel;
            }
            else {
                ibus_switch::periodic();
                out::setSwitches();
            }
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    etl::outl<terminal>("c: "_pgm, counter++);
                }
                else if (learnTimer == t) {
                    learn = false;
                }
                else if (eepromTimer == t) {
                    appData.expire();
                }
            });
        });
    }
}

