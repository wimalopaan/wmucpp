#define NDEBUG

#define USE_SBUS

#include "board.h"
#include "swout.h"

using servo_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer>;
using servo = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

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

using ibus_switch = IBus::Switch::Switch3<servo_pa, sw, out>;

auto& appData = eeprom::data();

int main() {
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
   
    servo::init<AVR::BaudRate<100000>, FullDuplex, true, 1>(); // 8E2
    
    systemTimer::init();

    eeprom::init();
    [[maybe_unused]] bool changed = false;
    
    {
        if (!((appData[Storage::AVKey::Magic].pwmValue() == 43))) {
            appData[Storage::AVKey::Magic].pwmValue(43);
            appData[Storage::AVKey::Ch0] = Storage::ApplData::value_type{};
            appData[Storage::AVKey::Ch1] = Storage::ApplData::value_type{};
            appData[Storage::AVKey::Ch2] = Storage::ApplData::value_type{};
            appData[Storage::AVKey::Ch3] = Storage::ApplData::value_type{};
            appData[Storage::AVKey::Ch4] = Storage::ApplData::value_type{};
            appData[Storage::AVKey::Ch5] = Storage::ApplData::value_type{};
            appData[Storage::AVKey::Ch6] = Storage::ApplData::value_type{};
            appData[Storage::AVKey::Ch7] = Storage::ApplData::value_type{};
            appData.change();
            changed  = true;
        }
    }
    
    out::init();
    
    const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);

    etl::outl<terminal>("multi8dsbus"_pgm);

//    uint16_t counter{};
    
    while(true) {
        eeprom::saveIfNeeded([&]{
        });
        servo::periodic();
        
        systemTimer::periodic([&]{
            ibus_switch::periodic();
            out::setSwitches();
            
            servo_pa::ratePeriodic();
            
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    etl::out<terminal>("sw: "_pgm);
                    for(const auto& s: sw::switches()) {
                        etl::out<terminal>((uint8_t) s);
                    }
                    etl::outl<terminal>(" c: "_pgm, servo_pa::c, " ch: "_pgm, ibus_switch::lv, 
                                        " s: "_pgm, (uint8_t)servo_pa::mState,
                                        " lo: "_pgm, ibus_switch::lastOnIndex.toInt()
                                        );
                    appData.expire();
                }
            });
        });
    }
}
