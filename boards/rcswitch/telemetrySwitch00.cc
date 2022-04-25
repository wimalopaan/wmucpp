#define NDEBUG

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/gpiorflags.h>
#include <mcu/internals/eeprom.h>

#include <external/sbus/frskyD.h>
#include <external/hal/alarmtimer.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

namespace  {
    static constexpr auto fRtc = 1000_Hz;

    using eepromGpior = AVR::GPIORFlags<AVR::Component::GPIOR<0>, etl::NamedConstant<0>, etl::NamedConstant<2>>;
    
    struct Data final : public EEProm::DataBase<Data, eepromGpior> {
        static inline constexpr uint8_t marker{42};
        
        inline auto magic() const {
            return mMagic;
        }
        
        inline bool& active() {
            return mActive;
        }
        
        inline void clear() {
            mMagic = marker;   
            mActive = false;   
            change();
        }
    private:
        uint8_t mMagic{};
        bool mActive{false};
    };
}

using s11 = ActiveLow<Pin<Port<A>, 4>, Input>; 
using s12 = ActiveLow<Pin<Port<A>, 5>, Input>; 
using t1  = ActiveLow<Pin<Port<A>, 6>, Input>; 
using s2  = ActiveLow<Pin<Port<B>, 3>, Input>; 
using t2  = ActiveLow<Pin<Port<B>, 2>, Input>; 

using o1 = ActiveLow<Pin<Port<A>, 7>, Output>; 
using o2 = ActiveLow<Pin<Port<B>, 0>, Output>; 
using o3 = ActiveLow<Pin<Port<A>, 2>, Output>; 
using o4 = ActiveLow<Pin<Port<A>, 3>, Output>; 

using ccp = Cpu::Ccp<>;
using clock = Clock<>;
using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Alt1>;
using frskyD = External::FrskyD::Output::Generator<usart0Position, systemTimer, void, 5>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position>>;

using nvm = EEProm::Controller<Data>;
static inline auto& appData = nvm::data();

struct FSM {
    static inline constexpr External::Tick<systemTimer> initTicks{300_ms};
    static inline constexpr External::Tick<systemTimer> eepromTicks{1000_ms};

    enum class State : uint8_t {Undefined, CheckPress, Run, Inactive};
    
    static inline void init() {
        nvm::init();
        if (nvm::data().magic() != nvm::data().marker) {
            nvm::data().clear();
        }
        o1::init();        
        o2::init();        
        o3::init();        
        o4::init(); 
        
        o1::activate();
        o2::activate();
        o3::activate();
        o4::activate();
        
        s11::init();        
        s12::init();        
        t1::init();        
        s2::init();        
        t2::init();        
        frskyD::init();
        frskyD::usart::txOpenDrain();
    }
    static inline void periodic() {
        frskyD::periodic();
        nvm::saveIfNeeded();
    }
    static inline void ratePeriodic() {
        (++mEepromTicks).on(eepromTicks, []{
            appData.expire();
        });
        ++mStateTicks;
        switch(mState) {
        case State::Undefined:
            mStateTicks.on(initTicks, []{
                mState = State::CheckPress;
            });
            break;
        case State::CheckPress:
            if (t1::isActive()) {
                if (appData.active()) {
                    appData.active() = false;
                    mState = State::Inactive;
                }
                else {
                    appData.active() = true;
                    mState = State::Run;
                }
                appData.change();
            }
            else {
                if (appData.active()) {
                    mState = State::Run;
                }
                else {
                    mState = State::Inactive;
                }
            }
            break;          
        case State::Inactive:
            break;
        case State::Run:
            update();
            frskyD::ratePeriodic();
            break;
        }
    }
private:
    static inline void update() {
        const uint8_t v = []{
            if (s11::isActive()) {
                return 1;
            }
            if (s12::isActive()) {
                return 2;
            }
            return 0;
        }();
        frskyD::set(0, External::FrskyD::ID::DIY1, v);
        frskyD::set(1, External::FrskyD::ID::DIY2, t1::isActive() ? 1 : 0);
        frskyD::set(2, External::FrskyD::ID::DIY3, s2::isActive() ? 1 : 0);
        frskyD::set(3, External::FrskyD::ID::DIY4, t2::isActive() ? 1 : 0);
        
        uint8_t b{0x00};
        if (s11::isActive()) {
            b |= 0x01;
        }
        if (s12::isActive()) {
            b |= 0x02;
        }
        if (t1::isActive()) {
            b |= 0x04;
        }
        if (s2::isActive()) {
            b |= 0x08;
        }
        if (t2::isActive()) {
            b |= 0x10;
        }
        frskyD::set(4, External::FrskyD::ID::DIY5, b);
    }
    inline static State mState{State::Undefined};
    static inline External::Tick<systemTimer> mStateTicks;
    static inline External::Tick<systemTimer> mEepromTicks;
};

using fsm = FSM;

int main() {
    portmux::init();
    ccp::unlock([]{
        clock::prescale<1>();
    });
    systemTimer::init();

    fsm::init();    
    while(true) {
        fsm::periodic();
        systemTimer::periodic([&]{
            fsm::ratePeriodic();
        });
    }
}

