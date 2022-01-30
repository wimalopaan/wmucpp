#include "devices.h"

template<typename Devices>
struct GlobalFsm {
    using devs = Devices;
    using sm1 = devs::sm1;    
    using sm1_pa = sm1::protocoll_adapter_type;
    using axis_index_t = sm1_pa::index_t;
    
    using sm2 = devs::sm2;    
    using sm2_pa = sm2::protocoll_adapter_type;

    using robo = devs::robo;    
    using sbus1 = devs::sbus1;    
    using sbus2 = devs::sbus2;    

    using term = devs::terminal;
    
    using nvm = devs::eeprom;
    static inline auto& appData = nvm::data();
    
    using timer = devs::systemTimer;
    
    using test1 = devs::test1Pin;
    using test2 = devs::test2Pin;
    
    static inline constexpr External::Tick<timer> initTicks{100_ms};
    static inline constexpr External::Tick<timer> debugTicks{500_ms};
    static inline constexpr External::Tick<timer> eepromTicks{1000_ms};
    static inline constexpr External::Tick<timer> sbusTicks{14_ms};
    static_assert(sbusTicks.value > 1);
    
    static inline void init() {
        devs::init();
        test1::init();
        test2::init();
        sm1::template init<AVR::BaudRate<38400>>();
        sm2::template init<AVR::BaudRate<38400>>();
        robo::template init<AVR::BaudRate<9600>>();    
        sbus1::init();
        sbus2::init();
    }
    static inline void periodic() {
        test1::toggle();
        nvm::saveIfNeeded([&]{
            etl::outl<term>("save eep"_pgm);
        });
        sm1::periodic();   
        sm2::periodic();   
        robo::periodic();   
        sbus1::periodic();   
        sbus2::periodic();   
    }
    
    enum class State : uint8_t {Start, Run, TransferSM1, TransferSM2};

    static inline void debug() {
        
    }
    
    static inline void ratePeriodic() {
        (++mDebugTicks).on(debugTicks, debug);
        (++mEepromTicks).on(eepromTicks, []{
            appData.expire();
        });
        (++mSBusTicks).on(sbusTicks, []{
            sbus1::ratePeriodic();   
            sbus2::ratePeriodic();   
        });
        ++mStateTicks;
        const auto oldState = mState;
        switch(mState) {
        case State::Start:
            mStateTicks.on(initTicks, []{
                mState = State::Run;
            });
            break;
        case State::Run:
            break;
        case State::TransferSM1:
        {
            using i_t = sbus1::index_type;
            using v_t = sbus1::value_type;
            
            int16_t v0 = 2 * (sm1_pa::axis(axis_index_t{0}) - 8192) + sbus1::sbus_mid;
            int16_t v1 = 2 * (sm1_pa::axis(axis_index_t{1}) - 8192) + sbus1::sbus_mid;
            int16_t v2 = 2 * (sm1_pa::axis(axis_index_t{2}) - 8192) + sbus1::sbus_mid;
            int16_t v3 = 2 * (sm1_pa::axis(axis_index_t{3}) - 8192) + sbus1::sbus_mid;
            int16_t v4 = 2 * (sm1_pa::axis(axis_index_t{4}) - 8192) + sbus1::sbus_mid;
            int16_t v5 = 2 * (sm1_pa::axis(axis_index_t{5}) - 8192) + sbus1::sbus_mid;
            sbus1::set(i_t{10}, v_t{v0});
            sbus1::set(i_t{11}, v_t{v1});
            sbus1::set(i_t{12}, v_t{v2});
            sbus1::set(i_t{13}, v_t{v3});
            sbus1::set(i_t{14}, v_t{v4});
            sbus1::set(i_t{15}, v_t{v5});
            
        }
            break;
        case State::TransferSM2:
            break;
        }
        if (mState != oldState) {
            mStateTicks.reset();
            switch(mState) {
            case State::Start:
                break;
            case State::Run:
                break;
            case State::TransferSM1:
                break;
            case State::TransferSM2:
                break;
            }
        }
    }
    
    inline static State mState{State::Start};
    static inline External::Tick<timer> mSBusTicks;
    static inline External::Tick<timer> mStateTicks;
    static inline External::Tick<timer> mDebugTicks;
    static inline External::Tick<timer> mEepromTicks;
    static inline External::Tick<timer> mRoboTicks;    
};

using devices = Devices<>;
using gfsm = GlobalFsm<devices>;

int main() {
    gfsm::init();    
    while(true) {
        gfsm::periodic(); 
        devices::systemTimer::periodic([&]{
            gfsm::ratePeriodic();
        });
    }
}

#ifndef NDEBUG
[[noreturn]] inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], unsigned int line [[maybe_unused]]) noexcept {
    etl::outl<devices::terminal>("Assertion failed: "_pgm, expr, etl::Char{','}, file, etl::Char{','}, line);
    while(true) {
//        terminalDevice::periodic();
//        dbg1::toggle();
    }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
