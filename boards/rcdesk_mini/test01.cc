#define NDEBUG

#define USE_CTR_AS_DEBUG

#include "devices.h"

template<typename Devices>
struct GlobalFsm {
    using devs = Devices;
    using sm1 = devs::sm1;    
    using sm1_pa = sm1::protocoll_adapter_type;
    using axis_index_t = sm1_pa::index_t;
    
    using sm2 = devs::sm2;    
    using sm2_pa = sm2::protocoll_adapter_type;

#if defined(USE_CTR_AS_DEBUG)
    using dbgPin = devs::debug;
#endif
    
    using robo = devs::robo;    
    using robo_pa = devs::robo_pa;    
     
    using hc05 = devs::hc05;
    
    using dbg = devs::dbg;    
//    using sbus1 = devs::sbus1;    
    using sbus2 = devs::sbus2;    
    
    using led1 = devs::led1;    

    using term = devs::terminal;
    
    using nvm = devs::eeprom;
    static inline auto& appData = nvm::data();
    
    using timer = devs::systemTimer;
    
    using test1 = devs::test1Pin;
    using test2 = devs::test2Pin;
    
    static inline constexpr External::Tick<timer> initTicks{100_ms};
    static inline constexpr External::Tick<timer> debugTicks{500_ms};
    static inline constexpr External::Tick<timer> eepromTicks{1000_ms};
    static inline constexpr External::Tick<timer> waitTicks{2000_ms};
    
    static inline void init() {
        devs::init();
        test1::init();
        test2::init();
        sm1::template init<AVR::BaudRate<38400>>();
        sm2::template init<AVR::BaudRate<38400>>();
        hc05::init();
        
        dbg::template init<AVR::BaudRate<9600>>();    
//        sbus1::init();
        sbus2::init();

        led1::init();
    }
    static inline void periodic() {
#if defined(USE_CTR_AS_DEBUG)
        dbgPin::toggle();
#endif
        test1::toggle();
        nvm::saveIfNeeded([&]{
            etl::outl<term>("save eep"_pgm);
        });
        sm1::periodic();   
        sm2::periodic();   
        hc05::periodic();   
        
        dbg::periodic();   
//        sbus1::periodic();   
        sbus2::periodic(); 
    }
    
    enum class State : uint8_t {Start, InitSM1, InitSM1Wait, InitSM1End, InitSM2, InitSM2Wait, InitSM2End, 
                                BTSetName, 
                                Run, TransferSM1, TransferSM2};

    static inline void debug() {
        etl::outl<term>("test01: "_pgm, sm1_pa::axis(axis_index_t{0}));
    }
    
    
    static inline void ratePeriodic() {
        (++mDebugTicks).on(debugTicks, debug);
        (++mEepromTicks).on(eepromTicks, []{
            appData.expire();
        });
//        sbus1::ratePeriodic();   
        sbus2::ratePeriodic();   
        hc05::ratePeriodic();
        
        ++mStateTicks;
        const auto oldState = mState;
        switch(mState) {
        case State::Start:
            mStateTicks.on(waitTicks, []{
                mState = State::InitSM1;
            });
            break;
        case State::InitSM1:
            break;
        case State::InitSM1Wait:
            if (sm1::isEmpty()) {
                mState = State::InitSM1End;
            }
            break;
        case State::InitSM1End:
            mStateTicks.on(initTicks, []{
                mState = State::InitSM2;
            });
            break;
        case State::InitSM2:
            break;
        case State::InitSM2Wait:
            if (sm2::isEmpty()) {
                mState = State::InitSM2End;
            }
            break;
        case State::InitSM2End:
            mStateTicks.on(waitTicks, []{
                mState = State::BTSetName;
            });
            break;
        case State::BTSetName:
            mStateTicks.on(waitTicks, []{
                mState = State::Run;
            });
            break;
        case State::Run:
            UpdateFSM::process();
//            mState = State::TransferSM1;
            break;
        case State::TransferSM1:
        {
            using i_t = sbus2::index_type;
            using v_t = sbus2::value_type;
            
            int16_t v0 = 2 * (sm1_pa::axis(axis_index_t{0}) - 8192) + sbus2::sbus_mid;
            int16_t v1 = 2 * (sm1_pa::axis(axis_index_t{1}) - 8192) + sbus2::sbus_mid;
            int16_t v2 = 2 * (sm1_pa::axis(axis_index_t{2}) - 8192) + sbus2::sbus_mid;
            int16_t v3 = 2 * (sm1_pa::axis(axis_index_t{3}) - 8192) + sbus2::sbus_mid;
            int16_t v4 = 2 * (sm1_pa::axis(axis_index_t{4}) - 8192) + sbus2::sbus_mid;
            int16_t v5 = 2 * (sm1_pa::axis(axis_index_t{5}) - 8192) + sbus2::sbus_mid;
            sbus2::set(i_t{10}, v_t(v0));
            sbus2::set(i_t{11}, v_t(v1));
            sbus2::set(i_t{12}, v_t(v2));
            sbus2::set(i_t{13}, v_t(v3));
            sbus2::set(i_t{14}, v_t(v4));
            sbus2::set(i_t{15}, v_t(v5));
//            mState = State::TransferSM2;
        }
            break;
        case State::TransferSM2:
        {
            using i_t = sbus2::index_type;
            using v_t = sbus2::value_type;
            
            int16_t v0 = 2 * (sm2_pa::axis(axis_index_t{0}) - 8192) + sbus2::sbus_mid;
            int16_t v1 = 2 * (sm2_pa::axis(axis_index_t{1}) - 8192) + sbus2::sbus_mid;
            int16_t v2 = 2 * (sm2_pa::axis(axis_index_t{2}) - 8192) + sbus2::sbus_mid;
            int16_t v3 = 2 * (sm2_pa::axis(axis_index_t{3}) - 8192) + sbus2::sbus_mid;
            int16_t v4 = 2 * (sm2_pa::axis(axis_index_t{4}) - 8192) + sbus2::sbus_mid;
            int16_t v5 = 2 * (sm2_pa::axis(axis_index_t{5}) - 8192) + sbus2::sbus_mid;
            sbus2::set(i_t{0}, v_t(v0));
            sbus2::set(i_t{1}, v_t(v1));
            sbus2::set(i_t{2}, v_t(v2));
            sbus2::set(i_t{3}, v_t(v3));
            sbus2::set(i_t{4}, v_t(v4));
            sbus2::set(i_t{5}, v_t(v5));
            mState = State::TransferSM1;
        }
            break;
        }
        if (mState != oldState) {
            mStateTicks.reset();
            switch(mState) {
            case State::Start:
                break;
            case State::InitSM1:
                etl::outl<term>("S InitSM1"_pgm);
                sm1::put(0xad_B);
                sm1::put(sm1_pa::endSymbol);
                sm1::put(0xae_B);
                sm1::put(sm1_pa::endSymbol);
                mState = State::InitSM1Wait;
                break;                
            case State::InitSM1Wait:
                etl::outl<term>("S InitSM1Wait"_pgm);
                break;
            case State::InitSM1End:
                etl::outl<term>("S InitSM1End"_pgm);
                break;
            case State::InitSM2:
                etl::outl<term>("S InitSM2"_pgm);
                sm2::put(0xad_B);
                sm2::put(sm2_pa::endSymbol);
                sm2::put(0xae_B);
                sm2::put(sm2_pa::endSymbol);
                mState = State::InitSM2Wait;
                break;                
            case State::InitSM2Wait:
                etl::outl<term>("S InitSM2Wait"_pgm);
                break;
            case State::InitSM2End:
                etl::outl<term>("S InitSM2End"_pgm);
                break;       
            case State::BTSetName:
                etl::outl<term>("S BTSetName"_pgm);
                hc05::event(hc05::Event::SetName);
                break;
            case State::Run:
                etl::outl<term>("S Run"_pgm);
                led1::activate();
                break;
            case State::TransferSM1:
                break;
            case State::TransferSM2:
                break;
            }
        }
    }
private:
    struct UpdateFSM {
        enum class State : uint8_t {A1, A2, A3, A4, A5, A6, B1, B2, B3, B4, B5, B6, Robo};
        using i_t = sbus2::index_type;
        using v_t = sbus2::value_type;

        template<typename PA, uint8_t N, uint8_t Offset>
        static inline void sm() {
            const int16_t v = 2 * (PA::axis(axis_index_t{N}) - 8192) + sbus2::sbus_mid;
            sbus2::set(i_t{N + Offset}, v_t(v));
        }
        static inline void process() {
            switch(mState) {
            case State::A1:
                sm<sm1_pa, 0, 0>();
                mState = State::B1;
                break;
            case State::A2:
                sm<sm1_pa, 1, 0>();
                mState = State::B2;
                break;
            case State::A3:
                sm<sm1_pa, 2, 0>();
                mState = State::B3;
                break;
            case State::A4:
                sm<sm1_pa, 3, 0>();
                mState = State::B4;
                break;
            case State::A5:
                sm<sm1_pa, 4, 0>();
                mState = State::B5;
                break;
            case State::A6:
                sm<sm1_pa, 5, 0>();
                mState = State::B6;
                break;
            case State::B1:
                sm<sm2_pa, 0, 6>();
                mState = State::A2;
                break;
            case State::B2:
                sm<sm2_pa, 1, 6>();
                mState = State::A3;
                break;
            case State::B3:
                sm<sm2_pa, 2, 6>();
                mState = State::A4;
                break;
            case State::B4:
                sm<sm2_pa, 3, 6>();
                mState = State::A5;
                break;
            case State::B5:
                sm<sm2_pa, 4, 6>();
                mState = State::A6;
                break;
            case State::B6:
                sm<sm2_pa, 5, 6>();
                mState = State::Robo;
                break;
            case State::Robo:
                robo_pa::whenTargetChanged([](const robo_pa::Target t, const auto index){
                    if (t == robo_pa::Target::Prop) {
                        if (index < 4) {
//                            sbus2::set(i_t{index + 12}, v_t(v));                            
                        }
                    }
                });
                mState = State::A1;
                break;
            }
        }  
    private:
        static inline State mState{State::A1};     
    };
    
    inline static State mState{State::Start};
    static inline External::Tick<timer> mSBusTicks;
    static inline External::Tick<timer> mStateTicks;
    static inline External::Tick<timer> mDebugTicks;
    static inline External::Tick<timer> mEepromTicks;
    static inline External::Tick<timer> mRoboTicks;    
};

using devices = Devices<1>;
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
 
