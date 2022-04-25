#define NDEBUG

#define USE_CTR_AS_DEBUG
#define USE_FRSKYD_TELEMETRY_OUT

#include "devices.h"

struct Menu {
    static inline void select(const uint8_t i, const auto f) {
    }
    template<typename Out>
    static inline void out() {
        if (mMenu == 0) {
            for(const auto& s : mPage1) {
                etl::outl<Out>(s);
            }
        }
    }
private:
    static inline uint8_t mMenu{};
    static inline uint8_t mItem{};
    
    static inline std::array<AVR::Pgm::StringView, 8> mPage1{
        "Eingang1 (SBus) normal"_pgm,
        "Eingang1 (SBus) intertiert"_pgm,
        "Ausgang1 (SBus) normal"_pgm,
        "Ausgang1 (SBus) intertiert"_pgm,
        "Eingang2 (IBus) normal"_pgm,
        "Eingang2 (IBus) intertiert"_pgm,
        "Ausgang2 (SumDV3) normal"_pgm,
        "Ausgang2 (SumDV3) intertiert"_pgm
    };
};

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
    using robo_out = devs::robo_out;    
    using toggle = External::QtRobo::Toggle<robo_out>;
    
    using hc05 = devs::hc05;
    
    using sbus1 = devs::sbus1;    
    using lut2 = devs::lut2;    
    using dbg = devs::dbg;    

    using frskyD = devs::frskyD;
    
    using sumd = devs::sumd2;    
    using lut1 = devs::lut1;
    
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
        nvm::init();
        if (nvm::data().magic() != nvm::data().marker) {
            nvm::data().clear();
        }
        devs::init();
        test1::init();
        test2::init();
        sm1::template init<AVR::BaudRate<38400>>();
        sm2::template init<AVR::BaudRate<38400>>();
        hc05::init();

        if constexpr(!std::is_same_v<dbg, void>) {
            //        dbg::template init<AVR::BaudRate<115200>>();    
            dbg::template init<AVR::BaudRate<100000>, AVR::FullDuplex, true, 1>(); // 8E2            
        }        
        
        if constexpr(!std::is_same_v<frskyD, void>) {
            frskyD::init();
            
            frskyD::set(0, External::FrskyD::ID::DIY1, 123);
            frskyD::set(1, External::FrskyD::ID::DIY2, 123);
            frskyD::set(2, External::FrskyD::ID::DIY3, 123);
            frskyD::set(3, External::FrskyD::ID::DIY4, 123);
            frskyD::set(4, External::FrskyD::ID::DIY5, 123);
            frskyD::set(5, External::FrskyD::ID::DIY6, 123);
            
        }
        
        if (std::any(nvm::data().template serial<0>().flags & Data::outputInverted)) {
            sbus1::usart::txPinDisable();
            lut2::init(std::byte{0x33}); // route TXD (inverted) to lut2-out     
        }
        if (std::any(nvm::data().template serial<0>().flags & Data::inputInverted)) {
            sbus1::usart::rxInvert(true);  
        }
        
        sumd::init();
        if (std::any(nvm::data().template serial<1>().flags & Data::outputInverted)) {
            sumd::usart::txPinDisable();
            lut1::init(std::byte{0x33}); // route TXD (inverted) to lut1-out 
        }

        led1::init();
    }

    static inline void periodic() {
#if defined(USE_CTR_AS_DEBUG)
        dbgPin::toggle();
#endif
        nvm::saveIfNeeded([&]{
            etl::outl<term>("save eep"_pgm);
        });
        sm1::periodic();   
        sm2::periodic();   
        hc05::periodic();   
        
        if constexpr(!std::is_same_v<dbg, void>) {
            dbg::periodic();   
        }
        if constexpr(!std::is_same_v<frskyD, void>) {
            frskyD::periodic();
        }
        sumd::periodic(); 
    }
    
    enum class State : uint8_t {Start, InitSM1, InitSM1Wait, InitSM1End, InitSM2, InitSM2Wait, InitSM2End, 
                                BTSetName, 
                                Run};

    static inline uint16_t ft;
    
    static inline void debug() {
        frskyD::set(0, External::FrskyD::ID::DIY1, ++ft);

        using i_t = sumd::index_type;
        etl::outl<term>("test03"_pgm, " ch0: "_pgm, sumd::raw(i_t{0}), " ft: "_pgm, ft);
    }
    
    static inline void ratePeriodic() {
        (++mDebugTicks).on(debugTicks, debug);
        (++mEepromTicks).on(eepromTicks, []{
            appData.expire();
        });
        
        sumd::ratePeriodic();   
        hc05::ratePeriodic();
        
        if constexpr(!std::is_same_v<frskyD, void>) {
            frskyD::ratePeriodic();
        }
        
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
            }
        }
    }
private:
    struct UpdateFSM {
        enum class State : uint8_t {A1, A2, A3, A4, A5, A6, B1, B2, B3, B4, B5, B6, Robo};
        using i_t = sumd::index_type;
        using v_t = sumd::value_type;

        template<typename PA, uint8_t N, uint8_t Offset>
        static inline void sm() {
            const auto v = PA::axis(axis_index_t{N});
            const v_t sv = etl::scaleTo<v_t>(v);
            sumd::set(i_t{N + Offset}, sv);
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
                    if (const uint8_t idx = index.toInt(); index) {
                        if (t == robo_pa::Target::Prop) {
                            if (idx < 16) {
                                const etl::uint_ranged<uint8_t> v(robo_pa::propValues[idx]);
                                const v_t sv = etl::scaleTo<v_t>(v);
                                etl::outl<term>("p: "_pgm, idx, " sv: "_pgm, sv.toInt());
                                sumd::set(i_t(idx + 16), sv);                            
                            }
                        }
                        else if (t == robo_pa::Target::Toggle) {
                            if (idx < 64) {
                                etl::outl<term>("t: "_pgm, idx);
                                sumd::setSwitch(i_t{idx}, robo_pa::toggleValues[idx]);
                                toggle::put(idx, robo_pa::toggleValues[idx]);                        
                            }
                        }
                        else if (t == robo_pa::Target::Switch) {
                            if (idx < 64) {
                                etl::outl<term>("s: "_pgm, idx, " : "_pgm, robo_pa::switchValues[idx]);
                                typename sumd::command_t c;
                                c.first = std::byte{idx};
                                c.second = std::byte{robo_pa::switchValues[idx]};
                                sumd::setCmd(c);
                            }
                        }
                        else if (t == robo_pa::Target::Menu) {
                            etl::outl<term>("m sel: "_pgm, idx);
                            Menu::select(idx, [](){
                                
                            });
                            Menu::template out<robo_out>();
                        }
                    }
                    else {
                        if (t == robo_pa::Target::Menu) {
                            etl::outl<term>("m txt: "_pgm);
                            Menu::template out<robo_out>();
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

using devices = Devices<3>;
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
 
