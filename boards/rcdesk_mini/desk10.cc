#define NDEBUG

#define USE_CTR_AS_DEBUG

#include "devices.h"
 
template<typename NVM, typename SBusPA, typename IBusPA>
struct Menu {
    using nvm = NVM;
    using sbus_pa = SBusPA;
    using ibus_pa = IBusPA;
    
    static inline void select(const uint8_t i, const auto f) {
        switch(i) {
        case 0:
            nvm::data().template serial<0>().flags ^= Data::inputInverted;
            nvm::data().change();
            break;
        case 1:
            nvm::data().template serial<0>().flags ^= Data::outputInverted;
            nvm::data().change();
            break;
        case 2:
            nvm::data().template serial<1>().flags ^= Data::inputInverted;
            nvm::data().change();
            break;
        case 3:
            nvm::data().template serial<1>().flags ^= Data::outputInverted;
            nvm::data().change();
            break;
        default:
            break;
        }
        
        f();
    }
    template<typename Out>
    static inline void out() {
        etl::out<Out>(mPage1[0]);
        if (std::any(nvm::data().template serial<0>().flags & Data::inputInverted)) {
            etl::out<Out>(niText.second);
        }
        else {
            etl::out<Out>(niText.first);
        }
        etl::outl<Out>(" (Pakete: "_pgm, sbus_pa::packages(), ")"_pgm);
        
        etl::out<Out>(mPage1[1]);
        if (std::any(nvm::data().template serial<0>().flags & Data::outputInverted)) {
            etl::outl<Out>(niText.second);
        }
        else {
            etl::outl<Out>(niText.first);
        }
        
        etl::out<Out>(mPage1[2]);
        if (std::any(nvm::data().template serial<1>().flags & Data::inputInverted)) {
            etl::out<Out>(niText.second);
        }
        else {
            etl::out<Out>(niText.first);
        }
        etl::outl<Out>(" (Pakete: "_pgm, ibus_pa::packages(), ")"_pgm);
        
        etl::out<Out>(mPage1[3]);
        if (std::any(nvm::data().template serial<1>().flags & Data::outputInverted)) {
            etl::outl<Out>(niText.second);
        }
        else {
            etl::outl<Out>(niText.first);
        }
        etl::outl<Out>(footerMenu);
    }
private:
    static inline std::array<AVR::Pgm::StringView, 4> mPage1{
        "Eingang1 (SBus)   : "_pgm,
        "Ausgang1 (SBus)   : "_pgm,
        "Eingang2 (IBus)   : "_pgm,
        "Ausgang2 (SumDV3) : "_pgm,
    };
    
    static inline std::pair niText{"normal"_pgm, "invertiert"_pgm};
    static inline auto footerMenu{"Eintrag auswaehlen (aendern) [$m:<zeilennummer><enter>]\n\rMenu anzeigen [$m<enter>]"_pgm};
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
    using log = External::QtRobo::Logging<robo_out, 0>;
    
    using hc05 = devs::hc05;
    
    using lut2 = devs::lut2;    

    using sumd = devs::sumd2;    
    using lut1 = devs::lut1;
    using ibus_pa = devs::ibus2_pa;
    using blinkLed = devs::blinkLed;    
    using bcount_t = blinkLed::count_type;

    using sbus = devs::sbus1;
    using sbus_pa = devs::sbus_pa;
    using term = devs::terminal;
    
    using nvm = devs::eeprom;
    static inline auto& appData = nvm::data();
    
    using timer = devs::systemTimer;
    
    using test1 = devs::test1Pin;
    using test2 = devs::test2Pin;
    
    using menu = Menu<nvm, sbus_pa, ibus_pa>;
    
    static inline constexpr External::Tick<timer> initTicks{100_ms};
    static inline constexpr External::Tick<timer> debugTicks{500_ms};
    static inline constexpr External::Tick<timer> checkTicks{500_ms};
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

        sbus::template init<AVR::BaudRate<100000>, AVR::FullDuplex, true, 1>(); // 8E2            
        sumd::init();
        
        updateUsarts();

        blinkLed::init();
    }

    static inline void updateUsarts() {
        if (std::any(nvm::data().template serial<0>().flags & Data::outputInverted)) {
            sbus::txPinDisable();
            lut2::init(std::byte{0x33}); // route TXD (inverted) to lut2-out     
            lut2::enable();
        }
        else {
            lut2::disable();
            sbus::txPinEnable();
        }
        if (std::any(nvm::data().template serial<0>().flags & Data::inputInverted)) {
            sbus::rxInvert(true);  
        }
        else {
            sbus::rxInvert(false);  
        }
        if (std::any(nvm::data().template serial<1>().flags & Data::outputInverted)) {
            sumd::usart::txPinDisable();
            lut1::init(std::byte{0x33}); // route TXD (inverted) to lut1-out 
            lut1::enable();
        }
        else {
            lut1::disable();
            sumd::usart::txPinEnable();
        }
        if (std::any(nvm::data().template serial<1>().flags & Data::inputInverted)) {
            sumd::usart::rxInvert(true);  
        }        
        else {
            sumd::usart::rxInvert(false);  
        }
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
        sbus::periodic();   
        sumd::periodic(); 
    }
    
    enum class State : uint8_t {Start, InitSM1, InitSM1Wait, InitSM1End, InitSM2, InitSM2Wait, InitSM2End, 
                                BTSetName, 
                                Run1, Run2, Run3, Run4};

    static inline void debug() {
        using i_t = sumd::index_type;
//        etl::outl<term>("desk10"_pgm, " ch0: "_pgm, sumd::raw(i_t{0}));
        etl::outl<term>("desk10"_pgm, " sm1: "_pgm, sm1Packages, " sm2: "_pgm, sm2Packages);
    }
    
    static inline void ratePeriodic() {
        (++mDebugTicks).on(debugTicks, debug);
        (++mEepromTicks).on(eepromTicks, []{
            appData.expire();
        });
        
        sumd::ratePeriodic();   
        hc05::ratePeriodic();
        sbus_pa::ratePeriodic();
        ibus_pa::ratePeriodic();
        blinkLed::ratePeriodic();
        
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
                mState = State::Run1;
            });
            break;
        case State::Run1:
            (++mCheckTicks).on(checkTicks, []{
                if (sm1_pa::packages() != sm1Packages) {
                    sm1Active = true;
                    sm1Packages = sm1_pa::packages();
                }
                else {
                    sm1Active = false;
                }
                mState = State::Run2;
            });
            UpdateFSM::process();
            break;
        case State::Run2:
            (++mCheckTicks).on(checkTicks, []{
                if (sm2_pa::packages() != sm2Packages) {
                    sm2Active = true;
                    sm2Packages = sm2_pa::packages();
                }
                else {
                    sm2Active = false;
                }
                mState = State::Run3;
            });
            UpdateFSM::process();
            break;
        case State::Run3:
            (++mCheckTicks).on(checkTicks, []{
                if (sbus_pa::packages() != sbPackages) {
                    sbusActive = true;
                    sbPackages = sbus_pa::packages();
                    blinkCount.set(1);
                }
                else {
                    blinkCount.set(0);
                    sbusActive = false;
                }
                mState = State::Run4;
            });
            UpdateFSM::process();
            break;
        case State::Run4:
            (++mCheckTicks).on(checkTicks, []{
                if (ibus_pa::packages() != ibPackages) {
                    ibusActive = true;
                    ibPackages = ibus_pa::packages();
                    blinkCount += 2;
                }
                else {
                    ibusActive = false;
                }
                mState = State::Run1;
            });
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
                blinkLed::blink(bcount_t{5});                
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
                blinkLed::blink(bcount_t{6});                
                etl::outl<term>("S BTSetName"_pgm);
                hc05::event(hc05::Event::SetName);
                break;
            case State::Run1:
                etl::outl<term>("S Run1"_pgm);
                blinkCount += 1;
                blinkLed::blink(blinkCount);
                break;
            case State::Run2:
                break;
            case State::Run3:
                break;
            case State::Run4:
                break;
            }
        }
    }
private:
    struct UpdateFSM {
        enum class State : uint8_t {A1, A2, A3, A4, A5, A6, 
                                    B1, B2, B3, B4, B5, B6, 
                                    Robo,
                                    SB0_3, SB4_7, SB8_11, SB12_15, SB_SW,
                                    IB0_3, IB4_7, IB8_11, IB12_15
                                   };
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
                            menu::select(idx, []{
                                updateUsarts();
                            });          
                            menu::template out<robo_out>();
                        }
                    }
                    else {
                        if (t == robo_pa::Target::Menu) {
                            etl::outl<term>("m txt: "_pgm);
                            menu::template out<robo_out>();
                        }
                    }
                });
                mState = State::SB_SW;
                break;
            case State::SB_SW:
                sumd::setSwitch(i_t{14}, std::any(sbus_pa::switches() & External::SBus::ch17));
                sumd::setSwitch(i_t{15}, std::any(sbus_pa::switches() & External::SBus::ch18));
                if (sm1Active || sm2Active) {
                    mState = State::SB12_15;
                }
                else {
                    mState = State::SB0_3;
                }
                break;
            case State::SB0_3:
                if (const auto v = sbus_pa::value(0); v) {
                    const v_t sv0 = etl::scaleTo<v_t>(v.toRanged());
                    sumd::set(i_t{0}, sv0);                            
                    const v_t sv1 = etl::scaleTo<v_t>(sbus_pa::value(1).toRanged());
                    sumd::set(i_t{1}, sv1);                            
                    const v_t sv2 = etl::scaleTo<v_t>(sbus_pa::value(2).toRanged());
                    sumd::set(i_t{2}, sv2);                            
                    const v_t sv3 = etl::scaleTo<v_t>(sbus_pa::value(3).toRanged());
                    sumd::set(i_t{3}, sv3);                            
                }
                mState = State::SB4_7;
                break;
            case State::SB4_7:
                if (const auto v = sbus_pa::value(4); v) {
                    const v_t sv0 = etl::scaleTo<v_t>(v.toRanged());
                    sumd::set(i_t{4}, sv0);                            
                    const v_t sv1 = etl::scaleTo<v_t>(sbus_pa::value(5).toRanged());
                    sumd::set(i_t{5}, sv1);                            
                    const v_t sv2 = etl::scaleTo<v_t>(sbus_pa::value(6).toRanged());
                    sumd::set(i_t{6}, sv2);                            
                    const v_t sv3 = etl::scaleTo<v_t>(sbus_pa::value(7).toRanged());
                    sumd::set(i_t{7}, sv3);                            
                }
                mState = State::SB8_11;
                break;
            case State::SB8_11:
                if (const auto v = sbus_pa::value(8); v) {
                    const v_t sv0 = etl::scaleTo<v_t>(v.toRanged());
                    sumd::set(i_t{8}, sv0);                            
                    const v_t sv1 = etl::scaleTo<v_t>(sbus_pa::value(9).toRanged());
                    sumd::set(i_t{9}, sv1);                            
                    const v_t sv2 = etl::scaleTo<v_t>(sbus_pa::value(10).toRanged());
                    sumd::set(i_t{10}, sv2);                            
                    const v_t sv3 = etl::scaleTo<v_t>(sbus_pa::value(11).toRanged());
                    sumd::set(i_t{11}, sv3);                            
                }
                mState = State::SB12_15;
                break;
            case State::SB12_15:
                if (const auto v = sbus_pa::value(12); v) {
                    const v_t sv0 = etl::scaleTo<v_t>(v.toRanged());
                    sumd::set(i_t{12}, sv0);                            
                    const v_t sv1 = etl::scaleTo<v_t>(sbus_pa::value(13).toRanged());
                    sumd::set(i_t{13}, sv1);                            
                    const v_t sv2 = etl::scaleTo<v_t>(sbus_pa::value(14).toRanged());
                    sumd::set(i_t{14}, sv2);                            
                    const v_t sv3 = etl::scaleTo<v_t>(sbus_pa::value(15).toRanged());
                    sumd::set(i_t{15}, sv3);                            
                }
                mState = State::IB0_3;
                break;
            case State::IB0_3:
                if (const auto v = ibus_pa::value(0); v) {
                    const v_t sv0 = etl::scaleTo<v_t>(v.toRanged());
                    sumd::set(i_t{16}, sv0);                            
                    const v_t sv1 = etl::scaleTo<v_t>(ibus_pa::value(1).toRanged());
                    sumd::set(i_t{17}, sv1);                            
                    const v_t sv2 = etl::scaleTo<v_t>(ibus_pa::value(2).toRanged());
                    sumd::set(i_t{18}, sv2);                            
                    const v_t sv3 = etl::scaleTo<v_t>(ibus_pa::value(3).toRanged());
                    sumd::set(i_t{19}, sv3);                            
                }
                mState = State::IB4_7;
                break;
            case State::IB4_7:
                if (const auto v = ibus_pa::value(4); v) {
                    const v_t sv0 = etl::scaleTo<v_t>(v.toRanged());
                    sumd::set(i_t{20}, sv0);                            
                    const v_t sv1 = etl::scaleTo<v_t>(ibus_pa::value(5).toRanged());
                    sumd::set(i_t{21}, sv1);                            
                    const v_t sv2 = etl::scaleTo<v_t>(ibus_pa::value(6).toRanged());
                    sumd::set(i_t{22}, sv2);                            
                    const v_t sv3 = etl::scaleTo<v_t>(ibus_pa::value(7).toRanged());
                    sumd::set(i_t{23}, sv3);                            
                }
                mState = State::IB8_11;
                break;
            case State::IB8_11:
                if (const auto v = ibus_pa::value(8); v) {
                    const v_t sv0 = etl::scaleTo<v_t>(v.toRanged());
                    sumd::set(i_t{24}, sv0);                            
                    const v_t sv1 = etl::scaleTo<v_t>(ibus_pa::value(9).toRanged());
                    sumd::set(i_t{25}, sv1);                            
                    const v_t sv2 = etl::scaleTo<v_t>(ibus_pa::value(10).toRanged());
                    sumd::set(i_t{26}, sv2);                            
                    const v_t sv3 = etl::scaleTo<v_t>(ibus_pa::value(11).toRanged());
                    sumd::set(i_t{27}, sv3);                            
                }
                mState = State::IB12_15;
                break;
            case State::IB12_15:
                if (const auto v = ibus_pa::value(12); v) {
                    const v_t sv0 = etl::scaleTo<v_t>(v.toRanged());
                    sumd::set(i_t{28}, sv0);                            
                    const v_t sv1 = etl::scaleTo<v_t>(ibus_pa::value(13).toRanged());
                    sumd::set(i_t{29}, sv1);                            
                    const v_t sv2 = etl::scaleTo<v_t>(ibus_pa::value(14).toRanged());
                    sumd::set(i_t{30}, sv2);                            
                    const v_t sv3 = etl::scaleTo<v_t>(ibus_pa::value(15).toRanged());
                    sumd::set(i_t{31}, sv3);                            
                }
                if (sm1Active || sm2Active) {
                    mState = State::A1;
                }
                else {
                    mState = State::SB_SW;
                }
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
    static inline External::Tick<timer> mCheckTicks;    
    
    static inline uint16_t sm1Packages{};
    static inline uint16_t sm2Packages{};
    static inline uint16_t sbPackages{};
    static inline uint16_t ibPackages{};
    
    static inline bool sm1Active{true};
    static inline bool sm2Active{true};
    static inline bool sbusActive{true};
    static inline bool ibusActive{true};
    
    static inline bcount_t blinkCount{0};
};

using devices = Devices<10>;
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
 
