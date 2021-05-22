#define NDEBUG

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/pwm.h>
#include <mcu/internals/eeprom.h>
#include <mcu/internals/spi.h>
#include <mcu/internals/ccl.h>
#include <mcu/internals/sigrow.h>
#include <mcu/internals/event.h>
#include <external/hal/adccontroller.h>
#include <external/hal/alarmtimer.h>
#include <external/bluetooth/roboremo.h>
#include <external/bluetooth/qtrobo.h>
#include <external/sbus/sbus.h>
#include <external/sbus/sport.h>
#include <external/ibus/ibus2.h>
#include <external/solutions/rc/busscan.h>

#include <external/hott/sumdprotocolladapter.h>
#include <external/hott/experimental/sensor.h>
#include <external/hott/experimental/adapter.h>
#include <external/hott/menu.h>

#include <external/solutions/button.h>
#include <external/solutions/blinker.h>
#include <external/solutions/analogsensor.h>
#include <external/solutions/series01/swuart.h>
#include <external/solutions/series01/rpm.h>
#include <external/solutions/apa102.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

#include "generic.h"

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

namespace  {
    constexpr auto fRtc = 1000_Hz;
}

namespace External {
    namespace PCA9745 {
        struct Registers {
            struct Ramp {
                std::byte rate{};
                std::byte step{};
                std::byte hold{};
                std::byte iref{};
            };
    //        std::byte mode0{};
    //        std::byte mode1{};
    //        std::byte grppwm{};
    //        std::byte grpfreq{};
            std::array<std::byte, 16> pwms{};
            std::array<std::byte, 16> irefs{};
            std::array<Ramp, 4> ramps{};
            std::array<std::byte, 2> grad_modes{};
            std::array<std::byte, 4> grad_grps{};
            std::byte grad_cntl;
    //        std::byte offset;
    //        std::byte pwmall;
    //        std::byte irefall;
        };
    //    static_assert(sizeof(Registers) == 0x42);
        
        template<template<typename Command> typename Spi, AVR::Concepts::Pin OePin, AVR::Concepts::Pin IRef1Pin, AVR::Concepts::Pin IRef2Pin, typename NVM>
        struct Controller {
            
            struct Address {
                std::byte raw{0xff}; 
            };
            
            struct Read{};
            struct Write{};
            
            struct Command {
                using value_type = std::byte;
                
                Command() = default;
                
                explicit Command(Write, uint8_t a, std::byte v) : address{((std::byte{a} & 0x7f_B) << 1) | 0x00_B}, data{v} {}        
                
                template<uint8_t N>
                std::byte get() const {
                    if constexpr(N == 0) {
                        return address.raw;
                    }
                    else if constexpr(N == 1) {
                        return data;
                    }
                    else {
                        static_assert(std::false_v<Spi>);
                    }
                }
                constexpr const std::byte& operator[](uint8_t i) const {
                    if (i == 0) {
                        return address.raw;
                    }         
                    return data;
                }
                static constexpr uint8_t size() {
                    return 2;
                }
            private:
                Address address{};
                std::byte data{0xff};
            };
            
            using spi = Spi<Command>;
            using oePin = OePin;
            
            using iref1Pin = IRef1Pin;
            using iref2Pin = IRef2Pin;

            using ggroup_t = etl::uint_ranged<uint8_t, 0, 3>;
            using giref_t  = etl::uint_ranged<uint8_t, 0, 255>;
            using grrate_t = etl::uint_ranged<uint8_t, 0, 63>;
            using grupdn_t = etl::uint_ranged<uint8_t, 0, 3>;
            using grstep_t = etl::uint_ranged<uint8_t, 0, 127>;
            using ghonoff_t = etl::uint_ranged<uint8_t, 0, 3>;
            using ghtime_t = etl::uint_ranged<uint8_t, 0, 7>;
        
            using index_type = etl::uint_ranged<uint8_t, 0, 15>;
            
            
            enum class Current : uint8_t {C0, C1, C2, C3};
            
            static inline void current(const Current c) {
                switch(c) {
                case Current::C0:
                    iref1Pin::template dir<AVR::Input>();
                    iref2Pin::template dir<AVR::Input>();
                    break;
                case Current::C1:
                    iref1Pin::template dir<AVR::Input>();
                    iref2Pin::template dir<AVR::Output>();
                    iref2Pin::low();
                    break;
                case Current::C2:
                    iref1Pin::template dir<AVR::Output>();
                    iref1Pin::low();
                    iref2Pin::template dir<AVR::Input>();
                    break;
                case Current::C3:
                    iref1Pin::template dir<AVR::Output>();
                    iref1Pin::low();
                    iref2Pin::template dir<AVR::Output>();
                    iref2Pin::low();
                    break;
                default:
                    iref1Pin::template dir<AVR::Input>();
                    iref2Pin::template dir<AVR::Output>();
                    iref2Pin::low();
                }
            }    
            
            static inline void init() {
                current(Current::C1);
                
                oePin::template dir<AVR::Output>();
                oePin::low();
                
                spi::init();
               
                spi::put(Command{Write{}, 0, 0x00_B}); // normal mode
                spi::put(Command{Write{}, 1, 0x14_B}); // clear / exponnetial
                spi::put(Command{Write{}, 0x40, 0x00_B}); // pwmall
                spi::put(Command{Write{}, 0x41, 0x00_B}); // irefall
                
                spi::flush();
            
                for(index_type i{};; ++i) {
                    pwm_out(i);
                    if (i.isTop()) break;
                }
                spi::flush();
                
                for(index_type i{};; ++i) {
                    current_out(i);
                    if (i.isTop()) break;
                }
                spi::flush();

                for(ggroup_t g{};; ++g) {
                    gradation_out(g);
                    if (g.isTop()) break;
                }
                spi::flush();
                
                registers.grad_cntl = 0xff_B; // continous
                for(ggroup_t g{};; ++g) {
                    gradationStart(g);
                    if (g.isTop()) break;
                }
                spi::flush();
                
                off();
            }
            
            
            enum class LedMode : uint8_t {Off = 0, On, Individual1, Individual2}; 
            
            static inline void out(const index_type i, const LedMode m) {
                const uint8_t offset = i.toInt() / 4;
                const uint8_t shift  = (i.toInt() % 4) * 2;
                
                const std::byte b    = std::byte{m} << shift;
                const std::byte mask = 0x03_B << shift;
                        
                ledouts[offset] = (ledouts[offset] & ~mask) | b;
                spi::put(Command{Write{}, uint8_t(2 + offset), ledouts[offset]});            
            }
            
            static inline void off() {
                for(uint8_t i{0}; auto& l : ledouts) {
                    l = 0x00_B;
                    spi::put(Command{Write{}, i, l});
                    ++i;            
                }
            }
           
            static inline void pwm_out(const index_type i) {
                spi::put(Command{Write{}, uint8_t(0x08 + i), registers.pwms[i]});            
            }
            static inline void pwm(const index_type i, const uint8_t pwm) {
                registers.pwms[i] = std::byte{pwm};
                pwm_out(i);
            }
        
            static inline void current_out(const index_type i) {
                spi::put(Command{Write{}, uint8_t(0x18 + i), registers.irefs[i]});            
            }
            static inline void current(const index_type i, const uint8_t iref) {
                registers.irefs[i] = std::byte{iref};
                current_out(i);
            }
            
            
            static inline void enableGradation_out() {
                spi::put(Command{Write{}, uint8_t(0x38), registers.grad_modes[0]}); //normal or gradiation
                spi::put(Command{Write{}, uint8_t(0x39), registers.grad_modes[1]}); //normal or gradiation
            }

            static inline void enableGradation(const index_type i, const bool on) {
                const uint8_t offset = i.toInt() / 8;
                const uint8_t shift  = i.toInt() % 8;
                const std::byte b    = on ? (0x01_B << shift) : 0x00_B;
                const std::byte mask = 0x01_B << shift;
                
                registers.grad_modes[offset] = (registers.grad_modes[offset] & ~mask) | b;
                enableGradation_out();
            }
            
            static inline void group_out(const uint8_t offset) {
                spi::put(Command{Write{}, uint8_t(0x3a + offset), registers.grad_grps[offset]});            
            }

            static inline void group(const index_type i, const ggroup_t g) {
                const uint8_t offset = i.toInt() / 4;
                const uint8_t shift  = (i.toInt() % 4) * 2;
                
                const std::byte b    = std::byte{g.toInt()} << shift;
                const std::byte mask = 0x03_B << shift;
                        
                registers.grad_grps[offset] = (registers.grad_grps[offset] & ~mask) | b;
                group_out(offset);
            }
        
            static inline void gradationStop(const ggroup_t g) {
                const std::byte b = 0x03_B << (g.toInt() * 2);
                registers.grad_cntl = (registers.grad_cntl & ~b);
                spi::put(Command{Write{}, 0x3e, registers.grad_cntl});            
            }
            static inline void gradationStart(const ggroup_t g) {
                const std::byte b = 0x03_B << (g.toInt() * 2);
                registers.grad_cntl = (registers.grad_cntl & ~b) | b;
                spi::put(Command{Write{}, 0x3e, registers.grad_cntl});            
            }

            static inline void gradation_out(const ggroup_t g) {
                spi::put(Command{Write{}, uint8_t(0x28 + g * 4), registers.ramps[g].rate});            
                spi::put(Command{Write{}, uint8_t(0x29 + g * 4), registers.ramps[g].step});                    
                spi::put(Command{Write{}, uint8_t(0x2a + g * 4), registers.ramps[g].hold});                    
                spi::put(Command{Write{}, uint8_t(0x2b + g * 4), registers.ramps[g].iref});            
            }
            
            static inline void gradationI(const ggroup_t g, const giref_t i) {
                registers.ramps[g].iref = std::byte{i.toInt()};
                gradation_out(g);
            }    
            static inline void gradationR(const ggroup_t g, const grrate_t r) {
                registers.ramps[g].rate = (registers.ramps[g].rate & 0xc0_B) | std::byte(r.toInt());
                gradation_out(g);
            }    
            static inline void gradationR(const ggroup_t g, const grupdn_t ud) {
                registers.ramps[g].rate = (registers.ramps[g].rate & 0x3f_B) | (std::byte(ud.toInt()) << 6);
                gradation_out(g);
            }    
            static inline void gradationR(const ggroup_t g, const grstep_t s) {
                registers.ramps[g].step = std::byte(s.toInt());
                gradation_out(g);
            }    
            static inline void gradationH(const ggroup_t g, const ghonoff_t h) {
                registers.ramps[g].hold = (registers.ramps[g].rate & 0x3f_B) | (std::byte(h.toInt()) << 6);
                gradation_out(g);
            }    
            static inline void gradationHon(const ggroup_t g, const ghtime_t o) {
                registers.ramps[g].hold = (registers.ramps[g].rate & 0xC7_B) | (std::byte(o.toInt()) << 3);
                gradation_out(g);
            }    
            static inline void gradationHoff(const ggroup_t g, const ghtime_t o) {
                registers.ramps[g].hold = (registers.ramps[g].rate & 0xF8_B) | std::byte(o.toInt());
                gradation_out(g);
            }    
            
            static inline void periodic() {
                spi::periodic();
            }
//        private:
            static inline std::array<std::byte, 4> ledouts{};
            static inline Registers& registers = NVM::data().registers;
        };
    }
}


template<uint8_t N, typename PCA, typename Proto, typename Term>
struct PcaAdapter {
    using pca = PCA;
    using index_t = typename PCA::index_type;
    using protocol_t = Proto;

//    protocol_t::_;
    
    static_assert(N <= 1);
    static inline constexpr uint8_t offset = N * 8;
    
    static_assert(index_t::Upper >= (RCSwitch::index_t::Upper + offset));
    
    static inline void reset() {
    }
    
    static inline void off() {
        PCA::off();
    }
    static inline void off(const RCSwitch::index_t i) {
        const index_t idx(i.toInt() + offset);
        etl::outl<Term>("off i: "_pgm, i);
        PCA::out(idx, PCA::LedMode::Off);
    }
    static inline void on(const RCSwitch::index_t i, const protocol_t::mode_t m) {
        static index_t li;
        static typename protocol_t::mode_t lm;
        
        const index_t idx(i.toInt() + offset);
        
        if ((li != idx) || (lm != m)) {
            etl::outl<Term>("on i: "_pgm, idx.toInt(), " m: "_pgm, m);
            li = idx;
            lm = m;
        }

        switch(m) {
        case 0:
            PCA::out(idx, PCA::LedMode::Off);
            break;
        case 1:
            PCA::out(idx, PCA::LedMode::On);
            break;
        case 2:
            PCA::out(idx, PCA::LedMode::Individual1);
            break;
        case 3:
            PCA::out(idx, PCA::LedMode::Individual2);
            break;
        default:
            PCA::out(idx, PCA::LedMode::On);
            break;
        }
    }
    
    using ggroup_t = etl::uint_ranged<uint8_t, 0, 3>;    
    using gparam_t = etl::uint_ranged<uint8_t, 0, 6>;    
    
    static inline void setGradation(const ggroup_t g, gparam_t p, const protocol_t::pvalue_t v) {
        if (p == 0) {
            const auto vv = etl::scaleTo<typename PCA::giref_t>(v);
            etl::outl<Term>("gr: "_pgm, g.toInt(), " i: "_pgm, vv);
            PCA::gradationI(g, vv);
        }
        else if (p == 1) {
            const auto vv = etl::scaleTo<typename PCA::grrate_t>(v);
            etl::outl<Term>("gr: "_pgm, g.toInt(), " r: "_pgm, vv);
            PCA::gradationR(g, vv);
        }
        else if (p == 2) {
            etl::outl<Term>("gr: "_pgm, g.toInt(), " up down: "_pgm, v);
            PCA::gradationR(g, typename PCA::grupdn_t{v});
        }
        else if (p == 3) {
            const auto vv = etl::scaleTo<typename PCA::grstep_t>(v);
            etl::outl<Term>("gr: "_pgm, g.toInt(), " step: "_pgm, vv);
            PCA::gradationR(g, vv);
        }
        else if (p == 4) {
            etl::outl<Term>("gr: "_pgm, g.toInt(), " h onoff: "_pgm, v);
            PCA::gradationH(g, typename PCA::ghonoff_t{v});
        }
        else if (p == 5) {
            etl::outl<Term>("gr: "_pgm, g.toInt(), " h on t: "_pgm, v);
            PCA::gradationHon(g, typename PCA::ghtime_t{v});
        }
        else if (p == 6) {
            etl::outl<Term>("gr: "_pgm, g.toInt(), " h off t: "_pgm, v);
            PCA::gradationHoff(g, typename PCA::ghtime_t{v});
        }
    }
    
    static inline void set(const RCSwitch::index_t i, const RCSwitch::param_t p, const protocol_t::pvalue_t v) {
        constexpr uint8_t scale = 256 / (protocol_t::pvalue_t::Upper + 1);
        static typename protocol_t::pvalue_t last;
        const index_t idx(i.toInt() + offset);
        
        if (v != last) {
            last = v;
            etl::outl<Term>("set i: "_pgm, idx.toInt(), " p: "_pgm, p, " v: "_pgm, v);
            
            if (p == protocol_t::rampParam1) { // ramp-up group 0/1
                const uint8_t rgroup = idx / 8;
                const uint8_t rparam = idx % 8;
                PCA::out(index_t{0}, PCA::LedMode::On);
                setGradation(ggroup_t{rgroup}, gparam_t{rparam}, v);
            }
            else if (p == protocol_t::rampParam2) { // ramp-up group 2/3
                const uint8_t rgroup = idx / 8 + 2;
                const uint8_t rparam = idx % 8;
                PCA::out(index_t{0}, PCA::LedMode::On);
                setGradation(ggroup_t{rgroup}, gparam_t{rparam}, v);
            }
            else if (p == protocol_t::pwm) {
                PCA::out(idx, PCA::LedMode::Individual1);
                const uint8_t vv = v * scale; 
                PCA::pwm(idx, vv);
                etl::outl<Term>(" pwm: "_pgm, vv);
            }
            else if (p == protocol_t::blink1Intervall) {
                PCA::out(idx, PCA::LedMode::On);
                const uint8_t vv = v * scale; 
                PCA::current(idx, vv);  
                etl::outl<Term>(" cur: "_pgm, vv);
            }
            else if (p == protocol_t::blink2Intervall) {
                PCA::out(idx, PCA::LedMode::On);
                if (v == 0) {
                    etl::outl<Term>(" grad: off"_pgm);
                    PCA::enableGradation(idx, false);
                }
                else {
                    const uint8_t g = std::min(uint8_t{4}, v.toInt()) - 1;
                    etl::outl<Term>(" grad g: "_pgm, g);
                    PCA::gradationStop(ggroup_t{g});
                    PCA::group(idx, ggroup_t{g});
                    PCA::enableGradation(idx, true);
                    etl::outl<Term>(" "_pgm, PCA::registers.ramps[g].iref, " "_pgm, PCA::registers.ramps[g].step, 
                            " "_pgm, PCA::registers.ramps[g].hold, " "_pgm, PCA::registers.ramps[g].rate, 
                            " "_pgm, PCA::registers.grad_cntl,
                                    " "_pgm, PCA::registers.grad_modes[0],
                            " "_pgm, PCA::registers.grad_modes[1],
                            " "_pgm, PCA::registers.grad_grps[0]
                                    );
                    PCA::gradationStart(ggroup_t{g});
                }
            }
            else if (p == protocol_t::maxCurrent) {
                etl::outl<Term>(" maxC: "_pgm, v);
                PCA::out(index_t{0}, PCA::LedMode::On);
                PCA::current(typename PCA::Current{v.toInt()});                
            }
            
        }
    }
};

template<typename BusDevs>
struct GlobalFSM {
    using bus_type = BusDevs::bus_type;
    using devs = bus_type::devs;
    using pca = devs::pca9745;
    
    using led = devs::scanLedPin;
    using nvm = devs::eeprom;
    
    using timer = BusDevs::systemTimer;
    
    using servo = BusDevs::servo;    
    using servo_pa = BusDevs::servo_pa;
    using value_t = servo_pa::value_type;
    using channel_t = servo_pa::channel_t;

    using search_t = etl::uint_ranged_circular<uint8_t, 0, nvm::data_t::channel_t::Upper>;
    
    using protocol_t = BusDevs::BusParam::proto_type;
    
    using term = BusDevs::terminal;
    using termDev = term::device_type;

    using actor0 = PcaAdapter<0, pca, protocol_t, term>;
    using actor1 = PcaAdapter<1, pca, protocol_t, term>;
    
    using input = External::Digital<BusDevs, Meta::List<actor0, actor1>>;
    
    inline static auto& data() {
        return nvm::data();
    }
    
    inline static void init(const bool invert) {
        led::inactivate();
        nvm::init();

        if constexpr(External::Bus::isIBus<bus_type>::value) {
            servo::template init<BaudRate<115200>>();
            servo::template txEnable<true>();
            servo::txPinEnable();
            etl::outl<term>("IB"_pgm);
        }
        else if constexpr(External::Bus::isSBus<bus_type>::value) {
            servo::template init<AVR::BaudRate<100000>, FullDuplex, true, 1>(); // 8E2
            if (invert) {
                servo::rxInvert(true);
                etl::outl<term>("SB I"_pgm);
            }
            else {
                etl::outl<term>("SB"_pgm);
            }
        }
        else {
            static_assert(std::false_v<bus_type>, "bus type not supported");
        }

        if (data().mMagic != BusDevs::magic) {
            data().mMagic = BusDevs::magic;
            data().clear();
            data().change();
            etl::outl<term>("e init"_pgm);
        }
        
        pca::init();
        input::init();
    }
    inline static void periodic() {
        pca::periodic();        
        servo::periodic();
        nvm::saveIfNeeded([&]{
            etl::outl<term>("ep s"_pgm);
        });
    }

    inline static constexpr auto learnTimeout = 4000_ms;
    inline static constexpr auto scanTimeout = 50_ms;
    
//    static_assert(learnTimeout > (scanTimeout * (ch_t::Upper + 1) * 2), "");
    
    static constexpr External::Tick<timer> learnTimeoutTicks{learnTimeout};
    static constexpr auto showTimeout = 2000_ms;
    static constexpr External::Tick<timer> initTimeoutTicks{100_ms};
    static constexpr External::Tick<timer> showTimeoutTicks{showTimeout};
    static constexpr External::Tick<timer> debugTimeoutTicks{500_ms};
    static constexpr External::Tick<timer> eepromTimeoutTicks{1000_ms};
    
    using blinker = External::Blinker2<led, timer, 100_ms, showTimeout>;
    using bcount_t = blinker::count_type;
    
    enum class State : uint8_t {Undefined, Init, Error, 
                                ShowBus,
                                SearchChannel, 
                                ShowAddress, LearnTimeout,
                                Run};
    
    inline static void ratePeriodic() {
        servo_pa::ratePeriodic();
        blinker::ratePeriodic();
        input::periodic();
        
        (++mEepromTick).on(eepromTimeoutTicks, []{
            nvm::data().expire();
        });
        
        const auto oldState = mState;
        
        (++mDebugTick).on(debugTimeoutTicks, []{
//            etl::outl<term>("lv: "_pgm, input::lchv);   
//            etl::outl<term>("lp: "_pgm, input::lpp, " lv: "_pgm, input::lpv);   
        });
        
        ++mStateTick;
        switch(mState) {
        case State::Undefined:
            mStateTick.on(initTimeoutTicks, []{
                mState = State::Init;
            });
            break;
        case State::Init:
            mStateTick.on(initTimeoutTicks, []{
                mState = State::ShowBus;
            });
            break;
        case State::ShowBus:
            mStateTick.on(showTimeoutTicks, []{
                mState = State::SearchChannel;
            });
            break;
        case State::SearchChannel:
            if (search()) {
                mState = State::ShowAddress;
            }
            mStateTick.on(learnTimeoutTicks, []{
                mState = State::LearnTimeout;
            });
            break;
        case State::ShowAddress:
            mStateTick.on(showTimeoutTicks, []{
                mState = State::Run;
            });
            break;
        case State::LearnTimeout:
            etl::outl<term>("timeout ch: "_pgm, nvm::data().mChannel.toInt(), " adr: "_pgm, nvm::data().mAddress.toInt());
            if (data().mChannel && data().mAddress) {
                mState = State::Run;
            }
            else {
                mState = State::Error;
            }
            break;
        case State::Error:
            break;
        case State::Run:
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                blinker::off();
                break;
            case State::ShowBus:
                if constexpr(External::Bus::isIBus<bus_type>::value) {
                    etl::outl<term>("S Ibu"_pgm);
                    blinker::onePeriod(bcount_t{1});
                }
                else if constexpr(External::Bus::isSBus<bus_type>::value) {
                    etl::outl<term>("S Sbu"_pgm);
                    blinker::onePeriod(bcount_t{2});
                }
                break;
            case State::SearchChannel:
                blinker::blink(bcount_t{5});
                etl::outl<term>("S Sea"_pgm);
                break;
            case State::ShowAddress:
                blinker::blink(bcount_t{data().mAddress.toInt()});
                etl::outl<term>("S Sea"_pgm);
                break;
            case State::LearnTimeout:
                break;
            case State::Error:
                blinker::blink(bcount_t{9});
                etl::outl<term>("S Sea"_pgm);
                break;    
            case State::Run:
                blinker::off();
                etl::outl<term>("S Run"_pgm);
                break;
            }
        }
    }
    
private:
    inline static search_t learnChannel{search_t::Upper};
    static inline External::Tick<timer> mDebugTick{};
    static inline External::Tick<timer> mEepromTick{};
    static inline External::Tick<timer> mStateTick{};
    static inline State mState{State::Undefined};

    static inline bool search() {
        const channel_t ch = learnChannel.template to<channel_t>();
        if (const auto lc = servo_pa::valueMapped(ch); lc && protocol_t::isLearnCode(lc)) {
            if (const auto pv = protocol_t::toParameterValue(lc).toInt(); (pv >= 1) && ((pv - 1) <= RCSwitch::addr_t::Upper)) {
                const uint8_t addr = pv - 1;
                data().mChannel = learnChannel;
                data().mAddress = addr;
                data().change();
                return true;
            }
        }   
        --learnChannel;
        return false;
    }

};

template<typename Regs>
struct Data final : public EEProm::DataBase<Data<Regs>> {
    uint8_t mMagic{};
    
    using channel_t = etl::uint_ranged_NaN<uint8_t, 0, 15>;
    channel_t mChannel{};
    
    using addr_t = etl::uint_ranged_NaN<uint8_t, 0, RCSwitch::addr_t::Upper>;
    addr_t mAddress{};
    
    Regs registers{};
    
    inline void clear() {
        mChannel = 14;
        mAddress = 0;
//        mChannel.setNaN();
//        mAddress.setNaN();

//        registers.grad_cntl = 0xff_B;
        
        registers.ramps[0].rate = 0xc1_B; // ramp up down, step = 1
        registers.ramps[0].step = 0x04_B; // 0,5ms x 5
        registers.ramps[0].hold = 0xd2_B; // hold on/off, 0,5s 0,5s
        registers.ramps[0].iref = 200_B;  // Iref = 200

        registers.ramps[1].rate = 0xc1_B; // ramp up down
        registers.ramps[1].step = 0x02_B; // 0,5ms * 2
        registers.ramps[1].hold = 0xc9_B; // hold on/off , 0,25s, 0,25s
        registers.ramps[1].iref = 200_B;

        registers.ramps[2].rate = 0x81_B; // ramp up, step = 1
        registers.ramps[2].step = 0x44_B; // 8ms  * 4
        registers.ramps[2].hold = 0xe0_B; // hold on, 1s hold off, 0s 
        registers.ramps[2].iref = 200_B;
        
        registers.ramps[3].rate = 0x41_B; // ramp down, step = 1
        registers.ramps[3].step = 0x48_B; // 8ms * 8
        registers.ramps[3].hold = 0xc5_B; // hold off, 2s
        registers.ramps[3].iref = 200_B;
        
        for(auto& pwm: registers.pwms) {
            pwm = 0x0f_B;
        }
        for(auto& pwm: registers.irefs) {
            pwm = 0x0f_B;
        }
        
        registers.grad_modes[0] = 0x00_B;
        registers.grad_modes[1] = 0x00_B;
        
        registers.grad_grps[0] = 0x00_B;
        registers.grad_grps[1] = 0x00_B;
        registers.grad_grps[2] = 0x00_B;
        registers.grad_grps[3] = 0x00_B;
        
//        registers.grad_cntl = 0xff_B;
    }
};

template<typename HWRev = void, typename MCU = DefaultMcuType>
struct Devices {
    using ccp = Cpu::Ccp<>;
    using clock = Clock<>;
    using sigrow = SigRow<>;
    
    using iref1Pin = Pin<Port<A>, 6>;
    using iref2Pin = Pin<Port<A>, 7>;
    
    using csPin = Pin<Port<B>, 0>;
    using oePin = Pin<Port<B>, 1>;
    
    using ledPin = Pin<Port<A>, 4>;
    
    using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;
    
    using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>; 
    using servoPosition = usart0Position;
    
    using scanDevPosition = usart0Position;
    using ppmDevPosition = void;
    using evrouter = void;
    using scan_term_dev = void;
    using scanLedPin = AVR::ActiveHigh<ledPin, Output>;
    
    using spiPosition = Portmux::Position<Component::Spi<0>, Portmux::Default>;
    
    template<typename Command>
    using spi = AVR::SpiSS<spiPosition, Command, AVR::QueueLength<128>, csPin>;
    
    using portmux = Portmux::StaticMapper<Meta::List<usart0Position, spiPosition>>;

    using eeprom = EEProm::Controller<Data<External::PCA9745::Registers>>;
    
    using pca9745 = External::PCA9745::Controller<spi, oePin, iref1Pin, iref2Pin, eeprom>;
    
//    std::integral_constant<uint16_t, sizeof(typename pca9745::Registers)>::_;
    
    static inline void init() {
        using portmux = Portmux::StaticMapper<Meta::List<usart0Position, spiPosition>>; 
        portmux::init();
        ccp::unlock([]{
            clock::template prescale<1>();
        });
        systemTimer::init(); 
    }
    static inline void periodic() {}
};

template<typename Bus>
struct BusDevs;

template<typename Devs>
struct BusDevs<External::Bus::IBusIBus<Devs>> {
    inline static constexpr uint8_t magic{42};
    
    using bus_type = External::Bus::IBusIBus<Devs>;
    
    struct BusParam {
        using bus_t = bus_type;
        using proto_type = RCSwitch::Protocol2<RCSwitch::High>;
    };
    
    using systemTimer = Devs::systemTimer;
    
    using servo_pa = IBus2::Servo::ProtocollAdapter<0>;
    using servo = Usart<typename Devs::servoPosition, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
    
    using terminal = etl::basic_ostream<servo>;
    
    using eeprom = Devs::eeprom;
};

template<typename Devs>
struct BusDevs<External::Bus::SBusSPort<Devs>> {
    inline static constexpr uint8_t magic{43};
    using bus_type = External::Bus::SBusSPort<Devs>;
    
    struct BusParam {
        using bus_t = bus_type;
        using proto_type = RCSwitch::Protocol2<RCSwitch::Low>;
    };
    
    using systemTimer = Devs::systemTimer;
    
    using servo_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer>;
    using servo = Usart<typename Devs::servoPosition, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;
    
    using terminal = etl::basic_ostream<servo>;

    using eeprom = Devs::eeprom;
};

template<typename BusSystem, typename MCU = DefaultMcuType>
struct Application {
    using devs = BusDevs<BusSystem>;
    
    inline static void run(const bool inverted = false) {
        if constexpr(External::Bus::isIBus<BusSystem>::value || External::Bus::isSBus<BusSystem>::value) {
            using terminal = devs::terminal;
            using systemTimer = devs::systemTimer;
            using gfsm = GlobalFSM<devs>;
            
            gfsm::init(inverted);
            etl::outl<terminal>("test20"_pgm);
            while(true) {
                gfsm::periodic(); 
                systemTimer::periodic([&]{
                    gfsm::ratePeriodic();
                });
            }
        }
    }
};

using devices = Devices<>;
using scanner = External::Scanner2<devices, Application, Meta::List<External::Bus::IBusIBus<devices>, External::Bus::SBusSPort<devices>>>;

int main() {
    scanner::run();
}

#ifndef NDEBUG
[[noreturn]] inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], const unsigned int line [[maybe_unused]]) noexcept {
    //    etl::outl<terminal>("Assertion failed: "_pgm, expr, etl::Char{','}, file, etl::Char{','}, line);
    while(true) {
        //        assertPin::toggle();
    }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, const unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
