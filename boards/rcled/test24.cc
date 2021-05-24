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
#include <std/tuple>

#include <etl/output.h>
#include <etl/meta.h>

#include "generic.h"
#include "pca9745.h"

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

namespace  {
    constexpr auto fRtc = 1000_Hz;
}

template<typename PCA, typename Timer, typename Term>
struct PatternGenerator {
    static inline External::PCA9745::Registers::Ramp ramp = {.rate = 0x01_B, // up = 0x81 down = 0x41
                                                             .step = 0x01_B, // 0,5ms * 4
                                                             .hold = 0x00_B, // no hold 
                                                             .iref = 255_B,
                                                            };
    static inline void setGradationUp(External::PCA9745::Registers::Ramp& r) {
        r.rate = (r.rate & ~0xc0_B) | 0x80_B;
    }
    static inline void setGradationDown(External::PCA9745::Registers::Ramp& r) {
        r.rate = (r.rate & ~0xc0_B) | 0x40_B;
    }
    static inline void setGradationStop(External::PCA9745::Registers::Ramp& r) {
        r.rate = (r.rate & ~0xc0_B) | 0x00_B;
    }
    
    
    static constexpr External::Tick<Timer> seqTimeoutTicks{75_ms};
    
    enum class State : uint8_t {Normal, Sequence};
    
    static inline void out(const PCA::index_type idx, const PCA::LedMode m) {
        if (mState == State::Normal) {
            PCA::out(idx, m);
        }
    }
    
    enum class Event : uint8_t {None, StartSequence, StopSequence};
    
    static inline void send(const Event e) {
        mEvent = e;
    }
    
    static inline void ratePeriodic() {
        const auto oldState = mState;
        ++mStateTick;
        ++mSeqTick;
        switch(mState) {
        case State::Normal:
            if (event() == Event::StartSequence) {
                mState = State::Sequence;
            }
            break;
        case State::Sequence:
            if (event() == Event::StopSequence) {
                mState = State::Normal;
            }
            mSeqTick.on(seqTimeoutTicks, []{
                sequenceNext();
            });
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Normal:
                break;
            case State::Sequence:
                initSequence();
                break;
            }
        }
    }
private:
    using ggroup_t = PCA::ggroup_t;
    using led_idx_t = etl::uint_ranged_circular<uint8_t, 0, 7>;
    using phase_t = etl::uint_ranged_circular<uint8_t, 0, 3>;

    static inline led_idx_t led;    
    static inline led_idx_t led0;    
    static inline led_idx_t led1;    
    static inline led_idx_t led2;    
    static inline led_idx_t led3;    
    static inline phase_t   phase;    
    
    static inline void sequenceNext() {
        switch(phase) {
        case 0:
            PCA::out(led0.toRanged(), PCA::LedMode::Off);
            PCA::out(led.toRanged(), PCA::LedMode::On);
            PCA::enableGradation(led0.toRanged(), false);
            PCA::enableGradation(led.toRanged(), true);
            led0 = led;

            PCA::group(led.toRanged(), ggroup_t{0});
            setGradationUp(ramp);
            PCA::pushGradation(ggroup_t{0, etl::RangeCheck<false>{}}, ramp);        

            setGradationStop(ramp);
            PCA::pushGradation(ggroup_t{1, etl::RangeCheck<false>{}}, ramp);        
            
            setGradationDown(ramp);
            PCA::pushGradation(ggroup_t{2, etl::RangeCheck<false>{}}, ramp);        

            setGradationStop(ramp);
            PCA::pushGradation(ggroup_t{3, etl::RangeCheck<false>{}}, ramp);        
            break;
        case 1:
            PCA::out(led1.toRanged(), PCA::LedMode::Off);
            PCA::out(led.toRanged(), PCA::LedMode::On);
            PCA::enableGradation(led1.toRanged(), false);
            PCA::enableGradation(led.toRanged(), true);
            led1 = led;

            setGradationStop(ramp);
            PCA::pushGradation(ggroup_t{0, etl::RangeCheck<false>{}}, ramp);        
            
            PCA::group(led.toRanged(), ggroup_t{1});
            setGradationUp(ramp);
            PCA::pushGradation(ggroup_t{1, etl::RangeCheck<false>{}}, ramp);        
            
            setGradationStop(ramp);
            PCA::pushGradation(ggroup_t{2, etl::RangeCheck<false>{}}, ramp);        

            setGradationDown(ramp);
            PCA::pushGradation(ggroup_t{3, etl::RangeCheck<false>{}}, ramp);        
            break;
        case 2:
            PCA::out(led2.toRanged(), PCA::LedMode::Off);
            PCA::out(led.toRanged(), PCA::LedMode::On);
            PCA::enableGradation(led2.toRanged(), false);
            PCA::enableGradation(led.toRanged(), true);
            led2 = led;

            setGradationDown(ramp);
            PCA::pushGradation(ggroup_t{0, etl::RangeCheck<false>{}}, ramp);        

            setGradationStop(ramp);
            PCA::pushGradation(ggroup_t{1, etl::RangeCheck<false>{}}, ramp);        
            
            PCA::group(led.toRanged(), ggroup_t{2});
            setGradationUp(ramp);
            PCA::pushGradation(ggroup_t{2, etl::RangeCheck<false>{}}, ramp);        

            setGradationStop(ramp);
            PCA::pushGradation(ggroup_t{3, etl::RangeCheck<false>{}}, ramp);        
            break;
        case 3:
            PCA::out(led3.toRanged(), PCA::LedMode::Off);
            PCA::out(led.toRanged(), PCA::LedMode::On);
            PCA::enableGradation(led3.toRanged(), false);
            PCA::enableGradation(led.toRanged(), true);
            led3 = led;

            setGradationStop(ramp);
            PCA::pushGradation(ggroup_t{0, etl::RangeCheck<false>{}}, ramp);        

            setGradationDown(ramp);
            PCA::pushGradation(ggroup_t{1, etl::RangeCheck<false>{}}, ramp);        

            setGradationStop(ramp);
            PCA::pushGradation(ggroup_t{2, etl::RangeCheck<false>{}}, ramp);        
            
            PCA::group(led.toRanged(), ggroup_t{3});
            setGradationUp(ramp);
            PCA::pushGradation(ggroup_t{3, etl::RangeCheck<false>{}}, ramp);        
            break;
        }        
        PCA::gradationStartSingle(0b1010'1010_B);
        
        ++led;
        ++phase;
    }

    static inline void initSequence() {
        etl::outl<Term>("is"_pgm);
        
        PCA::gradationStop();
        PCA::pushGradation(ggroup_t{0, etl::RangeCheck<false>{}}, ramp);        
        PCA::pushGradation(ggroup_t{1, etl::RangeCheck<false>{}}, ramp);        
        PCA::pushGradation(ggroup_t{2, etl::RangeCheck<false>{}}, ramp);        
        PCA::pushGradation(ggroup_t{3, etl::RangeCheck<false>{}}, ramp);        
    }
    
    static inline Event event() {
        Event e{Event::None};
        swap(mEvent, e);
        return e;
    }
    
    static inline Event mEvent{Event::None};
    static inline External::Tick<Timer> mStateTick{};
    static inline External::Tick<Timer> mSeqTick{};
    static inline State mState{State::Normal};
};

template<uint8_t N, typename PCA, typename PatGen, typename Proto, typename Term>
struct PcaAdapter {
    using pca = PCA;
    using index_t = typename PCA::index_type;
    using protocol_t = Proto;

    static_assert(N <= 1);
    static inline constexpr uint8_t offset = N * 8;
    
    static_assert(index_t::Upper >= (RCSwitch::index_t::Upper + offset));
    
    static inline void reset() {}
    
    static inline void off() {
        PCA::off();
    }
    static inline void off(const RCSwitch::index_t i) {
        const index_t idx(i.toInt() + offset);
        PatGen::out(idx, PCA::LedMode::Off);
    }

    static inline void on(const RCSwitch::index_t i, const protocol_t::mode_t m) {
        const index_t idx(i.toInt() + offset);
        
        switch(m) {
        case 0:
            PatGen::out(idx, PCA::LedMode::Off);
            break;
        case 1:
            PatGen::out(idx, PCA::LedMode::On);
            break;
        case 2:
            PatGen::out(idx, PCA::LedMode::Individual1);
            break;
        case 3:
            PatGen::out(idx, PCA::LedMode::Individual2);
            break;
        default:
            PatGen::out(idx, PCA::LedMode::On);
            break;
        }
    }
    
    using ggroup_t = etl::uint_ranged<uint8_t, 0, 3>;    
    using gparam_t = etl::uint_ranged<uint8_t, 0, 6>;    
    
    static inline void setGradation(const ggroup_t g, gparam_t p, const protocol_t::pvalue_t v) {
        if (p == 0) {
            const auto vv = etl::scaleTo<typename PCA::giref_t>(v);
            PCA::gradationI(g, vv);
        }
        else if (p == 1) {
            const auto vv = etl::scaleTo<typename PCA::grrate_t>(v);
            PCA::gradationR(g, vv);
        }
        else if (p == 2) {
            PCA::gradationR(g, typename PCA::grupdn_t{v});
        }
        else if (p == 3) {
            const auto vv = etl::scaleTo<typename PCA::grstep_t>(v);
            PCA::gradationR(g, vv);
        }
        else if (p == 4) {
            PCA::gradationH(g, typename PCA::ghonoff_t{v});
        }
        else if (p == 5) {
            PCA::gradationHon(g, typename PCA::ghtime_t{v});
        }
        else if (p == 6) {
            PCA::gradationHoff(g, typename PCA::ghtime_t{v});
        }
    }

    template<typename T>
    static inline constexpr bool equalsAndSet(T& a, const T& b) {
        if (b == a) {
            return true;
        }
        a = b;
        return false;
    }

    struct ParamTuple {
        RCSwitch::index_t i;
        RCSwitch::param_t p;
        protocol_t::pvalue_t v;
        bool operator==(const ParamTuple& b) {
            return (i == b.i) && (p == b.p) && (v == b.v);
        }
    };

    static inline void set(const RCSwitch::index_t i, const RCSwitch::param_t p, const protocol_t::pvalue_t v) {
        constexpr uint8_t scale = 256 / (protocol_t::pvalue_t::Upper + 1);
        const index_t idx(i.toInt() + offset);
        
//        using tp_t = std::tuple<RCSwitch::index_t, RCSwitch::param_t, typename protocol_t::pvalue_t>; 
//        static tp_t last;
//        if (equalsAndSet(last, tp_t{i, p, v})) return;
        
        static ParamTuple last;
        if (equalsAndSet(last, ParamTuple{i, p, v})) return;
        
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
        }
        else if (p == protocol_t::blink1Intervall) {
            PCA::out(idx, PCA::LedMode::On);
            const uint8_t vv = v * scale; 
            PCA::current(idx, vv);  
        }
        else if (p == protocol_t::blink2Intervall) {
            PCA::out(idx, PCA::LedMode::On);
            if (v == 0) {
                PCA::enableGradation(idx, false);
            }
            else {
                const uint8_t g = std::min(uint8_t{4}, v.toInt()) - 1;
                PCA::gradationStop(ggroup_t{g});
                PCA::group(idx, ggroup_t{g});
                PCA::enableGradation(idx, true);
                PCA::gradationStart(ggroup_t{g});
            }
        }
        else if (p == protocol_t::maxCurrent) {
            PCA::out(index_t{0}, PCA::LedMode::On);
            PCA::current(External::PCA9745::Current{v.toInt()});                
        }
        else if (p == protocol_t::testMode) {
            if (v == 0) {
                PatGen::send(PatGen::Event::StopSequence);                
            }
            else {
                PatGen::send(PatGen::Event::StartSequence);
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

    using patg = PatternGenerator<pca, timer, term>;
    
    using actor0 = PcaAdapter<0, pca, patg, protocol_t, term>;
    using actor1 = PcaAdapter<1, pca, patg, protocol_t, term>;
    
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
        }
        else if constexpr(External::Bus::isSBus<bus_type>::value) {
            servo::template init<AVR::BaudRate<100000>, FullDuplex, true, 1>(); // 8E2
            if (invert) {
                servo::rxInvert(true);
            }
        }
        else {
            static_assert(std::false_v<bus_type>, "bus type not supported");
        }

        if (data().mMagic != BusDevs::magic) {
            data().mMagic = BusDevs::magic;
            data().clear();
            data().change();
        }
        
        pca::init();
        input::init();
    }
    inline static void periodic() {
        pca::periodic();        
        servo::periodic();
        nvm::saveIfNeeded([&]{
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
        patg::ratePeriodic();
        blinker::ratePeriodic();
        input::periodic();
        
        (++mEepromTick).on(eepromTimeoutTicks, []{
            nvm::data().expire();
        });
        
        const auto oldState = mState;
        
        (++mDebugTick).on(debugTimeoutTicks, []{
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
                    blinker::onePeriod(bcount_t{1});
                }
                else if constexpr(External::Bus::isSBus<bus_type>::value) {
                    blinker::onePeriod(bcount_t{2});
                }
                break;
            case State::SearchChannel:
                blinker::blink(bcount_t{5});
                break;
            case State::ShowAddress:
                blinker::blink(bcount_t{data().mAddress.toInt()});
                break;
            case State::LearnTimeout:
                break;
            case State::Error:
                blinker::blink(bcount_t{9});
                break;    
            case State::Run:
                blinker::off();
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

template<typename Regs, typename Current>
struct Data final : public EEProm::DataBase<Data<Regs, Current>> {
    uint8_t mMagic{};
    
    using channel_t = etl::uint_ranged_NaN<uint8_t, 0, 15>;
    channel_t mChannel{};
    
    using addr_t = etl::uint_ranged_NaN<uint8_t, 0, RCSwitch::addr_t::Upper>;
    addr_t mAddress{};
    
    Regs registers{};

    Current current{};
    
    inline void clear() {
        mChannel = 14;
        mAddress = 0;
//        mChannel.setNaN();
//        mAddress.setNaN();

        current = Current::C1;
        
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
        
        registers.grad_cntl = 0xff_B;
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

    using eeprom = EEProm::Controller<Data<External::PCA9745::Registers, External::PCA9745::Current>>;
    
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
            etl::outl<terminal>("test23"_pgm);
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
