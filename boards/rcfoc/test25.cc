#define NDEBUG

#define LEARN_DOWN

#ifndef GITMAJOR
# define VERSION_NUMBER 0001
#endif

#ifndef NDEBUG
static unsigned int assertKey{1234};
#endif

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
#include <mcu/internals/syscfg.h>

#include <mcu/pgm/pgmarray.h>

#include <external/hal/adccontroller.h>
#include <external/hal/alarmtimer.h>
#include <external/bluetooth/roboremo.h>
#include <external/bluetooth/qtrobo.h>
#include <external/sbus/sbus.h>
#include <external/sbus/sport.h>
#include <external/ibus/ibus2.h>
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
#include <external/solutions/series01/sppm_in.h>
#include <external/solutions/rc/busscan.h>

#include <std/chrono>
#include <stdlib.h>

#include <etl/output.h>
#include <etl/meta.h>
#include <etl/control.h>

#include "driver.h"
#include "encoder.h"
#include "devices.h"
#include "busses.h"
#include "link.h"

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

#ifndef NDEBUG
namespace xassert {
    etl::StringBuffer<160> ab;
    etl::StringBuffer<10> aline;
    bool on{false};
}
#endif

namespace  {
    constexpr auto fRtc = 1000_Hz;
}

template<typename T, auto L, auto U>
constexpr auto cyclic_diff(const etl::uint_ranged<T, L, U>& a, const etl::uint_ranged<T, L, U>& b) {
    using diff_t = std::make_signed_t<T>;
    constexpr diff_t adelta2 = (U - L) / 2;
    constexpr diff_t adelta  = (U - L);
    
    using result_t = etl::int_ranged<diff_t, -adelta2, adelta2>; 
    
//    std::integral_constant<diff_t, adelta2>::_;
    
    const diff_t as(a);
    const diff_t bs(b);
    
    const diff_t diff = as - bs;
    if (diff > 0) {
        if (diff > adelta2) {
            return result_t{as - (bs + adelta)};
        }
        else {
            return result_t{as - bs};
        }
    }
    else {
        if (diff < -adelta2) {
            return result_t{as - (bs - adelta)};
        }
        else {
            return result_t{as - bs};
        }
    }
}

template<typename BusDevs>
struct GlobalFsm {
    using gfsm = GlobalFsm<BusDevs>;
    
    using devs = BusDevs::devs;
    using Timer = devs::systemTimer;
    
    using blinkLed = External::Blinker2<typename devs::led, Timer, 100_ms, 2500_ms>;
    using count_type = blinkLed::count_type;
    
    using enable = devs::enable;
    
    using dbg = devs::dbgPin;
    
    using encoder = devs::encoder;
    using m_angle_t = encoder::angle_type;
    
    using m_diff_t = decltype(cyclic_diff(m_angle_t{}, m_angle_t{}));
    using m_absdiff_t = decltype(cyclic_diff(m_angle_t{}, m_angle_t{}).rabs());
//    m_diff_t::_;
//    m_absdiff_t::_;

    using driver = Driver<typename devs::pwm, std::integral_constant<size_t, 2048>, std::integral_constant<uint8_t, 11>, m_absdiff_t>;
    using e_angle_t = driver::angle_type;
    
    static inline constexpr m_angle_t zeroAngle{1023};
    
    using sout = BusDevs::sbusGen;

    using link_dev = BusDevs::link_dev;
    using link = BusDevs::link;
    using link_pa = BusDevs::link_pa;
    
    using lut3 = devs::lut3;
    using Adc = devs::adcController;
    
    using terminal = BusDevs::terminal;
    using TermDev = terminal::device_type;
    
    using adc_i_t = Adc::index_type;
    
    using sbus_value_t = typename sout::value_type;
    using sbus_index_t = typename sout::index_type;
    
    enum class State : uint8_t {Undefined, Init, Enable,
                                CheckCurrentStart, CheckCurrentEnd,
                                CheckDirStart, CheckDirEnd, CheckPolesStart, CheckPolesEnd,
                                ZeroStart, ZeroEnd, 
                                Run, Error};
    
    static constexpr External::Tick<Timer> startupTicks{100_ms};
    static constexpr External::Tick<Timer> enableTicks{500_ms};
    static constexpr External::Tick<Timer> debugTicks{300_ms};
    static constexpr External::Tick<Timer> zeroGTicks{5000_ms};
    static constexpr External::Tick<Timer> zeroTicks{10_ms};
    
    static inline void init() {
        TermDev::template init<AVR::BaudRate<115200>>();
        
        blinkLed::init();
        enable::init();
        
        Adc::template init<false>(); // no pullups
        Adc::mcu_adc_type::nsamples(4);
        
        encoder::init();
        driver::init();
        
        sout::init();

        link_dev::template init<AVR::BaudRate<115200>>();
        
        dbg::template dir<Output>();
    }
    
    static inline void periodic() {
        TermDev::periodic();
        link_dev::periodic();
        Adc::periodic();
        
        encoder::periodic();
        driver::periodic();
        
        Adc::periodic();
        
        encoder::template read<true>(); // increased time
//        encoder::read(); 
        
        sout::periodic();
        
    }
    
    static inline void ratePeriodic() {
        dbg::toggle();
        
        driver::ratePeriodic();
        sout::ratePeriodic();
        blinkLed::ratePeriodic();
        
        const auto oldState = mState;
        ++mStateTick;
        
        const m_angle_t angle = encoder::angle();
        
        link::template set<0>(etl::nth_byte<0>(angle.toInt()));
        link::template set<1>(etl::nth_byte<1>(angle.toInt()));
        link::template send<link_dev>();

        uint16_t lv = uint8_t(link_pa::data()[0]);
        lv |= uint16_t(link_pa::data()[1]) << 8;
        const m_angle_t la{lv};

        const auto analog_i = Adc::value(adc_i_t{0});
        const auto analog_v = Adc::value(adc_i_t{1});
        const auto analog_te = Adc::value(adc_i_t{2});
        const auto analog_ti = Adc::value(adc_i_t{4});
        
        (++mDebugTick).on(debugTicks, [&]{
            etl::outl<terminal>("ma: "_pgm, angle.toInt(), " eo: "_pgm, eoffset.toInt(), " l: "_pgm, lv,
                                " i: "_pgm, analog_i, " v: "_pgm, analog_v, " te: "_pgm, analog_te, " ti: "_pgm, analog_ti,
                                " cs: "_pgm, checkStart, " ce: "_pgm, checkEnd, " ld: "_pgm, ld, " ccs: "_pgm, currStart,
                                " c: "_pgm, c);
            etl::outl<terminal>(mPid);
        });
        
        switch(mState) {
        case State::Undefined:
            mStateTick.on(startupTicks, []{
                mState = State::Init;
            });
            break;
        case State::Init:
            mStateTick.on(enableTicks, []{
                mState = State::Enable;
            });
            break;
        case State::Enable:
            mStateTick.on(enableTicks, []{
                mState = State::CheckCurrentStart;
                currStart.setToBottom();
            });
            break;
        case State::CheckCurrentStart:
            ++eoffset;
            driver::angle(eoffset);
            driver::scale(currStart);
            if (eoffset > (3 * driver::electric_convolution)) {
                mState = State::CheckCurrentEnd;
            }
            break;
        case State::CheckCurrentEnd:
            if (const auto d = cyclic_diff(checkStart, checkEnd); d.rabs() >= (2 * m_absdiff_t::Upper / driver::poles)) {
                ld = d.rabs();
                mState = State::CheckDirStart;
                currEnd = currStart;
            }
            else {
                ld = d.rabs();
                currStart += m_absdiff_t::Upper / 10;
                mState = State::CheckCurrentStart;
            }
            break;
        case State::CheckDirStart:
            ++eoffset;
            driver::angle(eoffset);
            if (eoffset == driver::electric_convolution) {
                mState = State::CheckDirEnd;
            }
            break;
        case State::CheckDirEnd:
            if (const auto diff = cyclic_diff(checkEnd, checkStart); diff > 0) {
                mState = State::ZeroStart;
            }
            else {
                mState = State::Error;
            }
            break;
        case State::CheckPolesStart:
            ++eangle;
            driver::angle(eangle);
            if (eangle == eoffset) {
                mState = State::CheckPolesEnd;
            }
            break;
        case State::CheckPolesEnd:
            if (const auto d = cyclic_diff(m_angle_t{0}, angle); abs(d) < 10) {
                mState = State::Run;
            }
            else {
                mState = State::Error;
            }
            break;
        case State::ZeroStart:
            ++eoffset;
            driver::angle(eoffset);
            if (const auto d = cyclic_diff(m_angle_t{0}, angle); d.rabs() < 10) {
                eoffset = driver::normalizedOffset();
                mState = State::ZeroEnd;
            }
            break;
        case State::ZeroEnd:
            mStateTick.on(zeroGTicks, []{
                mState = State::CheckPolesStart;
            });
            (++mSubStateTick).on(zeroTicks, [&]{
                ++c;
                if (const auto d = cyclic_diff(angle, m_angle_t{0}); d < 0) {
                    ld = d;
                    ++eoffset;
                }
                else if (d > 0) {
                    ld = d;
                    --eoffset;
                }
                else {
                    ld = d;
                }
                driver::angle(eoffset);
            });
            break;
        case State::Run:
        {
            m_angle_t target{la};
            
            auto d = cyclic_diff(target, angle);
            d *= 3;

//            const auto cv = mPid.correctionValue(d.toInt(), 0);
            
//            driver::torque(m_diff_t{cv}, angle, eoffset);
//            driver::scale(m_absdiff_t{abs(cv)});
            
            
            const auto da = d.rabs();
//            const m_absdiff_t ccv{da + currEnd};
            const m_absdiff_t ccv{da + (3 * currEnd) / 4};
            ld = ccv;

            driver::torque(d, angle, eoffset);
            driver::scale(ccv);
            
            const auto sb0 = AVR::Pgm::scaleTo<sbus_value_t>(angle);
            const auto sb1 = AVR::Pgm::scaleTo<sbus_value_t>(la);
            sout::set(sbus_index_t{0}, sb0);
            sout::set(sbus_index_t{1}, sb1);
        }
            break;
        case State::Error:
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                etl::outl<terminal>("S: Init"_pgm);
                blinkLed::blink(count_type{1});
                enable::activate();
                break;
            case State::Enable:
                etl::outl<terminal>("S: Enable"_pgm);
                break;
            case State::CheckCurrentStart:
                eoffset.setToBottom();
                checkStart = angle;
                etl::outl<terminal>("S: CCS"_pgm);
                break;
            case State::CheckCurrentEnd:
                checkEnd = angle;
                etl::outl<terminal>("S: CCE"_pgm);
                break;
            case State::CheckDirStart:
                etl::outl<terminal>("S: CDS"_pgm);
                driver::scale(currEnd);
                checkStart = angle;
                eoffset.setToBottom();
                break;
            case State::CheckDirEnd:
                etl::outl<terminal>("S: CDE"_pgm);
                checkEnd = angle;
                break;
            case State::CheckPolesStart:
                eangle = eoffset;
                checkStart = angle;
//                driver::scale(currEnd);
                etl::outl<terminal>("S: CPS"_pgm);
                break;
            case State::CheckPolesEnd:
                checkEnd = angle;
                etl::outl<terminal>("S: CPE"_pgm);
                break;
            case State::ZeroStart:
                driver::scale(m_absdiff_t{currEnd + currEnd});
                etl::outl<terminal>("S: ZS"_pgm);
                break;
            case State::ZeroEnd:
                etl::outl<terminal>("S: ZE"_pgm);
                break;
            case State::Run:
                etl::outl<terminal>("S: Run"_pgm);
                break;
            case State::Error:
                etl::outl<terminal>("S: Error"_pgm);
                break;
            }
        }
    }
private:
    static inline Control::PID<int16_t, float> mPid{2.0, 0.005, 9.0, 500, 500};
    
    static inline uint16_t c{0};
    static inline int16_t ld{0};
    
    static inline m_absdiff_t currStart{0};
    static inline m_absdiff_t currEnd{0};
    static inline m_angle_t checkStart{0};
    static inline m_angle_t checkEnd{0};
    
    static inline e_angle_t eangle{0};
    static inline e_angle_t eoffset{0};
    static inline State mState{State::Undefined};
    static inline External::Tick<Timer> mStateTick;
    static inline External::Tick<Timer> mSubStateTick;
    static inline External::Tick<Timer> mDebugTick;
};


template<typename BusSystem, typename MCU = DefaultMcuType>
struct Application {
    using busdevs = BusDevs<BusSystem>;
    
    inline static void run() {
        if constexpr(External::Bus::isNoBus<BusSystem>::value) {
            using terminal = busdevs::terminal;
            using systemTimer = busdevs::systemTimer;
            using gfsm = GlobalFsm<busdevs>;
            
            gfsm::init();
            
            etl::outl<terminal>("foc_t25_hw01"_pgm);
            
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
using app = Application<External::Bus::NoBus<devices>>;

int main() {
    devices::init();
    app::run();
}

#ifndef NDEBUG
/*[[noreturn]] */inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], const unsigned int line [[maybe_unused]]) noexcept {
    xassert::ab.clear();
    xassert::ab.insertAt(0, expr);
    etl::itoa(line, xassert::aline);
    xassert::ab.insertAt(20, xassert::aline);
    xassert::ab.insertAt(30, file);
    xassert::on = true;
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, const unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
