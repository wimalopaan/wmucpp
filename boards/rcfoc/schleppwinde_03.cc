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
#include "telemetry.h"

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

template<typename Encoder>
struct EncoderStats {
    using encoder = Encoder;
    using m_angle_t = encoder::angle_type;
    using m_diff_t = decltype(cyclic_diff(m_angle_t{}, m_angle_t{}));
    using m_absdiff_t = decltype(cyclic_diff(m_angle_t{}, m_angle_t{}).rabs());

    static inline auto angle() {
        const auto a = encoder::angle();
        mSpeed = cyclic_diff(a, mAngleEnd);
        mFullConv += wrapAround(a, mAngleEnd);
        mAngleEnd = a;
        return a;
    }
    
    static inline int32_t diff() {
        return int32_t(mAngleEnd.toInt()) - mAngleStart.toInt() + mFullConv * int32_t(m_angle_t::Upper + 1);
    }

    static inline int32_t diff(const int32_t& offset) {
        return int32_t(mAngleEnd.toInt()) - mAngleStart.toInt() + mFullConv * int32_t(m_angle_t::Upper + 1) - offset;
    }
    
    static inline m_diff_t speed() {
        return mSpeed;
    }
    
    static inline void reset() {
        mAngleStart = mAngleEnd;
        mFullConv = 0;
    }
    static inline void start() {
        reset();
    }
private:
    template<typename T>
    static inline auto wrapAround(const T& first, const T& second) {
        if (first.toInt() > (second.toInt() + (T::Upper - T::Lower)/2)) {
            return -1;
        }
        else if ((first.toInt() + (T::Upper - T::Lower)/2) < second.toInt()) {
            return 1;
        }
        return 0;
    }
    static inline m_angle_t mAngleStart{0};
    static inline m_angle_t mAngleEnd{0};
    static inline int16_t mFullConv{0};
    static inline m_diff_t mSpeed{0};
};

template<typename BusDevs>
struct GlobalFsm {
    using gfsm = GlobalFsm<BusDevs>;
    using bus_type = BusDevs::bus_type;
    
    using devs = BusDevs::devs;
    using Timer = devs::systemTimer;
    
    using blinkLed = External::Blinker2<typename devs::led, Timer, 100_ms, 2500_ms>;
    using count_type = blinkLed::count_type;
    
    using enable = devs::enable;
    
    using dbg = devs::dbgPin;
    
    using encoder = devs::encoder;
    using m_angle_t = encoder::angle_type;
    
    using stats = EncoderStats<encoder>;
    
    using m_diff_t = decltype(cyclic_diff(m_angle_t{}, m_angle_t{}));
    using m_absdiff_t = decltype(cyclic_diff(m_angle_t{}, m_angle_t{}).rabs());

//    using driver = Driver<typename devs::pwm, std::integral_constant<size_t, 2048>, std::integral_constant<uint8_t, 11>, m_absdiff_t>;

    // 2205
    using driver = Driver<typename devs::pwm, std::integral_constant<size_t, 2048>, std::integral_constant<uint8_t, 7>, m_absdiff_t>;

    using e_angle_t = driver::angle_type;
    
    static inline constexpr m_angle_t zeroAngle{1023};
    
    using lut3 = devs::lut3;
    using Adc = devs::adcController;
    
    using terminal = BusDevs::terminal;
    using TermDev = terminal::device_type;

    static inline constexpr bool useBus = !External::Bus::isNoBus<bus_type>::value;
    using servo = std::conditional_t<useBus, typename BusDevs::servo, void>;
    using servo_pa = std::conditional_t<useBus, typename BusDevs::servo_pa, External::Hal::NullProtocollAdapter<>>;

    using servo_value_t = servo_pa::value_type;
    static inline constexpr auto servo_mid  = []{
        if constexpr(useBus) {
            return (servo_value_t::Upper + servo_value_t::Lower) / 2;
        }
        return 0u;
    }();
    static inline constexpr auto servo_span = []{
        if constexpr(useBus) {
            return (servo_value_t::Upper - servo_value_t::Lower);
        }
        return 0u;
    }();
    static inline constexpr auto chThreshUp = servo_mid + servo_span / 4;
    static inline constexpr auto chThreshDn = servo_mid - servo_span / 4;
    
    using adc_i_t = Adc::index_type;

    using sensor = BusDevs::sensor;
    using angle_p = BusDevs::AngleProvider;
    using state_p = BusDevs::StateProvider;
    
    enum class State : uint8_t {Undefined = 0, 
                                Init = 10, Enable,
                                CheckCurrentStart = 20, CheckCurrentEnd,
                                CheckDirStart, CheckDirEnd, CheckPolesStart, CheckPolesEnd,
                                ZeroStart, ZeroEnd,
                                RepositionStart, RepositionEnd,
                                PreRun, 
                                Hold = 40, Forward, Backward, Stalled, EndFW, EndBW, Coast,
                                Error = 50};
    
    static constexpr External::Tick<Timer> startupTicks{100_ms};
    static constexpr External::Tick<Timer> enableTicks{500_ms};
    static constexpr External::Tick<Timer> debugTicks{300_ms};
    static constexpr External::Tick<Timer> zeroGTicks{1000_ms};
    static constexpr External::Tick<Timer> zeroTicks{10_ms};

//    static constexpr External::Tick<Timer> speedTicks{100_ms};
    
    
    static inline void init(const bool inverted) {
        TermDev::template init<AVR::BaudRate<115200>>();

        if constexpr(External::Bus::isIBus<bus_type>::value) {
            sensor::init();
            sensor::uart::txOpenDrain();

            servo::template init<BaudRate<115200>>();
            etl::outl<terminal>("IB"_pgm);
        }
        else if constexpr(External::Bus::isSBus<bus_type>::value) {
            lut3::init(std::byte{0x33}); // route TXD (inverted) to lut1-out 
            sensor::init();
            sensor::uart::txPinDisable();

            servo::template init<AVR::BaudRate<100000>, FullDuplex, true, 1>(); // 8E2
            if (inverted) {
                servo::rxInvert(true);
                etl::outl<terminal>("SB I"_pgm);
            }
            else {
                etl::outl<terminal>("SB"_pgm);
            }
        }
        else {
            static_assert(std::false_v<bus_type>, "bus type not supported");
        }
        
        blinkLed::init();
        enable::init();
        
        Adc::template init<false>(); // no pullups
        Adc::mcu_adc_type::nsamples(4);
        
        encoder::init();
        driver::init();
        
        dbg::template dir<Output>();
    }
    
    static inline void periodic() {
        TermDev::periodic();
        Adc::periodic();
        
        encoder::periodic();
        driver::periodic();
        
        encoder::template read<true>(); // increased time
        
        if constexpr(useBus) {
            servo::periodic();
            sensor::periodic();
        }
    }
    
    static inline void ratePeriodic() {
        dbg::toggle();
        
        driver::ratePeriodic();
        blinkLed::ratePeriodic();

        servo_pa::ratePeriodic();
        sensor::ratePeriodic();
        
        const auto oldState = mState;
        ++mStateTick;
        
        const m_angle_t angle = stats::angle();

        const auto analog_i = Adc::value(adc_i_t{0});
        const auto analog_v = Adc::value(adc_i_t{1});
        const auto analog_te = Adc::value(adc_i_t{2});
        const auto analog_ti = Adc::value(adc_i_t{4});
        
        (++mDebugTick).on(debugTicks, [&]{
            etl::outl<terminal>(
                        "cv: "_pgm, stats::diff(), " sp: "_pgm, stats::speed().toInt(), " pos: "_pgm, mPosition, " ls: "_pgm, mLastSpeed.toInt()
//                        "ma: "_pgm, angle.toInt(), " eo: "_pgm, eoffset.toInt(), 
//                        " i: "_pgm, analog_i, " v: "_pgm, analog_v, " te: "_pgm, analog_te, " ti: "_pgm, analog_ti,
//                        " cs: "_pgm, checkStart, " ce: "_pgm, checkEnd, " ld: "_pgm, ld, " ccs: "_pgm, currStart
                        );
//            etl::outl<terminal>(mPid);
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
            mStateTick.on(enableTicks, [&]{
                mState = State::CheckCurrentStart;
                currStart.setToBottom();
                stats::start();
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
//            lastAngle = angle;
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
            if (const auto d = cyclic_diff(m_angle_t{0}, angle); abs(d) < 50) {
                mState = State::RepositionStart;
                mLastSpeed.set(0);
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
        case State::RepositionStart:
        {
            if (stats::speed() > 3) {
                mLastSpeed -= 1;    
            }
            else if (stats::speed() < -3) {
                mLastSpeed += 1;    
            }
            else {
                if (stats::diff() > 0) {
                    mLastSpeed -=1;
                }
                else if (stats::diff() < 0) {
                    mLastSpeed +=1;
                }
            }
            m_angle_t target = circularAdd(lastAngle, mLastSpeed.toInt());
            setFOC(target, angle);
//            lastAngle = angle;
            
            if (abs(stats::diff()) < 50) {
                mState = State::RepositionEnd;
            }
        }
            break;
        case State::RepositionEnd:
            mStateTick.on(enableTicks, []{
                mPosition = 0;
                mState = State::PreRun;
            });
            break;
        case State::PreRun:
            if (const auto s = servo_pa::value(0); s) {
                mState = State::Hold;
            }
            break;
        case State::Hold:
            if (const auto v = servo_pa::value(0); v.isTop()) {
                mState = State::Coast;
            }
            else if (const auto v = scaledWithdeadband(servo_pa::value(0)); v > 0) {
                if (mPosition <= mPosMax) {
                    mState = State::Forward;
                }
            }
            else if (const auto v = scaledWithdeadband(servo_pa::value(0)); v < 0) {
                if (mPosition >=  mPosMin) {
                    mState = State::Backward;
                }
            }
            else {
                const auto d = -stats::diff(mPosition);
                setFOC(d, angle);
            }
            break;
        case State::Forward:
            if (const auto v = servo_pa::value(0); v.isTop()) {
                mState = State::Coast;
            }
            else if (const auto s = scaledWithdeadband(servo_pa::value(0)); s > 0) {
                if (s < 10) {
                    mStateTick.on(External::Tick<Timer>{10_ms}, [&]{
                        mPosition += s;
                    });
                }
                else {
                    mPosition += s / 10;
                }
                if (mPosition <= mPosMax) {
                    const auto d = -stats::diff(mPosition);
                    setFOC(d, angle);
                    if ((abs(d) > m_angle_t::Upper) && (stats::speed() == 0)) {
                        mPosition = stats::diff();
                        mState = State::Stalled;
                    }
                }
                else {
                    mState = State::EndFW;
                }
            }
            else {
                mState = State::Hold;
            }
            break;
        case State::Backward:
            if (const auto v = servo_pa::value(0); v.isTop()) {
                mState = State::Coast;
            }
            else if (const auto s = scaledWithdeadband(servo_pa::value(0)); s < 0) {
                if (s > -10) {
                    mStateTick.on(External::Tick<Timer>{10_ms}, [&]{
                        mPosition += s;
                    });
                }
                else {
                    mPosition += s / 10;
                }
                if (mPosition >= mPosMin) {
                    const auto d = -stats::diff(mPosition);
                    setFOC(d, angle);
                    if ((abs(d) > m_angle_t::Upper) && (stats::speed() == 0)) {
                        mPosition = stats::diff();
                        mState = State::Stalled;
                    }
                }
                else {
                    mState = State::EndBW;
                }
            }
            else {
                mState = State::Hold;
            }
            break;
        case State::EndFW:
            if (const auto v = servo_pa::value(0); v.isTop()) {
                mState = State::Coast;
            }
            else if (const auto s = scaledWithdeadband(servo_pa::value(0)); s < 0) {
                mState = State::Backward;
            }
            else {
                const auto d = -stats::diff(mPosition);
                setFOC(d, angle);
            }
            break;
        case State::EndBW:
            if (const auto v = servo_pa::value(0); v.isTop()) {
                mState = State::Coast;
            }
            else if (const auto s = scaledWithdeadband(servo_pa::value(0)); s > 0) {
                mState = State::Forward;
            }
            else {
                const auto d = -stats::diff(mPosition);
                setFOC(d, angle);
            }
            break;
        case State::Coast:
            if (const auto v = scaledWithdeadband(servo_pa::value(0)); v == 0) {
                mLastSpeed.set(0);
                driver::scale(m_absdiff_t{0});        
                enable::activate();
                mState = State::RepositionStart;
            }
            break;
        case State::Stalled:
            if (const auto v = servo_pa::value(0); v.isTop()) {
                mState = State::Coast;
            }
            else if (const auto v = scaledWithdeadband(servo_pa::value(0)); v == 0) {
                mState = State::Hold;
            }
            else {
                const auto d = -stats::diff(mPosition);
                setFOC(d, angle);
            }
            break;
        case State::Error:
            break;
        }
        
        angle_p::mAngle = stats::diff() / 10;
        lastAngle = angle;

        if (oldState != mState) {
            state_p::mState = static_cast<uint16_t>(mState);
            mStateTick.reset();
            mSubStateTick.reset();
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
                etl::outl<terminal>("S: CCS"_pgm);
                eoffset.setToBottom();
                checkStart = angle;
//                lastAngle = angle;
                break;
            case State::CheckCurrentEnd:
                etl::outl<terminal>("S: CCE"_pgm);
                checkEnd = angle;
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
                etl::outl<terminal>("S: CPS"_pgm);
                eangle = eoffset;
                checkStart = angle;
//                driver::scale(currEnd);
                break;
            case State::CheckPolesEnd:
                etl::outl<terminal>("S: CPE"_pgm);
                checkEnd = angle;
                break;
            case State::ZeroStart:
                etl::outl<terminal>("S: ZS"_pgm);
                driver::scale(m_absdiff_t{currEnd + currEnd});
                break;
            case State::ZeroEnd:
                etl::outl<terminal>("S: ZE"_pgm);
                break;
            case State::PreRun:
                etl::outl<terminal>("S: PreRun"_pgm);
                break;
            case State::Hold:
                etl::outl<terminal>("S: Hold"_pgm);
                break;
            case State::Stalled:
                etl::outl<terminal>("S: Stalled"_pgm);
                break;
            case State::Forward:
                etl::outl<terminal>("S: FW"_pgm);
                break;
            case State::Backward:
                etl::outl<terminal>("S: BW"_pgm);
                break;
            case State::RepositionStart:
                etl::outl<terminal>("S: RPS"_pgm);
                if (stats::diff() < 0) {
                    mLastSpeed.set(10);
                }
                if (stats::diff() > 0) {
                    mLastSpeed.set(-10);
                }
                break;
            case State::RepositionEnd:
                etl::outl<terminal>("S: RPE"_pgm);
                break;
            case State::EndFW:
                etl::outl<terminal>("S: End FW"_pgm);
                break;
            case State::EndBW:
                etl::outl<terminal>("S: End BW"_pgm);
                break;
            case State::Coast:
                etl::outl<terminal>("S: Coast"_pgm);
                enable::inactivate();
                driver::scale(m_absdiff_t{0});        
                break;
            case State::Error:
                etl::outl<terminal>("S: Error"_pgm);
                enable::inactivate();
                blinkLed::blink(count_type{5});
                break;
            }
        }
    }
private:
    
    inline static auto scaledWithdeadband(const servo_value_t& s) {
        using result_t = std::make_signed_t<typename servo_value_t::value_type>;
        if (s) {
            if (s.toInt() > (servo_mid + mDead)) {
                const result_t v = (s.toInt() - servo_mid - mDead) / 10;
                return v;
            }
            else if (s.toInt() < (servo_mid - mDead)) {
                const result_t v = (servo_mid - s.toInt() - mDead) / 10;
                return -v;
            }
        }
        return result_t{0};
    }
    
    template<typename T, auto U, typename A>
    inline static auto circularAdd(const etl::uint_ranged<T, 0, U>& v, const A& a) {
        A t = v + a;        
        while (t > A{U}) {
            t -= U; 
        }
        while (t < 0) {
            t += U;
        }
        return etl::uint_ranged<T, 0, U>(t);
    }
     
    static inline m_angle_t coastAngle(const m_angle_t& last, const m_angle_t& angle) {
        int16_t dd = std::clamp(int16_t(angle.toInt() - last.toInt()), -50, 50);

        if (dd <= -2) {
            dd *= 4;
            dd -= 30;
        }
        else if (dd >= 2) {
            dd *= 4;
            dd += 30;
        }
        
        int16_t ta = angle + dd;
        
        ld = dd;
        
        static_assert(m_angle_t::Lower == 0);
        if (ta < 0) {
            ta += m_angle_t::Upper;
        }            
        else if (ta > int16_t(m_angle_t::Upper)) {
            ta -= m_angle_t::Upper;
        }            
        return m_angle_t(ta);
    }
    
    static inline void setFOC(const m_angle_t& target, const m_angle_t& angle) {
        auto d = cyclic_diff(target, angle);
        
        const auto da = d.rabs();
        const m_absdiff_t ccv{da + (3 * currEnd) / 4};
        driver::torque(d, angle, eoffset);
        driver::scale(ccv);        
    }
    static inline void setFOC(const int32_t& d, const m_angle_t& angle) {
        const auto da = abs(d);
        const m_absdiff_t ccv{da + (3 * currEnd) / 4};
        driver::torque(d, angle, eoffset);
        driver::scale(ccv);        
    }

    static inline m_angle_t lastAngle;
    static inline m_diff_t mLastSpeed{0};
    
    static inline Control::PID<int16_t, float> mPid{2.0, 0.005, 9.0, 500, 500};

    static inline uint16_t mDead{10};     
    static inline servo_value_t ls;
    static inline int32_t mPosition{};
    static inline int32_t mPosMax{5 * m_angle_t::Upper};
    static inline int32_t mPosMin{};
    
//    static inline uint16_t c{0};
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
    
    inline static void run(const bool inverted = false) {
        if constexpr(External::Bus::isNoBus<BusSystem>::value || External::Bus::isSBus<BusSystem>::value || External::Bus::isIBus<BusSystem>::value) {
            using terminal = busdevs::terminal;
            using systemTimer = busdevs::systemTimer;
            using gfsm = GlobalFsm<busdevs>;
            
            gfsm::init(inverted);
            
            etl::outl<terminal>("schleppwinde_hw02"_pgm);
            
            while(true) {
                gfsm::periodic(); 
                systemTimer::periodic([&]{
                    gfsm::ratePeriodic();
                });
            }
        }
    }
};

using devices = Devices<0>;
//using app = Application<External::Bus::NoBus<devices>>;
//using app = Application<External::Bus::SBusSPort<devices>>;
using app = Application<External::Bus::IBusIBus<devices>>;

int main() {
    devices::init();
    app::run(true);
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
