#pragma once

#include <etl/control.h>

namespace External {
    
    using namespace AVR;
    using namespace std::literals::chrono;
    using namespace External::Units::literals;
    
    template<typename PPM, typename ADC, typename Timer, typename ValueType, typename TermDev = void>
    struct Servo {
        using terminal = etl::basic_ostream<TermDev>;
        
        enum class State : uint8_t {Undefined = 0, Init, 
                                    CalibrateADStart = 10, CalibrateADRun1, CalibrateADTest1, CalibrateADRun2, CalibrateADTest2, CalibrateADFinish,
                                    CalibrateDeadStartP = 20, CalibrateDeadSetP, CalibrateDeadTestP,
                                    CalibrateDeadStartN = 30, CalibrateDeadSetN, CalibrateDeadTestN,
                                    NeutralStart = 40, NeutralReachForward, NeutralReachBackward, Neutral,
//                                    Stop = 50, Forward, Backward,
                                    Run = 60};
        enum class Event : uint8_t {None, Reset, Calibrate, Run};
    
        static constexpr External::Tick<Timer> settleTicks{20_ms};
        static constexpr External::Tick<Timer> adTestTicks{300_ms};
        
        inline static Event mEvent{Event::None};

        using ppm_t = PPM::ranged_type;
        using adc_t = ADC::value_type;
    //    adc_t::_;
        
        struct PositionProvider {
            inline static constexpr auto valueId = External::SPort::ValueId::DIY;
            inline static constexpr auto ibus_type = IBus2::Type::type::ID_S85;
            inline static constexpr void init() {
            }
            inline static constexpr uint32_t value() {
                const uint16_t d = std::max(int16_t(mActual) - int16_t(amin), 0);
                const uint16_t v = (uint32_t(d) * 1024) / (amax - amin);
                return std::min(v, 1024U);
            }
            inline static void set(const adc_t v) {
                mActual = v;
            }
        private:
            inline static uint16_t mActual{};
        };
        
        
        inline static void init() {}
        
        inline static bool setupFinished() {
            return (mState == State::Neutral);
        }
        
        inline static Event event() {
            Event e{Event::None};
            using std::swap;
            swap(mEvent, e);
            return e;
        }    
        
        static inline void reset() {
            mEvent = Event::Reset;
        }
        static inline void calibrate() {
            mEvent = Event::Calibrate;        
        }
        static inline void run() {
            mEvent = Event::Run;
        }
        
        static inline uint16_t adiff() {
            return aDiff;
        }
        
        static inline uint16_t span() {
            return adelta;
        }
        
        template<auto L, auto U>
        static inline void position(const etl::uint_ranged_NaN<uint16_t, L, U>& pv) {
            if (pv) {
                const uint16_t sv = pv.toInt();
                targetPos = amin + ((uint32_t)(sv - L) * (amax - amin)) / (U - L);
            }
        }
        
        inline static constexpr adc_t::value_type adc_mid = (adc_t::Upper + adc_t::Lower) / 2;
    
        inline static constexpr int16_t minimumASpan = 3000; // 3V
        
        inline static constexpr int16_t ppm_mid = (ppm_t::Upper + ppm_t::Lower) / 2;
        inline static constexpr int16_t ppm_delta = (ppm_t::Upper - ppm_t::Lower) / 2;
        
    //    std::integral_constant<uint16_t, ppm_mid>::_;
    //    std::integral_constant<uint16_t, ppm_delta>::_;
        
        template<auto L, auto U>
        static inline void speed(const etl::uint_ranged_NaN<uint16_t, L, U>& pv) {
            if (pv) {
                const auto s = AVR::Pgm::scaleTo<ppm_t>(pv);
                if (s > ppm_mid) {
                    servoSpeedMax = std::min(s - ppm_mid, ppm_t::Upper - std::max(deadN, deadP) - deadOffset);
                }
            }
        }
        
        inline static int16_t error(const int16_t actual) {
            constexpr uint8_t hysterese = 0;
            if (targetPos > (actual + hysterese)) {
                const int16_t diff = targetPos - actual;
                if (diff > adelta2) {
                    return targetPos - (actual + adelta);
                }
                else {
                    return targetPos - actual;
                }
            }
            else if (targetPos < (actual - hysterese)) {
                const int16_t diff = targetPos - actual;
                if (diff < -adelta2) {
                    return targetPos - (actual - adelta);
                }
                else {
                    return targetPos - actual;
                }
            }
            return 0;
        }
        
        static inline void periodic() {}
        
        static inline void ratePeriodic() {
            const auto oldState = mState;
            const auto e = event();        
            const adc_t actual = ADC::value();
            PositionProvider::set(actual);
            
            ++stateTicks;
            switch(mState) {
            case State::Undefined:
                mState = State::Init;
                break;
            case State::Init:
                if (e == Event::Calibrate) {
                    mState = State::CalibrateDeadStartP;
                }
                break;
            case State::CalibrateADStart:
                stateTicks.on(adTestTicks, []{
                    mState = State::CalibrateADRun1;
                });
                break;
            case State::CalibrateADRun1:
                stateTicks.on(settleTicks, []{
                    mState = State::CalibrateADTest1;
                });
                if (actual > amax) {
                    amax = actual;
                }
                break;
            case State::CalibrateADTest1:
                if (amax == amaxLast) {
                    mState = State::CalibrateADRun2;
                    amin = actual;
                }
                else {
                    mState = State::CalibrateADRun1;
                }
                break;
            case State::CalibrateADRun2:
                stateTicks.on(settleTicks, []{
                    mState = State::CalibrateADTest2;
                });
                if (actual < amin) {
                    amin = actual;
                }
                break;
            case State::CalibrateADTest2:
                if (amin == aminLast) {
                    mState = State::CalibrateADFinish;
                }
                else {
                    mState = State::CalibrateADRun2;
                }
                break;
            case State::CalibrateADFinish:
                if (adelta < minimumASpan) {
                    mState = State::CalibrateADStart;
                }
                stateTicks.on(adTestTicks, []{
                    mState = State::NeutralStart;
                });
                break;
            case State::CalibrateDeadStartP:
                stateTicks.on(settleTicks, []{
                    mState = State::CalibrateDeadSetP;
                });
                break;
            case State::CalibrateDeadSetP:
                stateTicks.on(settleTicks, []{
                    mState = State::CalibrateDeadTestP;
                });
                break;
            case State::CalibrateDeadTestP:
                if (actual > (lastAnalog + 10)) {
                    mState = State::CalibrateDeadStartN;
                }
                else {
                    if (deadP < servoSpeedMax) {
                        deadP += 1;
                    }
                    mState = State::CalibrateDeadSetP;
                }
                break;            
            case State::CalibrateDeadStartN:
                stateTicks.on(settleTicks, []{
                    mState = State::CalibrateDeadSetN;
                });
                break;
            case State::CalibrateDeadSetN:
                stateTicks.on(settleTicks, []{
                    mState = State::CalibrateDeadTestN;
                });
                break;
            case State::CalibrateDeadTestN:
                if (actual < (lastAnalog - 10)) {
                    mState = State::CalibrateADStart;
                }
                else {
                    if (deadN < servoSpeedMax) {
                        deadN += 1;
                    }
                    mState = State::CalibrateDeadSetN;
                }
                break;            
            case State::NeutralStart:
                if (error(actual) > 0) {
                    mState = State::NeutralReachForward;
                }
                else if (error(actual) < 0) {
                    mState = State::NeutralReachBackward;
                }
                break;
            case State::NeutralReachForward:
                if (error(actual) <= 0) {
                    mState = State::Neutral;
                }
                break;
            case State::NeutralReachBackward:
                if (error(actual) >= 0) {
                    mState = State::Neutral;
                }
                break;
            case State::Neutral:
                if (e == Event::Run) {
                    mState = State::Run;
                }
                break;
            case State::Run:
            {
                const auto e = error(actual); 
                const auto cv = mPd.correctionValue(e, 0);
                if (cv > 0) {
                    PPM::set(ppm_t{ppm_mid + cv + (deadP) / 2});
                } 
                else if (cv < 0) {
                    PPM::set(ppm_t{ppm_mid + cv - (deadN) / 2});
                } 
                else {
                    PPM::set(ppm_t{ppm_mid});
                }
                if (e >= 0) {
                    aDiff = e;
                }
                else {
                    aDiff = -e;
                }
            }
                break;
//            case State::Stop:
//            {
//                constexpr int16_t h = 0;
//                if (error(actual) > h) {
//                    mState = State::Forward;
//                }
//                else if (error(actual) < -h) {
//                    mState = State::Backward;
//                }
//                else {
//                    aDiff = 0;
//                }
//            }
//                break;
//            case State::Forward:
//                if (const auto err = error(actual); err <= 0) {
//                    mState = State::Stop;
//                }
//                else {
//                    aDiff = err;
//                    const auto cv = std::min(aDiff / errorDiv, servoSpeedMax);
//                    PPM::set(ppm_t{ppm_mid + deadP + deadOffset + cv});
//                }
//                break;
//            case State::Backward:
//                if (const auto err = error(actual); err >= 0) {
//                    mState = State::Stop;
//                }
//                else {
//                    aDiff = -err;
//                    const auto cv = std::min(aDiff / errorDiv, servoSpeedMax);
//                    PPM::set(ppm_t{ppm_mid - deadN - deadOffset - cv});
//                }
//                break;
            }
            if (oldState != mState) {
                stateTicks.reset();
                switch(mState) {
                case State::Undefined:
    //                etl::outl<terminal>("FB: U"_pgm);
                    break;
                case State::Init:
    //                etl::outl<terminal>("FB: I"_pgm);
                    PPM::set(ppm_t{ppm_mid});
                    break;
                case State::CalibrateADStart:
                    etl::outl<terminal>("FB: C AD Start"_pgm);
                    aminLast = amaxLast = amax = amin = actual;
                    PPM::set(ppm_t{ppm_mid + deadP + 20});
                    break;
                case State::CalibrateADRun1:
    //                etl::outl<terminal>("FB: C AD Run1"_pgm);
                    amaxLast = amax;
                    break;
                case State::CalibrateADTest1:
    //                etl::outl<terminal>("FB: C AD Test1"_pgm);
                    break;
                case State::CalibrateADRun2:
    //                etl::outl<terminal>("FB: C AD Run2"_pgm);
                    aminLast = amin;
                    break;
                case State::CalibrateADTest2:
    //                etl::outl<terminal>("FB: C AD Test2"_pgm);
                    break;
                case State::CalibrateADFinish:
                    etl::outl<terminal>("FB: C AD Fin"_pgm);
                    PPM::set(ppm_t{ppm_mid});
                    adelta = amax - amin;
                    adelta2 = adelta / 2;
                    targetPos = (amax + amin) / 2;
                    break;
                case State::CalibrateDeadStartP:
    //                etl::outl<terminal>("FB: C D StartP "_pgm);
                    deadP = deadInitialValue;
                    PPM::set(ppm_t{ppm_mid});
                    break;
                case State::CalibrateDeadSetP:
    //                etl::outl<terminal>("FB: C D SetP "_pgm);
                    lastAnalog = actual;
                    PPM::set(ppm_t{ppm_mid + deadP});
                    break;
                case State::CalibrateDeadTestP:
    //                etl::outl<terminal>("FB: C D TestP "_pgm);
                    break;
                case State::CalibrateDeadStartN:
    //                etl::outl<terminal>("FB: C D StartN "_pgm);
                    deadN = deadInitialValue;
                    PPM::set(ppm_t{ppm_mid});
                    break;
                case State::CalibrateDeadSetN:
    //                etl::outl<terminal>("FB: C D SetN "_pgm);
                    lastAnalog = actual;
                    PPM::set(ppm_t{ppm_mid - deadN});
                    break;
                case State::CalibrateDeadTestN:
    //                etl::outl<terminal>("FB: C D TestN "_pgm);
                    break;
                case State::NeutralStart:
    //                etl::outl<terminal>("FB: NS "_pgm);
                    PPM::set(ppm_t{ppm_mid});
                    targetPos = (amax + amin) / 2;
                    break;
                case State::NeutralReachForward:
    //                etl::outl<terminal>("FB: NRF "_pgm);
                    PPM::set(ppm_t{ppm_mid + deadP + 20});
                    break;
                case State::NeutralReachBackward:
    //                etl::outl<terminal>("FB: NRB "_pgm);
                    PPM::set(ppm_t{ppm_mid - deadN - 20});
                    break;
                case State::Neutral:
    //                etl::outl<terminal>("FB: N"_pgm);
                    PPM::set(ppm_t{ppm_mid});
                    break;
//                case State::Stop:
//                    etl::outl<terminal>("FB: S"_pgm);
//                    PPM::set(ppm_t{ppm_mid});
//                    break;
//                case State::Forward:
//                    etl::outl<terminal>("FB: F"_pgm);
//                    break;
//                case State::Backward:
//                    etl::outl<terminal>("FB: B"_pgm);
//                    break;
                case State::Run:
                    etl::outl<terminal>("FB: Run"_pgm);
                    break;
                }
            }
        } 
        static inline void debug() {
    //        etl::outl<terminal>("FB a: "_pgm, av, " dp: "_pgm, deadP, " dn: "_pgm, deadN, " ami: "_pgm, amin, " ama: "_pgm, amax);
    //        etl::outl<terminal>("FB t: "_pgm, targetPos, " ami: "_pgm, amin, " ama: "_pgm, amax);
//            etl::outl<terminal>("FB t: "_pgm, targetPos, " ad: "_pgm, aDiff);
        }
    private:
        inline static adc_t amin{adc_mid};
        inline static adc_t amax{adc_mid};
        inline static adc_t aminLast{adc_mid};
        inline static adc_t amaxLast{adc_mid};
        inline static int16_t adelta{adc_mid};
        inline static int16_t adelta2{adc_mid};
        
        inline static constexpr uint16_t deadInitialValue{50};
        
        inline static uint16_t deadP{deadInitialValue};
        inline static uint16_t deadN{deadInitialValue};
        inline static int16_t deadOffset{0};
    
        inline static uint16_t errorDiv = 2;
        inline static uint16_t servoSpeedMax = (ppm_delta / 4);
    
        static inline Control::PD<int16_t, float> mPd{2.0, 6.0, 500, int16_t(servoSpeedMax)};
        
        inline static int16_t targetPos{};
        
        inline static adc_t lastAnalog;
    
        inline static uint16_t aDiff{0};
        inline static External::Tick<Timer> stateTicks;
        inline static State mState{State::Undefined};
    };
}
