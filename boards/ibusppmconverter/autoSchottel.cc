// analogInput fuer feedbach: 120K/2µ2F RC Tiefpass

// SO2 = Servo (Ch N + 0)
// SO1 = ESC   (Ch N + 1)
// Q0  = Thru  (Ch N + 2) (Config im Debug-Mode)
// SO3 = Feedback-In
// SO4 = Config (LED im Debug-Mode)

#define AUTO_BUS // SCAN für SBus / IBus

#include "board.h" // sets NDEBUG

// im Debug geht der Through-Channel nicht (ist dann Config-Pin)
//#undef NDEBUG

#include <etl/control.h>

template<typename PPM, typename ADC, typename Timer>
struct Servo {
    enum class State : uint8_t {Undefined = 0, Init, 
                                CalibrateADStart = 10, CalibrateADRun, CalibrateADTest, CalibrateADFinish,
                                CalibrateDeadStartP = 20, CalibrateDeadSetP, CalibrateDeadTestP,
                                CalibrateDeadStartN = 30, CalibrateDeadSetN, CalibrateDeadTestN,
                                NeutralStart = 40, NeutralReachForward, NeutralReachBackward, Neutral,
                                Stop = 50, Forward, Backward};
    enum class Event : uint8_t {None, Reset, Calibrate, Run};

    static constexpr External::Tick<Timer> settleTicks{40_ms};
    static constexpr External::Tick<Timer> adTestTicks{300_ms};
    
    inline static Event mEvent{Event::None};

//    using fpq10 = etl::FixedPoint<int16_t, 10>;
////    static inline Control::PID<int16_t, fpq10> pid{fpq10{1.0}, fpq10{0.0}, fpq10{2.0}, 100, 100};
//    static inline Control::PID<int16_t, float> pid{2.0, 0.0001, 20.0, 100, 200};

    inline static void init() {}
    
    inline static bool setupFinished() {
        return (mState == State::Neutral);
    }
    
    inline static Event event() {
        Event e{Event::None};
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
    
    template<auto L, auto U>
    static inline void position(const etl::uint_ranged_NaN<uint16_t, L, U>& pv) {
        if (pv) {
            const uint16_t sv = pv.toInt();
            targetPos = amin + ((uint32_t)(sv - L) * (amax - amin)) / (U - L);
        }
    }
    
    using ppm_t = PPM::ranged_type;
    using adc_t = ADC::value_type;
    
    inline static constexpr int16_t ppm_mid = (ppm_t::Upper + ppm_t::Lower) / 2;
    inline static constexpr int16_t ppm_delta = (ppm_t::Upper - ppm_t::Lower) / 2;
    
//    std::integral_constant<uint16_t, ppm_mid>::_;
//    std::integral_constant<uint16_t, ppm_delta>::_;
    
    inline static adc_t amin{512};
    inline static adc_t amax{512};
    inline static adc_t aminLast{512};
    inline static adc_t amaxLast{512};
    inline static int16_t adelta{512};
    inline static int16_t adelta2{512};
    
    inline static uint16_t deadP{100};
    inline static uint16_t deadN{deadP};

    inline static int16_t targetPos;
    
    inline static adc_t lastAnalog;
    
    inline static int16_t error(const int16_t actual) {
        constexpr uint8_t hysterese = 1;
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
            stateTicks.on(settleTicks, []{
                mState = State::CalibrateADRun;
            });
            break;
        case State::CalibrateADRun:
            stateTicks.on(adTestTicks, []{
                mState = State::CalibrateADTest;
            });
            if (actual > amax) {
                amax = actual;
            }
            if (actual < amin) {
                amin = actual;
            }
            break;
        case State::CalibrateADTest:
            if ((amax == amaxLast) && (amin == aminLast)) {
                mState = State::CalibrateADFinish;
            }
            else {
                mState = State::CalibrateADRun;
            }
            break;
        case State::CalibrateADFinish:
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
            if (actual > (lastAnalog + 1)) {
                mState = State::CalibrateDeadStartN;
            }
            else {
                deadP += 1;
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
            if (actual < (lastAnalog - 1)) {
                mState = State::CalibrateADStart;
            }
            else {
                deadN += 1;
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
                mState = State::Stop;
            }
            break;
        case State::Stop:
            if (error(actual) > 0) {
                mState = State::Forward;
            }
            else if (error(actual) < 0) {
                mState = State::Backward;
            }
            else {
                aDiff = 0;
            }
            break;
        case State::Forward:
            if (const auto err = error(actual); err <= 0) {
                mState = State::Stop;
            }
            else {
                aDiff = err;
                const auto cv = std::min(err * 2, ppm_delta / 4);
                PPM::set(ppm_t{ppm_mid + deadP + 30 + cv});
            }
            break;
        case State::Backward:
            if (const auto err = error(actual); err >= 0) {
                mState = State::Stop;
            }
            else {
                aDiff = -err;
                const auto cv = std::max(err * 2, -ppm_delta / 4);
                PPM::set(ppm_t{ppm_mid - deadN - 30 + cv});
            }
            break;
        }
        if (oldState != mState) {
            stateTicks.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                PPM::set(ppm_t{ppm_mid});
                break;
            case State::CalibrateADStart:
                amax = amin = actual;
                PPM::set(ppm_t{ppm_mid + deadP + 50});
                break;
            case State::CalibrateADRun:
                amaxLast = amax;
                aminLast = amin;
                break;
            case State::CalibrateADTest:
                break;
            case State::CalibrateADFinish:
                adelta = amax - amin;
                adelta2 = adelta / 2;
                PPM::set(ppm_t{ppm_mid});
                targetPos = (amax + amin) / 2;
                break;
            case State::CalibrateDeadStartP:
                deadP = 50;
                PPM::set(ppm_t{ppm_mid});
                break;
            case State::CalibrateDeadSetP:
                lastAnalog = actual;
                PPM::set(ppm_t{ppm_mid + deadP});
                break;
            case State::CalibrateDeadTestP:
                break;
            case State::CalibrateDeadStartN:
                deadN = 50;
                PPM::set(ppm_t{ppm_mid});
                break;
            case State::CalibrateDeadSetN:
                lastAnalog = actual;
                PPM::set(ppm_t{ppm_mid - deadN});
                break;
            case State::CalibrateDeadTestN:
                break;
            case State::NeutralStart:
                targetPos = (amax + amin) / 2;
                break;
            case State::NeutralReachForward:
                PPM::set(ppm_t{ppm_mid + deadP + 50});
                break;
            case State::NeutralReachBackward:
                PPM::set(ppm_t{ppm_mid - deadN - 50});
                break;
            case State::Neutral:
                PPM::set(ppm_t{ppm_mid});
                break;
            case State::Stop:
                PPM::set(ppm_t{ppm_mid});
                break;
            case State::Forward:
                break;
            case State::Backward:
                break;
            }
        }
    } 
private:
    inline static uint16_t aDiff{0};
    inline static External::Tick<Timer> stateTicks;
    inline static State mState{State::Undefined};
};

template<typename PPM, uint8_t N>
struct PPMAdapter {
    using ranged_type = PPM::ranged_type;
    using index_t = PPM::index_type;
    inline static void set(const ranged_type& v) {
        PPM::ppmRaw(index_t{N}, v.toInt());
    }
};

template<typename ADC, uint8_t N>
struct ADCAdapter {
    using value_type = ADC::value_type; 
    using index_type = ADC::index_type; 
    inline static value_type value() {
        return ADC::value(index_type{N});
    }
};

template<typename PPM, uint8_t N, typename Timer, typename V, typename Pos>
struct EscFsm {
    enum class State : uint8_t {Undefined, Init, Off, OffWait, Run};

    static constexpr External::Tick<Timer> initTimeoutTicks{100_ms};
    static constexpr External::Tick<Timer> thrTimeoutTicks{5_ms};
    static constexpr External::Tick<Timer> offTimeoutTicks{500_ms};

    using value_type = V;
    using ranged_type = value_type::ranged_type;
    
    using index_type = PPM::index_type;;
    
    static inline constexpr uint16_t offOffset{10};
    static inline constexpr ranged_type offValue = ranged_type{ranged_type::Mid};
    static inline constexpr ranged_type offThresh = ranged_type{ranged_type::Mid + offOffset};
    
    inline static void set(const value_type& v) {
        if (v) {
            mTarget = v.toRanged();
        }
    }
    inline static void off() {
        mTarget = offValue;
    }    
        
    inline static void init() {
    }    
    inline static void periodic() {
    }    
    inline static void ratePeriodic() {
        const auto oldState = mState;
        ++mStateTicks;
        switch(mState) {
        case State::Undefined:
            mState = State::Init;
            break;
        case State::Init:
            mStateTicks.on(initTimeoutTicks, []{
                mState = State::Off;
            });
            break;
        case State::Off:
            if (mTarget > offThresh) {
                mState = State::Run;
            }
            break;
        case State::OffWait:
            mStateTicks.on(offTimeoutTicks, []{
                mState = State::Off;
            });
            break;
        case State::Run:
            if ((mTarget <= offThresh) && (mActual <= offThresh)) {
                mState = State::OffWait;
            }
            (++mThrTicks).on(thrTimeoutTicks, []{
                if (mTarget > mActual) {
                    mActual += 1;
                }            
                if (mTarget < mActual) {
                    mActual -= 1;
                }            
                PPM::ppm(index_type{N}, mActual);            
            });
            break;
        }
        if (oldState != mState) {
            mStateTicks.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                mTarget = mActual = offValue;
                break;
            case State::Off:
                mTarget = mActual = offValue;
                PPM::ppm(index_type{N}, mActual);
                break;
            case State::OffWait:
                break;
            case State::Run:
                break;
            }
        }
    }    
//private:
//    ranged_type::_;
    static inline ranged_type mTarget;
    static inline ranged_type mActual;
    static inline State mState{State::Undefined};
    static inline External::Tick<Timer> mStateTicks;    
    static inline External::Tick<Timer> mThrTicks;    
};

template<typename BusDevs>
struct GFSM {
    using devs = BusDevs::devs;
    using PPM = devs::ppm;
    using ADC = devs::adcController;
    using Timer = devs::systemTimer;
    using configPin = devs::configPin;
    
    using bus_type = BusDevs::bus_type;
    using servo = BusDevs::servo;
    using PA = BusDevs::servo_pa;
    using terminal = BusDevs::terminal;
    
    using index_t = PPM::index_type;
    using channel_t = PA::channel_t;
    using search_t = etl::uint_ranged_circular<uint8_t, channel_t::Lower, 15>;
    using value_t = PA::value_type;
    using adcv_t = ADC::value_type; 
    
    static inline constexpr value_t chThreshH = value_t{value_t::Mid + (value_t::Upper - value_t::Lower) / 4};
    static inline constexpr value_t chThreshL = value_t{value_t::Mid + (value_t::Upper - value_t::Lower) / 6};

    using NVM = BusDevs::eeprom;
    
    using ppm1 = PPMAdapter<PPM, 0>;
    using adc1 = ADCAdapter<ADC, 0>;
  
    using ppm_t  = PPM::ranged_type;
    using out360 = Servo<ppm1, adc1, Timer>;

    using esc = EscFsm<PPM, 1, Timer, value_t, out360>;
    
    template<typename V>
    inline static void setThru(const V& v) {
        using index_t = PPM::index_type;
        PPM::ppm(index_t{2}, v);        
    }
    
    
    inline static auto& data() {
        return NVM::data();
    }
    
    enum class State : uint8_t {Undefined, 
                                ShowBus,
                                CheckConfig, SearchChannel, ShowChannel, ShowChannel2,
                                Init, Calibrate, Run};

    static constexpr auto showTimeout = 1000_ms;
    
    static constexpr External::Tick<Timer> initTimeoutTicks{100_ms};
    static constexpr External::Tick<Timer> debugTimeoutTicks{500_ms};
    static constexpr External::Tick<Timer> eepromTimeoutTicks{1000_ms};
    static constexpr External::Tick<Timer> showTimeoutTicks{showTimeout};
    
    using blinker = External::Blinker2<typename devs::led, Timer, 100_ms, showTimeout>;
    using bcount_t = blinker::count_type;
    
    static inline void init(const bool inverted) {
        NVM::init();
        if (data().magic() != 42) {
            data().magic() = 42;
            data().clear();
            data().change();
        }
        if (const auto ch = data().channel(); ch) {
            searchCh = search_t{ch.toInt()};
        }
        
        wdt::init<ccp>();        
        reset::noWatchDog([]{
            uninitialzed::reset();
        });
        reset::onWatchDog([]{
            uninitialzed::counter = uninitialzed::counter + 1;        
        });
        blinker::init();

#ifndef NDEBUG
        if constexpr(External::Bus::isIBus<bus_type>::value) {
            servo::template init<BaudRate<115200>>();
        }
        else if constexpr(External::Bus::isSBus<bus_type>::value) {
            servo::template init<AVR::BaudRate<100000>, FullDuplex, true, 1>(); // 8E2
            if (inverted) {
                servo::rxInvert(true);
            }
        }
        else {
            static_assert(std::false_v<bus_type>, "bus type not supported");
        }
#else        
        if constexpr(External::Bus::isIBus<bus_type>::value) {
            servo::template init<BaudRate<115200>, HalfDuplex>();
            servo::template txEnable<false>();
        }
        else if constexpr(External::Bus::isSBus<bus_type>::value) {
            servo::template init<AVR::BaudRate<100000>, HalfDuplex, true, 1>(); // 8E2
            servo::template txEnable<false>();
            if (inverted) {
                servo::txInvert(true);
            }
        }
        else {
            static_assert(std::false_v<bus_type>, "bus type not supported");
        }
#endif
        PPM::init();
        ADC::init();
        ADC::mcu_adc_type::nsamples(6);

        out360::init();
        esc::init();
                    
        configPin::init();
        
    }

    static inline void periodic() {
        servo::periodic();
        ADC::periodic();        
        NVM::saveIfNeeded([&]{
            etl::outl<terminal>("ep s"_pgm);
        });
        out360::periodic();
        esc::periodic();
    }

    static inline void ratePeriodic() {
        wdt::reset();
        PA::ratePeriodic();
        blinker::ratePeriodic();
        out360::ratePeriodic();
        esc::ratePeriodic();
            
        (++debugTicks).on(debugTimeoutTicks, []{
//            etl::outl<terminal>("e: "_pgm, (uint8_t)esc::mState, " "_pgm, esc::mTarget.toInt(), " "_pgm, esc::mActual.toInt());
            etl::outl<terminal>("e: "_pgm, (uint8_t)esc::mState, " "_pgm, esc::mTarget.toInt(), " "_pgm, esc::mActual.toInt(), " "_pgm, out360::adiff());
        });
        (++mEepromTicks).on(eepromTimeoutTicks, []{
            NVM::data().expire();
        });
        
        const auto oldState = mState;
        ++stateTicks;        
        switch(mState) {
        case State::Undefined:
            mState = State::Init;
            break;
        case State::Init:
            stateTicks.on(initTimeoutTicks, []{
                mState = State::ShowBus;
            });
            break;
        case State::ShowBus:
            stateTicks.on(showTimeoutTicks, []{
                mState = State::CheckConfig;
            });
            break;
        case State::CheckConfig:
            if (configPin::isActive()) {
                mState = State::SearchChannel;
            }
            else {
                mState = State::Calibrate;
            }
            break;
        case State::SearchChannel:
            if (const auto v = PA::value(searchCh.toInt()); v) {
                if (v.toInt() > chThreshH.toInt()) {
                    mState = State::ShowChannel;
                }
                else {
                    ++searchCh;
                }
            }
            break;
        case State::ShowChannel:
            if (const auto v = PA::value(searchCh.toInt()); v) {
                if (v.toInt() < chThreshL.toInt()) {
                    mState = State::ShowChannel2;
                }
            }
            break;
        case State::ShowChannel2:
            stateTicks.on(showTimeoutTicks, []{
                mState = State::Calibrate;
            });
            break;
        case State::Calibrate:
            if (out360::setupFinished()) {
                mState = State::Run;
            }
            break;
        case State::Run:
            const auto dir = PA::value(searchCh + 1);
            out360::position(dir);
            if (const auto d = out360::adiff(); d < 100) {
                const auto throttle = PA::value(searchCh + 0);
                esc::set(throttle);
            }
            else {
                esc::off();
            }
            const auto thru = PA::value(searchCh + 2);
            setThru(thru);
            break;
        }
        if (oldState != mState) {
            stateTicks.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                etl::outl<terminal>("S Ini"_pgm);
                out360::reset();
                break;
            case State::ShowBus:
                etl::outl<terminal>("S Sbu"_pgm);
                if constexpr(External::Bus::isIBus<bus_type>::value) {
                    blinker::onePeriod(bcount_t{1});
                }
                else if constexpr(External::Bus::isSBus<bus_type>::value) {
                    blinker::onePeriod(bcount_t{2});
                }
                break;
            case State::CheckConfig:
                etl::outl<terminal>("S CkC"_pgm);
                break;
            case State::SearchChannel:
                blinker::blink(bcount_t{5});
                etl::outl<terminal>("S Sea"_pgm);
                break;
            case State::ShowChannel:
                etl::outl<terminal>("S Sch "_pgm, searchCh.toInt());
                blinker::off();
                data().channel() = channel_t{searchCh.toInt()};
                data().change();
                break;
            case State::ShowChannel2:
                blinker::steady();
                etl::outl<terminal>("S Sch2 "_pgm, searchCh.toInt());
                break;
            case State::Calibrate:
                blinker::off();
                blinker::blink(bcount_t{4});
                etl::outl<terminal>("S Cal"_pgm);
                out360::calibrate();
                break;
            case State::Run:
                blinker::off();
                etl::outl<terminal>("S Run"_pgm);
                out360::run();
//                setEsc(ppm_t{PPM::ocMedium});
                break;
            }
        }
    }
private:    
    inline static search_t searchCh{0};
    inline static External::Tick<Timer> debugTicks;
    inline static External::Tick<Timer> stateTicks;
    inline static External::Tick<Timer> mEepromTicks;
    static inline State mState{State::Undefined};
};

template<typename HWRev = void, typename MCU = DefaultMcuType>
struct Devices {
    using ccp = Cpu::Ccp<>;
    using clock = Clock<>;
    using sigrow = SigRow<>;
    
    using systemTimer = SystemTimer<Component::Rtc<0>, External::Bus::fRtc>;
    
//    using scanTermPosition = void;
    
    using servoPosition = usart0Position;
    using scanDevPosition = usart0Position;
    
    using ppmDevPosition = void;  
    using evrouter = void;
    
#ifndef NDEBUG
    using scanLedPin = AVR::ActiveHigh<so4Pin, Output>;
#else
    using scanLedPin = AVR::ActiveLow<daisyChain, Output>;
#endif

#ifndef NDEBUG
    using configPin = AVR::ActiveLow<q0Pin, Input>;
#else
    using configPin = AVR::ActiveLow<so4Pin, Input>;
#endif
    
    using adcController = External::Hal::AdcController<adc, Meta::NList<5>>; // 1e = temp
    
    using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;
#ifndef NDEBUG
    using ppm = External::Ppm::PpmOut<tcaPosition, Meta::List<PWM::WO<0>, PWM::WO<1>>>;
#else
    using ppm = External::Ppm::PpmOut<tcaPosition, Meta::List<PWM::WO<0>, PWM::WO<1>, PWM::WO<2>>>;
#endif
    
    using portmux = Portmux::StaticMapper<Meta::List<usart0Position, tcaPosition>>;
    
    using led = scanLedPin;
    
    using scan_term_dev = void;
        
    static inline void init() {
        portmux::init();
        ccp::unlock([]{
            clock::template prescale<1>();
        });
        systemTimer::init(); 
    }
};

template<typename Bus>
struct BusDevs;

template<typename Devs>
struct BusDevs<External::Bus::IBusIBus<Devs>> {
    using bus_type = External::Bus::IBusIBus<Devs>;
    
    struct BusParam {
        using bus_t = bus_type;
        using proto_type = RCSwitch::Protocol2<RCSwitch::High>;
    };
    
    using devs = Devs;
    
    using servo_pa = IBus2::Servo::ProtocollAdapter<0>;
    using servo = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

    using terminal = etl::basic_ostream<servo>;
    
    using eeprom = EEProm::Controller<Storage::ApplDataSchottel<typename servo_pa::channel_t, bus_type>>;
};

template<typename Devs>
struct BusDevs<External::Bus::SBusSPort<Devs>> {
    using bus_type = External::Bus::SBusSPort<Devs>;
    
    struct BusParam {
        using bus_t = bus_type;
        using proto_type = RCSwitch::Protocol2<RCSwitch::Low>;
    };
    
    using devs = Devs;
    
    using servo_pa = External::SBus::Servo::ProtocollAdapter<0, systemTimer>;
    using servo = AVR::Usart<usart0Position, servo_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

    using terminal = etl::basic_ostream<servo>;
    
    using eeprom = EEProm::Controller<Storage::ApplDataSchottel<typename servo_pa::channel_t, bus_type>>;

};

template<typename BusSystem, typename MCU = DefaultMcuType>
struct Application {
    using busdevs = BusDevs<BusSystem>;
    
    inline static void run(const bool inverted = false) {
        if constexpr(External::Bus::isIBus<BusSystem>::value || External::Bus::isSBus<BusSystem>::value) {
            using devs = busdevs::devs;
            using systemTimer = devs::systemTimer;
            using gfsm = GFSM<busdevs>;
            
            gfsm::init(inverted);
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

#ifndef NDEBUG
using scanner = External::Scanner<devices, Application>;
#else
using scanner = External::Scanner<devices, Application, AVR::HalfDuplex>;
#endif

int main() {
    scanner::run();
}

#ifndef NDEBUG
[[noreturn]] inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], unsigned int line [[maybe_unused]]) noexcept {
    while(true) {
//        devices::scan_term_dev::periodic();
    }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
