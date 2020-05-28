#define NDEBUG

#include "board.h"
#include "swout.h"

template<typename Timer, typename Out, typename NVM, typename Input = void>
struct FSM {
    inline static constexpr auto size = Out::size();
    
    static_assert(Out::size() == Input::size());
    
    inline static constexpr auto intervall = Timer::intervall;
    inline static constexpr Storage::tick_type initTime{1000_ms};
    //        std::integral_constant<uint16_t, initTime.value>::_;
    
    
//    using ppm_type = decltype(Input::ppm());
    using ppm_type = Input::value_type;
    //    ppm_type::_;
    
    inline static constexpr auto ppmMedium = (ppm_type::Upper + ppm_type::Lower) / 2;
    inline static constexpr auto ppmSpan   = (ppm_type::Upper - ppm_type::Lower) / 2;
    
    inline static constexpr auto ppmDead  = ppmMedium + ppmSpan / 20;    
    inline static constexpr auto ppmDead2 = ppmMedium - ppmSpan / 20;    
    inline static constexpr auto ppmFull  = ppm_type::Upper - ppmSpan / 10;    
    //    std::integral_constant<uint16_t, ppm::ppmMax>::_;
    //    std::integral_constant<uint16_t, ppm::medium>::_;
    //    std::integral_constant<uint16_t, ppmDead>::_;

    using nvm_t = std::remove_cvref_t<decltype(NVM::data())>;
    using nvm_data_t = nvm_t::value_type;
    using pwm_type = nvm_data_t::pwm_type;
    
    enum class State : uint8_t {Init, Run, 
                                LearnPWMWait, LearnPWMStart, LearnPWM, LearnPWMEnd,
                                LearnBlinkIntWait, LearnBlinkIntStart, LearnBlinkInt, LearnBlinkIntEnd,
                                LearnBlinkDurStart, LearnBlinkDur, LearnBlinkDurEnd,
                               };
    
    inline static void init() {
        Input::init();
        Out::init();
    }
    
    inline static void ratePeriodic() {
        auto lastState = mState;
        ++stateTicks;
        switch(mState) {
        case State::Init:
            stateTicks.on(initTime, []{
                mState = State::Run;
            });
            break;
        case State::Run:
            if (getConfigValue() == 2) {
                if (numberOfOnSwitches() > 1) {
                    resetAll();                    
                }
                else {
                    if (const auto c = getFirstSelected()) {
                        if (c <= Out::pwm_index_t::Upper) {
                            mLearn.set(c);
                            Input::enable(false);
                            mState = State::LearnPWMWait;
                        }
                        else {
                            resetAll();
                            mState = State::Run;
                        }
                    }
                }
            }
            else if (getConfigValue() == 4) {
                if (numberOfOnSwitches() > 1) {
                    resetAll();                    
                }
                else {
                    if (const auto c = getFirstSelected()) {
                        mLearn.set(c);
                        Input::enable(false);
                        Out::setSwitchOff(mLearn);
                        mState = State::LearnBlinkIntWait;
                    }
                }
            }
            else {
                Out::setSwitches();
            }
            break;
        case State::LearnPWMWait:
            if (Input::ppm() < ppmDead2) {
                mState = State::LearnPWMStart;
            }
            break;
        case State::LearnPWMStart:
            if (Input::ppm() > ppmDead) {
                mState = State::LearnPWM;
            }
            else {
                NVM::data()[mLearn].pwmValue(1);
                Out::setSwitch(mLearn);
            }
            break;
        case State::LearnBlinkIntWait:
            if (Input::ppm() < ppmDead2) {
                mState = State::LearnBlinkIntStart;
            }
            break;
        case State::LearnBlinkIntStart:
            if (Input::ppm() > ppmDead) {
                Out::setSwitchOn(mLearn);
                mState = State::LearnBlinkInt;
            }
            break;
        case State::LearnBlinkDurStart:
            if (Input::ppm() > ppmDead) {
                mState = State::LearnBlinkDur;
            }
            break;
        case State::LearnPWM:
            if (getConfigValue() == 2) {
                mState = State::LearnPWMEnd;
            }
            else {
                if (const auto x = Input::ppm(); x > ppmMedium) {
                    if (x > ppmFull) {
                        NVM::data()[mLearn].pwmValue(pwm_type{});    
                    }
                    else {
                        uint8_t p = (((uint32_t)x - ppmMedium) * Out::pwmMax) / ppmSpan;
                        Out::pwm(mLearn, p);
                    }
                    Out::setSwitch(mLearn);
                }
            }
            break;
        case State::LearnBlinkInt:
            if (getConfigValue() == 4) {
                mState = State::LearnBlinkIntEnd;
            }
            else {
                if (const auto x = Input::ppm(); x > ppmMedium) {
                    uint8_t p = (((uint32_t)x - ppmMedium) * Out::blinkMax) / ppm::span;
                    Out::duration(mLearn, Storage::tick_type::fromRaw(p / 2));
                    Out::intervall(mLearn, Storage::tick_type::fromRaw(p));
                    Out::setSwitch(mLearn);
                }
            }
            break;
        case State::LearnBlinkDur:
            if (getConfigValue() == 4) {
                mState = State::LearnBlinkDurEnd;
            }
            else {
                if (const auto x = Input::ppm(); x > ppmMedium) {
                    uint8_t p = (((uint32_t)x - ppmMedium) * (Out::intervall(mLearn).value.toInt() - 1)) / ppm::span;
                    Out::duration(mLearn, Storage::tick_type::fromRaw(p));
                    Out::setSwitch(mLearn);
                }
            }
            break;
        case State::LearnPWMEnd:
            if (const auto x = Input::ppm(); x < ppmDead2) {
                NVM::data().change();
                Out::setSwitchOff(mLearn);
                resetAll();
                mState = State::Run;
            }
            break;
        case State::LearnBlinkIntEnd:
            if (Input::ppm() < ppmDead2) {
                mState = State::LearnBlinkDurStart;
            }
            break;
        case State::LearnBlinkDurEnd:
            if (const auto x = Input::ppm(); x < ppmDead2) {
                NVM::data().change();
                resetAll();
                Input::enable(true);
                mState = State::Run;
            }
            break;
        }
        if (lastState != mState) {
            stateTicks.reset();
        }
    }
//    private:
    inline static etl::uint_ranged_NaN<uint8_t, 0, 7> getFirstSelected() {
        for(uint8_t i = 0; i < Input::switches().size(); ++i) {
            if (Input::switches()[i] == Input::SwState::On) {
                return {i};
            }
        }
        return {};
    }
    inline static uint8_t numberOfOnSwitches() {
        uint8_t c{};
        for(const auto& sw: Input::switches()) {
            if (sw == Input::SwState::On) {
                ++c;
            }
        }
        return c;
    }
    inline static void resetAll() {
        Input::reset();
        Input::enable(true);
    }
    inline static Out::index_t mLearn;
    inline static State mState{State::Init};
    inline static Storage::tick_type stateTicks;
};


//template<typename Timer, typename Out, typename NVM, typename Input = void>
//struct FSM {
//    inline static constexpr auto size = Out::size();
    
//    static_assert(Out::size() == Input::size());
    
    
//    inline static void init() {
//        Out::init();
//    }
    
//    inline static void ratePeriodic() {
        
//    }
    
    
//};

using terminalDevice = AVR::Usart<usart0Position, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>>;
using terminal = etl::basic_ostream<terminalDevice>;

using mk4 = External::Graupner::MultiSwitch2N<ppm, 4>;

using out = External::Output<ledList, pwm, mk4, eeprom>;

using fsm = FSM<SoftTimer, out, eeprom, mk4>;

auto& appData = eeprom::data();

int main() {
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
    
    evrouter::init();
   
    terminalDevice::init<BaudRate<9600>>();
    terminalDevice::rxEnable<false>();
    
    adcController::init();
    
    systemTimer::init();

    eeprom::init();
    [[maybe_unused]] bool changed = false;
    
    {
        if (!((appData[Storage::AVKey::Magic].pwmValue() == 44))) {
            appData[Storage::AVKey::Magic].pwmValue(44);
            appData[Storage::AVKey::Ch0] = Storage::ApplData::value_type{};
            appData[Storage::AVKey::Ch1] = Storage::ApplData::value_type{};
            appData[Storage::AVKey::Ch2] = Storage::ApplData::value_type{};
            appData[Storage::AVKey::Ch3] = Storage::ApplData::value_type{};
            appData[Storage::AVKey::Ch4] = Storage::ApplData::value_type{};
            appData[Storage::AVKey::Ch5] = Storage::ApplData::value_type{};
            appData[Storage::AVKey::Ch6] = Storage::ApplData::value_type{};
            appData[Storage::AVKey::Ch7] = Storage::ApplData::value_type{};
            appData.change();
            changed  = true;
        }
    }
    
    fsm::init();
    
    const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);

    const auto tickTimer = alarmTimer::create(SoftTimer::intervall, External::Hal::AlarmFlags::Periodic);
    
    etl::outl<terminal>("multi8mk4"_pgm);

    while(true) {
        eeprom::saveIfNeeded([&]{
            //            fsm::load();                
        });
        terminalDevice::periodic();
        adcController::periodic();
        mk4::periodic();        

        systemTimer::periodic([&]{
            alarmTimer::periodic([&](const auto& t){
                if (tickTimer == t) {
                    fsm::ratePeriodic();
                }
                else if (periodicTimer == t) {
//                    etl::out<terminal>("sw: [ "_pgm);
//                    for(const auto l : mk4::swStates) {
//                        etl::out<terminal>(uint8_t(l), " "_pgm);
//                    }
//                    etl::outl<terminal>(" ]"_pgm);
//                    etl::out<terminal>("v: [ "_pgm);
//                    for(const auto l : mk4::pValues) {
//                        etl::out<terminal>(uint16_t(l), " "_pgm);
//                    }
//                    etl::outl<terminal>(" ]"_pgm);

                    etl::outl<terminal>(" ] a: "_pgm, getConfigValue(), " s: "_pgm, (uint8_t)fsm::mState);
                    
                    
                    appData.expire();
                }
            });
        });
    }
}

