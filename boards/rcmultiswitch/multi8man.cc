#define NDEBUG

// Blinkmodus -> Jumper 2
// dann Blinkfrequenz einstellen (on/off getrennt)
// 1) einen einschalten
// 2) Jumper 2
// 3) Blinkfrequenz einstellen
// 4) Jumper 3
// 5) Tastverh. einstellen


// keine Freilaufdiode notwendig
// PWM-Modus über die Jumper einstellbar (zunächst für Kanäle 1-3)
//    PWM-Lernen:
//    1) normales Einschalten eines(!) Kanals
//    2) Lern-Modus -> Jumper 1 (ist mehr als ein Kanal an, geht alles aus)
//    3) langsam auf hoch auf max, dann neutral. Max wird gespeichert -> EEProm
//    4) Jumper 1 weg -> alles aus
//    5) dann einschalten bewirkt die PWM


#include "board.h"

namespace External {
    template<typename OutList, typename PWM, typename StateProvider, typename NVM>
    struct Output {
        inline static constexpr uint8_t size() {
            return Meta::size_v<OutList>;
        }
        static_assert(size() == StateProvider::size());
        
        using pwm_index_t = typename PWM::index_type;
        using index_t = etl::uint_ranged<uint8_t, 0, size() - 1>;
        
        inline static constexpr auto pwmMax = PWM::pwmMax;
 
        using nvm_t = std::remove_cvref_t<decltype(NVM::data())>;
        using nvm_data_t = nvm_t::value_type;
        using tick_t = nvm_data_t::tick_t;
        
        inline static constexpr auto blinkMax = tick_t::max();
        
        static inline void init() {
            PWM::init();
            StateProvider::init();
            Meta::visit<OutList>([]<typename L>(Meta::Wrapper<L>){
                                     L::off();
                                     L::template dir<AVR::Output>();
                                 });
            for (uint8_t out = 0; out < StateProvider::size(); ++out) {
                if (out <= pwm_index_t::Upper) {
                    const auto p = pwm_index_t(out);
                    if (auto v = NVM::data()[out].pwmValue()) {
                        PWM::pwm(p, 0);
                        PWM::on(p);
                    }
                }
            }
            
        }        
        
        inline static void setSwitchOff(const index_t index) {
            if (index <= pwm_index_t::Upper) {
                const auto li = pwm_index_t(index);
                if (const auto v = NVM::data()[index].pwmValue()) {
                    PWM::pwm(li, 0);
                }
                else {
                    off(index);
                }
            }
            else {
                off(index);
            }
        }
        inline static void setSwitchOn(const index_t index) {
            if (index <= pwm_index_t::Upper) {
                const auto li = pwm_index_t(index);
                if (const auto v = NVM::data()[index].pwmValue()) {
                    PWM::on(li);
                    PWM::pwm(li, v.toInt());
                }
                else {
                    on(index);
                }
            }
            else {
                on(index);
            }    
        }
        
        inline static void setSwitch(const index_t index) {
            if (StateProvider::switches()[index] == StateProvider::SwState::Off) {
                setSwitchOff(index);
                blinkTicks[index] = NVM::data()[index].blinks()[0].intervall;
            }
            else if (StateProvider::switches()[index] == StateProvider::SwState::On) {
                if (NVM::data()[index].blinks()[0].duration) {
                    blinkTicks[index].match(NVM::data()[index].blinks()[0].duration, [&]{
                        setSwitchOff(index);
                    });
                    blinkTicks[index].on(NVM::data()[index].blinks()[0].intervall, [&]{
                        setSwitchOn(index);
                    });
                    ++blinkTicks[index];
                }
                else {
                    setSwitchOn(index);
                }
            }
        }
        inline static void setSwitches() {
            for(uint8_t s = 0; s <= index_t::Upper; ++s) {
                setSwitch(index_t{s});
            }        
        }
//    private:
        inline static void on(const index_t index) {
            Meta::visitAt<OutList>(index, []<typename L>(Meta::Wrapper<L>){
                                       L::on();
                                   });
        }
        inline static void off(const index_t index) {
            Meta::visitAt<OutList>(index, []<typename L>(Meta::Wrapper<L>){
                                       L::off();
                                   });
        }
        
        inline static std::array<Storage::tick_type, size()> blinkTicks;
    };
}

template<typename Timer, typename Out, typename NVM, typename Input = void>
struct FSM {
    inline static constexpr auto size = Out::size();
    
    static_assert(Out::size() == Input::size());
    
    inline static constexpr auto intervall = Timer::intervall;
    inline static constexpr Storage::tick_type initTime{1000_ms};
    //        std::integral_constant<uint16_t, initTime.value>::_;
    
    
    using ppm_type = decltype(Input::ppm());
//    ppm_type::_;
    
    inline static constexpr auto ppmMedium = (ppm_type::Upper + ppm_type::Lower) / 2;
    inline static constexpr auto ppmSpan   = (ppm_type::Upper - ppm_type::Lower) / 2;
    
    inline static constexpr auto ppmDead = ppmMedium + ppmSpan / 20;    
    inline static constexpr auto ppmDead2 = ppmMedium - ppmSpan / 20;    
    //    std::integral_constant<uint16_t, ppm::ppmMax>::_;
    //    std::integral_constant<uint16_t, ppm::medium>::_;
    //    std::integral_constant<uint16_t, ppmDead>::_;
    
    enum class State : uint8_t {Init, Run, 
                                LearnPWMStart, LearnPWM, LearnPWMEnd,
                                LearnBlinkIntStart, LearnBlinkInt, LearnBlinkIntEnd,
                                LearnBlinkDurStart, LearnBlinkDur, LearnBlinkDurEnd,
                               };
    
    inline static void init() {
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
                    if (auto c = getFirstSelected()) {
                        mLearn.set(c);
                        Input::enable(false);
                        mState = State::LearnPWMStart;
                    }
                }
            }
            else if (getConfigValue() == 4) {
                if (numberOfOnSwitches() > 1) {
                    resetAll();                    
                }
                else {
                    if (auto c = getFirstSelected()) {
                        mLearn.set(c);
                        Input::enable(false);
                        mState = State::LearnBlinkIntStart;
                    }
                }
            }
            else {
                Out::setSwitches();
            }
            break;
        case State::LearnPWMStart:
            if (getConfigValue() == 2) {
                if (Input::ppm() > ppmDead) {
                    mState = State::LearnPWM;
                }
                else {
                    NVM::data()[mLearn].pwmValue(1);
                    Out::setSwitch(mLearn);
                }
            }
            else {
                resetAll();
                mState = State::Run;
            }
            break;
        case State::LearnBlinkIntStart:
            if (getConfigValue() == 4) {
                if (Input::ppm() > ppmDead) {
                    mState = State::LearnBlinkInt;
                }
            }
            else {
                resetAll();
                mState = State::Run;
            }
            break;
        case State::LearnBlinkDurStart:
            if (getConfigValue() == 4) {
                if (Input::ppm() > ppmDead) {
                    mState = State::LearnBlinkDur;
                }
            }
            break;
        case State::LearnPWM:
            if (getConfigValue() == 2) {
                if (const auto x = Input::ppm(); x > ppmMedium) {
                    uint8_t p = (((uint32_t)x - ppmMedium) * Out::pwmMax) / ppmSpan;
                    NVM::data()[mLearn].pwmValue(p);
                    Out::setSwitch(mLearn);
                }
            }
            else {
                mState = State::LearnPWMEnd;
            }
            break;
        case State::LearnBlinkInt:
            if (getConfigValue() == 4) {
                if (const auto x = Input::ppm(); x > ppmMedium) {
                    uint8_t p = (((uint32_t)x - ppmMedium) * Out::blinkMax) / ppm::span;
                    NVM::data()[mLearn].blinks()[0].duration = Storage::tick_type::fromRaw(p / 2);
                    NVM::data()[mLearn].blinks()[0].intervall = Storage::tick_type::fromRaw(p);
                    Out::setSwitch(mLearn);
                }
            }
            else {
                mState = State::LearnBlinkDurStart;
            }
            break;
        case State::LearnBlinkDur:
            if (getConfigValue() == 4) {
                if (const auto x = Input::ppm(); x > ppmMedium) {
                    uint8_t p = (((uint32_t)x - ppmMedium) * (NVM::data()[mLearn].blinks()[0].intervall.value.toInt() - 1)) / ppm::span;
                    NVM::data()[mLearn].blinks()[0].duration = Storage::tick_type::fromRaw(p);
                    Out::setSwitch(mLearn);
                }
            }
            else {
                mState = State::LearnBlinkDurEnd;
            }
            break;
        case State::LearnPWMEnd:
            if (const auto x = Input::ppm(); x < ppmDead2) {
                NVM::data().change();
                Out::setSwitchOff(mLearn);
                resetAll();
                Input::enable(true);
                mState = State::Run;
            }
            break;
        case State::LearnBlinkIntEnd:
            mState = State::LearnBlinkDurStart;
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
    //private:
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
    }
    
    inline static pwm::index_type mLearn;
    inline static State mState{State::Init};
    inline static Storage::tick_type stateTicks;
};

using terminalDevice = AVR::Usart<usart0Position, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<2>, AVR::SendQueueLength<512>>;
using terminal = etl::basic_ostream<terminalDevice>;

using multi = External::Ppm::MultiSwitch<ppm, 8>;

using out = External::Output<ledList, pwm, multi, eeprom>;

using fsm = FSM<SoftTimer, out, eeprom, multi>;

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
        if (!((appData[Storage::AVKey::Magic].pwmValue() == 43))) {
            appData[Storage::AVKey::Magic].pwmValue(43);
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
    
//    Meta::visit<ledList>([]<typename L>(Meta::Wrapper<L>){
//                             L::template dir<Output>();
//                         });
    
//    multi::init();
    
    fsm::init();
    
    const auto periodicTimer = alarmTimer::create(1000_ms, External::Hal::AlarmFlags::Periodic);
    const auto tickTimer = alarmTimer::create(SoftTimer::intervall, External::Hal::AlarmFlags::Periodic);
    
    etl::outl<terminal>("multi8man"_pgm);
    
    while(true) {
        eeprom::saveIfNeeded([&]{
            //            fsm::load();                
        });
        adcController::periodic();
        terminalDevice::periodic();
        multi::periodic();        
        
        systemTimer::periodic([&]{
            alarmTimer::periodic([&](const auto& t){
                if (tickTimer == t) {
                    fsm::ratePeriodic();
                }
                else if (periodicTimer == t) {
                    etl::out<terminal>("sw: [ "_pgm);
                    for(const auto l : multi::switches()) {
                        etl::out<terminal>(uint8_t(l), " "_pgm);
                    }
                    auto ii = appData[Storage::AVKey::Ch0].blinks()[0].intervall.value.toInt();
                    auto dd = appData[Storage::AVKey::Ch0].blinks()[0].duration.value.toInt();
                    etl::outl<terminal>(" ] a: "_pgm, getConfigValue(), " s: "_pgm, (uint8_t)fsm::mState, " bt0: "_pgm, out::blinkTicks[0].value.toInt(), " i: "_pgm, ii, " d: "_pgm, dd);
                    
                    appData.expire();
                }
            });
        });
    }
}

