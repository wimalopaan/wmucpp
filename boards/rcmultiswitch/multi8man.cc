#define NDEBUG

// keine Freilaufdiode notwendig
// PWM-Modus über die Jumper einstellbar (zunächst für Kanäle 1-3)
//    PWM-Lernen:
//    1) normales Einschalten eines(!) Kanals
//    2) Lern-Modus -> Jumper 1 (ist mehr als ein Kanal an, geht alles aus)
//    3) langsam auf hoch auf max, dann neutral. Max wird gespeichert -> EEProm
//    4) Jumper 1 weg -> alles aus
//    5) dann einschalten bewirkt die PWM


#include "board.h"

using terminalDevice = AVR::Usart<usart0Position, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<2>, AVR::SendQueueLength<512>>;
using terminal = etl::basic_ostream<terminalDevice>;

auto& appData = eeprom::data();

namespace External {
    namespace Ppm {
        template<typename Ppm, uint8_t N>
        struct MultiSwitch {
            enum class State : uint8_t {UnDefined, CountUp, CountUpPause, ToggleUp, CountDown, CountDownPause, ToggleDown};
            enum class SwState : uint8_t {Off, On};
            
            using index_t = etl::uint_ranged<uint8_t, 0, ((N / 2) - 1)>;
            
            inline static constexpr uint8_t size() {
                return N;
            }
            inline static void init() {
                Ppm::init();
            }
            inline static void reset() {
                for(auto& s : swStates) {
                    s = SwState::Off;
                }
            }
            
            inline static void periodic() {
                Ppm::onCapture([]{
                    const auto v = Ppm::value(); 
                    if (!v) return;
                    switch(mState) {
                    case State::UnDefined:
                        if (isUp(v)) {
                            mState = State::CountUp;
                            count.setToBottom();
                            index.setToBottom();
                        }
                        if (isDown(v)) {
                            mState = State::CountDown;
                            count.setToBottom();
                            index.setToBottom();
                        }
                        break;
                    case State::CountUp:
                        if (isUp(v)) {
                            ++count;
                            if (count > setCycles) {
                                toggle1(index);
                                mState = State::ToggleUp;
                            }
                        }
                        else {
                            if (isNeutral(v)) {
                                if (count > stepCycles) {
                                    mState = State::CountUpPause;
                                    count.setToBottom();
                                }
                                else {
                                    mState = State::UnDefined;
                                }
                            }
                        }
                        break;
                    case State::ToggleUp:
                        if (isNeutral(v)) {
                            mState = State::UnDefined;
                        }
                        break;
                    case State::CountUpPause:
                        if (isNeutral(v)) {
                            ++count;
                            if (count > resetCycles) {
                                mState = State::UnDefined;
                            }
                        }
                        else {
                            if (isUp(v)) {
                                if (count > stepCycles) {
                                    mState = State::CountUp;
                                    count.setToBottom();
                                    ++index;
                                }
                                else {
                                    mState = State::UnDefined;
                                }
                            }
                        }
                        break;
                    case State::CountDown:
                        if (isDown(v)) {
                            ++count;
                            if (count > setCycles) {
                                toggle2(index);
                                mState = State::ToggleDown;
                            }
                        }
                        else {
                            if (isNeutral(v)) {
                                if (count > stepCycles) {
                                    mState = State::CountDownPause;
                                    count.setToBottom();
                                }
                                else {
                                    mState = State::UnDefined;
                                }
                            }
                        }
                        break;
                    case State::ToggleDown:
                        if (isNeutral(v)) {
                            mState = State::UnDefined;
                        }
                        break;
                    case State::CountDownPause:
                        if (isNeutral(v)) {
                            ++count;
                            if (count > resetCycles) {
                                mState = State::UnDefined;
                            }
                        }
                        else {
                            if (isDown(v)) {
                                if (count > stepCycles) {
                                    mState = State::CountDown;
                                    count.setToBottom();
                                    ++index;
                                }
                                else {
                                    mState = State::UnDefined;
                                }
                            }
                        }
                        break;
                    }
                });
            }
            static inline const auto& switches() {
                return swStates;
            }
        private:
            static inline etl::uint_ranged<uint8_t, 0, 200> count;
            
            static inline constexpr uint8_t stepCycles{10};
            static inline constexpr uint8_t setCycles{50};
            static inline constexpr uint8_t resetCycles{50};
            
            static inline std::array<SwState, N> swStates{};
            static inline constexpr uint16_t hysterese = (Ppm::span / 16);
            static inline constexpr uint16_t thresh_up = Ppm::medium + (Ppm::span / 4) + hysterese;
            static inline constexpr uint16_t thresh_down = Ppm::medium - (Ppm::span / 4) - hysterese;
            static inline constexpr uint16_t thresh_neutral_low  = Ppm::medium - (Ppm::span / 4) - hysterese;
            static inline constexpr uint16_t thresh_neutral_high = Ppm::medium + (Ppm::span / 4) + hysterese;
            static inline index_t index;
            static inline State mState{State::UnDefined};
            
            static inline void toggle1(const index_t i) {
                if (swStates[i] == SwState::Off) {
                    swStates[i] = SwState::On;
                }
                else {
                    swStates[i] = SwState::Off;
                }
            }
            static inline void toggle2(const index_t k) {
                const auto i = k + (N/2);
                if (swStates[i] == SwState::Off) {
                    swStates[i] = SwState::On;
                }
                else {
                    swStates[i] = SwState::Off;
                }
            }
            static inline bool isNeutral(const auto v) {
                return (v >= thresh_neutral_low) && (v <= thresh_neutral_high);
            }
            static inline bool isUp(const auto v) {
                return (v >= thresh_up);
            }
            static inline bool isDown(const auto v) {
                return (v <= thresh_down);
            }
        };
    }
}

using multi = External::Ppm::MultiSwitch<ppm, 8>;

template<typename Timer, typename PWM>
struct FSM {
    inline static constexpr auto intervall = Timer::intervall;
    inline static constexpr External::Tick<Timer> initTime{1000_ms};
    
    inline static constexpr auto ppmDead = ppm::medium + ppm::span / 20;    
    
    enum class State : uint8_t {Init, Run, LearnStart, LearnPWM, LearnEnd};
    
    inline static void init() {
        PWM::init();
    }
    
    using index_t = etl::uint_ranged_circular<uint8_t, 0, multi::size() - 1>;
    using led_index_t = etl::uint_ranged<uint8_t, 0, 7>;
    
    static inline index_t index;
    inline static void setSwitches() {
        if (auto v = appData[mapToKey(index)]) {
            auto li = pwm::index_type(index);
            
            if (multi::switches()[index] == multi::SwState::Off) {
                pwm::pwm(li, 0);
            }            
            else if (multi::switches()[index] == multi::SwState::On) {
                pwm::on(li);
                pwm::pwm(li, v.toInt());
            }            
        }
        else {
            auto li = led_index_t(index);
            
            if (multi::switches()[index] == multi::SwState::Off) {
                Meta::visitAt<ledList>(li, []<typename L>(Meta::Wrapper<L>){
                                           L::off();
                                       });
            }            
            else if (multi::switches()[index] == multi::SwState::On) {
                Meta::visitAt<ledList>(li, []<typename L>(Meta::Wrapper<L>){
                                           L::on();
                                       });
            }            
        }
        ++index;
    }
    
    template<typename T>
    static inline Storage::AVKey mapToKey(const T i) {
        switch(i) {
        case 0:
            return Storage::AVKey::Pwm0;
        case 1:
            return Storage::AVKey::Pwm1;
        case 2:
            return Storage::AVKey::Pwm2;
        case 3:
            return Storage::AVKey::Pwm3;
        case 4:
            return Storage::AVKey::Pwm4;
        case 5:
            return Storage::AVKey::Pwm5;
        }
        return Storage::AVKey::Undefined;
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
            if (getConfigValue() > 0) {
                if (numberOfOnSwitches() > 1) {
                    resetAll();                    
                }
                else {
                    if (auto c = getFirstSelected()) {
                        if (c <= pwm::index_type::Upper) {
                            mLearn.set(c);
                            mState = State::LearnStart;
                        }
                    }
                }
            }
            else {
                setSwitches();
            }
            break;
        case State::LearnStart:
            if (getConfigValue() > 0) {
                if (ppm::value() > ppmDead) {
                    mState = State::LearnPWM;
                    pwm::pwm(mLearn, 0);
                    pwm::on(mLearn);
                }
            }
            else {
                resetAll();
                mState = State::Run;
            }
            break;
        case State::LearnPWM:
            if (getConfigValue() > 0) {
                if (auto x = ppm::value(); x > ppm::medium) {
                    uint8_t p = (((uint32_t)ppm::value() - ppm::medium) * pwm::pwmMax) / (ppm::span / 2);
                    pwm::pwm(mLearn, p);
                    appData[mapToKey(mLearn)] = p;
                }
            }
            else {
                mState = State::LearnEnd;
            }
            break;
        case State::LearnEnd:
            appData.change();
            pwm::off(mLearn);
            resetAll();
            Meta::visitAt<ledList>(mLearn, []<typename L>(Meta::Wrapper<L>){
                                           L::off();
                                       });
            mState = State::Run;
            break;
        }
        if (lastState != mState) {
            stateTicks.reset();
        }
    }
//private:
    inline static etl::uint_ranged_NaN<uint8_t, 0, 7> getFirstSelected() {
        for(uint8_t i = 0; i < multi::switches().size(); ++i) {
            if (multi::switches()[i] == multi::SwState::On) {
                return {i};
            }
        }
        return {};
    }
    
    inline static uint8_t numberOfOnSwitches() {
        uint8_t c{};
        for(const auto sw: multi::switches()) {
            if (sw == multi::SwState::On) {
                ++c;
            }
        }
        return c;
    }
    inline static void resetAll() {
        multi::reset();
    }
    
    inline static pwm::index_type mLearn;
    inline static State mState{State::Init};
    inline static External::Tick<Timer> stateTicks;
};

using fsm = FSM<systemTimer, pwm>;

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
        constexpr auto m0 = Storage::ApplData::value_type{43};
        constexpr auto m1 = Storage::ApplData::value_type{44};
        if (!((appData[Storage::AVKey::Magic0] == m0) && (appData[Storage::AVKey::Magic1] == m1))) {
            appData[Storage::AVKey::Magic0] = m0;
            appData[Storage::AVKey::Magic1] = m1;
            appData[Storage::AVKey::Pwm0] = Storage::ApplData::value_type{};
            appData[Storage::AVKey::Pwm1] = Storage::ApplData::value_type{};
            appData[Storage::AVKey::Pwm2] = Storage::ApplData::value_type{};
            appData[Storage::AVKey::Pwm3] = Storage::ApplData::value_type{};
            appData[Storage::AVKey::Pwm4] = Storage::ApplData::value_type{};
            appData[Storage::AVKey::Pwm5] = Storage::ApplData::value_type{};
            appData.change();
            changed  = true;
        }
    }
    
    Meta::visit<ledList>([]<typename L>(Meta::Wrapper<L>){
                             L::template dir<Output>();
                         });
    
    multi::init();
    
    fsm::init();
    
    const auto periodicTimer = alarmTimer::create(1000_ms, External::Hal::AlarmFlags::Periodic);
    
    etl::outl<terminal>("multi8man"_pgm);
    
    
    while(true) {
        eeprom::saveIfNeeded([&]{
//            fsm::load();                
        });
        adcController::periodic();
        terminalDevice::periodic();
        multi::periodic();        
        
        systemTimer::periodic([&]{
            fsm::ratePeriodic();
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    etl::out<terminal>("sw: [ "_pgm);
                    for(const auto l : multi::switches()) {
                        etl::out<terminal>(uint8_t(l), " "_pgm);
                    }
                    etl::outl<terminal>(" ] a: "_pgm, getConfigValue(), " s: "_pgm, (uint8_t)fsm::mState);

                    appData.expire();
                }
            });
        });
    }
}

