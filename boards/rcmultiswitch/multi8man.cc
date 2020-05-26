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

using terminalDevice = AVR::Usart<usart0Position, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<2>, AVR::SendQueueLength<512>>;
using terminal = etl::basic_ostream<terminalDevice>;

auto& appData = eeprom::data();

namespace External {
    namespace Ppm {
        template<typename Ppm, uint8_t N, typename MCU = DefaultMcuType>
        struct MultiSwitch {
            using gpior_t = typename MCU::Gpior; 
            static inline constexpr auto flags = AVR::getBaseAddr<gpior_t, 1>;
            static inline constexpr uint8_t enableMask = 0x01;

            enum class State : uint8_t {UnDefined, CountUp, CountUpPause, ToggleUp, CountDown, CountDownPause, ToggleDown};
            enum class SwState : uint8_t {Off, On};
            
            using index_t = etl::uint_ranged<uint8_t, 0, ((N / 2) - 1)>;
            
            inline static constexpr uint8_t size() {
                return N;
            }
            inline static void init() {
                Ppm::init();
                enable(true);
            }
            inline static void reset() {
                for(auto& s : swStates) {
                    s = SwState::Off;
                }
            }
            inline static void enable(const bool en = true) {
                if (en) {
                    flags()->data = flags()->data | enableMask;
                }
                else {
                    flags()->data = flags()->data & ~enableMask;
                }
            }            
            inline static void periodic() {
                if (flags()->data & enableMask) {
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
    inline static constexpr Storage::tick_type initTime{1000_ms};
//        std::integral_constant<uint16_t, initTime.value>::_;
    
    inline static constexpr auto ppmDead = ppm::medium + ppm::span / 20;    
    inline static constexpr auto ppmDead2 = ppm::medium - ppm::span / 20;    
//    std::integral_constant<uint16_t, ppm::ppmMax>::_;
//    std::integral_constant<uint16_t, ppm::medium>::_;
//    std::integral_constant<uint16_t, ppmDead>::_;
    
    enum class State : uint8_t {Init, Run, 
                                LearnPWMStart, LearnPWM, LearnPWMEnd,
                                LearnBlinkIntStart, LearnBlinkInt, LearnBlinkIntEnd,
                                LearnBlinkDurStart, LearnBlinkDur, LearnBlinkDurEnd,
                               };
    
    inline static void init() {
        PWM::init();
        Meta::visit<ledList>([]<typename L>(Meta::Wrapper<L>){
                                 L::off();
                                 L::template dir<Output>();
                             });
        for (uint8_t l = 0; l < multi::size(); ++l) {
            if (l <= pwm::index_type::Upper) {
                auto li = pwm::index_type(l);
                if (auto v = appData[mapToKey(l)].pwmValue()) {
                        pwm::pwm(li, 0);
                        pwm::on(li);
                }
            }
        }
    }
    
    using index_t = etl::uint_ranged<uint8_t, 0, multi::size() - 1>;
    
    inline static void setSwitchOff(const index_t index) {
        if (index <= pwm::index_type::Upper) {
            auto li = pwm::index_type(index);
            if (auto v = appData[mapToKey(index)].pwmValue()) {
                    pwm::pwm(li, 0);
            }
            else {
                Meta::visitAt<ledList>(index, []<typename L>(Meta::Wrapper<L>){
                                           L::off();
                                       });
            }
        }
        else {
            Meta::visitAt<ledList>(index, []<typename L>(Meta::Wrapper<L>){
                                       L::off();
                                   });
        }
    }
    inline static void setSwitchOn(const index_t index) {
        if (index <= pwm::index_type::Upper) {
            auto li = pwm::index_type(index);
            if (auto v = appData[mapToKey(index)].pwmValue()) {
                pwm::on(li);
                pwm::pwm(li, v.toInt());
            }
            else {
                Meta::visitAt<ledList>(index, []<typename L>(Meta::Wrapper<L>){
                                           L::on();
                                       });
            }
        }
        else {
            Meta::visitAt<ledList>(index, []<typename L>(Meta::Wrapper<L>){
                                       L::on();
                                   });
        }    
    }
    
    inline static void setSwitch(const index_t index) {
        if (multi::switches()[index] == multi::SwState::Off) {
            setSwitchOff(index);
            blinkTicks[index] = appData[mapToKey(index)].blinks()[0].intervall;
        }
        else if (multi::switches()[index] == multi::SwState::On) {
            if (appData[mapToKey(index)].blinks()[0].duration) {
                blinkTicks[index].match(appData[mapToKey(index)].blinks()[0].duration, [&]{
                    setSwitchOff(index);
                });
                blinkTicks[index].on(appData[mapToKey(index)].blinks()[0].intervall, [&]{
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
    
    template<typename T>
    static inline Storage::AVKey mapToKey(const T i) {
        switch(i) {
        case 0:
            return Storage::AVKey::Ch0;
        case 1:
            return Storage::AVKey::Ch1;
        case 2:
            return Storage::AVKey::Ch2;
        case 3:
            return Storage::AVKey::Ch3;
        case 4:
            return Storage::AVKey::Ch4;
        case 5:
            return Storage::AVKey::Ch5;
        case 6:
            return Storage::AVKey::Ch6;
        case 7:
            return Storage::AVKey::Ch7;
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
            if (getConfigValue() == 2) {
                if (numberOfOnSwitches() > 1) {
                    resetAll();                    
                }
                else {
                    if (auto c = getFirstSelected()) {
                        if (c <= pwm::index_type::Upper) {
                            mLearn.set(c);
                            multi::enable(false);
                            mState = State::LearnPWMStart;
                        }
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
                        multi::enable(false);
                        mState = State::LearnBlinkIntStart;
                    }
                }
            }
            else {
                setSwitches();
            }
            break;
        case State::LearnPWMStart:
            if (getConfigValue() == 2) {
                if (ppm::value() > ppmDead) {
                    mState = State::LearnPWM;
                }
                else {
                    pwm::on(mLearn);
                    appData[mapToKey(mLearn)].pwmValue(2);
                    setSwitch(mLearn);
                }
            }
            else {
                resetAll();
                mState = State::Run;
            }
            break;
        case State::LearnBlinkIntStart:
            if (getConfigValue() == 4) {
                if (ppm::value() > ppmDead) {
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
                if (ppm::value() > ppmDead) {
                    mState = State::LearnBlinkDur;
                }
            }
            break;
        case State::LearnPWM:
            if (getConfigValue() == 2) {
                if (uint32_t x = ppm::value(); x > ppm::medium) {
                    uint8_t p = (((uint32_t)x - ppm::medium) * pwm::pwmMax) / (ppm::span / 2);
                    appData[mapToKey(mLearn)].pwmValue(p);
                    setSwitch(mLearn);
                }
            }
            else {
                mState = State::LearnPWMEnd;
            }
            break;
        case State::LearnBlinkInt:
            if (getConfigValue() == 4) {
                if (auto x = ppm::value(); x > ppm::medium) {
                    uint8_t p = (((uint32_t)x - ppm::medium) * 99) / (ppm::span / 2);
                    appData[mapToKey(mLearn)].blinks()[0].duration = Storage::tick_type::fromRaw(p / 2);
                    appData[mapToKey(mLearn)].blinks()[0].intervall = Storage::tick_type::fromRaw(p);
                    setSwitch(mLearn);
                }
            }
            else {
                mState = State::LearnBlinkDurStart;
            }
            break;
        case State::LearnBlinkDur:
            if (getConfigValue() == 4) {
                if (auto x = ppm::value(); x > ppm::medium) {
                    uint8_t p = (((uint32_t)x - ppm::medium) * (appData[mapToKey(mLearn)].blinks()[0].intervall.value.toInt() - 1)) / (ppm::span / 2);
                    appData[mapToKey(mLearn)].blinks()[0].duration = Storage::tick_type::fromRaw(p);
                    setSwitch(mLearn);
                }
            }
            else {
                mState = State::LearnBlinkDurEnd;
            }
            break;
        case State::LearnPWMEnd:
            if (auto x = ppm::value(); x < ppmDead2) {
                appData.change();
                pwm::off(mLearn);
                resetAll();
                multi::enable(true);
                mState = State::Run;
            }
            break;
        case State::LearnBlinkIntEnd:
            mState = State::LearnBlinkDurStart;
            break;
        case State::LearnBlinkDurEnd:
            if (auto x = ppm::value(); x < ppmDead2) {
                appData.change();
                resetAll();
                multi::enable(true);
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
        for(uint8_t i = 0; i < multi::switches().size(); ++i) {
            if (multi::switches()[i] == multi::SwState::On) {
                return {i};
            }
        }
        return {};
    }
    
    inline static uint8_t numberOfOnSwitches() {
        uint8_t c{};
        for(const auto& sw: multi::switches()) {
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
    inline static Storage::tick_type stateTicks;
    inline static std::array<Storage::tick_type, 8> blinkTicks;
};

using fsm = FSM<SoftTimer, pwm>;

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
    
    Meta::visit<ledList>([]<typename L>(Meta::Wrapper<L>){
                             L::template dir<Output>();
                         });
    
    multi::init();
    
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
                    etl::outl<terminal>(" ] a: "_pgm, getConfigValue(), " s: "_pgm, (uint8_t)fsm::mState, " bt0: "_pgm, fsm::blinkTicks[0].value.toInt(), " i: "_pgm, ii, " d: "_pgm, dd);

                    appData.expire();
                }
            });
        });
    }
}

