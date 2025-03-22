/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2016 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

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

#define USE_PPM
#define SBUS_IBUS_NO_WARN

#include "board.h"
#include "swout.h"

using eeprom = EEProm::Controller<Storage::ApplData<etl::uint_ranged_NaN<uint8_t, 0, 7>>>;

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
    
    inline static constexpr auto ppmDead  = ppmMedium + ppmSpan / 20;    
    inline static constexpr auto ppmDead2 = ppmMedium - ppmSpan / 20;    
    inline static constexpr auto ppmFull  = ppm_type::Upper - ppmSpan / 10;    
    //    std::integral_constant<uint16_t, ppm::ppmMax>::_;
    //    std::integral_constant<uint16_t, ppm::medium>::_;
    //    std::integral_constant<uint16_t, ppmDead>::_;

    using nvm_t = std::remove_cvref_t<decltype(NVM::data())>;
    using nvm_data_t = nvm_t::value_type;
    using pwm_type = nvm_data_t::pwm_type;
    
    using blink_index_t = Out::blink_index_t;
    
    enum class State : uint8_t {Init, Run, 
                                LearnPWMStart, LearnPWM, LearnPWMEnd,
                                LearnBlinkIntStart, LearnBlinkInt, LearnBlinkIntEnd,
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
                    if (const auto c = getFirstSelected(); c) {
                        if (const auto ci = c.toInt(); ci <= Out::pwm_index_t::Upper) {
                            mLearn.set(ci);
                            Input::enable(false);
                            mState = State::LearnPWMStart;
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
                    if (const auto c = getFirstSelected(); c) {
                        mLearn.set(c.toInt());
                        Input::enable(false);
                        Out::setSwitchOff(mLearn);
                        mState = State::LearnBlinkIntStart;
                    }
                }
            }
            else {
                Out::setSwitches();
            }
            break;
        case State::LearnPWMStart:
            if (Input::ppm().toInt() > ppmDead) {
                mState = State::LearnPWM;
            }
            else if (Input::ppm().toInt() < ppmDead2) {
                resetAll();
                mState = State::Run;
            }
            else {
                NVM::data()[mLearn].pwmValue(1);
                Out::setSwitch(mLearn);
            }
            break;
        case State::LearnBlinkIntStart:
            if (const auto v = Input::ppm(); v) {
                const auto vi = v.toInt();
                if (vi > ppmDead) {
                    Out::setSwitchOn(mLearn);
                    mState = State::LearnBlinkInt;
                }
                else if (vi < ppmDead2) {
                    resetAll();
                    mState = State::Run;
                }
            }
            break;
        case State::LearnBlinkDurStart:
            if (Input::ppm().toInt() > ppmDead) {
                mState = State::LearnBlinkDur;
            }
            break;
        case State::LearnPWM:
            if (getConfigValue() == 2) {
                mState = State::LearnPWMEnd;
            }
            else {
                if (const auto x = Input::ppm(); x) {
                    if (const auto xi = x.toInt(); xi > ppmMedium) {
                        if (xi > ppmFull) {
                            NVM::data()[mLearn].pwmValue(pwm_type{});    
                        }
                        else {
                            uint8_t p = (((uint32_t)xi - ppmMedium) * Out::pwmMax) / ppmSpan;
                            NVM::data()[mLearn].pwmValue(p);
                        }
                        Out::setSwitch(mLearn);
                    }
                }
            }
            break;
        case State::LearnBlinkInt:
            if (getConfigValue() == 4) {
                mState = State::LearnBlinkIntEnd;
            }
            else {
                if (const auto x = Input::ppm(); x) {
                    if (const auto xi = Input::ppm().toInt(); xi > ppmMedium) {
                        uint8_t p = (((uint32_t)xi - ppmMedium) * Out::blinkMax) / ppm::span;
                        Out::duration(mLearn, Storage::tick_type::fromRaw(p / 2), blink_index_t{0});
                        Out::intervall(mLearn, Storage::tick_type::fromRaw(p), blink_index_t{0});
                        Out::setSwitch(mLearn);
                    }
                }
            }
            break;
        case State::LearnBlinkDur:
            if (getConfigValue() == 4) {
                mState = State::LearnBlinkDurEnd;
            }
            else {
                if (const auto x = Input::ppm(); x) {
                    if (const auto xi = Input::ppm().toInt(); xi > ppmMedium) {
                        uint8_t p = (((uint32_t)xi - ppmMedium) * (Out::intervall(mLearn, blink_index_t{0}).value.toInt() - 1)) / ppm::span;
                        Out::duration(mLearn, Storage::tick_type::fromRaw(p), blink_index_t{0});
    //                    NVM::data()[mLearn].blinks()[0].duration = Storage::tick_type::fromRaw(p);
                        Out::setSwitch(mLearn);
                    }
                }
            }
            break;
        case State::LearnPWMEnd:
            if (const auto x = Input::ppm(); x) {
                if (x.toInt() < ppmDead2) {
                    NVM::data().change();
                    Out::setSwitchOff(mLearn);
                    resetAll();
                    Input::enable(true);
                    mState = State::Run;
                }
            }
            break;
        case State::LearnBlinkIntEnd:
            if (Input::ppm().toInt() < ppmDead2) {
                mState = State::LearnBlinkDurStart;
            }
            break;
        case State::LearnBlinkDurEnd:
            if (const auto x = Input::ppm(); x) {
                if (x.toInt() < ppmDead2) {
                    NVM::data().change();
                    resetAll();
                    Input::enable(true);
                    mState = State::Run;
                }
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
    }
    inline static Out::index_t mLearn;
    inline static State mState{State::Init};
    inline static Storage::tick_type stateTicks;
};

using terminalDevice = AVR::Usart<usart0Position, External::Hal::NullProtocollAdapter<>, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<2>, AVR::SendQueueLength<512>>;
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
        if (!((appData[Storage::AVKey::Magic].pwmValue().toInt() == 43))) {
            appData[Storage::AVKey::Magic].pwmValue(43);
            appData.clear();
            appData.change();
            changed  = true;
        }
    }
    
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
                    etl::outl<terminal>(" ] a: "_pgm, getConfigValue(), " s: "_pgm, (uint8_t)fsm::mState, 
//                                        " bt0: "_pgm, out::blinkTicks[0].value.toInt(), 
                            " i: "_pgm, ii, " d: "_pgm, dd);
                    
                    appData.expire();
                }
            });
        });
    }
}

