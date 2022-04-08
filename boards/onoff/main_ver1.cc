/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017, 2018 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

// Clock = 8MHz / 8 = 1MHz, EEProm 
// sudo avrdude -p attiny85 -P usb -c avrisp2 -U lfuse:w:0x62:m -U hfuse:w:0xd7:m -U efuse:w:0xff:m

// sudo avrdude -p attiny85 -P usb -c avrisp2 -U flash:w:main.elf

// todo: Einschaltverzögerung 1s, 2s, 3s
// todo: EEPROM abspeichern config
// todo: Config-Mode nach Einschalten + Taster

//todo: Einschaltstrombegrenzung (kurze Impulse über 1ms)

#define NDEBUG

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/mcutimer.h"
#include "mcu/avr/sleep.h"
#include "mcu/avr/groups.h"
#include "hal/alarmtimer.h"
#include "hal/eeprom.h"
#include "util/disable.h"
#include "util/types.h"

template<auto T>
struct static_print {
    std::integral_constant<uint16_t, T> v;
    using type = typename decltype(v)::_;
};

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;

using systemTimer = AVR::Timer8Bit<0>;
using alarmTimer  = AlarmTimer<systemTimer, UseEvents<false>>;

using toneTimer = AVR::Timer8Bit<1>;

using mosfetPin = AVR::ActiveHigh<AVR::Pin<PortB, 4>, AVR::Output>;
using buttonPin = AVR::ActiveLow<AVR::Pin<PortB, 2>, AVR::Input>;
namespace detail {
    using buzzerPin1 = AVR::Pin<PortB, 1>;
    using buzzerPin0 = AVR::Pin<PortB, 0>;
}
using buzzerPins = AVR::PinSet<detail::buzzerPin0, detail::buzzerPin1>;
using testPin   = AVR::Pin<PortB, 3>;

struct Storage {
    enum class AVKey : uint8_t {OnDelay, 
                                SoftStart, 
                                _Number};
    
    class ApplData : public EEProm::DataBase<ApplData> {
    public:
        uint_NaN<uint8_t>& operator[](AVKey key) {
            return AValues[static_cast<uint8_t>(key)];
        }
        const uint_NaN<uint8_t>& operator[](AVKey key) const {
            return AValues[static_cast<uint8_t>(key)];
        }
        bool isValid() const {
            if (((*this)[AVKey::OnDelay].toInt() < 1) || ((*this)[AVKey::OnDelay].toInt() > 3)) return false;            
            if (((*this)[AVKey::SoftStart].toInt() < 1) || ((*this)[AVKey::SoftStart].toInt() > 2)) return false;            
            return true;
        }
        void initialize() {
            (*this)[AVKey::OnDelay] = 1;
            (*this)[AVKey::SoftStart] = 1;
            change();
        }
    private:
        std::array<uint_NaN<uint8_t>, static_cast<uint8_t>(AVKey::_Number)> AValues;
    };
    
    inline static uint8_t configValue = 0;
};

using eeprom = EEProm::Controller<Storage::ApplData>;
auto& appData = eeprom::data();

struct Parameter {
    inline static constexpr auto intervall = 100_ms;
    inline static constexpr uint8_t ticksPerSecond = 1000_ms / intervall;
};

using sleep = AVR::Sleep<>;

template<typename Timer, typename P>
struct Beeper {
    enum class Mode : uint8_t {Off, Periodic, OneShot};
    enum class Tone : uint8_t {Tone, ScaleUp, ScaleDown, Stakkato};
    
    inline static constexpr auto fTone     = 4200_Hz;
    inline static constexpr auto fToneHigh = 4300_Hz;
    inline static constexpr auto fToneLow  = 4100_Hz;
    
    inline static constexpr auto formFactor = 2;
    
    inline static constexpr auto beepDuration  = 200_ms;
    inline static constexpr auto beepTicks  = beepDuration / P::intervall;
    
    inline static constexpr auto stakkatoDuration  = 100_ms;
    inline static constexpr auto stakkatoTicks  = stakkatoDuration / P::intervall;
    
    using value_type = typename Util::TypeForValue_t<beepTicks>;
    
    inline static constexpr auto beepIntervall = 1000_ms;
    inline static constexpr auto beepIntervallTicks  = beepIntervall/ P::intervall;
    static_assert(beepIntervallTicks < std::numeric_limits<value_type>::max());
    
    inline static constexpr auto numberOfPhases = beepIntervallTicks / beepTicks;
    
    //    static_print<beepIntervallTicks> x;
    
    inline static constexpr auto tl = AVR::Util::calculate<Timer>(fToneLow);
    static_assert(tl, "falscher wert für p");
    
    inline static void init() {
        Timer::template prescale<tl.prescaler>();
        Timer::mode(AVR::TimerMode::CTCNoInt);
        normal();
    }
    
    inline static void tick() {
        uint8_t phase = ++mTicks / beepTicks;
        if (mMode != Mode::Off) {
            if (mTone == Tone::Tone) {
                if (phase == 0) {
                    buzzerPins::dir<AVR::Output>();
                }
                else {
                    buzzerPins::dir<AVR::Input>();
                    if (mMode == Mode::OneShot) {
                        mMode = Mode::Off;
                    }
                }
            }
            else if (mTone == Tone::ScaleUp) {
                if (phase == 0) {
                    low();
                    buzzerPins::dir<AVR::Output>();
                }
                else if (phase == 1) {
                    normal();
                }
                else if (phase == 2) {
                    high();
                }
                else {
                    buzzerPins::dir<AVR::Input>();
                }
            }
            else if (mTone == Tone::ScaleDown) {
                if (phase == 0) {
                    high();
                    buzzerPins::dir<AVR::Output>();
                }
                else if (phase == 1) {
                    normal();
                }
                else if (phase == 2) {
                    low();
                }
                else {
                    buzzerPins::dir<AVR::Input>();
                }
            }
            else if (mTone == Tone::Stakkato) {
                uint8_t sphase = mTicks / stakkatoTicks;
                if ((sphase < (2 * mStakkatos)) && ((sphase % 2) == 0)) {
                    buzzerPins::dir<AVR::Output>();
                }
                else {
                    buzzerPins::dir<AVR::Input>();
                }
            }
        }
        else {
            buzzerPins::dir<AVR::Input>();
        }
    }
    inline static void low() {
        Timer::template ocrc<tl.ocr>();
        Timer::template ocra<tl.ocr / formFactor>();
    }            
    inline static void high() {
        constexpr auto ocr = (tl.ocr * fToneLow) / fToneHigh;
        static_assert(ocr <= std::numeric_limits<value_type>::max());
        Timer::template ocrc<ocr>();
        Timer::template ocra<ocr / formFactor>();
    }            
    inline static void normal() {
        constexpr auto ocr = (tl.ocr * fToneLow) / fTone;
        static_assert(ocr <= std::numeric_limits<value_type>::max());
        Timer::template ocrc<ocr>();
        Timer::template ocra<ocr / formFactor>();
    }            
    inline static void on() {
        mTicks = 0;
        mMode = Mode::Periodic;
    }
    inline static void oneShot() {
        mTicks = 0;
        mMode = Mode::OneShot;
    }
    inline static void off() {
        mTicks = 0;
        mMode = Mode::Off;
    }
    inline static void mode(Mode m) {
        mTicks = 0;
        mMode = m;
    }
    inline static void tone(Tone t) {
        mTone = t;
    }
    inline static void stakkatos(uint8_t n) {
        mTicks = 0;
        mStakkatos = n; 
    }
private:
    inline static Mode mMode = Mode::Off;
    inline static Tone mTone = Tone::Tone;
    inline static uint_ranged_circular<value_type, 0, beepIntervallTicks> mTicks;
    inline static uint8_t mStakkatos = 1;
};

using beeper = Beeper<toneTimer, Parameter>;

template<typename P, typename Beeper>
struct FSM {
    inline static constexpr auto startPressDuration = 1000_ms;
    inline static constexpr auto  startPressTicks = startPressDuration / P::intervall;
    
    inline static constexpr auto sleepStartDuration = 3000_ms;
    inline static constexpr auto sleepStartTicks = sleepStartDuration / P::intervall;
    
    using value_type = typename Util::TypeForValue_t<sleepStartTicks>;
    
    enum class State : uint8_t {PowerOn, 
                                PreConfig, Config, 
                                OnDelay,
                                Warn, WarnOff, Off, WaitForOn, PreOn, On, WaitForOff, PreOff};
    
    inline static void tick() {
        State newState = state;
        switch(state) {
        case State::PowerOn:
            if (buttonPin::isActive()) {
                newState = State::PreConfig;
            }
            else {
                pressTicks = 0;
                if (++stateTicks >= beeper::beepIntervallTicks) {
                    newState = State::Warn;
                }
            }
            break;
        case State::PreConfig:
            if (buttonPin::isActive()) {
                if (++pressTicks > startPressTicks) {
                    newState = State::Config;
                }
            }
            else {
                newState = State::Warn;
            }
            break;
        case State::Config:
            if (buttonPin::isActive()) {
                stateTicks = 0;
                if (++pressTicks >= (3 * startPressTicks)) {
                    pressTicks = 0;

                    Storage::configValue += 1;
                    
                    if (Storage::configValue > 5) {
                        Storage::configValue = 1;
                    }
                    if (Storage::configValue < 1) {
                        Storage::configValue = 1;
                    } 
                    
                    if (Storage::configValue <= 3) {
                        beeper::low();
                        beeper::stakkatos(Storage::configValue);
                    }
                    else {
                        beeper::high();
                        beeper::stakkatos(Storage::configValue - 3);
                    }
                }
            }
            else {
                pressTicks = 0;
                if (stateTicks < std::numeric_limits<value_type>::max()) {
                    if (++stateTicks > (5 * startPressTicks)) {
                        if (Storage::configValue <= 3) {
                            appData[Storage::AVKey::OnDelay] = Storage::configValue;
                            appData.change();
                        }
                        else {
                            appData[Storage::AVKey::SoftStart] = Storage::configValue - 3;
                            appData.change();
                        }
                        beeper::tone(beeper::Tone::ScaleDown);
                    }
                    if (stateTicks > (7 * startPressTicks)) {
                        beeper::off();
                        stateTicks = std::numeric_limits<value_type>::max();
                    }
                }
            }
            break;
        case State::Warn:
            if (++stateTicks >= beeper::beepIntervallTicks) {
                newState = State::WarnOff;
            }
            break;
        case State::WarnOff:
            if (++stateTicks >= beeper::beepIntervallTicks) {
                newState = State::Off;
            }
            break;
        case State::Off:
            if (buttonPin::isActive()) {
                if (!buttonStillPressed) {
                    if (++pressTicks > startPressTicks) {
                        newState = State::WaitForOn;
                    }
                }
            }
            else {
                buttonStillPressed = false;
                pressTicks = 0;
                ++sleepTicks;
                if (sleepTicks > sleepStartTicks) {
                    sleep::down();
                    sleepTicks = 0;
                }
            }
            break;
        case State::WaitForOn:
            if (buttonPin::isActive()) {
                if (++pressTicks > (3 * startPressTicks)) {
                    newState = State::PreOn;
                }
            }
            else {
                newState = State::Off;
            }
            break;
        case State::PreOn:
            if (buttonPin::isActive()) {
                if (++pressTicks > startPressTicks) {
                    newState = State::OnDelay;
                    buttonStillPressed = true;
                }
            }
            else {
                buttonStillPressed = false;
                newState = State::Off;
            }
            break;
        case State::OnDelay:
        {
            auto n = appData[Storage::AVKey::OnDelay];
            if (n) {
                if (++stateTicks > (n.toInt() * Parameter::ticksPerSecond)) {
                    newState = State::On;
                } 
            }
            else {
                assert(false);
            }
        }
            break;
        case State::On:
            if (buttonPin::isActive()) {
                if (!buttonStillPressed) {
                    if (++pressTicks > startPressTicks) {
                        newState = State::WaitForOff;
                    }
                }
            }
            else {
                buttonStillPressed = false;
                pressTicks = 0;
            }
            break;
        case State::WaitForOff:
            if (buttonPin::isActive()) {
                if (++pressTicks > (3 * startPressTicks)) {
                    newState = State::PreOff;
                }
            }
            else {
                newState = State::On;
            }
            break;
        case State::PreOff:
            if (buttonPin::isActive()) {
                if (++pressTicks > startPressTicks) {
                    newState = State::Off;
                    buttonStillPressed = true;
                }
            }
            else {
                buttonStillPressed = false;
                newState = State::On;
            }
            break;
        default:
            assert(false);
            break;
        }
        if (newState != state) {
            pressTicks = 0;
            stateTicks = 0;            
            switch(state = newState) {
            case State::PowerOn:
                mosfetPin::inactivate();
                break;
            case State::PreConfig:
                break;
            case State::Config:
                Storage::configValue = 1;
                beeper::low();
                beeper::tone(beeper::Tone::Stakkato);
                beeper::stakkatos(Storage::configValue);
                beeper::on();
                break;
            case State::Warn:
                beeper::tone(beeper::Tone::ScaleUp);
                beeper::on();
                break;
            case State::WarnOff:
                beeper::off();
                beeper::tone(beeper::Tone::Tone);
                break;
            case State::Off:
                Beeper::low();
                Beeper::oneShot();
                mosfetPin::inactivate();
                break;
            case State::WaitForOn:
            case State::WaitForOff:
                Beeper::normal();
                Beeper::on();
                break;
            case State::PreOn:
                Beeper::high();
                break;
            case State::OnDelay:
                Beeper::high();
                Beeper::oneShot();
                break;
            case State::On:
                if (mosfetPin::activated()) {
                    beeper::high();
                    Beeper::oneShot();
                }
                softStart();
                break;
            case State::PreOff:
                Beeper::low();
                break;
            default:
                assert(false);
                break;
            }
        }
    }
    inline static void softStart() {
        if (!mosfetPin::activated()) {
            if (appData[Storage::AVKey::SoftStart].toInt() > 1) {
                for(uint8_t counter = 0; counter < 100; ++counter) {
                    mosfetPin::activate();        
                    Util::delay(5_us); // 20us on
                    mosfetPin::inactivate();        
                    Util::delay(200_us); // 200us off -> 1/11 PWM
                }
            }
            mosfetPin::activate();        
        }
    }
    inline static bool buttonStillPressed = false;
    inline static State state = State::PowerOn;
    inline static value_type pressTicks = 0;
    inline static value_type sleepTicks = 0;
    inline static value_type stateTicks = 0;
};

using fsm = FSM<Parameter, beeper>;

struct Button : public IsrBaseHandler<AVR::ISR::Int<0>> {
    inline static void isr() {}
};

using isrRegistrar = IsrRegistrar<Button>;

int main() {
    eeprom::init();
    
    if (!appData.isValid()) {
        appData.initialize();
    }

    isrRegistrar::init();
    
    mosfetPin::init();
    buttonPin::init();
    
    testPin::dir<AVR::Output>();
    testPin::off();
    
    systemTimer::setup<Config::Timer::frequency>(AVR::TimerMode::CTCNoInt);
    auto tickTimer = alarmTimer::create(Parameter::intervall, AlarmFlags::Periodic);
    auto eepromTimer = alarmTimer::create(500_ms, AlarmFlags::Periodic);
    
    beeper::init();
    
    constexpr auto interrupts = AVR::getBaseAddr<AVR::ATTiny85::Interrupt>;
    interrupts()->gifr.reset<AVR::ATTiny85::Interrupt::GIFlags::intf>();
    interrupts()->gimsk.set<AVR::ATTiny85::Interrupt::GIMask::ie>();
    
    sleep::init<sleep::PowerDown>();
    
    {
        Scoped<EnableInterrupt<>> ei;
        while(true) {
            if(eeprom::saveIfNeeded()) {
                testPin::on();
            }
            systemTimer::periodic<systemTimer::flags_type::ocf0a>([&](){
                alarmTimer::periodic([&](uint7_t timer){
                    if (timer == *tickTimer) {
                        fsm::tick();
                        beeper::tick();
                    }                
                    if (timer == *eepromTimer) {
                        appData.expire();
                        testPin::off();
                    }
                });
            });
        }
    }
}


ISR(INT0_vect) {
    isrRegistrar::isr<AVR::ISR::Int<0>>();
}

#ifndef NDEBUG
void assertFunction(const PgmStringView&, const PgmStringView&, unsigned int) noexcept {
    while(true) {
        testPin::toggle();
    }
}
#endif
