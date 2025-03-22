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

#define VERSION "1.11"
#define TITLE   "WM-Switch"

#define NDEBUG

#define USE_HOTT
#define SBUS_IBUS_NO_WARN

#include "board.h"
#include "swout.h"

#ifdef USE_HOTT

#define GAM
//#define AIR
//#define ESC
//#define VARIO
//#define GPS

#ifdef GAM
# define SUFFIX "GAM"
using sensor = Hott::Experimental::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<19200>, Hott::GamMsg, Hott::TextMsg, systemTimer>;
#endif
#ifdef AIR
# define SUFFIX "AIR"
using sensor = Hott::Experimental::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<19200>, Hott::AirMsg, Hott::TextMsg, systemTimer>;
#endif
#ifdef ESC
# define SUFFIX "ESC"
using sensor = Hott::Experimental::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<19200>, Hott::EscMsg, Hott::TextMsg, systemTimer>;
#endif
#ifdef VARIO
# define SUFFIX "VAR"
using sensor = Hott::Experimental::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<19200>, Hott::VarioMsg, Hott::TextMsg, systemTimer>;
#endif
#ifdef GPS
# define SUFFIX "GPS"
using sensor = Hott::Experimental::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<19200>, Hott::GpsMsg, Hott::TextMsg, systemTimer>;
#endif

#define FULLTITLE TITLE " " VERSION " " SUFFIX

using storage_t = Storage::ApplData<etl::uint_ranged_NaN<uint8_t, 0, 7>>;
using eeprom = EEProm::Controller<storage_t>;

template<auto N = 8>
struct SwitchStates {
    enum class SwState : uint8_t {Off, Steady, Blink1, Blink2};
    enum class Pattern : uint8_t {Off, Run};
    
    static constexpr void init() {}
    static constexpr uint8_t size() {
        return N;
    }
    static inline auto& switches() {
        return swStates;
    }
    static inline void off() {
        mPattern = Pattern::Off;
        for(auto& i : swStates) {
            i = SwState::Off;
        }
    }
    static inline void on() {
        mPattern = Pattern::Off;
        for(auto& i : swStates) {
            i = SwState::Steady;
        }
    }
    static inline void pattern(Pattern p) {
        mPattern = p;
    }
    static inline auto pattern() {
        return mPattern;
    }
    static inline void ratePeriodic() {
        switch(mPattern) {
        case Pattern::Off:
            break;
        case Pattern::Run:
            swStates[mIndex] = SwState::Off;
            ++mIndex;
            swStates[mIndex] = SwState::Steady;
            break;
        }
    }
private:
    static inline std::array<SwState, N> swStates{};
    static inline etl::uint_ranged_circular<uint8_t, 0, N-1> mIndex;
    static inline Pattern mPattern = Pattern::Off;
};

using sw = SwitchStates<>;

using out = External::Output<ledList, pwm, sw, eeprom>;

auto& appData = eeprom::data();

struct SwitchesMenu final : public Hott::Menu<8, false, 8> {
    struct Adapter {
        sw::SwState& operator[](const auto i) {
            if (i == lastSelected) {
                return lastState;
            }
            else {
                return sw::switches()[Storage::keyToInt(i)];    
            }
        }
        void change() {
            sw::switches()[Storage::keyToInt(lastSelected)] = lastState;
            lastSelected = Storage::AVKey::Undefined;
        }
        void select(const auto i) {
            lastSelected = i;
            lastState = sw::switches()[Storage::keyToInt(i)];    
        }
        inline static void format(const uint8_t vi, etl::span<6, etl::Char>& b) {
            const sw::SwState v{vi};
            switch(v) {
            case sw::SwState::Off:
                b.insertLeftFill("Off"_pgm);
                break;
            case sw::SwState::Steady:
                b.insertLeftFill("On"_pgm);
                break;
            case sw::SwState::Blink1:
                b.insertLeftFill("Blink1"_pgm);
                break;
            case sw::SwState::Blink2:
                b.insertLeftFill("Blink2"_pgm);
                break;
            }
        }
    private:
        Storage::AVKey lastSelected = Storage::AVKey::Undefined;
        sw::SwState lastState;
    };
    
    SwitchesMenu(auto* const parent) : Hott::Menu<8, false, 8>{parent, "Switches"_pgm, 
                                                               &mSW0, &mSW1, &mSW2, &mSW3, &mSW4, &mSW5, &mSW6, &mSW7} {}    
    Hott::TextWithValue<Storage::AVKey, Adapter, 6, Adapter> mSW0{"Output A :"_pgm, a, Storage::AVKey::Ch0, (uint8_t)sw::SwState::Blink2};
    Hott::TextWithValue<Storage::AVKey, Adapter, 6, Adapter> mSW1{"Output B :"_pgm, a, Storage::AVKey::Ch1, (uint8_t)sw::SwState::Blink2};
    Hott::TextWithValue<Storage::AVKey, Adapter, 6, Adapter> mSW2{"Output C :"_pgm, a, Storage::AVKey::Ch2, (uint8_t)sw::SwState::Blink2};
    Hott::TextWithValue<Storage::AVKey, Adapter, 6, Adapter> mSW3{"Output D :"_pgm, a, Storage::AVKey::Ch3, (uint8_t)sw::SwState::Blink2};
    Hott::TextWithValue<Storage::AVKey, Adapter, 6, Adapter> mSW4{"Output E :"_pgm, a, Storage::AVKey::Ch4, (uint8_t)sw::SwState::Blink2};
    Hott::TextWithValue<Storage::AVKey, Adapter, 6, Adapter> mSW5{"Output F :"_pgm, a, Storage::AVKey::Ch5, (uint8_t)sw::SwState::Blink2};
    Hott::TextWithValue<Storage::AVKey, Adapter, 6, Adapter> mSW6{"Output G :"_pgm, a, Storage::AVKey::Ch6, (uint8_t)sw::SwState::Blink2};
    Hott::TextWithValue<Storage::AVKey, Adapter, 6, Adapter> mSW7{"Output H :"_pgm, a, Storage::AVKey::Ch7, (uint8_t)sw::SwState::Blink2};
    
    Adapter a;
};


struct PwmMenu final : public Hott::Menu<8, false, 6> {
    struct Adapter {
        eeprom::data_t::value_type::pwm_type& operator[](const Storage::AVKey k) {
            return appData[k].pwmValue();
        }
        void change() {
            appData.change();
        }
        void select(const Storage::AVKey k) {
            sw::off();
            sw::switches()[Storage::keyToInt(k)] = sw::SwState::Steady;
            out::setSwitches();
        }
    };
    
    using pwm_type = storage_t::value_type::pwm_type;
    
    PwmMenu(auto* const parent) : Menu{parent, "Pwm Parameter"_pgm, &mPwm0, &mPwm1, &mPwm2, &mPwm3, &mPwm4, &mPwm5
                                       //, &mPwm6, &mPwm7
                                        } {}    
    Hott::TextWithValue<Storage::AVKey, Adapter, 3, void, 5> mPwm0{"PWM A :"_pgm, a0, Storage::AVKey::Ch0, pwm_type::Upper};
    Hott::TextWithValue<Storage::AVKey, Adapter, 3, void, 5> mPwm1{"PWM B :"_pgm, a1, Storage::AVKey::Ch1, pwm_type::Upper};
    Hott::TextWithValue<Storage::AVKey, Adapter, 3, void, 5> mPwm2{"PWM C :"_pgm, a2, Storage::AVKey::Ch2, pwm_type::Upper};
    Hott::TextWithValue<Storage::AVKey, Adapter, 3, void, 5> mPwm3{"PWM D :"_pgm, a3, Storage::AVKey::Ch3, pwm_type::Upper};
    Hott::TextWithValue<Storage::AVKey, Adapter, 3, void, 5> mPwm4{"PWM E :"_pgm, a4, Storage::AVKey::Ch4, pwm_type::Upper};
    Hott::TextWithValue<Storage::AVKey, Adapter, 3, void, 5> mPwm5{"PWM F :"_pgm, a5, Storage::AVKey::Ch5, pwm_type::Upper};
//    Hott::TextWithValue<Storage::AVKey, Adapter, 3, void, 5> mPwm6{"PWM G :"_pgm, a6, Storage::AVKey::Ch6, pwm_type::Upper};
//    Hott::TextWithValue<Storage::AVKey, Adapter, 3, void, 5> mPwm7{"PWM H :"_pgm, a7, Storage::AVKey::Ch7, pwm_type::Upper};
    
    Adapter a0;
    Adapter a1;
    Adapter a2;
    Adapter a3;
    Adapter a4;
    Adapter a5;
//    Adapter a6;
//    Adapter a7;
};

template<uint8_t I, auto ValueWidth = 3>
class Tick final : public UI::MenuItem<Hott::BufferString, Hott::key_t> {
public:
    static inline constexpr uint8_t valueWidth = ValueWidth;
    static inline constexpr uint8_t valueBeginColumn1 = Hott::BufferString::size() - ValueWidth;
    static inline constexpr uint8_t valueBeginColumn2 = Hott::BufferString::size() - 2*ValueWidth - 1;
    
    using value_span_type = etl::span<valueWidth, etl::Char>;
    
    Tick(const AVR::Pgm::StringView& text, Storage::AVKey number) : mTitle{text}, mNumber{number} {}
    
    void valueToText(const Storage::tick_type v, value_span_type buffer) const {
        if (v) {
            etl::itoa_r<10>(v.value.toInt(), buffer);
        }
        else {
            etl::fill(buffer, etl::Char{'-'});
        }
    }
    
    virtual void putTextInto(Hott::BufferString& buffer) const override {
        buffer[0] = etl::Char{' '};
        buffer.insertAtFill(1, mTitle);
        
        valueToText(appData[mNumber].blinks()[I].intervall, etl::make_span<valueBeginColumn1, valueWidth>(buffer));
        valueToText(appData[mNumber].blinks()[I].duration, etl::make_span<valueBeginColumn2, valueWidth>(buffer));
        
        if (mSelected) {
            etl::apply(etl::make_span<valueBeginColumn1, valueWidth>(buffer), [](auto& c) {c |= etl::Char{0x80};});
            etl::apply(etl::make_span<valueBeginColumn2, valueWidth>(buffer), [](auto& c) {c |= etl::Char{0x80};});
        }
    }
    using blink_index_t = out::blink_index_t;
    virtual MenuItem* processKey(const Hott::key_t key) override {
        switch (key) {
        case Hott::key_t::up:
            if (!appData[mNumber].blinks()[I].intervall) {
                ++appData[mNumber].blinks()[I].intervall;
            }
            ++appData[mNumber].blinks()[I].intervall;
            if (!appData[mNumber].blinks()[I].duration) {
                appData[mNumber].blinks()[I].duration = Storage::tick_type::fromRaw(appData[mNumber].blinks()[I].intervall.value.toInt() / 2); 
            }
            out::setSwitches();
            break;
        case Hott::key_t::down:
            --appData[mNumber].blinks()[I].intervall;
            if (!appData[mNumber].blinks()[I].duration) {
                appData[mNumber].blinks()[I].duration = Storage::tick_type::fromRaw(appData[mNumber].blinks()[I].intervall.value.toInt() / 2); 
            }
            out::setSwitches();
            break;
        case Hott::key_t::left:
            if (appData[mNumber].blinks()[I].duration < appData[mNumber].blinks()[I].intervall) {
                ++appData[mNumber].blinks()[I].duration;
            }
            out::setSwitches();
            break;
        case Hott::key_t::right:
            --appData[mNumber].blinks()[I].duration;
            out::setSwitches();
            break;
        case Hott::key_t::nokey:
            break;
        case Hott::key_t::set:
            if (mSelected) {
                appData.change();
            }
            else {
                sw::off();
                if constexpr(I == 0) {
                    sw::switches()[Storage::keyToInt(mNumber)] = sw::SwState::Blink1;
                }
                else {
                    sw::switches()[Storage::keyToInt(mNumber)] = sw::SwState::Blink2;
                }
                out::setSwitches();
            }
            mSelected = !mSelected;
            break;
        }
        return this;
    }
private:
    const AVR::Pgm::StringView mTitle;
    const Storage::AVKey mNumber;
};

struct Blink1Menu final : public Hott::Menu<8, false, 8> {
    Blink1Menu(auto* const parent) : Menu{parent, "Blink 1 Parameter"_pgm, &mB0, &mB1, &mB2, &mB3, &mB4, &mB5, &mB6, &mB7} {}    
    
    Tick<0> mB0{"Blink1 A D/I:"_pgm, Storage::AVKey::Ch0};
    Tick<0> mB1{"Blink1 B D/I:"_pgm, Storage::AVKey::Ch1};
    Tick<0> mB2{"Blink1 C D/I:"_pgm, Storage::AVKey::Ch2};
    Tick<0> mB3{"Blink1 D D/I:"_pgm, Storage::AVKey::Ch3};
    Tick<0> mB4{"Blink1 E D/I:"_pgm, Storage::AVKey::Ch4};
    Tick<0> mB5{"Blink1 F D/I:"_pgm, Storage::AVKey::Ch5};
    Tick<0> mB6{"Blink1 G D/I:"_pgm, Storage::AVKey::Ch6};
    Tick<0> mB7{"Blink1 H D/I:"_pgm, Storage::AVKey::Ch7};
};

struct Blink2Menu final : public Hott::Menu<8, false, 8> {
    Blink2Menu(auto* const parent) : Menu{parent, "Blink 2 Parameter"_pgm, &mB0, &mB1, &mB2, &mB3, &mB4, &mB5, &mB6, &mB7} {}    
    
    Tick<1> mB0{"Blink2 A D/I:"_pgm, Storage::AVKey::Ch0};
    Tick<1> mB1{"Blink2 B D/I:"_pgm, Storage::AVKey::Ch1};
    Tick<1> mB2{"Blink2 C D/I:"_pgm, Storage::AVKey::Ch2};
    Tick<1> mB3{"Blink2 D D/I:"_pgm, Storage::AVKey::Ch3};
    Tick<1> mB4{"Blink2 E D/I:"_pgm, Storage::AVKey::Ch4};
    Tick<1> mB5{"Blink2 F D/I:"_pgm, Storage::AVKey::Ch5};
    Tick<1> mB6{"Blink2 G D/I:"_pgm, Storage::AVKey::Ch6};
    Tick<1> mB7{"Blink2 H D/I:"_pgm, Storage::AVKey::Ch7};
};

template<typename CB, auto ValueWidth = 7>
class Button final : public UI::MenuItem<Hott::BufferString, Hott::key_t> {
public:
    static inline constexpr uint8_t valueBeginColumn = Hott::BufferString::size() - ValueWidth;
    static inline constexpr uint8_t valueWidth = ValueWidth;
    
    using value_span_type = etl::span<valueWidth, etl::Char>;
    
    Button(const AVR::Pgm::StringView& text) : mTitle{text} {}
    
    void valueToText(value_span_type buffer) const {
        buffer.insertLeft("<press>"_pgm);
    }
    
    virtual void putTextInto(Hott::BufferString& buffer) const override {
        buffer[0] = etl::Char{' '};
        buffer.insertAtFill(1, mTitle);
        valueToText(etl::make_span<valueBeginColumn, valueWidth>(buffer));
        if (mSelected) {
            etl::apply(etl::make_span<valueBeginColumn, valueWidth>(buffer), [](auto& c) {c |= etl::Char{0x80};});
        }
    }
    virtual MenuItem* processKey(const Hott::key_t key) override {
        switch (key) {
        case Hott::key_t::up:
        case Hott::key_t::down:
        case Hott::key_t::left:
        case Hott::key_t::right:
        case Hott::key_t::nokey:
            break;
        case Hott::key_t::set:
            if (mSelected) {
                CB::process();
            }
            mSelected = !mSelected;
            break;
        }
        return this;
    }
private:
    const AVR::Pgm::StringView mTitle;
};

struct SystemMenu final : public Hott::Menu<8, true, 4> {
    SystemMenu(auto* const parent) : Menu{parent, "Settings"_pgm, &mReset, &mOn, &mRun} {}    
    
    struct R {
        static inline void process() {
            appData.clear();
            appData.change();
        }
    };
    struct O {
        static inline void process() {
            sw::on();
        }
    };
    struct P1 {
        static inline void process() {
            if (sw::pattern() == sw::Pattern::Off) {
                sw::pattern(sw::Pattern::Run);
            }
            else {
                sw::pattern(sw::Pattern::Off);
                sw::off();
            }
        }
    };
    Button<R> mReset{"Reset"_pgm};
    Button<O> mOn{"Turn all on"_pgm};
    Button<P1> mRun{"Running on/off"_pgm};
};

struct RCMenu final : public Hott::Menu<8, true, 7> {
    inline static constexpr uint8_t valueTextLength{6};
    
    RCMenu() : Hott::Menu<8, true, 7>{nullptr, ASSERTSTRING(FULLTITLE), &mAllOff, &mSW, &mPwm, &mBlink1, &mBlink2, &mSystem} {}
    
private:
    struct R {
        static inline void process() {
            sw::off();
        }
    };
    Button<R> mAllOff{"Turn all off:"_pgm};
    SwitchesMenu mSW{this};
    PwmMenu mPwm{this};
    Blink1Menu mBlink1{this};
    Blink2Menu mBlink2{this};
    SystemMenu mSystem{this};
};

using menu = Hott::BasePage<sensor, RCMenu>;

#endif

int main() {
    
//    lvPin::dir<Output>();
    
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
    
    systemTimer::init();
    sensor::init();
    menu::init();
    
    eeprom::init();
    
    if (!((appData.magic() == 44))) {
        appData.magic() = 44;
        appData.clear();
        appData.change();
    }
    
    const auto tickTimer = alarmTimer::create(SoftTimer::intervall, External::Hal::AlarmFlags::Periodic);
    const auto eepromTimer = alarmTimer::create(3000_ms, External::Hal::AlarmFlags::Periodic);
    
    out::init();
    
    while(true) {
        eeprom::saveIfNeeded([&]{
        });
        sensor::periodic();
        menu::periodic();
        
        systemTimer::periodic([&]{
            sensor::ratePeriodic();
            alarmTimer::periodic([&](const auto& t){
                if (tickTimer == t) {
                    sw::ratePeriodic();
                    out::setSwitches();
                }
                else if (eepromTimer == t) {
                    appData.expire();
                }
            });
        });
    }
}

#ifndef NDEBUG
[[noreturn]] inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], unsigned int line [[maybe_unused]]) noexcept {
#if !(defined(USE_IBUS) || defined(USE_HOTT))
    etl::outl<terminal>("Assertion failed: "_pgm, expr, etl::Char{','}, file, etl::Char{','}, line);
#endif
    while(true) {
//        dbg1::toggle();
    }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
