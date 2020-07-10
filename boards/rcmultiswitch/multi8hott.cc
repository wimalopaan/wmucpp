#define NDEBUG

#define USE_HOTT

#include "board.h"
#include "swout.h"

#ifdef USE_HOTT

using sensor = Hott::Experimental::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<19200>, Hott::GamMsg, Hott::TextMsg, systemTimer>;

using storage_t = Storage::ApplData<etl::uint_ranged_NaN<uint8_t, 0, 7>>;
using eeprom = EEProm::Controller<storage_t>;

template<auto N = 8>
struct SwitchStates {
    enum class SwState : uint8_t {Off, On, Blink1 = On, Steady, Blink2};
    
    static constexpr void init() {
    }
    static constexpr uint8_t size() {
        return N;
    }
    static inline auto& switches() {
        return swStates;
    }
private:
    static inline std::array<SwState, N> swStates{};
    
};

using sw = SwitchStates<>;

using out = External::Output<ledList, pwm, sw, eeprom>;

template<typename Provider, auto ValueWidth = 3>
class Switch final : public UI::MenuItem<Hott::BufferString, Hott::key_t> {
    struct OnOff{
        inline static void format(const Provider::SwState v, etl::span<3, etl::Char>& b) {
            if (v == Provider::SwState::On) {
                b.insertLeftFill("On"_pgm);
            }
            else {
                b.insertLeftFill("Off"_pgm);
            }
        }
    };
public:
    static inline constexpr uint8_t valueBeginColumn = Hott::BufferString::size() - ValueWidth;
    static inline constexpr uint8_t valueWidth = ValueWidth;
    
    using value_span_type = etl::span<valueWidth, etl::Char>;

    Switch(const AVR::Pgm::StringView& text, const uint8_t number) : mTitle{text}, mNumber{number} {}
    
    void valueToText(const auto v, value_span_type buffer) const {
        OnOff::format(v, buffer);
    }
    
    virtual void putTextInto(Hott::BufferString& buffer) const override {
        buffer[0] = etl::Char{' '};
        buffer.insertAtFill(1, mTitle);
        
        valueToText(Provider::switches()[mNumber], etl::make_span<valueBeginColumn, valueWidth>(buffer));
        
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
            if (Provider::switches()[mNumber] == Provider::SwState::On) {
                Provider::switches()[mNumber] = Provider::SwState::Off;
            }
            else {
                Provider::switches()[mNumber] = Provider::SwState::On;
            }
            break;
        }
        return this;
    }
private:
    const AVR::Pgm::StringView mTitle;
    const etl::uint_ranged<uint8_t, 0, Provider::size() - 1> mNumber;
};

struct SwitchesMenu final : public Hott::Menu<8, false, 8> {
    SwitchesMenu(auto* const parent) : Hott::Menu<8, false, 8>{parent, "Switches"_pgm, 
                                                               &mSW0, &mSW1, &mSW2, &mSW3, &mSW4, &mSW5, &mSW6, &mSW7} {}    
    Switch<sw> mSW0{"A :"_pgm, 0};
    Switch<sw> mSW1{"B :"_pgm, 1};
    Switch<sw> mSW2{"C :"_pgm, 2};
    Switch<sw> mSW3{"D :"_pgm, 3};
    Switch<sw> mSW4{"E :"_pgm, 4};
    Switch<sw> mSW5{"F :"_pgm, 5};
    Switch<sw> mSW6{"G :"_pgm, 6};
    Switch<sw> mSW7{"H :"_pgm, 7};
};

auto& appData = eeprom::data();

struct PwmMenu final : public Hott::Menu<8, false, 8> {
    struct Adapter {
        eeprom::data_t::value_type::pwm_type& operator[](const Storage::AVKey k) {
            return appData[k].pwmValue();
        }
        void change() {
            appData.change();
        }
    };

    using pwm_type = storage_t::value_type::pwm_type;
    
    PwmMenu(auto* const parent) : Menu{parent, "Pwm"_pgm, &mPwm0, &mPwm1, &mPwm2, &mPwm3, &mPwm4, &mPwm5, &mPwm6, &mPwm7} {}    
    Hott::TextWithValue<Storage::AVKey, Adapter> mPwm0{"PWM A :"_pgm, a0, Storage::AVKey::Ch0, pwm_type::Upper};
    Hott::TextWithValue<Storage::AVKey, Adapter> mPwm1{"PWM B :"_pgm, a1, Storage::AVKey::Ch1, pwm_type::Upper};
    Hott::TextWithValue<Storage::AVKey, Adapter> mPwm2{"PWM C :"_pgm, a2, Storage::AVKey::Ch2, pwm_type::Upper};
    Hott::TextWithValue<Storage::AVKey, Adapter> mPwm3{"PWM D :"_pgm, a3, Storage::AVKey::Ch3, pwm_type::Upper};
    Hott::TextWithValue<Storage::AVKey, Adapter> mPwm4{"PWM E :"_pgm, a4, Storage::AVKey::Ch4, pwm_type::Upper};
    Hott::TextWithValue<Storage::AVKey, Adapter> mPwm5{"PWM F :"_pgm, a5, Storage::AVKey::Ch5, pwm_type::Upper};
    Hott::TextWithValue<Storage::AVKey, Adapter> mPwm6{"PWM G :"_pgm, a6, Storage::AVKey::Ch6, pwm_type::Upper};
    Hott::TextWithValue<Storage::AVKey, Adapter> mPwm7{"PWM H :"_pgm, a7, Storage::AVKey::Ch7, pwm_type::Upper};
    
    Adapter a0;
    Adapter a1;
    Adapter a2;
    Adapter a3;
    Adapter a4;
    Adapter a5;
    Adapter a6;
    Adapter a7;
};

template<auto ValueWidth = 3>
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

        valueToText(appData[mNumber].blinks()[0].intervall, etl::make_span<valueBeginColumn1, valueWidth>(buffer));
        valueToText(appData[mNumber].blinks()[0].duration, etl::make_span<valueBeginColumn2, valueWidth>(buffer));

        if (mSelected) {
            etl::apply(etl::make_span<valueBeginColumn1, valueWidth>(buffer), [](auto& c) {c |= etl::Char{0x80};});
            etl::apply(etl::make_span<valueBeginColumn2, valueWidth>(buffer), [](auto& c) {c |= etl::Char{0x80};});
        }
    }
    virtual MenuItem* processKey(const Hott::key_t key) override {
        switch (key) {
        case Hott::key_t::up:
            ++appData[mNumber].blinks()[0].intervall;
            out::setSwitches();
            break;
        case Hott::key_t::down:
            --appData[mNumber].blinks()[0].intervall;
            out::setSwitches();
            break;
        case Hott::key_t::left:
            if (appData[mNumber].blinks()[0].duration < appData[mNumber].blinks()[0].intervall) {
                ++appData[mNumber].blinks()[0].duration;
            }
            out::setSwitches();
            break;
        case Hott::key_t::right:
            --appData[mNumber].blinks()[0].duration;
            out::setSwitches();
            break;
        case Hott::key_t::nokey:
            break;
        case Hott::key_t::set:
            if (mSelected) {
                appData.change();
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

struct BlinkMenu final : public Hott::Menu<8, false, 8> {
    BlinkMenu(auto* const parent) : Menu{parent, "Blink"_pgm, &mB0, &mB1, &mB2, &mB3, &mB4, &mB5, &mB6, &mB7} {}    

    Tick<> mB0{"Blink A D/I:"_pgm, Storage::AVKey::Ch0};
    Tick<> mB1{"Blink B D/I:"_pgm, Storage::AVKey::Ch1};
    Tick<> mB2{"Blink C D/I:"_pgm, Storage::AVKey::Ch2};
    Tick<> mB3{"Blink D D/I:"_pgm, Storage::AVKey::Ch3};
    Tick<> mB4{"Blink E D/I:"_pgm, Storage::AVKey::Ch4};
    Tick<> mB5{"Blink F D/I:"_pgm, Storage::AVKey::Ch5};
    Tick<> mB6{"Blink G D/I:"_pgm, Storage::AVKey::Ch6};
    Tick<> mB7{"Blink H D/I:"_pgm, Storage::AVKey::Ch7};
};


template<typename CB, auto ValueWidth = 5>
class Button final : public UI::MenuItem<Hott::BufferString, Hott::key_t> {
public:
    static inline constexpr uint8_t valueBeginColumn = Hott::BufferString::size() - ValueWidth;
    static inline constexpr uint8_t valueWidth = ValueWidth;
    
    using value_span_type = etl::span<valueWidth, etl::Char>;

    Button(const AVR::Pgm::StringView& text) : mTitle{text} {}
    
    void valueToText(value_span_type buffer) const {
        buffer.insertLeft("press"_pgm);
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

struct SystemMenu final : public Hott::Menu<8, true, 2> {
    SystemMenu(auto* const parent) : Menu{parent, "Einstellungen"_pgm, &mReset} {}    

    struct R {
        static inline void process() {
            appData.clear();
            appData.change();
        }
    };
    Button<R> mReset{"Reset"_pgm};
};

struct RCMenu final : public Hott::Menu<8, true, 5> {
    inline static constexpr uint8_t valueTextLength{6};
    
    RCMenu() : Hott::Menu<8, true, 5>{nullptr, "WM Switch 1.3"_pgm, &mSW, &mPwm, &mBlink, &mSystem} {}

private:
    SwitchesMenu mSW{this};
    PwmMenu mPwm{this};
    BlinkMenu mBlink{this};
    SystemMenu mSystem{this};
};

using menu = Hott::BasePage<sensor, RCMenu>;

#endif

int main() {
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
   
    systemTimer::init();
    sensor::init();
    menu::init();
    
    eeprom::init();
    
    {
        if (!((appData[Storage::AVKey::Magic].pwmValue().toInt() == 44))) {
            appData[Storage::AVKey::Magic].pwmValue(44);
            appData.clear();
            appData.change();
        }
    }
    
    const auto tickTimer = alarmTimer::create(SoftTimer::intervall, External::Hal::AlarmFlags::Periodic);
    const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);

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
                    out::setSwitches();
                }
                else if (periodicTimer == t) {
                    appData.expire();
                }
            });
        });
    }
}

