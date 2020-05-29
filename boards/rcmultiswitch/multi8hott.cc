#define NDEBUG

#define USE_HOTT

#include "board.h"
#include "swout.h"

using sensor = Hott::Experimental::Sensor<usart0Position, AVR::Usart, AVR::BaudRate<19200>, Hott::GamMsg, Hott::TextMsg, systemTimer>;

#ifdef USE_HOTT

template<auto Columns = 4, auto ValueWidth = 3>
class Switch final : public UI::MenuItem<Hott::BufferString, Hott::key_t> {
public:
    static inline constexpr uint8_t valueBeginColumn = Hott::BufferString::size() - ValueWidth;
    static inline constexpr uint8_t valueWidth = ValueWidth;
    
    using value_span_type = etl::span<valueWidth, etl::Char>;

    Switch(const AVR::Pgm::StringView& text, uint8_t number) : mTitle{text}, mNumber{number} {}
    
    virtual void putTextInto(Hott::BufferString& buffer) const override {
        buffer[0] = etl::Char{' '};
        buffer.insertAtFill(1, mTitle);
        
//        if (value) {
//            valueToText(*value, etl::make_span<valueBeginColumn, valueWidth>(buffer));
//        }
//        else {
//            etl::fill(etl::make_span<valueBeginColumn, valueWidth>(buffer), Char{'-'});
//        }
        
//        if (mSelected) {
//            etl::apply(etl::make_span<valueBeginColumn, valueWidth>(buffer), [](auto& c) {c |= etl::Char{0x80};});
//        }
    }
//    virtual MenuItem* processKey(Hott::key_t key) override {
////        auto& v = mProvider[mKey];
//        switch (key) {
//        case Hott::key_t::up:
//            if (mSelected) {
////                if (v) {
////                    if (*v > 0) {
////                        --v;
////                    }
////                    else {
////                        v.setNaN();
////                    }
////                }
////                else {
////                    *v = mMax;
////                }
//            }
//            break;
//        case Hott::key_t::down:
//            if (mSelected) {
////                if (v) {
////                    if (*v < mMax) {
////                        ++v;
////                    }
////                    else {
////                        v.setNaN();
////                    }
////                }
////                else {
////                    *v = 0;
////                }
//            }
//            break;
//        case Hott::key_t::left:
//            --mColumn;
//            break;
//        case Hott::key_t::right:
//            ++mColumn;
//            break;
//        case Hott::key_t::set:
//            if (mSelected) {
////                mProvider.change();
//            }
//            mSelected = !mSelected;
//            break;
//        case Hott::key_t::nokey:
//            break;
//        }
//        return this;
//    }
private:
    const AVR::Pgm::StringView mTitle;
    const Hott::key_t mKey = Hott::key_t::nokey;
    uint8_t mNumber = 0;
    etl::uint_ranged_circular<uint8_t, 0, Columns - 1> mColumn;
};

struct SwitchesMenu final : public Hott::Menu<8, false, 4> {
    SwitchesMenu(auto* const parent) : Hott::Menu<8, false, 4>{parent, "Switches"_pgm, &mSW0, &mSW1, &mSW2, &mSW3} {
//        title(false);
    }    
    Switch<> mSW0{"A"_pgm, 0};
    Switch<> mSW1{"B"_pgm, 1};
    Switch<> mSW2{"C"_pgm, 2};
    Switch<> mSW3{"D"_pgm, 3};
//    Switch mSW4{3};
//    Switch mSW5{3};
//    Switch mSW6{3};
//    Switch mSW7{3};
};

struct PwmMenu final : public Hott::Menu<8, true, 2> {
    PwmMenu(auto* const parent) : Menu<8, true, 2>{parent, "Pwm"_pgm, &mSW0} {}    
    Switch<> mSW0{"A"_pgm, 0};
};

struct BlinkMenu final : public Hott::Menu<8, true, 2> {
    BlinkMenu(auto* const parent) : Menu<8, true, 2>{parent, "Blink"_pgm, &mSW0} {}    
    Switch<> mSW0{"A"_pgm, 0};
};


struct RCMenu final : public Hott::Menu<8, true, 4> {
    inline static constexpr uint8_t valueTextLength{6};
    
    struct YesNo{
        inline static void format(const uint8_t v, etl::span<3, etl::Char>& b) {
            if (v == 0) {
                b.insertLeftFill("no"_pgm);
            }
            else {
                b.insertLeftFill("yes"_pgm);
            }
        }
    };
    
    RCMenu() : Hott::Menu<8, true, 4>{nullptr, "WM Switch 0.2"_pgm, &mSW, &mPwm, &mBlink} {}

private:
    SwitchesMenu mSW{this};
    PwmMenu mPwm{this};
    BlinkMenu mBlink{this};
};

template<typename PA, typename TopMenu>
class HottMenu final {
    HottMenu() = delete;
public:
    inline static void init() {
        clear();
    }
    inline static void periodic() {
        PA::processKey([&](Hott::key_t k){
            if (const auto n = mMenu->processKey(k); n != mMenu) {
                clear();
                if (n) {
                    mMenu = n;
                }
                else {
                    PA::esc();
                }
            }
        });
        PA::notSending([&]{
            mMenu->textTo(PA::text());
        });
    }
private:
    inline static void clear() {
        for(auto& line : PA::text()) {
            line.clear();
        }
    }
    inline static TopMenu mTopMenu;
    inline static Hott::IMenu<PA::menuLines>* mMenu = &mTopMenu;
};

using menu = HottMenu<sensor, RCMenu>;
#endif

int main() {
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
   
    systemTimer::init();
    sensor::init();

    menu::init();
    
    const auto periodicTimer = alarmTimer::create(1000_ms, External::Hal::AlarmFlags::Periodic);

//    etl::outl<terminal>("multi8mk4hott"_pgm);

//    uint16_t counter{};
    
    while(true) {
        sensor::periodic();
        menu::periodic();
        
        systemTimer::periodic([&]{
            sensor::ratePeriodic();
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
//                    etl::outl<terminal>("c: "_pgm, counter++, " mc: "_pgm, sumd::hasMultiChannel(), " nc: "_pgm, sumd::numberOfChannels());
//                    etl::outl<terminal>("mc0: "_pgm, (uint8_t)sumd::mChannel(0));
                }
            });
        });
    }
}

