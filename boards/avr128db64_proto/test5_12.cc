#define NDEBUG

#include "devices.h"

template<typename Devices>
struct GlobalFsm {
    using devs = Devices;
    
    enum State {Undefined, Init, On, On2, BT_SetName};
    
    static inline constexpr External::Tick<typename devs::systemTimer> powerTicks{1000_ms};
    static inline constexpr External::Tick<typename devs::systemTimer> debugTicks{500_ms};
    static inline constexpr External::Tick<typename devs::systemTimer> outTicks{100_ms};
    
    using blink1_t = devs::blinkLed1::count_type;
    using blink2_t = devs::blinkLed2::count_type;
    
    using terminal1 = devs::terminal1;
    using terminal2 = devs::terminal2;
    
    using channel_t = devs::sbus_pa::channel_t;
    
    static inline constexpr External::Crgb red{External::Red{255}, External::Green{0}, External::Blue{0}};
    static inline constexpr External::Crgb green{External::Red{0}, External::Green{255}, External::Blue{0}};
    static inline constexpr External::Crgb blue{External::Red{0}, External::Green{0}, External::Blue{255}};
    static inline constexpr External::Crgb c1{External::Red{255}, External::Green{255}, External::Blue{0}};
    static inline constexpr External::Crgb c2{External::Red{255}, External::Green{0}, External::Blue{255}};
    static inline constexpr External::Crgb c3{External::Red{0}, External::Green{255}, External::Blue{255}};
    
    using colors_t = std::array<External::Crgb, 6>; 
    using cindex_type = etl::uint_ranged_circular<uint8_t, 0, colors_t::size() - 1>;
    using index_t = devs::ledStripe::index_type;
    
    static void init() {
        devs::pc1::init();
        devs::pc2::init();
        devs::pc1::activate();
        devs::pc2::activate();
        
        devs::init();
        
        devs::blinkLed1::init();
        devs::blinkLed2::init();
        
        devs::blinkLed1::blink(blink1_t{4});
        devs::blinkLed2::blink(blink2_t{2});
        
        devs::serial1::template init<AVR::BaudRate<9600>>();
        devs::serial2::template init<AVR::BaudRate<9600>>();
        
        devs::buzz::init();
        
        devs::ds3231::init();
        devs::oled::init();
        
        devs::ledStripe::init();
        
        devs::hc05::init();
        
        devs::adcController::init();
        
        devs::lut2::init(std::byte{0x0f}); // route TXD (inverted) to lut2-out         
        devs::lut2::enable();
        devs::sensor::init();
        devs::sensor::uart::txPinDisable();
        
        devs::sbus::template init<AVR::BaudRate<100000>, AVR::FullDuplex, true, 1>(); // 8E2 
        devs::sbus::rxInvert(true);
        devs::sbus::txPinDisable();
        
        devs::rotary::init(127);
        devs::rotaryButton::init();
        
        devs::dac::init();
        
        devs::pwm1::template init<Meta::List<AVR::PWM::WO<0>>>();
        devs::pwm1::frequency(60000);
        devs::pwm1::template on<Meta::List<AVR::PWM::WO<0>>>();
        devs::pwm1::template duty<Meta::List<AVR::PWM::WO<0>>>(30000);
        devs::lut0::init(std::byte{0x55}); 
        devs::lut0::enable();
        devs::lutPin1::template dir<AVR::Output>();
        devs::lutPin1::on();
        
        devs::pwm2::template init<Meta::List<AVR::PWM::WO<0>>>();
        devs::pwm2::frequency(60000);
        devs::pwm2::template on<Meta::List<AVR::PWM::WO<0>>>();
        devs::pwm2::template duty<Meta::List<AVR::PWM::WO<0>>>(10000);
        devs::lut5::init(std::byte{0x55}); 
        devs::lut5::enable();
        devs::lutPin2::template dir<AVR::Output>();
        devs::lutPin2::on();
        
    } 
    static void periodic() {
        devs::la0::toggle();
        devs::ds3231::periodic();
        devs::serial1::periodic();
        devs::serial2::periodic();
        devs::ledStripe::periodic();
        devs::hc05::periodic();
        devs::adcController::periodic();
        
        devs::sbus::periodic();
        devs::sensor::periodic();
        
        devs::oled::periodic();
    }
    static void ratePeriodic() {
        devs::la1::toggle();
        
        devs::blinkLed1::ratePeriodic();
        devs::blinkLed2::ratePeriodic();
        
        devs::ds3231::ratePeriodic();
        devs::oled::ratePeriodic();
        
        devs::hc05::ratePeriodic();
        
        devs::sbus_pa::ratePeriodic();
        devs::sensor::ratePeriodic();
        
        devs::rotary::rateProcess();
        devs::rotaryButton::ratePeriodic();
        
        const uint16_t r = devs::rotary::value();

        
        const auto oldState{mState};
        ++mStateTicks;
        
        switch(mState) {
        case State::Undefined:
            mStateTicks.on(powerTicks, []{
                mState = State::Init;
            });
            break;
        case State::Init:
            if (devs::ds3231::isIdle()) {
                mState = State::BT_SetName;
            }
            break;
        case State::BT_SetName:
            mStateTicks.on(powerTicks, []{
                mState = State::On;
            });
            break;
        case State::On:
            devs::dac::put(r);
            (++mDebugTicks).on(debugTicks, [&]{
                etl::outl<terminal1>("serial1: "_pgm, "test3_12"_pgm);                
                etl::outl<terminal2>("serial2: "_pgm, "sbus p: "_pgm, devs::sbus_pa::packages(), " v[0]: "_pgm, devs::sbus_pa::value(0).toInt(), " s.port p: "_pgm, devs::sensor::ProtocollAdapter::requests());
                etl::outl<terminal2>("serial2: "_pgm, "rot: "_pgm, r);                
            });
            (++mLedTicks).on(outTicks, []{
                ++color;
                devs::ledStripe::set(index_t{0}, colors[color]);
                ++color;
                devs::ledStripe::set(index_t{1}, colors[color]);
                ++color;
                devs::ledStripe::set(index_t{2}, colors[color]);
                ++color;
                devs::ledStripe::set(index_t{3}, colors[color]);
                ++color;
                devs::ledStripe::set(index_t{4}, colors[color]);
                ++color;
                devs::ledStripe::set(index_t{5}, colors[color]);
                ++color;
                devs::ledStripe::set(index_t{6}, colors[color]);
                ++color;
                devs::ledStripe::set(index_t{7}, colors[color]);
                devs::ledStripe::out();
            });
            if (const auto e = devs::rotaryButton::event(); e == devs::rotaryButton::Press::Short) {
                mState = State::On2;
            }
            break;
        case State::On2:
            if (const auto e = devs::rotaryButton::event(); e == devs::rotaryButton::Press::Short) {
                mState = State::On;
            }
            break;
#ifdef NDEBUG
        default:
            __builtin_unreachable();
            break;
#endif     
        }
        if (oldState != mState) {
            mStateTicks.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                devs::rtcI2C::write(AVR::Twi::Address{0b1101000}, {0x0e, 0x40});                
                break;
            case State::On:
                devs::blinkLed1::blink(blink1_t{4});
                break;
            case State::BT_SetName:
                devs::hc05::event(devs::hc05::Event::SetName);
                break;
            case State::On2:
                devs::blinkLed1::off();
                break;
#ifdef NDEBUG
            default:
                __builtin_unreachable();
                break;
#endif     
            }
        }
    }     
private:    
    static inline State mState{State::Undefined};
    static inline External::Tick<typename devs::systemTimer> mStateTicks;
    static inline External::Tick<typename devs::systemTimer> mDebugTicks;
    static inline External::Tick<typename devs::systemTimer> mLedTicks;
    inline static constexpr uint8_t color_scale = 10;
    inline static constexpr colors_t colors{red / color_scale, c1 / color_scale, green / color_scale, c2 / color_scale, blue / color_scale, c3 / color_scale};
    inline static cindex_type color{};
};

using devices = Devices<4
>;
using gfsm = GlobalFsm<devices>;

int main() {
    gfsm::init();    
    while(true) {
        gfsm::periodic(); 
        devices::systemTimer::periodic([&]{
            gfsm::ratePeriodic();
        });
    }
    
}
#ifndef NDEBUG
/*[[noreturn]] */inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], const unsigned int line [[maybe_unused]]) noexcept {
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, const unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif