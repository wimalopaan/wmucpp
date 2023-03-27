#define NDEBUG

#include "devices.h"

template<typename Devices>
struct GlobalFsm {
    using devs = Devices;

    enum class State : uint8_t {Undefined, Init, On, Wait, Out, BT_SetName};
    
    static inline constexpr External::Tick<typename devs::systemTimer> powerTicks{1000_ms};
    static inline constexpr External::Tick<typename devs::systemTimer> debugTicks{500_ms};
    
    using blink1_t = devs::blinkLed1::count_type;
    using blink2_t = devs::blinkLed2::count_type;
    
    using terminal1 = devs::terminal1;
    using terminal2 = devs::terminal2;
    
    static inline constexpr External::Tick<typename devs::systemTimer> outTicks{500_ms};
    
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
        devs::init();
        
        devs::blinkLed1::init();
        devs::blinkLed2::init();

        devs::blinkLed1::blink(blink1_t{4});
        devs::blinkLed2::blink(blink2_t{2});

        devs::serial1::template init<AVR::BaudRate<9600>>();
        devs::serial2::template init<AVR::BaudRate<9600>>();
        
        devs::buzz::init();
        
        devs::ds3231::init();

        devs::ledStripe::init();
        
        devs::hc05::init();
        
        devs::adcController::init();
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
    } 
    static void ratePeriodic() {
        devs::la1::toggle();        

        devs::blinkLed1::ratePeriodic();
        devs::blinkLed2::ratePeriodic();

        devs::ds3231::ratePeriodic();
        
        devs::hc05::ratePeriodic();

        devs::sbus_pa::ratePeriodic();
        devs::sensor::ratePeriodic();
        
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
            mStateTicks.on(debugTicks, []{
                etl::outl<terminal1>("serial1: "_pgm, "test3_09"_pgm);                
                etl::outl<terminal2>("serial2: "_pgm, "test3_09"_pgm);                
                mState = State::Out;
            });
            break;
        case State::Out:
            mStateTicks.on(outTicks, []{
                mState = State::Wait;
            });
            break;
        case State::Wait:
            mStateTicks.on(outTicks, []{
                mState = State::On;
            });
            break;
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
            case State::Out:
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
                break;
            case State::Wait:
                break;
            }
        }
    }     
private:    
    static inline State mState{State::Undefined};
    static inline External::Tick<typename devs::systemTimer> mStateTicks;
    inline static constexpr uint8_t color_scale = 10;
    inline static constexpr colors_t colors{red / color_scale, c1 / color_scale, green / color_scale, c2 / color_scale, blue / color_scale, c3 / color_scale};
    inline static cindex_type color{};
};

using devices = Devices<2>;
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
