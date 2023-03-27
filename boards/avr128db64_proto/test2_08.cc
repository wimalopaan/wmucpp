#define NDEBUG

#include "devices.h"

template<typename Devices>
struct GlobalFsm {
    using devs = Devices;

    enum class State : uint8_t {Undefined, Init, On, Wait, Out};
    
    static inline constexpr External::Tick<typename devs::systemTimer> powerTicks{1000_ms};
    static inline constexpr External::Tick<typename devs::systemTimer> debugTicks{500_ms};
    
    using blink1_t = devs::blinkLed1::count_type;
    using blink2_t = devs::blinkLed2::count_type;
    
    using terminal1 = etl::basic_ostream<typename devs::serial1>;
    using terminal2 = etl::basic_ostream<typename devs::serial2>;
    
    static inline constexpr External::Tick<typename devs::systemTimer> outTicks{500_ms};
    
    static constexpr External::Crgb red{External::Red{255}, External::Green{0}, External::Blue{0}};
    static constexpr External::Crgb green{External::Red{0}, External::Green{255}, External::Blue{0}};
    static constexpr External::Crgb blue{External::Red{0}, External::Green{0}, External::Blue{255}};
    static constexpr External::Crgb c1{External::Red{255}, External::Green{255}, External::Blue{0}};
    static constexpr External::Crgb c2{External::Red{255}, External::Green{0}, External::Blue{255}};
    static constexpr External::Crgb c3{External::Red{0}, External::Green{255}, External::Blue{255}};
    
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
    } 
    static void periodic() {
        devs::la0::toggle();
        devs::ds3231::periodic();
        devs::serial1::periodic();
        devs::serial2::periodic();
        devs::ledStripe::periodic();
    } 
    static void ratePeriodic() {
        devs::la1::toggle();        

        devs::blinkLed1::ratePeriodic();
        devs::blinkLed2::ratePeriodic();

        devs::ds3231::ratePeriodic();
        
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
                mState = State::On;
            }
            break;
        case State::On:
            mStateTicks.on(debugTicks, []{
                etl::outl<terminal1>("serial1: test07"_pgm);                
                etl::outl<terminal2>("serial2: test07"_pgm);                
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
            case State::Out:
                ++color;
                devs::ledStripe::set(index_t{0}, colors[color]);
                ++color;
                devs::ledStripe::set(index_t{1}, colors[color]);
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
    inline static constexpr colors_t colors{red, c1, green, c2, blue, c3};
    inline static cindex_type color{};
};

using devices = Devices<1>;
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
