#define NDEBUG

#include "devices.h"

template<typename Devices>
struct GlobalFsm {
    using devs = Devices;
    
    enum class State : uint8_t {Undefined, Wait, Out};
    
    static inline constexpr External::Tick<typename devs::systemTimer> outTicks{500_ms};
    
    inline static constexpr External::Crgb red{External::Red{255}, External::Green{0}, External::Blue{0}};
    inline static constexpr External::Crgb green{External::Red{0}, External::Green{255}, External::Blue{0}};
    inline static constexpr External::Crgb blue{External::Red{0}, External::Green{0}, External::Blue{255}};
    inline static constexpr External::Crgb c1{External::Red{255}, External::Green{255}, External::Blue{0}};
    inline static constexpr External::Crgb c2{External::Red{255}, External::Green{0}, External::Blue{255}};
    inline static constexpr External::Crgb c3{External::Red{0}, External::Green{255}, External::Blue{255}};
    
    using colors_t = std::array<External::Crgb, 6>; 
    inline static constexpr uint8_t f = 10;
    inline static constexpr colors_t colors{red / f, c1 / f, green / f, c2 / f, blue / f, c3 / f};
    using cindex_type = etl::uint_ranged_circular<uint8_t, 0, colors_t::size() - 1>;
    inline static cindex_type color{};
    using index_t = devs::ledStripe::index_type;
    
    static void init() {
        devs::init();
        
        devs::blinkLed1::init();
        devs::blinkLed2::init();
        
        using blink1_t = devs::blinkLed1::count_type;
        devs::blinkLed1::blink(blink1_t{4});
        
        using blink2_t = devs::blinkLed2::count_type;
        devs::blinkLed2::blink(blink2_t{2});
        
        devs::ledStripe::init();
    } 
    static void periodic() {
        devs::la0::toggle();
        devs::ledStripe::periodic();
    } 
    static void ratePeriodic() {
        devs::la1::toggle();        
        
        devs::blinkLed1::ratePeriodic();
        devs::blinkLed2::ratePeriodic();
        
        const auto oldState{mState};
        ++mStateTicks;
        
        switch(mState) {
        case State::Undefined:
            mStateTicks.on(outTicks, []{
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
                mState = State::Out;
            });
            break;
        }
        if (oldState != mState) {
            mStateTicks.reset();
            switch(mState) {
            case State::Undefined:
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
