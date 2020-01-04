#define NDEBUG

#include <mcu/avr.h>
#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/event.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/eeprom.h>
#include <mcu/internals/ccl.h>
#include <mcu/internals/spi.h>

#include <external/hal/alarmtimer.h>
#include <external/solutions/tick.h>
#include <external/solutions/button.h>
#include <external/solutions/apa102.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/sweep.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

namespace Parameter {
    constexpr auto fRtc = 500_Hz;
}

using ccp = Cpu::Ccp<>;
using clock = Clock<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>;
using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::Default>;
using spiPosition = Portmux::Position<Component::Spi<0>, Portmux::Default>;

using systemTimer = SystemTimer<Component::Rtc<0>, Parameter::fRtc>;
using alarmTimer = External::Hal::AlarmTimer<systemTimer, 8>;

using spi = AVR::Spi<spiPosition, AVR::QueueLength<128>,  AVR::UseInterrupts<false>>;

using PortA = Port<A>;
using PortB = Port<B>;

using buttonPin = ActiveLow<Pin<PortA, 7>, Input>;

using button = External::Button<buttonPin, systemTimer, External::Tick<systemTimer>{100_ms}, External::Tick<systemTimer>{3000_ms}>;

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, tcaPosition, spiPosition>>;

using terminalDevice = AVR::Usart<usart0Position, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>>;
using terminal = etl::basic_ostream<terminalDevice>;

template<typename Stripe, typename Timer>
struct PatternGenerator {
    enum class Mode : uint8_t {
        LeftRight, RightLeft, MidBorder,
        LeftRightCircular, RightLeftCircular, MidBorderCircular,
        StaticRightLeft, StaticCenter
    };
    using tick_t = External::Tick<Timer>;
    
    static constexpr auto intervall = Timer::intervall;
    inline static tick_t intervallTicks{10_ms};
    
    using stripe = Stripe;
    using index_type = stripe::index_type;
    using size_type = stripe::size_type;

    inline static constexpr size_type width{10};
    inline static constexpr size_type mid{stripe::size() / 2 + 1};
    static_assert(width < (stripe::size() / 2 - 1));
    static_assert((stripe::size() - 1 - (mid - 1)) == (mid - 1));
    
    static constexpr External::Crgb red{External::Red{255}, External::Green{0}, External::Blue{0}};
    static constexpr External::Crgb green{External::Red{0}, External::Green{255}, External::Blue{0}};
    static constexpr External::Crgb blue{External::Red{0}, External::Green{0}, External::Blue{255}};
    static constexpr External::Crgb c1{External::Red{255}, External::Green{255}, External::Blue{0}};
    static constexpr External::Crgb c2{External::Red{255}, External::Green{0}, External::Blue{255}};
    static constexpr External::Crgb c3{External::Red{0}, External::Green{255}, External::Blue{255}};
    
    using colors_t = std::array<External::Crgb, 6>; 
    inline static colors_t colors{red, c1, green, c2, blue, c3};
    using cindex_type = etl::uint_ranged_circular<uint8_t, 0, colors_t::size() - 1>;
    inline static cindex_type color{};
    
    using modes_t = std::array<Mode, 8>; 
    inline static modes_t modes{Mode::LeftRight, Mode::LeftRightCircular, 
                Mode::MidBorder, Mode::MidBorderCircular, 
                Mode::RightLeft, Mode::RightLeftCircular,
                Mode::StaticRightLeft, Mode::StaticCenter
                               };
    
    using mindex_type = etl::uint_ranged_circular<uint8_t, 0, modes_t::size() - 1>;
    inline static mindex_type mode{};

    using speeds_t = std::array<tick_t, 3>; 
    inline static speeds_t speeds{tick_t{5_ms}, tick_t{10_ms}, tick_t{30_ms}};
    using speed_type = etl::uint_ranged_circular<uint8_t, 0, speeds_t::size() - 1>;
    inline static speed_type speed{};

    using sweep_t = etl::Sweep<typename stripe::size_type, 0, stripe::size() - 1, width>;
    using sweep_r = etl::Sweep<typename stripe::size_type, mid - 1, stripe::size() - 1, width>;
    using sweep_l = etl::Sweep<typename stripe::size_type, 0, mid - 1, width>;
    
    inline static void init() {
        stripe::init();
        stripe::clear();
        stripe::out();
    }
    inline static void periodic() {
        stripe::periodic();
    }
    
    inline static bool changeColor() {
        ++color;    
        return color.isBottom();
    }

    inline static bool changeMode() {
        mMode = modes[++mode];
        return mode.isBottom();
    }

    inline static bool changeSpeed() {
        ++speed;
        intervallTicks = speeds[speed];
        return speed.isBottom();
    }
    
    inline static void tick() {
        ++mStateTicks;
        switch(mMode) {
        case Mode::LeftRight:
            leftRight();
            break;
        case Mode::RightLeft:
            rightLeft();
            break;
        case Mode::MidBorder:
            midBorder();
            break;
        case Mode::LeftRightCircular:
            leftRightCircular();
            break;
        case Mode::RightLeftCircular:
            rightLeftCircular();
            break;
        case Mode::MidBorderCircular:
            midBorderCircular();
            break;
        case Mode::StaticCenter:
            staticCenter();
            break;
        case Mode::StaticRightLeft:
            staticRightLeft();
            break;
        }
    }
private:
    inline static void staticCenter() {
        mStateTicks.on(intervallTicks, [&]{
            stripe::clear();
            index_type left{mid - width / 2 - 1};
            index_type right{mid + width / 2 - 1 - 1};
            stripe::set(left, right, colors[color]);
            stripe::out();
        });
    }
    inline static void staticRightLeft() {
        mStateTicks.on(intervallTicks, [&]{
            stripe::clear();
            index_type left{width / 2};
            stripe::set(index_type{0}, left, colors[color]);
            index_type right{stripe::size() - 1 - width / 2};
            stripe::set(right, index_type{stripe::size() - 1}, colors[color]);
            stripe::out();
        });
    }

    inline static void si(sweep_t& sw) {
        stripe::clear();
        auto [l, r] = sw.next();
        stripe::set(l, r, colors[color]);
        stripe::out();
    }
    
    inline static void si(sweep_l& sl, sweep_r& sr) {
        stripe::clear();
        auto [lr, rr] = sr.next();
        auto [ll, rl] = sl.next();
        stripe::set(lr, rr, colors[color]);
        stripe::set(ll, rl, colors[color]);
        stripe::out();
    }
        
    inline static void leftRight() {
        static sweep_t sw{sweep_t::Mode::LeftToRight};
        mStateTicks.on(intervallTicks, [&]{
            si(sw);
        });
    }
    inline static void rightLeft() {
        static sweep_t sw{sweep_t::Mode::RightToLeft};
        mStateTicks.on(intervallTicks, [&]{
            si(sw);
        });
    }
    inline static void leftRightCircular() {
       static sweep_t sw{sweep_t::Mode::LeftToRightCirclar};
       mStateTicks.on(intervallTicks, [&]{
           si(sw);
       });
    }
    inline static void rightLeftCircular() {
        static sweep_t sw{sweep_t::Mode::RightToLeftCircular};
        mStateTicks.on(intervallTicks, [&]{
            si(sw);
        });
    }
                            
    inline static void midBorderCircular() {       
        static sweep_r sr{sweep_r::Mode::LeftToRightCirclar};
        static sweep_l sl{sweep_l::Mode::RightToLeftCircular};
        mStateTicks.on(intervallTicks, [&]{
            si(sl, sr);
        });
    }
    inline static void midBorder() {
        static sweep_r sr{sweep_r::Mode::LeftToRight};
        static sweep_l sl{sweep_l::Mode::RightToLeft};
        mStateTicks.on(intervallTicks, [&]{
            si(sl, sr);
        });
    }
    inline static tick_t mStateTicks{};
    inline static Mode mMode{Mode::MidBorder};
};

using patGen = PatternGenerator<External::LedStripe<spi, External::APA102, 145>, systemTimer>;

int main() {
    ccp::unlock([]{
        clock::prescale<1>();
    });
    
    portmux::init();    
    
    patGen::init();
    
    button::init();
    
    systemTimer::init();
    
    terminalDevice::init<AVR::BaudRate<9600>>();
    
    const auto periodicTimer = alarmTimer::create(500_ms, External::Hal::AlarmFlags::Periodic);
    const auto modeTimer = alarmTimer::create(5000_ms, External::Hal::AlarmFlags::Periodic);
    
    while(true) {
        terminalDevice::periodic();
        patGen::periodic();
        
        systemTimer::periodic([&]{
            patGen::tick();
            button::periodic([&](button::Press press){
                if (press == button::Press::Long) {
                    patGen::changeMode();                    
                }
            });
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    etl::outl<terminal>("test00"_pgm);                   
                    patGen::changeColor();
                }
                if (modeTimer == t) {
                    if (patGen::changeMode()) {
//                        patGen::changeSpeed();
                    }
                }
            });
        });
    }
}


