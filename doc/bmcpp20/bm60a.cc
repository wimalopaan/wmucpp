#define NDEBUG

#include "mcu/avr8.h"
#include "mcu/avr/isr.h"
#include "mcu/avr/mcutimer.h"
#include "mcu/ports.h"
#include "hal/constantrate.h"
#include "std/array.h"
#include "std/concepts.h"
#include "util/disable.h"
#include "util/bits.h"
#include "util/rational.h"
#include "units/percent.h"
#include "container/stringbuffer.h"
#include "console.h"
#include "simavr/simavrdebugconsole.h"

using terminalDevice = SimAVRDebugConsole;
using terminal = std::basic_ostream<terminalDevice>;

using namespace std::literals::quantity;

volatile uint8_t y = 42;
volatile std::percent z{50};

using constantRateTimer = AVR::Timer16Bit<1>;
constexpr const auto constantRatePeriod = 1000_us;

struct CTHandler : public IsrBaseHandler<AVR::ISR::Timer<1>::CompareA> {
    static void isr() {
        ++mCounter;
    }
    inline static volatile uint16_t mCounter = 0;
};

using isrRegistrar = IsrRegistrar<CTHandler>;

struct Measure {
    std::milliseconds time;
    StringBuffer<20> name;
    
};
const uint32_t iterations = 100000;
std::array<Measure, 20> times;

int main() {
    isrRegistrar::init();
    Scoped<EnableInterrupt> interruptEnabler;
    
    constexpr std::hertz constantRateFrequency = 1 / constantRatePeriod;
    constexpr auto tsd = AVR::Util::calculate<constantRateTimer>(constantRateFrequency);
    static_assert(tsd, "wrong parameter");
    constantRateTimer::prescale<tsd.prescaler>();
    constantRateTimer::ocra<tsd.ocr>();
    constantRateTimer::mode(AVR::TimerMode::CTC);
    
    uint8_t algo = 0;
    {
        y = 0;
        uint32_t start = CTHandler::mCounter;
        for(uint32_t n = 0; n < iterations; ++n) {
            ++y;
        }
        uint32_t end = CTHandler::mCounter;           
        times[algo].name.insertAt(0, "empty"_pgm);
        times[algo++].time = std::milliseconds{static_cast<uint16_t>(end - start)};
    }
    {
        y = 0;
        uint32_t start = CTHandler::mCounter;
        for(uint32_t n = 0; n < iterations; ++n) {
            y = std::expand1(z, uint8_t(0), uint8_t(200));
            ++y;
        }
        uint32_t end = CTHandler::mCounter;           
        times[algo].name.insertAt(0, "Rational 8bit"_pgm);
        times[algo++].time = std::milliseconds{static_cast<uint16_t>(end - start)} - times[0].time;
    }
    {
        y = 0;
        uint32_t start = CTHandler::mCounter;
        for(uint32_t n = 0; n < iterations; ++n) {
            y = std::expand(z, 0u, 200u);
            ++y;
        }
        uint32_t end = CTHandler::mCounter;           
        times[algo].name.insertAt(0, "direct"_pgm);
        times[algo++].time = std::milliseconds{static_cast<uint16_t>(end - start)} - times[0].time;
    }
    {
        y = 0;
        uint32_t start = CTHandler::mCounter;
        for(uint32_t n = 0; n < iterations; ++n) {
            y = std::expand1(z, 0u, 200u);
            ++y;
        }
        uint32_t end = CTHandler::mCounter;           
        times[algo].name.insertAt(0, "Rational 16"_pgm);
        times[algo++].time = std::milliseconds{static_cast<uint16_t>(end - start)} - times[0].time;
    }
    
    for(uint8_t i = 1; i < times.size; ++i) {
        std::outl<terminal>(times[i].name, " : "_pgm, times[i].time);
    }        
    
    {
        auto x = std::expand1(z, uint8_t(0), uint8_t(200));
        std::outl<terminal>(x);
    }
    {
        auto x = std::expand(z, 0u, 200u);
        std::outl<terminal>(x);
    }
    {
        auto x = std::expand1(z, 0u, 200u);
        std::outl<terminal>(x);
    }
    
    while(true) {}
}

ISR(TIMER1_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<1>::CompareA>();
}

template<typename L>
void assertFunction(const PgmStringView&, const PgmStringView&, L) noexcept {
    while(true) {}
}
