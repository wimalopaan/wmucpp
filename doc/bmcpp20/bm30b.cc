#define NDEBUG

#include <stdint.h>

//#include "mcu/avr8.h"
//#include "mcu/avr/isr.h"
//#include "mcu/avr/mcutimer.h"
//#include "mcu/ports.h"
//#include "hal/constantrate.h"
//#include "std/array.h"
//#include "std/concepts.h"
//#include "util/disable.h"
//#include "util/bits.h"
//#include "units/percent.h"
//#include "container/stringbuffer.h"
//#include "console.h"
//#include "simavr/simavrdebugconsole.h"

//using terminalDevice = SimAVRDebugConsole;
//using terminal = std::basic_ostream<terminalDevice>;

//using namespace std::literals::quantity;

volatile uint8_t y = 42;
//volatile uint16_t y2 = 42;
//volatile std::percent z = 0_ppc;
volatile uint8_t z  = 0;

//namespace detail {
//    template<typename T>
//    class LookupScaler {
//    private:
//        typedef typename Util::enclosingType<T>::type E;
//        inline static constexpr auto lf = [](){
//            std::array<E, std::numeric_limits<T>::max() + 1> data;
//            for(E i = 1; i <= std::numeric_limits<T>::max(); ++i) {
//                data[i] = (100 * 256) / i;
//            }
//            return data;
//        }();
//    public:
//        static std::percent scale(T value, T min, T max) {
//            typedef typename Util::enclosingType<E>::type R;
//            return std::percent{(uint8_t)(((value - min) * (R)lf[max - min]) / 256)};
//        }
//    };
//    template<typename T, T min, T max>
//    class LookupScalerFixed {
//    private:
//        typedef typename Util::enclosingType<T>::type E;
//        inline static constexpr auto lf = [](){
//            std::array<E, std::numeric_limits<T>::max() + 1> data;
//            for(E i = 1; i <= std::numeric_limits<T>::max(); ++i) {
//                data[i] = ((i - min) * 100u) / (max - min);
//            }
//            return data;
//        }();
//    public:
//        static std::percent scale(T value) {
//            return std::percent{(uint8_t)lf[value]};
//        }
//    };
//} //!detail

//template<typename T = uint8_t>
//std::percent lookupScale(T value, T min, T max) {
//    typedef typename std::remove_cv_t<T> U;
//    if (value < min) {
//        return std::percent{0U};
//    }
//    else if (value > max) {
//        return std::percent{100U};
//    }
//    return detail::LookupScaler<U>::scale(value, min, max);
//}

//template<uint64_t min = 0, uint64_t max = std::numeric_limits<uint64_t>::max(), typename T = uint8_t>
//std::percent lookupScale(T value) {
//    typedef typename std::remove_cv_t<T> U;
//    if (value < min) {
//        return std::percent{0u};
//    }
//    else if (value > max) {
//        return std::percent{100u};
//    }
//    return detail::LookupScalerFixed<U, U(min), U(max)>::scale(value);
//}

//std::percent scale_naiv(uint8_t value, uint8_t min, uint8_t max) {
//    if (value < min) {
//        return std::percent{0U};
//    }
//    else if (value > max) {
//        return std::percent{100U};
//    }
//    return std::percent{(uint8_t)(((uint16_t)((value - min) * 100u)) / (uint8_t)(max - min))};
//}

static inline
uint8_t scale (uint8_t value, uint8_t min, uint8_t max)
{
    return ((value - min) * 100u) / ((uint16_t) (uint8_t)(max - min));
}

uint8_t scale1 (uint8_t value)
{
    return scale (value, 0, 255);
}

//using constantRateTimer = AVR::Timer16Bit<1>;
//constexpr const auto constantRatePeriod = 1000_us;

//struct CTHandler : public IsrBaseHandler<AVR::ISR::Timer<1>::CompareA> {
//    static void isr() {
//        ++mCounter;
//    }
//    inline static volatile uint16_t mCounter = 0;
//};

//using isrRegistrar = IsrRegistrar<CTHandler>;

//struct Measure {
//    std::milliseconds time;
//    StringBuffer<20> name;
    
//};
const uint16_t iterations = 10000;
//std::array<Measure, 20> times;

//extern "C" {
//// todo: checking bounds
//uint8_t ascale(uint8_t, uint8_t, uint8_t);
//}

int main() {
//    isrRegistrar::init();
//    Scoped<EnableInterrupt> interruptEnabler;
    
//    constexpr std::hertz constantRateFrequency = 1 / constantRatePeriod;
//    constexpr auto tsd = AVR::Util::calculate<constantRateTimer>(constantRateFrequency);
//    static_assert(tsd, "wrong parameter");
//    constantRateTimer::prescale<tsd.prescaler>();
//    constantRateTimer::ocra<tsd.ocr>();
//    constantRateTimer::mode(AVR::TimerMode::CTC);
    
    [[maybe_unused]] uint8_t algo = 0;
//    {
//        y = 0;
//        uint32_t start = CTHandler::mCounter;
//        for(uint16_t n = 0; n < iterations; ++n) {
//            z = scale_naiv(y, 0, 255);
//            ++y;
//        }
//        uint32_t end = CTHandler::mCounter;           
//        times[algo].name.insertAt(0, "1 naiv"_pgm);
//        times[algo++].time = std::milliseconds{static_cast<uint16_t>(end - start)};
//    }
//    {
//        y = 0;
//        uint32_t start = CTHandler::mCounter;
//        for(uint16_t n = 0; n < iterations; ++n) {
//            z = std::scale(y, 0, 255);
//            ++y;
//        }
//        uint32_t end = CTHandler::mCounter;            
//        times[algo].name.insertAt(0, "2 scale var"_pgm);
//        times[algo++].time = std::milliseconds{static_cast<uint16_t>(end - start)};
//    }
//    {
//        y = 0;
//        uint32_t start = CTHandler::mCounter;
//        for(uint16_t n = 0; n < iterations; ++n) {
//            z = std::scale(y);
//            ++y;
//        }
//        uint32_t end = CTHandler::mCounter;            
//        times[algo].name.insertAt(0, "3 scale d/a"_pgm);
//        times[algo++].time = std::milliseconds{static_cast<uint16_t>(end - start)};
//    }
//    {
//        y = 0;
//        uint32_t start = CTHandler::mCounter;
//        for(uint16_t n = 0; n < iterations; ++n) {
//            z = std::fastScale<0, 255>(y);
//            ++y;
//        }
//        uint32_t end = CTHandler::mCounter;            
//        times[algo].name.insertAt(0, "4 fast"_pgm);
//        times[algo++].time = std::milliseconds{static_cast<uint16_t>(end - start)};
//    }
//    {
//        y = 0;
//        uint32_t start = CTHandler::mCounter;
//        for(uint16_t n = 0; n < iterations; ++n) {
//            z = lookupScale(y, uint8_t(0), uint8_t(255));
//            ++y;
//        }
//        uint32_t end = CTHandler::mCounter;            
//        times[algo].name.insertAt(0, "5 loUp Var"_pgm);
//        times[algo++].time = std::milliseconds{static_cast<uint16_t>(end - start)};
//    }
//    {
//        y = 0;
//        uint32_t start = CTHandler::mCounter;
//        for(uint16_t n = 0; n < iterations; ++n) {
//            z = lookupScale<0, 255>(y);
//            ++y;
//        }
//        uint32_t end = CTHandler::mCounter;            
//        times[algo].name.insertAt(0, "6 loUp Fixed"_pgm);
//        times[algo++].time = std::milliseconds{static_cast<uint16_t>(end - start)};
//    }
//    {
//        y = 0;
//        uint32_t start = CTHandler::mCounter;
//        for(uint16_t n = 0; n < iterations; ++n) {
//            z = std::percent{ascale(y, 0, 255)};
//            ++y;
//        }
//        uint32_t end = CTHandler::mCounter;            
//        times[algo].name.insertAt(0, "7 asm"_pgm);
//        times[algo++].time = std::milliseconds{static_cast<uint16_t>(end - start)};
//    }
    {
        y = 0;
//        uint32_t start = CTHandler::mCounter;
        for(uint16_t n = 0; n < iterations; ++n) {
//            z = std::percent{scale1(y)};
            z = scale1(y);
            ++y;
        }
//        uint32_t end = CTHandler::mCounter;            
//        times[algo].name.insertAt(0, "8 scale1"_pgm);
//        times[algo++].time = std::milliseconds{static_cast<uint16_t>(end - start)};
    }
//    {
//        y2 = 0;
//        uint32_t start = CTHandler::mCounter;
//        for(uint16_t n = 0; n < iterations; ++n) {
//            z = std::fastScale<0, 255>(y);
//            ++y;
//        }
//        uint32_t end = CTHandler::mCounter;            
//        times[algo].name.insertAt(0, "9 fast 16bit"_pgm);
//        times[algo++].time = std::milliseconds{static_cast<uint16_t>(end - start)};
//    }
//    {
//        y2 = 0;
//        uint32_t start = CTHandler::mCounter;
//        for(uint16_t n = 0; n < iterations; ++n) {
//            z = std::scale(y2, 0, 255);
//            ++y;
//        }
//        uint32_t end = CTHandler::mCounter;            
//        times[algo].name.insertAt(0, "10 scale 16bit var"_pgm);
//        times[algo++].time = std::milliseconds{static_cast<uint16_t>(end - start)};
//    }
    
//    for(uint8_t i = 0; i < times.size; ++i) {
//        std::outl<terminal>(times[i].name, " : "_pgm, times[i].time);
//    }        
    
//    y2 = y = 42;
//    {
//        auto p = scale_naiv(y, uint8_t(0), uint8_t(255));
//        std::outl<terminal>(p);
//    }
//    {
//        auto p = std::scale(y, uint8_t(0), uint8_t(255));
//        std::outl<terminal>(p);
//    }
//    {
//        auto p = std::scale(y);
//        std::outl<terminal>(p);
//    }
//    {
//        auto p = std::fastScale<0, 255>(y);
//        std::outl<terminal>(p);
//    }
//    {
//        auto p = lookupScale(y, uint8_t(0), uint8_t(255));
//        std::outl<terminal>(p);
//    }
//    {
//        auto p = lookupScale<0, 255>(y);
//        std::outl<terminal>(p);
//    }
//    {
//        auto p = std::percent{ascale(y, 0, 255)};
//        std::outl<terminal>(p);
//    }
//    {
//        auto p = std::percent{scale1(y)};
//        std::outl<terminal>(p);
//    }
//    {
//        auto p = std::fastScale<0, 255>(y2);
//        std::outl<terminal>(p);
//    }
    
//    {
//        for(auto v : std::FastDownScaler<uint8_t, 0, 150>::data.divisions) {
//            std::outl<terminal>(v);
//        }
//    }
    
    
    while(true) {}
}

//ISR(TIMER1_COMPA_vect) {
//    isrRegistrar::isr<AVR::ISR::Timer<1>::CompareA>();
//}

//template<typename L>
//void assertFunction(const PgmStringView&, const PgmStringView&, L) noexcept {
//    while(true) {}
//}
