#define NDEBUG

#include <mcu/avr.h>
#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/sigrow.h>
#include <mcu/internals/ccl.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/pwm.h>

#include <external/solutions/blinker.h>
#include <external/solutions/apa102.h>

#include <external/hal/adccontroller.h>

#include <etl/meta.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

namespace Meta {
    template<template<size_t> typename, typename L>
    struct for_each;
    
    template<template<size_t> typename F, size_t... II>
    struct for_each<F, std::index_sequence<II...>> {
        using type = Meta::List<typename F<II>::type...>;  
    };
    
    template<template<size_t> typename F, typename L>
    using for_each_t = for_each<F, L>::type;
    
}


template<typename LineList> 
struct Charly;
template<typename... Lines> 
struct Charly<Meta::List<Lines...>> {
    using linesList = Meta::List<Lines...>;
    static inline constexpr size_t numberOfLeds = sizeof...(Lines) * (sizeof...(Lines) - 1);
    using ledIndizes = std::make_index_sequence<numberOfLeds>;

    static inline constexpr auto permutations = []{
        std::array<std::pair<uint8_t, uint8_t>, numberOfLeds> d;
        uint8_t l = 0;
        for(uint8_t i = 0; i < sizeof...(Lines); ++i) {
            for(uint8_t k = i + 1; k < sizeof...(Lines); ++k) {
                d[l++] = {i, k};
                d[l++] = {k, i};
            }
        }
        return d;
    }();
    
    template<size_t I>
    struct IndizesToTypes {
        using l1 = Meta::nth_element<permutations[I].first, linesList>;
        using l2 = Meta::nth_element<permutations[I].second, linesList>;
        using type = std::pair<l1, l2>;
    };
    
    using leds = Meta::for_each_t<IndizesToTypes, ledIndizes>;
    
    static inline void init() {
    }
    static inline void periodic() {
    }
    static inline void set(const uint8_t led) {
        if (led < Meta::size_v<leds>) {
            Meta::visitAt<leds>(led, []<typename Led1, typename Led2>(Meta::Wrapper<std::pair<Led1, Led2>>){
                                Led1::template dir<Output>();
                                Led2::template dir<Output>();
           });
        }
    }
};

template<typename ADC, typename Display>
struct VUMeter {
    using adc = ADC;
    using display = Display;
    
    template<size_t Steps>    
    struct GenerateSquare {
        constexpr auto operator()() {
            std::array<uint16_t, Steps> data;
            for(uint16_t i = 0; i < Steps; ++i) {
                int16_t v = i - 127;
                data[i] = v * v;
            }
            return data;
        }
    };
    using Square = AVR::Pgm::Util::Converter<GenerateSquare<256>>::pgm_type;
    
    static inline void init() {
        adc::template init<etl::DisbaleInterrupt<etl::NoDisableEnable>>();
        display::init();
    }
    static inline void periodic() {
        display::periodic();
        adc::whenConversionReady([]{
            const uint8_t v = adc::value() >> 2;
            const uint16_t s = Square::value(v);
            display::set(s);
        });
    }
};

template<auto HWRev = 0, typename MCU = DefaultMcuType>
struct Devices;

template<typename MCU>
struct Devices<0, MCU> {
    inline static constexpr megahertz fMcu{F_CPU / 1'000'000};
    inline static constexpr auto fSample = 32'000_Hz;
    
    using ccp = Cpu::Ccp<>;
    using clock = Clock<>;
    using sigrow = SigRow<>;

    using led = Pin<Port<C>, 6>; 
    
    
    using line0 = Pin<Port<C>, 4>; 
    using line1 = Pin<Port<C>, 5>; 
    using line2 = Pin<Port<C>, 6>; 
    using line3 = Pin<Port<C>, 7>; 
    using lines = Meta::List<line0, line1, line2, line3>;
    using charly = Charly<lines>;

    using adc = Adc<Component::Adc<0>, Resolution<10>, Vref::V4_096>;
    
    using vumeter = VUMeter<adc, charly>;
    
    using dbgPin = Pin<Port<D>, 0>; 
    using dbg = ActiveHigh<dbgPin, Output>;
    
    using portmux = AVR::Portmux::StaticMapper<Meta::List<>>;
    
    static inline void init() {
        portmux::init();
        ccp::unlock([]{
            clock::template init<fMcu>();
        });
        led::template dir<Output>();
        led::off();
        
        vumeter::init();
    }
};

template<typename Devices>
struct GlobalFsm {
    using devs = Devices;
    using vumeter = devs::vumeter;
    static void init() {
        devs::init();
    } 
    static void periodic() {
        ScopedPin<typename devs::dbg> measure;
        vumeter::periodic();        
    }
};

using devices = Devices<0>;
using gfsm = GlobalFsm<devices>;

int main() {
    gfsm::init();    
    while(true) {
        gfsm::periodic(); 
    }
}
