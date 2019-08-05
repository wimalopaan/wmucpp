#pragma once

#include "../common/register.h"

#include <array>
#include <type_traits>
#include <etl/algorithm.h>

#include <external/units/physical.h>

namespace AVR {
    template<uint8_t N>
    using Size = etl::NamedConstant<N>;
    
    using register_type = std::byte;
    
    namespace Util {
        namespace Twi {
            template<typename CS>
            constexpr std::array<PrescalerPair<CS>, 4> prescalerValues = {
                PrescalerPair<CS>{CS::twps1 | CS::twps0, 64},
                PrescalerPair<CS>{CS::twps1           , 16},
                PrescalerPair<CS>{            CS::twps0,  4},
                PrescalerPair<CS>{static_cast<CS>(0) ,  1},
            };
            
        }
        namespace Timer {
            template<typename CS>
            constexpr std::array<PrescalerPair<CS>, 6> prescalerValues10Bit = {
                PrescalerPair<CS>{CS::cs2          | CS::cs0, 1024},
                PrescalerPair<CS>{CS::cs2                   , 256},
                PrescalerPair<CS>{         CS::cs1 | CS::cs0, 64},
                PrescalerPair<CS>{         CS::cs1          , 8},
                PrescalerPair<CS>{                   CS::cs0, 1},
                PrescalerPair<CS>{static_cast<CS>(0)        , 0},
            };
            
//            template<typename CS, typename MCU = DefaultMcuType>
//            constexpr CS csMask10Bit = {CS::cs2 | CS::cs1 | CS::cs0};
            
            template<typename CS>
            constexpr std::array<PrescalerPair<CS>, 8> prescalerValues10BitExtended = {
                PrescalerPair<CS>{CS::cs2 | CS::cs1 | CS::cs0, 1024},
                PrescalerPair<CS>{CS::cs2 | CS::cs1          , 256},
                PrescalerPair<CS>{CS::cs2           | CS::cs0, 128},
                PrescalerPair<CS>{CS::cs2                    , 64},
                PrescalerPair<CS>{          CS::cs1 | CS::cs0, 32},
                PrescalerPair<CS>{          CS::cs1          , 8},
                PrescalerPair<CS>{                    CS::cs0, 1},
                PrescalerPair<CS>{static_cast<CS>(0)         , 0}
            };

            template<typename CS>
            constexpr std::array<PrescalerPair<CS>, 16> prescalerValues14Bit = {
                PrescalerPair<CS>{CS::cs3 | CS::cs2 | CS::cs1 | CS::cs0, 16384},
                PrescalerPair<CS>{CS::cs3 | CS::cs2 | CS::cs1          , 8192},
                PrescalerPair<CS>{CS::cs3 | CS::cs2 |           CS::cs0, 4096},
                PrescalerPair<CS>{CS::cs3 | CS::cs2                    , 2048},
                PrescalerPair<CS>{CS::cs3 |           CS::cs1 | CS::cs0, 1024},
                PrescalerPair<CS>{CS::cs3 |           CS::cs1          , 512},
                PrescalerPair<CS>{CS::cs3 |                     CS::cs0, 256},
                PrescalerPair<CS>{CS::cs3                              , 128},
                PrescalerPair<CS>{          CS::cs2 | CS::cs1 | CS::cs0, 64},
                PrescalerPair<CS>{          CS::cs2 | CS::cs1          , 32},
                PrescalerPair<CS>{          CS::cs2 |           CS::cs0, 16},
                PrescalerPair<CS>{          CS::cs2                    , 8},
                PrescalerPair<CS>{                    CS::cs1 | CS::cs0, 4},
                PrescalerPair<CS>{                    CS::cs1          , 2},
                PrescalerPair<CS>{                              CS::cs0, 1},
                PrescalerPair<CS>{static_cast<CS>(0)                   , 0}
            };
            
        }
    }
    
    namespace Util::Timer {
        using megahertz = External::Units::megahertz;
        using hertz     = External::Units::hertz;
        
        using namespace Project;

//        template<typename T>
//        struct TimerSetupData final {
//            const uint16_t prescaler = 0;
//            const T ocr = 0;
//            const hertz f{0};
//            const bool isExact = false;
//            explicit constexpr operator bool() const {
//                return (prescaler > 0) && (ocr > 0);
//            }
//        };
        
//        template<typename T, auto N>
//        constexpr std::array<typename AVR::PrescalerPair<T>::scale_type, N> prescalerValues(const std::array<AVR::PrescalerPair<T>, N>& a) {
//            std::array<typename AVR::PrescalerPair<T>::scale_type, N> values;
//            for(uint8_t i = 0; i < N; ++i) {
//                values[i] = a[i].scale;
//            }
//            return values;
//        }
        
//        template<uint16_t Prescale, typename T, int N>
//        constexpr typename AVR::PrescalerPair<T>::bits_type bitsFrom(const std::array<AVR::PrescalerPair<T>, N>& a) {
//            for(const auto pair: a) {
//                if (pair.scale == Prescale) {
//                    return pair.bits;
//                }
//            }
//            return static_cast<typename AVR::PrescalerPair<T>::bits_type>(0);
//        }
//        template<typename T, int N>
//        constexpr typename AVR::PrescalerPair<T>::bits_type bitsFrom(uint16_t Prescale, const std::array<AVR::PrescalerPair<T>, N>& a) {
//            for(const auto pair: a) {
//                if (pair.scale == Prescale) {
//                    return pair.bits;
//                }
//            }
//            return static_cast<typename AVR::PrescalerPair<T>::bits_type>(0);
//        }
        
//        template<typename T, uint16_t N>
//        constexpr uint16_t bitsToPrescale(T bits, const std::array<AVR::PrescalerPair<T>, N>& a) {
//            for(const auto& pair : a) {
//                if (bits == pair.bits) {
//                    return pair.scale;
//                }
//            }
//            return 0;
//        }
        
//        template<typename MCUTimer, uint8_t N>
//        constexpr TimerSetupData<typename MCUTimer::value_type> calculate(const hertz& ftimer) {
//            using pBits = typename MCUTimer::template PrescalerBits<N>;
//            auto p = prescalerValues(pBits::values);
            
//            for(const auto& p : etl::sort(p)) { // aufsteigend
//                if (p > 0) {
//                    const auto tv = (Project::Config::fMcu / ftimer) / p;
//                    if ((tv > 0) && (tv < std::numeric_limits<typename MCUTimer::value_type>::max())) {
//                        const bool exact = ((Project::Config::fMcu.value / p) % tv) == 0;
//                        return {p, static_cast<typename MCUTimer::value_type>(tv), Project::Config::fMcu / tv / uint32_t(p), exact};
//                    }
//                }
//            }
//            return {};
//        }
        
//        template<typename MCUTimer>
//        constexpr uint16_t prescalerForAbove(const hertz& ftimer) {
//            using pBits = typename MCUTimer::mcu_timer_type::template PrescalerBits<MCUTimer::number>;
//            auto p = prescalerValues(pBits::values);
//            for(const auto& p : etl::sort(p, std::less<uint16_t>())) {
//                if (p > 0) {
//                    auto f = Config::fMcu / p;
//                    if (f >= ftimer) {
//                        return p;
//                    }
//                }
//            }
//            return 0;
//        }
        
//        template<typename MCUTimer, typename T>
//        constexpr uint16_t calculatePpmInParameter() {
//            using namespace std::literals::chrono;
//            using pBits = typename MCUTimer::mcu_timer_type::template PrescalerBits<MCUTimer::number>;
//            auto p = prescalerValues(pBits::values);
            
//            for(const auto& p : etl::sort(p)) {
//                if (p > 0) {
//                    const hertz f = Config::fMcu / p;
//                    const uint16_t ppmMin = 1_ms * f;
//                    const uint16_t ppmMax = 2_ms * f;
//                    if ((ppmMax < std::numeric_limits<T>::max()) && (ppmMin > 1)) {
//                        return p;
//                    }
//                }
//            }
//            return 0;
//        }
        
//        template<typename MCUTimer, typename T>
//        constexpr uint16_t calculatePpmOutParameter() {
//            using namespace std::literals::chrono;
//            using pBits = typename MCUTimer::mcu_timer_type::template PrescalerBits<MCUTimer::number>;
//            auto p = prescalerValues(pBits::values);
            
//            for(const auto& p : etl::sort(p)) {
//                if (p > 0) {
//                    const hertz f = Config::fMcu / p;
//                    const uint32_t ppmMin = 1_ms * f;
//                    const uint32_t ppmMax = 20_ms * f;
//                    if ((ppmMax < std::numeric_limits<T>::max()) && (ppmMin > 1)) {
//                        return p;
//                    }
//                }
//            }
//            return 0;
//        }
        
//        template<typename MCUTimer>
//        constexpr TimerSetupData<typename MCUTimer::value_type> caculateForExactFrequencyAbove(const hertz& f) {
//            using pBits = typename MCUTimer::mcu_timer_type::template PrescalerBits<MCUTimer::number>;
//            auto p = prescalerValues(pBits::values);
            
//            for(const auto& p : etl::sort(p, std::greater<uint16_t>())) { // absteigend
//                if (p > 0) {
//                    const auto timerStartValue = (Config::fMcu / f) / p;
//                    const decltype(timerStartValue) tmax = std::numeric_limits<typename MCUTimer::value_type>::max();
//                    for(auto tx = std::min(timerStartValue, tmax); tx > 0; --tx) {
//                        const auto fx = (Config::fMcu / p) / tx;
//                        if (fx <= (2 * f)) {
//                            uint32_t uf = Config::fMcu.value;
//                            const auto rx = uf % (p * tx); 
//                            if (rx == 0) {
//                                return {p, static_cast<typename MCUTimer::value_type>(tx), fx, true};
//                            }
//                        }
//                    }
//                }
//            }
//            return{0};
//        }
        
        
    }
    
}

#if defined(__AVR_ATmega1284P__)
# include "atmega1284p.h"
#elif defined(__AVR_ATmega328P__)
# include "atmega328p.h"
#elif defined(__AVR_ATmega88P__)
# include "atmega88p.h"
#elif defined(__AVR_ATmega168P__)
# include "atmega168p.h"
#elif defined(__AVR_ATmega328PB__)
# include "atmega328pb.h"
#elif defined(__AVR_ATmega8__)
# include "atmega8.h"
#elif defined(__AVR_ATmega324PB__)
# include "atmega324pb.h"
#elif defined(__AVR_ATtiny85__)
# include "../tiny/attiny85.h"
#elif defined(__AVR_ATtiny25__)
# include "attiny25.h"
#elif defined(__AVR_ATtiny84__)
# include "attiny84.h"
#else
# warning
typedef AVR::ATMegaNone DefaultMcuType;
#endif
