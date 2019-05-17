#pragma once

#include "../../config.h"
#include <cstdint>
#include <external/units/physical.h>

#include "groups.h"

namespace AVR {
    using namespace Project;
    
    namespace Ppm::Util {
        using namespace AVR::Util::Timer;
        using namespace std::literals::chrono;
        using namespace External::Units::literals;
        
        template<auto TimerNumber, typename MCU = DefaultMcuType, typename VT = typename TimerParameter<TimerNumber, MCU>::value_type>
        constexpr uint16_t calculatePpmInParameter() {
            using value_type  = VT;        
            using pBits = prescaler_bits_t<TimerNumber>;

            auto p = prescalerValues(pBits::values);
            
            for(const auto& p : etl::sort(p)) { // aufsteigend
                if (p > 0) {
                    const hertz f = Config::fMcu / p;
                    const uint16_t ppmMin = 1_ms * f;
                    const uint16_t ppmMax = 2_ms * f;
                    if ((ppmMax < std::numeric_limits<value_type>::max()) && (ppmMin > 1)) {
                        return p;
                    }
                }
            }
            return 0;
        }

        template<auto TimerNumber, typename MCU = DefaultMcuType, typename VT = typename TimerParameter<TimerNumber, MCU>::value_type>
        constexpr uint16_t calculateCPpmInParameter() {
            using value_type  = VT;        
            using pBits = prescaler_bits_t<TimerNumber>;
            auto p = prescalerValues(pBits::values);
            
            for(const auto& p : etl::sort(p)) { // aufsteigend
                if (p > 0) {
                    const hertz f = Config::fMcu / p;
                    const etl::enclosing_t<value_type> ppmMin = 1_ms * f;
                    const etl::enclosing_t<value_type> ppmMax = 4_ms * f;
                    if ((ppmMax < std::numeric_limits<value_type>::max()) && (ppmMin > 1)) {
                        return p;
                    }
                }
            }
            return 0;
        }
        
        template<auto TimerNumber, typename MCU = DefaultMcuType, typename VT = typename TimerParameter<TimerNumber, MCU>::value_type>
        constexpr uint16_t calculateCPpmInParameter(const VT& max) {
            using value_type  = VT;        
            using pBits = prescaler_bits_t<TimerNumber>;
            auto p = prescalerValues(pBits::values);
            
            for(const auto& p : etl::sort(p)) { // aufsteigend
                if (p > 0) {
                    const hertz f = Config::fMcu / p;
                    const etl::enclosing_t<value_type> ppmMin = 1_ms * f;
                    const etl::enclosing_t<value_type> ppmMax = 4_ms * f;
                    if ((ppmMax <= max) && (ppmMax < std::numeric_limits<value_type>::max()) && (ppmMin > 1)) {
                        return p;
                    }
                }
            }
            return 0;
        }
        template<auto TimerNumber, typename MCU = DefaultMcuType, typename VT = typename TimerParameter<TimerNumber, MCU>::value_type>
        constexpr uint16_t calculateSPpmInParameter(const VT& max) {
            using value_type  = VT;        
            using pBits = prescaler_bits_t<TimerNumber>;
            auto p = prescalerValues(pBits::values);
            
            for(const auto& p : etl::sort(p)) { // aufsteigend
                if (p > 0) {
                    const hertz f = Config::fMcu / p;
                    const etl::enclosing_t<value_type> ppmMin = 1_ms * f;
                    const etl::enclosing_t<value_type> ppmMax = 20_ms * f;
                    if ((ppmMax <= max) && (ppmMax < std::numeric_limits<value_type>::max()) && (ppmMin > 1)) {
                        return p;
                    }
                }
            }
            return 0;
        }
        
        template<auto TimerNumber, typename MCU = DefaultMcuType>
        constexpr uint16_t calculatePpmOutParameter() {
            using value_type  = typename TimerParameter<TimerNumber, MCU>::value_type;        
            using pBits = prescaler_bits_t<TimerNumber>;
            auto p = prescalerValues(pBits::values);
            
            for(const auto& p : etl::sort(p)) { // aufsteigend
                if (p > 0) {
                    const hertz f = Config::fMcu / p;
                    const uint32_t ppmMin = 1_ms * f;
                    const uint32_t ppmMax = 20_ms * f;
                    if ((ppmMax < std::numeric_limits<value_type>::max()) && (ppmMin > 1)) {
                        return p;
                    }
                }
            }
            return 0;
        }
    }
    
    template<auto TimerNumber>
    class PPMParameter {
        PPMParameter() = delete;
    public:
        static constexpr uint32_t prescaler = AVR::Ppm::Util::calculatePpmOutParameter<TimerNumber>();
        static_assert(prescaler > 0, "wrong prescaler");
        static constexpr hertz timerFrequency = Config::fMcu / prescaler;
        static constexpr uint16_t ocMin = 1_ms * timerFrequency;
        static_assert(ocMin > 0, "wrong oc value");
        static constexpr uint16_t ocMax = 2_ms * timerFrequency;
        static_assert(ocMax > 0, "wrong oc value");
        static constexpr uint16_t ocDelta = ocMax - ocMin;
        static_assert(ocDelta > 0, "wrong oc value");
        static constexpr uint16_t ocFrame = 20_ms * timerFrequency;
        static_assert(ocFrame > 0, "wrong oc value");
        static constexpr uint16_t ocMedium = (ocMin + ocMax) / 2;
        static_assert(ocMedium> 0, "wrong oc value");
    
        static constexpr uint16_t ccPulse = 500_us * timerFrequency;
        static_assert(ccPulse > 0, "wrong oc value");
        static constexpr uint16_t ccPulseFrame = ocMax + ccPulse;
        static_assert(ccPulse > 0, "wrong oc value");
    };
}
