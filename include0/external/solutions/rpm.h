/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017, 2018 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <cstdint>
#include <limits>
#include <chrono>

#include <etl/rational.h>
#include <etl/concepts.h>
#include <etl/algorithm.h>

#include "mcu/common/timer.h"
#include "mcu/common/concepts.h"

#include "external/units/physical.h"

namespace External {
    
    using namespace External::Units;
    using namespace AVR::Util::Timer;

    namespace Rpm::Util {
        
        template<auto TimerNumber, typename MCU = DefaultMcuType>
        constexpr uint16_t calculateRpm(const RPM& rpmMin, const RPM& rpmMax, uint32_t tvMin = 50) {
            using value_type  = typename TimerParameter<TimerNumber, MCU>::value_type;  
            using pBits = prescaler_bits_t<TimerNumber>;
            
            constexpr auto sortedPRow = []{
                auto p = prescalerValues(pBits::values);
                etl::sort(p, std::greater<uint16_t>()); // absteigend
                return p;
            }();
            
            for(const auto& p : sortedPRow) {
                if (p > 0) {
                    uint32_t tvalMax = (Config::fMcu.value * 60) / (p * rpmMin.value()); 
                    uint32_t tvalMin = (Config::fMcu.value * 60) / (p * rpmMax.value()); 
                    if ((tvalMax < std::numeric_limits<value_type>::max()) && (tvalMin > tvMin)) {
                        return p;
                    }
                }
            }
            return 0;
        }
    }
        
    template<AVR::Concepts::ComponentNumber TN, const RPM& MaxRpm, const RPM& MinRpm, uint8_t MinMeasurements = 2, typename MCU = DefaultMcuType>
    class RpmWithIcp final {
        RpmWithIcp() = delete;
    public:
        static inline constexpr auto TimerNumber = TN::value;
        using mcu_timer_type = mcu_timer_t<TimerNumber>;
        
        using value_type  = typename TimerParameter<TimerNumber, MCU>::value_type;        
        using ta = typename TimerParameter<TimerNumber, MCU>::ta;        
        using tb = typename TimerParameter<TimerNumber, MCU>::tb;        
        using flags_type = typename TimerParameter<TimerNumber, MCU>::mcu_timer_interrupts_flags_type;
        using ocAPin = typename TimerParameter<TimerNumber, MCU>::ocAPin;
        using ocBPin = typename TimerParameter<TimerNumber, MCU>::ocBPin;
        
        static constexpr auto mcu_timer = TimerParameter<TimerNumber, MCU>::mcu_timer;
        static constexpr auto mcu_timer_interrupts = TimerParameter<TimerNumber, MCU>::mcu_timer_interrupts;
                
        static_assert(std::is_same_v<value_type, uint16_t>, "timer must be 16bit with icp");
        static constexpr uint32_t minPeriod = 50;
        
        static constexpr auto prescaler = Rpm::Util::calculateRpm<TimerNumber>(MaxRpm, MinRpm, minPeriod);
        static_assert(prescaler > 0);
        
        static constexpr hertz fTimer = Config::fMcu / (uint32_t)prescaler;
        
        inline static constexpr void init() {
            constexpr auto bits = bitsFrom<prescaler>(mcu_timer_type::template PrescalerBits<TimerNumber>::values);            
            static_assert(isset(bits), "wrong prescaler");
            mcu_timer()->tccrb.template set<tb::icnc | tb::ices | bits>();
        }
        
        inline static constexpr value_type period() {
            return mActualPeriod;
        }
        
        inline static constexpr value_type filteredPeriod() {
            if (mMeasurements >= MinMeasurements) {
                if (mActualPeriod >= minPeriod) {
                    return period();
                }
            }
            return 0;
        }
        
        inline static constexpr void reset() {
            mMeasurements.setToBottom();
        }
        
        inline static bool check() {
            static uint8_t lastNumberOfMeasurements = 0;
            if (mMeasurements.isTop()) {
                mMeasurements.set(MinMeasurements);
            }
            else {
                if (mMeasurements == lastNumberOfMeasurements) {
                    reset();
                    return false;
                }
            }
            lastNumberOfMeasurements = mMeasurements;
            return true;
        }
        
        inline static constexpr hertz frequency() {
            if (filteredPeriod() > 0) {
                return fTimer / (uint32_t)filteredPeriod();
            }
            return {};
        }
        
        inline static constexpr RPM rpm() {
            if (frequency().value > 0) {
                return RPM{frequency()};
            }
            return RPM{};
        }
        
        inline static constexpr void periodic() {
            if (mcu_timer_interrupts()->tifr.template isSet<flags_type::icf>()) {
                mcu_timer_interrupts()->tifr.template reset<flags_type::icf>();
                value_type actual = *mcu_timer()->icr;
                if (actual >= mTimerStartValue) {
                    mActualPeriod = actual - mTimerStartValue;
                }
                else {
                    mActualPeriod = (std::numeric_limits<value_type>::module() - mTimerStartValue) + actual;
                }
                mTimerStartValue = actual;
                ++mMeasurements;
            }
        }
//    private:
        inline static value_type mTimerStartValue = 0;
        inline static value_type mActualPeriod = 0;
        inline static etl::uint_ranged<uint8_t> mMeasurements{};
    };
}
