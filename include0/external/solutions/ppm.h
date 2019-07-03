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

#include "external/units/physical.h"

namespace External {
    using namespace External::Units;
    using namespace External::Units::literals;
    using namespace std::literals::chrono;
    using namespace AVR::Util::Timer;
    
    namespace Ppm {
        
        template<bool B>
        struct RisingEdge : etl::NamedFlag<B> {};
        
        template<etl::Concepts::NamedConstant TimerNumber, etl::Concepts::NamedFlag RisingEdge = RisingEdge<true>, typename AVR::AtMega = DefaultMcuType>
        struct IcpPpm {
            using value_type = uint16_t;
            
            static constexpr auto mcu_timer = TimerParameter<TimerNumber::value, MCU>::mcu_timer;
            static constexpr auto mcu_timer_interrupts = TimerParameter<TimerNumber::value, MCU>::mcu_timer_interrupts;
            
            using flags_type = typename TimerParameter<TimerNumber::value, MCU>::mcu_timer_interrupts_flags_type;
            
            using tb = typename TimerParameter<TimerNumber::value, MCU>::tb;
            
            static inline constexpr uint16_t prescaler = AVR::Ppm::Util::calculateSPpmInParameter<TimerNumber::value, DefaultMcuType, value_type>(65000);
            //    using x1 = std::integral_constant<uint16_t, prescaler>::_;;
            //    static inline constexpr uint16_t prescaler = AVR::Ppm::Util::calculateCPpmInParameter<Timer::number>();
            
            static inline constexpr hertz f = Project::Config::fMcu / prescaler;
            
            static inline constexpr auto exact_intervall = duration_cast<milliseconds>(std::numeric_limits<value_type>::max() / f);
            static_assert(exact_intervall.value > 0);
            
            static inline constexpr value_type ppm_pause = 19_ms * f;
            static inline constexpr value_type ppm_min  = 1_ms * f;
            static inline constexpr value_type ppm_max  = 2_ms * f;
            static inline constexpr value_type ppm_width = ppm_max - ppm_min;
            static inline constexpr value_type span = ppm_width / 2;
            static inline constexpr value_type medium = (ppm_max + ppm_min) / 2;
            
//                        std::integral_constant<uint16_t, prescaler>::_;
            //            std::integral_constant<uint16_t, ppm_min>::_;
            //            std::integral_constant<uint16_t, ppm_max>::_;
            //            std::integral_constant<uint16_t, ppm_pause>::_;
            
            static inline void periodic() {
                if (mcu_timer_interrupts()->tifr.template isSet<flags_type::icf>()) {
                    
                    uint16_t diff = (*mcu_timer()->icr >= mSavedValue) ? (*mcu_timer()->icr - mSavedValue) : (std::numeric_limits<value_type>::max() - mSavedValue + *mcu_timer()->icr);
                    mSavedValue = *mcu_timer()->icr;
                    
                    if (mcu_timer()->tccrb.template isSet<tb::ices>()) {
                        if constexpr(RisingEdge::value) {
                            mPause = diff;
                            ++mSyncCounter;
                        }
                        else {
                            mPulse = diff;
                        }
                        mcu_timer()->tccrb.template clear<tb::ices>();
                    }
                    else {
                        if constexpr(RisingEdge::value) {
                            mPulse = diff;
                        }
                        else {
                            mPause = diff;
                            ++mSyncCounter;
                        }
                        mcu_timer()->tccrb.template add<tb::ices>();
                    }
                    mcu_timer_interrupts()->tifr.template reset<flags_type::icf>(); // reset
                } 
            }
            
            static inline auto pulseRaw() {
                return mPulse;
            }
            static inline auto pause() {
                return mPause;
            }
            static inline auto syncCount() {
                return mSyncCounter;
            }
            static inline etl::int_ranged_NaN<int16_t, -span, span> pulse() {
                if ((mPulse < ppm_min) || (mPulse > ppm_max)) {
                    return {};
                }
                else {
                    return (int16_t)(mPulse - medium);
                }
            }
            
            template<etl::Concepts::Callable Callable>
            inline static void overflow(const Callable& f) {
                if (mcu_timer_interrupts()->tifr.template isSet<flags_type::tov>()) {
                    f();
                    mcu_timer_interrupts()->tifr.template reset<flags_type::tov>(); // reset
                } 
            }
            
            static inline void init() {
                constexpr auto bits = bitsFrom<prescaler>(prescaler_bits_v<TimerNumber::value>);            
                static_assert(isset(bits), "wrong prescaler");
                if constexpr(RisingEdge::value) {
                    mcu_timer()->tccrb.template set<bits | tb::icnc | tb::ices>();
                }
                else {
                    mcu_timer()->tccrb.template set<bits | tb::icnc>();
                }
            }
            
        private:
            inline static uint8_t  mSyncCounter = 0;
            inline static uint16_t mSavedValue = 0;
            inline static uint16_t mPulse = 0;
            inline static uint16_t mPause = 0;
        };
    }
}
