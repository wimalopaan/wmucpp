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

#include <mcu/common/concepts.h>

#include <etl/algorithm.h>
#include "../external/units/percent.h"


namespace External {
    
    using namespace External::Units::literals;
    
    template<AVR::Concepts::Actor PWMChannel, AVR::Concepts::Pin dirPin, etl::Concepts::Ranged InputType>
    struct TLE5205 final {
        
        TLE5205() = delete;
        
        enum class Mode : uint8_t {UniDirectional, BiDirectonal};
        
        using value_type = typename PWMChannel::value_type;
        using input_type = typename InputType::value_type;
        
        inline static constexpr input_type medium = (InputType::Upper + InputType::Lower) / 2;
        inline static constexpr input_type span = (InputType::Upper - InputType::Lower) / 2;

        inline static constexpr value_type hysterese = (10_ppc * std::numeric_limits<value_type>::max());
        
//        std::integral_constant<value_type, hysterese>::_;
        
        inline static constexpr void init() {
            dirPin::template dir<AVR::Output>();
        }        
        
        template<typename T>
        inline static constexpr void set(const T& v) {
            if (!v) return;
            if (mMode == Mode::BiDirectonal) {
                auto s = etl::distanceToCenter(v);
                value_type t = s.template mapTo<value_type>();
                if (v.toInt() >= medium) {
                    dirPin::on();
                }
                else {
                    dirPin::off();
                }
                set(t);
            }
            else {
                value_type t = v.template mapTo<value_type>();
                set(t);
            }
        }
        
        inline static constexpr void mode(Mode m) {
            mMode = m;
        }
        
    private:
        inline static void set(value_type v) {
            if (v < hysterese) {
                v = 0;
            }
            PWMChannel::set(v);
        }
        inline static Mode mMode = Mode::UniDirectional;
    };
}
