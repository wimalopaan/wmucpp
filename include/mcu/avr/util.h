/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "std/limits.h"

namespace AVR {
namespace Util {

template<typename T>
struct TimerSetupData final {
    const uint16_t prescaler;
    const T ocr;
};

template<typename MCUTimer>
constexpr TimerSetupData<typename MCUTimer::value_type> calculate(const std::hertz& ftimer) {
//    static_assert(MCUTimer::hasOcrA || MCUTimer::hasOcrB, "need ocra or ocrb");
    using pRow = typename MCUTimer::mcu_timer_type::template PrescalerRow<MCUTimer::number>;
    for(const auto& p : pRow::values) {
        const auto tv = (Config::fMcu / ftimer) / p;
        if (tv < std::numeric_limits<typename MCUTimer::value_type>::max()) {
            return {p, static_cast<typename MCUTimer::value_type>(tv)};
        }
    }
    return {0, 0};
}

}
}
