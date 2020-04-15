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
#include <chrono>

#include <external/units/physical.h>

#include "../common/concepts.h"

namespace AVR {
    template<const auto& ResetInterval, typename MCU = DefaultMcuType>
    struct WatchDog;
    
    template<const auto& ResetInterval, AVR::Concepts::At01Series MCU>
    struct WatchDog<ResetInterval, MCU> {
        inline static constexpr auto mcu_wd = AVR::getBaseAddr<typename MCU::WatchDog>;
        
        inline static constexpr auto resetInterval = External::Units::duration_cast<std::chrono::milliseconds>(ResetInterval);
        
        inline static constexpr auto pv = []{
            for(auto pv : MCU::WatchDog::periodValues) {
                if (pv.timeout > (resetInterval * 2)) {
                    return pv.bits;
                }
            }
        }();
        
//        std::integral_constant<decltype(resetInterval), resetInterval>::_;
//        std::integral_constant<decltype(pv), pv>::_;
        
        template<typename ConfProt>
        inline static void init() {
            mcu_wd()->status.template waitForCleared<MCU::WatchDog::Status_t::busy>();
            ConfProt::unlock([]{
                mcu_wd()->ctrla.template set<pv>();
            });
        }
        template<typename ConfProt>
        inline static void off() {
            mcu_wd()->status.template waitForCleared<MCU::WatchDog::Status_t::busy>();
            ConfProt::unlock([]{
                mcu_wd()->ctrla.template set<MCU::WatchDog::CtrlA2_t::off>();
            });
        }

        inline static void reset() {
            __asm__ __volatile__ ( "wdr" "\n\t" :: ); 
        }
    private:
    };
}
