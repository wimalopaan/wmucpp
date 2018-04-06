/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <type_traits>

namespace MCU {
    namespace Ressource {
#ifdef USE_DEPRECATED
        template<typename RessourceType, uint8_t RessourceNumber = 0, uint8_t RessourceElement = 0>
        struct [[deprecated("use Hal::Controller instead")]] Type {
            inline static constexpr uint8_t maximumNumberOfElementsInRessource = 16;
            inline static constexpr uint8_t ressource_bit_number = RessourceNumber * maximumNumberOfElementsInRessource + RessourceElement;
            static_assert(ressource_bit_number < (8 * sizeof(uint64_t)));
        
            inline static constexpr uint64_t mask = (1 << ressource_bit_number);
            
            typedef RessourceType resource_type;
        };
    
        template<typename... RR>
        struct [[deprecated("use Hal::Controller instead")]] Registrar {
            static_assert(sizeof...(RR) < 8, "too much ressources");
            static inline constexpr uint64_t all = (RR::ressource_type::mask | ... );
            static_assert(sizeof...(RR) == Util::numberOfOnes(all), "multiple use of ressource");
            static void init() {}
        };
#endif
    }    
}
