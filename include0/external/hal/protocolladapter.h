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
#include <cstddef>

#include <etl/fifo.h>

namespace External {
    namespace Hal {
        template<uint8_t Size>
        struct ActivatableBuffer {
            static inline bool isActive() {
                return active;
            }  
            static inline void activate(const bool on) {
                active = on;
            }
            static inline void clear() {
                fifo.clear();
            }
            static inline void push(const std::byte b) {
                if (active) {
                    fifo.push_back(b);
                }
            }  
        private:
            static inline bool active{false};
            static inline etl::FiFo<std::byte, Size> fifo;
        };
        
        template<typename Buf = void>
        class NullProtocollAdapter final {
        public:
            using buffer_t = Buf;
            using value_type = void;
            NullProtocollAdapter() = delete;
            static inline constexpr bool process(const std::byte) {
                return false;
            }
            static inline void ratePeriodic() {}
        };
    }
}
