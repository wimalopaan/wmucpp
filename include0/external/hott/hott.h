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

#include "sensorprotocoll.h"
#include "sensorprotocolladapter.h"
#include "sensorprotocollbuffer.h"
#include "sensortextprotocollbuffer.h"
#include "sumdprotocoll.h"
#include "sumdprotocolladapter.h"

namespace Hott {
    
    template<typename Usart>
    class SensorProtocoll final {
    public:
        SensorProtocoll() = delete;
    private:
        static inline constexpr uint8_t mNumberOfRows = 7;
        static inline constexpr uint8_t mNumberOfColumns = 2;
        
        static inline uint8_t mRow = 0;
        static inline uint8_t mColumn = 0;
        static inline uint8_t mKey = 0;
        
        static inline GamMsg hottBinaryResponse {};
        static inline TextMsg hottTextResponse {};
    };
    
    
}
