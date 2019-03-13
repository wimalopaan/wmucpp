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

namespace Hott {
    using namespace std;    
    using namespace etl;
    
    template<uint8_t M, auto id,  typename AsciiHandler, typename BinaryHandler, typename BCastHandler>
    class SensorProtocollAdapter final {
        enum class hottstate {Undefined = 0, Request1, RequestA1, NumberOfStates};
        SensorProtocollAdapter() = delete;
    public:
        inline static bool process(std::byte c) { // from isr only (1,5µs)
            static hottstate state = hottstate::Undefined;
            switch (state) {
            case hottstate::Undefined:
                if (c == Hott::msg_start) {
                    state = hottstate::Request1;
                }
                if (c == std::byte{0x7f}) {
                    state = hottstate::RequestA1;
                }
                break;
            case hottstate::RequestA1:
                if (c == std::byte{0x0f}) {
                    AsciiHandler::start();
                    BinaryHandler::stop();
                    BCastHandler::stop();
                    state = hottstate::Undefined;
                }
                else {
                    AsciiHandler::process(c);
                    state = hottstate::Undefined;
                }
                break;
            case hottstate::Request1:
                if (c == id) {
                    AsciiHandler::stop();
                    BinaryHandler::start();
                    BCastHandler::stop();
                    state = hottstate::Undefined;
                }
                else if (c == Hott::msg_start) {
                    AsciiHandler::stop();
                    BinaryHandler::stop();
                    BCastHandler::start();
                    state = hottstate::Undefined;
                }
                else {
                    state = hottstate::Undefined;
                }
                break;
            default:
                assert(false);
                break;
            }
            return true;
        }
    };
    
}
