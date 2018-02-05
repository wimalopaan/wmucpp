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

#include <cstdint>
#include "hal/event.h"
#include "sensorprotocoll.h"

namespace Hott {
    
    template<uint16_t N, uint16_t Thresh, typename CT = uint16_t>
    struct NullPA {
        inline static bool process(std::byte) { // from isr only
            static CT counter = 0;
            if (++counter > Thresh) {
                EventManager::enqueue({EventType::NullPAEvent, std::byte{N}});
                counter = 0;
            }
            return true;
        }    
    };
    
    template<uint8_t M, ::Util::NamedFlag useEvents, typename AsciiHandler, typename BinaryHandler, typename BCastHandler>
    class SensorProtocollAdapter final {
        enum class hottstate {Undefined = 0, Request1, RequestA1, NumberOfStates};
        static_assert(useEvents::value != !std::is_same<AsciiHandler, void>::value, "without events, an AsciiHandler must be used");
        static_assert(useEvents::value != !std::is_same<BinaryHandler, void>::value, "without events, an BinaryHandler must be used");
        static_assert(useEvents::value != !std::is_same<BCastHandler, void>::value, "without events, an BroadCastHandler must be used");
    public:
        SensorProtocollAdapter() = delete;
    private:
        inline static bool process(std::byte c) { // from isr only (1,5µs)
            static hottstate state = hottstate::Undefined;
            switch (state) {
            case hottstate::Undefined:
                if (c == std::byte{0x80}) {
                    state = hottstate::Request1;
                }
                if (c == std::byte{0x7f}) {
                    state = hottstate::RequestA1;
                }
                break;
            case hottstate::RequestA1:
                if (c == std::byte{0x0f}) {
                    if constexpr(useEvents::value) {
                        EventManager::enqueueISR({EventType::HottAsciiRequest, std::byte{0}});
                    }
                    else {
                        AsciiHandler::start();
                        BinaryHandler::stop();
                        BCastHandler::stop();
                    }
                    state = hottstate::Undefined;
                }
                else {
                    if constexpr(useEvents::value) {
                        EventManager::enqueueISR({EventType::HottAsciiKey, std::byte{c}});
                    }
                    else {
                        AsciiHandler::process(c);
                    }
                    state = hottstate::Undefined;
                }
                break;
            case hottstate::Request1:
                if (c == std::byte{0x8d}) {
                    if constexpr(useEvents::value) {
                        EventManager::enqueueISR({EventType::HottBinaryRequest, std::byte{0}});
                    }
                    else {
                        AsciiHandler::stop();
                        BinaryHandler::start();
                        BCastHandler::stop();
                    }
                    state = hottstate::Undefined;
                }
                else if (c == std::byte{0x80}) {
                    if constexpr(useEvents::value) {
                        EventManager::enqueueISR({EventType::HottSensorBroadcast, std::byte{0}});
                    }
                    else {
                        AsciiHandler::stop();
                        BinaryHandler::stop();
                        BCastHandler::start();
                    }
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
