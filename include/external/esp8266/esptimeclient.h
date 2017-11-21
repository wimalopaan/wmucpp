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

#include "esp8266.h"
#include "hal/event.h"
#include "std/time.h"

namespace ESP8266 {

template<typename ESP>
class TimeClient final {
    TimeClient() = delete;
public:
    static void init() {
        ESP::init();
        Util::delay(1_s);
        ESP::send(Esp8266::Command::Reset);
        Util::delay(1_s);
        ESP::send(Esp8266::Command::JoinAP1);
        Util::delay(1_s);
    }
    static void query() {
        
    }
    struct EspOkHandler: public EventHandler<EventType::Esp_OK> {
        static bool process(std::byte) {
            return true;
        }
    };
    struct EspIDataHandler: public EventHandler<EventType::Esp_IData> {
        static bool process(std::byte) {
            if (ESP::data().size() == 4) {
                time_t value = 0;
                for(uint8_t i = 0; i < 4; ++i) {
                    value <<= 8;
                    value += ESP::data()[i];
                }
                dt = value - NTP_OFFSET;
                EventManager::enqueue(EventType::EspTimeAvailable);
                return true;
            }
            return false;
        }
    };
    struct EspErrorHandler: public EventHandler<EventType::Esp_Error> {
        static bool process(std::byte) {
            EventManager::enqueue(EventType::EspTimeError);
            return true;
        }
    };
    class HandlerGroup final : public EventHandlerGroup<EspOkHandler, EspIDataHandler, EspErrorHandler>{
    };
private:
    inline static DateTime::TimeTm dt;
};


}
