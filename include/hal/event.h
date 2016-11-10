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

#include <stdint.h>

#include "config.h"
#include "mcu/avr8.h"
#include "util/bits.h"
#include "mcu/avr/mcutimer.h"
#include "container/fifo.h"
#include "util/disable.h"

enum class EventType : uint8_t {
    NoEvent,
    Test,
    Timer,
    UsartRecv0, UsartRecv1, UsartRecv2,
    UsartFe0, UsartFe1, UsartFe2,
    UsartUpe0, UsartUpe1, UsartUpe2,
    UsartDor0, UsartDor1, UsartDor2,
    SwUsartRecv0, SwUsartRecv1,
    Spi0, Spi1,
    HottBinaryRequest, HottAsciiRequest, HottSensorBroadcast, HottAsciiKey,
    Ppm1Up, Ppm1Down, Ppm2Up, Ppm2Down,
    ButtonPress,
    ButtonPress0, ButtonPress1, ButtonPress2, ButtonPress3, ButtonPress4, ButtonPress5, ButtonPress6, ButtonPress7,
};

template<typename T>
struct Event final {
    EventType type;
    T value;
};

typedef Event<uint8_t> Event8u_t;

namespace AVR {
template<uint8_t N, typename PA, typename MCU> class Usart;
template<uint8_t N, typename MCU> class Spi;
template<uint8_t N> class SWUsart;
}

namespace Hott {
template<uint8_t N> class SensorProtocollAdapter;
}

template<typename... PP>
class PeriodicGroup {
    friend void ::TIMER0_COMPA_vect();
public:
    static void periodic() {
        if (tickCounter > 0) {
            --tickCounter;
            (PP::periodic(),...); // fold
        }
    }
private:
    static uint8_t tickCounter;
};
template<typename... PP>
uint8_t PeriodicGroup<PP...>::tickCounter = 0;

template<typename... EE>
class EventHandlerGroup {
    template<int N, typename T, typename... TT>
    class Processor final {
    public:
        static void process(const Event8u_t& e) {
            if (e.type == T::eventType) {
                T::process(e.value);
            }
            Processor<N - 1, TT..., void>::process(e);
        }
    };
    template<typename... TT>
    class Processor<0, void, TT...> {
    public:
        static void process(const Event8u_t&) {}
    };
public:
    static void process(const Event8u_t& event) {
        Processor<sizeof...(EE), EE...>::process(event);
    }
};

class EventManager final
{
    template<uint8_t> friend class Hott::SensorProtocollAdapter;
    template<uint8_t N, typename MCU> friend class AVR::Usart;
    template<uint8_t N, typename MCU> friend class AVR::Spi;
    template<uint8_t N> friend class SWUsart;
public:
    EventManager() = delete;
    static bool enqueue(const Event8u_t& event) {
        return fifo().push_back(event); // lockfree fifo
    }

    template<typename PP, typename EE, typename P>
    static void run(const P& periodic) {
        while(true) {
            PP::periodic();
            periodic();
            if (auto event = fifo().pop_front()) {
                EE::process(*event);
            }
        }
    }
    template<typename PP, typename EE>
    static void run() {
        while(true) {
            PP::periodic();
            if (auto event = fifo().pop_front()) {
                EE::process(*event);
            }

        }
    }
private:
    static bool enqueueISR(const Event8u_t& event) {
        return fifo().push_back(event);
    }
    // header only: to avoid static data member
    static std::FiFo<Event8u_t, Config::EventManager::EventQueueLength>& fifo() {
        static std::FiFo<Event8u_t, Config::EventManager::EventQueueLength> fifo;
        return fifo;
    }
};

template<EventType Type>
struct EventHandler {
    EventHandler() = delete;
    friend class EventManager;
    static constexpr EventType eventType = Type;
};
