/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2026 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <mcu/avr.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/adc.h>

#include <external/solutions/tick.h>
#include <external/solutions/blinker.h>
#include <external/sbus/sbus.h>
#include <external/sbus/sport.h>
#include <external/ibus/ibus2.h>
#include <external/hal/adccontroller.h>

#include <etl/output.h>
#include <etl/meta.h>
#include <etl/event.h>

#include "project.h"
#include "crsf.h"
#include "crsf_cb.h"
#include "sbus_cb.h"
#include "sport_cb.h"
#include "ibus_cb.h"
#include "leds.h"
#include "command.h"
#include "stdcomp.h"
#include "protocoll.h"

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

struct SSMSW01; // Custom PCB, Standardlayout

template<typename HW, typename Config> struct Devices;

template<typename Config>
struct Devices<SSMSW01, Config> {
    using clock = Project::Clock<>;
    static inline constexpr auto fRtc = Config::fRtc;

    using protocoll = Config::protocoll;

    using systemTimer = AVR::SystemTimer<Component::Rtc<0>, fRtc>;

    using usart1Position = AVR::Portmux::Position<Component::Usart<1>, Portmux::Default>;
    using portmux = AVR::Portmux::StaticMapper<Meta::List<usart1Position>>;

    // output-to-pin
    using led15 = Pin<Port<A>, 7>;
    using led14 = Pin<Port<A>, 6>;
    using led13 = Pin<Port<A>, 5>;
    using led12 = Pin<Port<A>, 4>;
    using led11 = Pin<Port<A>, 3>;
    using led10 = Pin<Port<A>, 2>;
    using led9  = Pin<Port<A>, 1>;
    using led8  = Pin<Port<A>, 0>;
    using led7 = Pin<Port<D>, 7>;
    using led6 = Pin<Port<D>, 6>;
    using led5 = Pin<Port<D>, 5>;
    using led4 = Pin<Port<D>, 4>;
    using led3 = Pin<Port<D>, 3>;
    using led2 = Pin<Port<D>, 2>;
    using led1 = Pin<Port<D>, 1>;
    using led0 = Pin<Port<D>, 0>;

    using tp = NoPin;

    using ledPin  = Pin<Port<F>, 0>;
    using led     = ActiveHigh<ledPin, Output>;
    using in0Pin  = Pin<Port<F>, 6>;
    using in0     = ActiveLow<in0Pin, Input>;
    using aVBatt  = Pin<Port<F>, 1>;
    using blinker = External::Blinker2<led, systemTimer, 100_ms, 2000_ms>;

    using adc = AVR::Adc<AVR::Component::Adc<0>, AVR::Resolution<12>, AVR::Vref::V2_048>;
    using adcController = External::Hal::AdcController<adc, Meta::NList<17, 0x42>>; // 0x42 = temp

    using adr0Pin = Pin<Port<C>, 2>;
    using adr0    = ActiveLow<adr0Pin, Input>;
    using adr1Pin = Pin<Port<C>, 3>;
    using adr1    = ActiveLow<adr1Pin, Input>;

#ifdef USE_ACTIVE_LOW_OUPUTS
    template<typename L>
    using ledOutput = ActiveLow<L, Output>;
#else
    template<typename L>
    using ledOutput = ActiveHigh<L, Output>;
#endif

    using leds  = Leds<Meta::transform<ledOutput, Meta::List<led0, led1, led2, led3, led4, led5, led6, led7>>>;
    using leds2 = Leds<Meta::transform<ledOutput, Meta::List<led8, led9, led10, led11, led12, led13, led14, led15>>>;

    struct InputConfig;
    using input = Serial<protocoll, InputConfig>;

    struct InputConfig {
        using systemTimer = Devices::systemTimer;
        using uartPosition = Devices::usart1Position;
        using ledGroups = Meta::List<Devices::leds, Devices::leds2>;
    };

    using serial = input::serial;
    using protocoll_adapter = input::protocoll_adapter;
    using terminalDevice = input::terminalDevice;
    using terminal = input::terminal;

    using components = StandardComponents<clock, systemTimer, portmux, leds, leds2,
                        in0, blinker, adcController, adr0, adr1, input, serial, terminalDevice>;

    static inline void init() {
        tp::dir<Output>();
        components::init();
#ifdef DEBUG_OUTPUT
        if constexpr (!std::is_same_v<terminalDevice, serial> && !std::is_same_v<terminalDevice, void>) {
            terminalDevice::template init<BaudRate<DEBUG_BAUDRATE>>();
        }
#endif
    }
    static inline void periodic() {
        tp::toggle();
        components::periodic();
    }
    static inline void ratePeriodic() {
        components::ratePeriodic();
    }
};
