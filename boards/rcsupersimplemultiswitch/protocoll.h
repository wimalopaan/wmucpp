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
#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/ccl.h>
#include <mcu/internals/sigrow.h>

#include <external/solutions/tick.h>
#include <external/solutions/blinker.h>
#include <external/sbus/sbus.h>
#include <external/sbus/sport.h>
#include <external/ibus/ibus2.h>

#include <external/hal/adccontroller.h>

#include <etl/output.h>
#include <etl/meta.h>
#include <etl/event.h>

#include "crsf_cb.h"
#include "ibus_cb.h"
#include "sbus_cb.h"
#include "sport_cb.h"
#include "telemetry.h"
#include "link.h"

namespace Protocoll {
    struct Crsf;
    struct IBus;
    struct SBus;
    struct SPort;
};

template<typename Proto, typename Config>
struct Serial;


template<typename Config>
struct Serial<Protocoll::Crsf, Config> {
    using uartPosition = Config::uartPosition;
    using ledGroups = Config::ledGroups;

    struct CrsfAdapterConfig;
    using protocoll_adapter = Crsf::Adapter<CrsfAdapterConfig>;
    using serial = AVR::Usart<uartPosition, protocoll_adapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

#ifdef DEBUG_OUTPUT
    using terminalDevice = serial;
#else
    using terminalDevice = void;
#endif
    using terminal = etl::basic_ostream<terminalDevice>;

    using commandCallback = CrsfCommandCallback<ledGroups>;

    struct TelemetryConfig;
    using telemetry = Telemetry<TelemetryConfig>;

    struct CommandDecoderConfig;
    using commandDecoder = Crsf::Command<CommandDecoderConfig>;

    struct LinkDecoderConfig;
    using linkDecoder = Crsf::Link<LinkDecoderConfig>;

    struct CrsfAdapterConfig {
        using debug = etl::basic_ostream<void>;
        using decoder = Meta::List<commandDecoder, linkDecoder>;
    };
    struct CommandDecoderConfig {
        using debug = etl::basic_ostream<void>;
        using callback = commandCallback;
    };
    struct LinkDecoderConfig {
        using debug = etl::basic_ostream<void>;
        using callback = telemetry;
    };
    struct TelemetryConfig {
        using debug = etl::basic_ostream<void>;
#ifdef DEBUG_OUTPUT
        using crsf = void;
#else
        using crsf = Serial::serial;
#endif
    };
    static inline void init() {
        serial::template init<AVR::BaudRate<CRSF_BAUDRATE>>();
    }
};
template<typename Config>
struct Serial<Protocoll::IBus, Config> {
    using uartPosition = Config::uartPosition;
    using ledGroups = Config::ledGroups;

    struct CallbackConfig;
    using commandDecoder = IBusCommandCallback<CallbackConfig>;
    using protocoll_adapter = IBus2::Servo::ProtocollAdapter<0, void, commandDecoder>;
    using serial = AVR::Usart<uartPosition, protocoll_adapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<0>>;

#ifdef DEBUG_OUTPUT
    using terminalDevice = serial;
#else
    using terminalDevice = void;
#endif
    using terminal = etl::basic_ostream<terminalDevice>;

    struct CallbackConfig {
        using debug = terminal;
        using ledGroups = Serial::ledGroups;
    };
    static inline void init() {
        serial::template init<AVR::BaudRate<IBUS_BAUDRATE>>();
    }
};
template<typename Config>
struct Serial<Protocoll::SBus, Config> {
    using uartPosition = Config::uartPosition;
    using ledGroups = Config::ledGroups;
    using systemTimer = Config::systemTimer;

    struct CallbackConfig;
    using commandDecoder = SBusCommandCallback<CallbackConfig>;
    using protocoll_adapter = External::SBus::Servo::ProtocollAdapter<0, systemTimer, void, commandDecoder>;
    using serial = AVR::Usart<uartPosition, protocoll_adapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

#ifdef DEBUG_OUTPUT
    using terminalDevice = serial;
#else
    using terminalDevice = void;
#endif
    using terminal = etl::basic_ostream<terminalDevice>;

    struct CallbackConfig {
        using debug = terminal;
        using ledGroups = Serial::ledGroups;
    };
    static inline void init() {
        serial::template init<AVR::BaudRate<SBUS_BAUDRATE>, AVR::FullDuplex, true, 1>(); // 8E2
#ifndef SBUS_INVERT
        serial::rxInvert(true);
#endif
    }
};
template<typename Config>
struct Serial<Protocoll::SPort, Config> {
    using uartPosition = Config::uartPosition;
    using ledGroups = Config::ledGroups;
    using systemTimer = Config::systemTimer;

    struct CallbackConfig;
    using commandDecoder = SPortCommandCallback<CallbackConfig>;

    template<typename PA>
    using sensorUsart = AVR::Usart<uartPosition, PA, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<16>>;
    struct Provider {
        inline static constexpr auto valueId = External::SPort::ValueId::DIY;
        inline static uint32_t value() {
            return 1234;
        }
    };
    using sensor = External::SPort2::Sensor<SPORT_PHY, sensorUsart, systemTimer, Meta::List<Provider>, commandDecoder>;
    using protocoll_adapter = External::SBus::Servo::ProtocollAdapter<0, systemTimer, void, commandDecoder>;
    using serial = sensor;

    using terminalDevice = void;
    using terminal = etl::basic_ostream<terminalDevice>;

    struct CallbackConfig {
        using debug = terminal;
        using ledGroups = Serial::ledGroups;
    };
    static inline void init() {
        sensor::init();
    }
};
