/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2019 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
#include <mcu/internals/cppm.h>
#include <mcu/internals/constantrate.h>

#include <etl/output.h>
#include <etl/fixedpoint.h>
#include <etl/scoped.h>
#include <etl/meta.h>

#include <external/hal/alarmtimer.h>
#include <external/hott/hott.h>
#include <external/bluetooth/roboremo.h>
#include <external/bluetooth/qtrobo.h>
#include <external/hal/devicemapper.h>

#ifdef MEM
# include "util/memory.h"
#endif
 
namespace AVR {
    template<typename PinList, typename MCU = DefaultMcuType> struct Multiplexer;
    
    template<typename... Pins, typename MCU>
    struct Multiplexer<Meta::List<Pins...>, MCU> {
        inline static constexpr auto size = sizeof...(Pins);
        
        using index_type = etl::uint_ranged<uint8_t, 0, (1 << size) - 1>;
        
        constexpr inline static void init() {
            (Pins::template dir<AVR::Output>(), ...);
            (Pins::off(), ...);
        }
        
        template<auto... II>
        constexpr inline static void select(index_type number, std::index_sequence<II...>) {
            (((number.toInt() & (1 << II)) ? Pins::on() : Pins::off()), ...);    
        }
        
        constexpr inline static void select(index_type number) {
            select(number, std::make_index_sequence<sizeof...(Pins)>());
        }
    private:
    };
    
    template<typename... CC>
    struct Components {
        inline static constexpr void periodic() {
            (CC::periodic(), ...);    
        }
    };
    
}


using PortB = AVR::Port<AVR::B>;
using PortC = AVR::Port<AVR::C>;
using PortD = AVR::Port<AVR::D>;
using PortE = AVR::Port<AVR::E>;

using ppmInPin  = AVR::Pin<PortE, 2>;

using selectA0 = AVR::Pin<PortB, 0>;
using selectA1 = AVR::Pin<PortD, 7>;
using selectA2 = AVR::Pin<PortD, 6>;

using multiplexer = AVR::Multiplexer<Meta::List<selectA0, selectA1, selectA2>>;

using paired   = AVR::Pin<PortD, 5>;
using rxSelect = AVR::Pin<PortD, 3>;

using cppm = AVR::Cppm<1, AVR::A, 8, multiplexer, ppmInPin>;

#ifdef MATRIX
struct AsciiHandler;
struct BinaryHandler;
struct BCastHandler;
using sensorPA = Hott::SensorProtocollAdapter<0, Hott::gam_code, AsciiHandler, BinaryHandler, BCastHandler>;
using sensorUsart = AVR::Usart<AVR::Component::Uart<0>, sensorPA, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>> ;
using roboremoPA = External::RoboRemo::ProtocollAdapter<0, 16>;
using btUsart = AVR::Usart<AVR::Component::Uart<0>, roboremoPA, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>> ;
#endif


#if defined(SERVOTESTER) || defined(SCHALTMODUL)
using qtroboPA = External::QtRobo::ProtocollAdapter<0>;
using qtroboUsart = AVR::Usart<AVR::Component::Uart<0>, qtroboPA, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<512>> ;
#endif

using sensorData = Hott::GamProtocollBuffer<0>;
using menuData = Hott::SensorTextProtocollBuffer<Hott::gam_code, 0>;

#ifdef MATRIX
using crWriterSensorBinary = ConstanteRateWriter<sensorData, sensorUsart>;
using crWriterSensorText = ConstanteRateWriter<menuData, sensorUsart>;
#endif

using sumd = Hott::SumDProtocollAdapter<0, AVR::UseInterrupts<false>>;
using rcUsart = AVR::Usart<AVR::Component::Timer<1>, sumd, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>>;

namespace  {
    using namespace std::literals::chrono;
    constexpr auto interval = 10_ms;
//    constexpr auto interval = External::Units::duration_cast<std::chrono::milliseconds>(Hott::hottDelayBetweenBytes);
}

using systemClock = AVR::SystemTimer<AVR::Component::Timer<0>, interval>;
using alarmTimer = External::Hal::AlarmTimer<systemClock>;


