
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

#include <mcu/avr.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/cppm.h>
#include <mcu/internals/pwm.h>
#include <mcu/internals/constantrate.h>

#include <etl/output.h>
#include <etl/fixedpoint.h>
#include <etl/scoped.h>
#include <etl/meta.h>

#include <external/hal/alarmtimer.h>
#include <external/hott/hott.h>
#include <external/bluetooth/roboremo.h>
#include <external/hal/devicemapper.h>

#ifdef MEM
# include "util/memory.h"
#endif
 
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;
using PortE = AVR::Port<DefaultMcuType::PortRegister, AVR::E>;

struct AsciiHandler;
struct BinaryHandler;
struct BCastHandler;

using sensorPA = Hott::SensorProtocollAdapter<0, AsciiHandler, BinaryHandler, BCastHandler>;
using roboremoPA = External::RoboRemo::ProtocollAdapter<0, 16>;

using sensorUsart = AVR::Usart<0, sensorPA, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>> ;
using btUsart = AVR::Usart<0, roboremoPA, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>> ;

using sensorData = Hott::SensorProtocollBuffer<0>;
using crWriterSensorBinary = ConstanteRateWriter<sensorData, sensorUsart>;
using menuData = Hott::SensorTextProtocollBuffer<0>;
using crWriterSensorText = ConstanteRateWriter<menuData, sensorUsart>;

using sumd = Hott::SumDProtocollAdapter<0, AVR::UseInterrupts<false>>;
using rcUsart = AVR::Usart<1, sumd, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>>;

namespace  {
    using namespace std::literals::chrono;
//    constexpr auto interval = 10_ms;
    constexpr auto interval = External::Units::duration_cast<std::chrono::milliseconds>(Hott::hottDelayBetweenBytes);
}

using systemClock = AVR::SystemTimer<0, interval>;
using alarmTimer = External::Hal::AlarmTimer<systemClock>;


