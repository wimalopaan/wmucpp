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
#include <std/utility>

#include "components/cpu.h"
#include "components/clock.h"
#include "components/rtc.h"
#include "components/ccl2.h"
#include "components/port1.h"
#include "components/portmux1.h"
#include "components/tca.h"
#include "components/tcb.h"
#include "components/tcd.h"
#include "components/usart.h"
#include "components/event0.h"
#include "components/event1.h"
#include "components/adc.h"
#include "components/vref1.h"
#include "components/sleep.h"
#include "components/reset.h"
#include "components/watchdog.h"
#include "components/sigrow.h"
#include "components/spi.h"
#include "components/dac.h"
#include "components/gpior.h"

#include "components/bitmask_operators1.h"

namespace AVR {
    namespace Series1 {
    }
}
