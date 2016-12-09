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

#include "config.h"
#include "mcu/avr8.h"
#include "mcu/ports.h"

namespace AVR {

template<typename Name>
struct PCNumber;

template<>
struct PCNumber<A> {
    PCNumber() = delete;
    static constexpr uint8_t number = 0;
};
template<>
struct PCNumber<B> {
    PCNumber() = delete;
    static constexpr uint8_t number = 1;
};
template<>
struct PCNumber<C> {
    PCNumber() = delete;
    static constexpr uint8_t number = 2;
};
template<>
struct PCNumber<D> {
    PCNumber() = delete;
    static constexpr uint8_t number = 3;
};

template<typename Pin, typename MCU = DefaultMcuType>
class PinChange final {
    static constexpr auto interrupts = getBaseAddr<typename MCU::Interrupt>;
public:
    static constexpr uint8_t pcGroupNumber = PCNumber<typename Pin::port::name_type>::number;
    static constexpr auto pc = getBaseAddr<typename MCU::PCInterrupts, pcGroupNumber>;
    typedef MCU mcu_type;
    typedef Pin pin_type;

    PinChange() = delete;

    static void init() {
        Pin::template dir<Input>();
        Pin::on();
        pc()->pcmsk = Pin::pinMask;
        interrupts()->pcifr |= _BV(pcGroupNumber);
        interrupts()->pcicr |= _BV(pcGroupNumber);
    }
};

template<typename Pin>
class PinChange<Pin, ATTiny84> final {
    typedef ATTiny84 MCU;
    static constexpr auto interrupts = getBaseAddr<typename MCU::Interrupt>;
public:
    static constexpr uint8_t pcGroupNumber = PCNumber<typename Pin::port::name_type>::number;
    static constexpr auto pc = getBaseAddr<typename MCU::PCInterrupts, pcGroupNumber>;
    typedef MCU mcu_type;
    typedef Pin pin_type;

    PinChange() = delete;

    static void init() {
        Pin::template dir<Input>();
        Pin::pullup();
        pc()->pcmsk = Pin::pinMask;
        interrupts()->gifr |= _BV(pcGroupNumber + 4);
        interrupts()->gimsk |= _BV(pcGroupNumber + 4);
    }
};


}
