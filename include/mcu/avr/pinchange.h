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
#include "mcu/avr/isr.h"
#include "mcu/ports.h"

namespace AVR {

template<typename Name, typename MCU = DefaultMcuType>
struct PCNumber;

template<typename MCU>
struct PCNumber<A, MCU> {
    PCNumber() = delete;
    static constexpr uint8_t number = 0;
};
template<typename MCU>
struct PCNumber<B, MCU> {
    PCNumber() = delete;
    static constexpr uint8_t number = 1;
};
template<>
struct PCNumber<B, ATTiny85> {
    PCNumber() = delete;
    static constexpr uint8_t number = 0;
};
template<>
struct PCNumber<B, ATTiny25> {
    PCNumber() = delete;
    static constexpr uint8_t number = 0;
};
template<typename MCU>
struct PCNumber<C, MCU> {
    PCNumber() = delete;
    static constexpr uint8_t number = 2;
};
template<typename MCU>
struct PCNumber<D, MCU> {
    PCNumber() = delete;
    static constexpr uint8_t number = 3;
};

template<typename PSet, typename MCU = DefaultMcuType>
class PinChange final {
    static constexpr auto interrupts = getBaseAddr<typename MCU::Interrupt>;
public:
    static constexpr uint8_t pcGroupNumber = PCNumber<typename PSet::port_type::name_type>::number;
    static constexpr auto pc = getBaseAddr<typename MCU::PCInterrupts, pcGroupNumber>;
    typedef MCU mcu_type;
    typedef PSet pinset_type;

    typedef AVR::ISR::PcInt<pcGroupNumber> interrupt_type;
    
    PinChange() = delete;

    static void init() {
        PSet::template dir<Input>();
        PSet::allPullup();
        pc()->pcmsk = PSet::setMask;
        interrupts()->pcifr |= _BV(pcGroupNumber);
        interrupts()->pcicr |= _BV(pcGroupNumber);
    }
};

template<typename PinSet>
class PinChange<PinSet, ATTiny84> final {
    typedef ATTiny84 MCU;
    static constexpr auto interrupts = getBaseAddr<typename MCU::Interrupt>;
public:
    static constexpr uint8_t pcGroupNumber = PCNumber<typename PinSet::port_type::name_type>::number;
    static constexpr auto pc = getBaseAddr<typename MCU::PCInterrupts, pcGroupNumber>;
    typedef MCU mcu_type;
    typedef PinSet pinset_type;

    typedef AVR::ISR::PcInt<pcGroupNumber> interrupt_type;

    PinChange() = delete;

    static void init() {
        PinSet::template dir<Input>();
        PinSet::allPullup();
        pc()->pcmsk = PinSet::setMask;
        interrupts()->gifr |= _BV(pcGroupNumber + 4);
        interrupts()->gimsk |= _BV(pcGroupNumber + 4);
    }
};

template<typename PinSet>
class PinChange<PinSet, ATTiny25> final {
    typedef ATTiny25 MCU;
    static constexpr auto interrupts = getBaseAddr<typename MCU::Interrupt>;
public:
    static constexpr uint8_t pcGroupNumber = 0;
    static constexpr auto pc = getBaseAddr<typename MCU::PCInterrupts, pcGroupNumber>;
    typedef MCU mcu_type;
    typedef PinSet pinset_type;

    typedef AVR::ISR::PcInt<pcGroupNumber> interrupt_type;
    
    PinChange() = delete;

    static void init() {
        PinSet::template dir<Input>();
        PinSet::allPullup();
        pc()->pcmsk = PinSet::setMask;
#ifdef PCIF
        interrupts()->gifr |= _BV(PCIF);
        interrupts()->gimsk |= _BV(PCIE);
#endif
    }
};

}
