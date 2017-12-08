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

#include "config.h"
#include "mcu/avr8.h"
#include "mcu/avr/isr.h"
#include "mcu/ports.h"

namespace AVR {

template<typename Name, typename MCU>
struct PCNumber {};

template<typename MCU>
struct PCNumber<A, MCU> {
    PCNumber() = delete;
    static constexpr auto flag   = MCU::Interrupt::PCFlags::if0;
    static constexpr auto mask = MCU::Interrupt::PCMask::ie0;
    static constexpr uint8_t interruptNumber = 0;
};
template<typename MCU>
struct PCNumber<B, MCU> {
    PCNumber() = delete;
    static constexpr auto flag   = MCU::Interrupt::PCFlags::if1;
    static constexpr auto mask = MCU::Interrupt::PCMask::ie1;
    static constexpr uint8_t interruptNumber = 1;
};
template<typename MCU>
struct PCNumber<C, MCU> {
    PCNumber() = delete;
    static constexpr auto flag   = MCU::Interrupt::PCFlags::if2;
    static constexpr auto mask = MCU::Interrupt::PCMask::ie2;
    static constexpr uint8_t interruptNumber = 2;
};
template<typename MCU>
struct PCNumber<D, MCU> {
    PCNumber() = delete;
    static constexpr auto flag   = MCU::Interrupt::PCFlags::if3;
    static constexpr auto mask = MCU::Interrupt::PCMask::ie3;
    static constexpr uint8_t interruptNumber = 3;
};
#if defined(__AVR_ATmega328PB__)
template<>
struct PCNumber<B, ATMega328PB> {
    PCNumber() = delete;
    static constexpr auto flag   = ATMega328PB::Interrupt::PCFlags::if0;
    static constexpr auto mask = ATMega328PB::Interrupt::PCMask::ie0;
    static constexpr uint8_t interruptNumber = 0;
};
template<>
struct PCNumber<C, ATMega328PB> {
    PCNumber() = delete;
    static constexpr auto flag   = ATMega328PB::Interrupt::PCFlags::if1;
    static constexpr auto mask = ATMega328PB::Interrupt::PCMask::ie1;
    static constexpr uint8_t interruptNumber = 1;
};
template<>
struct PCNumber<D, ATMega328PB> {
    PCNumber() = delete;
    static constexpr auto flag   = ATMega328PB::Interrupt::PCFlags::if2;
    static constexpr auto mask = ATMega328PB::Interrupt::PCMask::ie2;
    static constexpr uint8_t interruptNumber = 2;
};
#endif
#if defined(__AVR_ATtiny84__)
template<>
struct PCNumber<A, ATTiny84> {
    PCNumber() = delete;
    static constexpr auto flag   = ATTiny84::Interrupt::GIFlags::pcif0;
    static constexpr auto enable = ATTiny84::Interrupt::GIMask::pcie0;
    static constexpr uint8_t interruptNumber = 0;
};
template<>
struct PCNumber<B, ATTiny84> {
    PCNumber() = delete;
    static constexpr auto flag   = ATTiny84::Interrupt::GIFlags::pcif1;
    static constexpr auto enable = ATTiny84::Interrupt::GIMask::pcie1;
    static constexpr uint8_t interruptNumber = 1;
};
#endif
template<typename PSet, typename MCU = DefaultMcuType>
class PinChange final {
    static constexpr auto interrupts = getBaseAddr<typename MCU::Interrupt>;
public:
    static constexpr uint8_t pcInterruptNumber = PCNumber<typename PSet::port_type::name_type, MCU>::interruptNumber;
    static constexpr auto pcGroupFlag = PCNumber<typename PSet::port_type::name_type, MCU>::flag;
    static constexpr auto pcGroupMask= PCNumber<typename PSet::port_type::name_type, MCU>::mask;
    static constexpr auto pcInterrupt = getBaseAddr<typename MCU::PCInterrupts, pcInterruptNumber>;
    typedef MCU mcu_type;
    typedef PSet pinset_type;

    typedef AVR::ISR::PcInt<pcInterruptNumber> interrupt_type;
    
    PinChange() = delete;

    static void init() {
        PSet::template dir<Input>();
        PSet::allPullup();
        *pcInterrupt()->pcmsk = PSet::setMask;
        interrupts()->pcifr.template reset<pcGroupFlag>();
        interrupts()->pcicr.template add<pcGroupMask>();
//        interrupts()->pcifr |= _BV(pcGroupNumber);
//        interrupts()->pcicr |= _BV(pcGroupNumber);
    }
};

#if defined(__AVR_ATtiny84__)
template<typename PinSet>
class PinChange<PinSet, ATTiny84> final {
    typedef ATTiny84 MCU;
    static constexpr auto interrupts = getBaseAddr<typename MCU::Interrupt>;
public:
    static constexpr uint8_t pcInterruptNumber = PCNumber<typename PinSet::port_type::name_type, MCU>::interruptNumber;
    static constexpr auto pcGroupFlag = PCNumber<typename PinSet::port_type::name_type, MCU>::flag;
    static constexpr auto pcGroupEnable = PCNumber<typename PinSet::port_type::name_type, MCU>::enable;
    static constexpr auto pcInterrupt = getBaseAddr<typename MCU::PCInterrupts, pcInterruptNumber>;
    static constexpr auto pc = getBaseAddr<typename MCU::PCInterrupts, pcInterruptNumber>;
    typedef MCU mcu_type;
    typedef PinSet pinset_type;

    typedef AVR::ISR::PcInt<pcInterruptNumber> interrupt_type;

    PinChange() = delete;

    static void init() {
        PinSet::template dir<Input>();
        PinSet::allPullup();
        *pc()->pcmsk = PinSet::setMask;
//        interrupts()->gifr |= _BV(pcGroupNumber + 4);
//        interrupts()->gimsk |= _BV(pcGroupNumber + 4);
        interrupts()->gifr.template add<pcGroupFlag>();
        interrupts()->gimsk.template add<pcGroupEnable>();
    }
};
#endif

#if defined(__AVR_ATtiny25__)
template<typename PinSet>
class PinChange<PinSet, ATTiny25> final {
    typedef ATTiny25 MCU;
    static constexpr auto interrupts = getBaseAddr<typename MCU::Interrupt>;
public:
    static constexpr uint8_t pcInterruptNumber = 0;
    static constexpr auto pc = getBaseAddr<typename MCU::PCInterrupts, pcInterruptNumber>;
    typedef MCU mcu_type;
    typedef PinSet pinset_type;

    typedef AVR::ISR::PcInt<pcInterruptNumber> interrupt_type;
    
    PinChange() = delete;

    static void init() {
        PinSet::template dir<Input>();
        PinSet::allPullup();
        *pc()->pcmsk = PinSet::setMask;
        interrupts()->gifr.template add<AVR::ATTiny25::Interrupt::GIFlags::pcif>();
        interrupts()->gimsk.template add<AVR::ATTiny25::Interrupt::GIMask::pcie>();
    }
};
#endif
#if defined(__AVR_ATtiny85__)
template<typename PinSet>
class PinChange<PinSet, ATTiny85> final {
    typedef ATTiny85 MCU;
    static constexpr auto interrupts = getBaseAddr<typename MCU::Interrupt>;
public:
    static constexpr uint8_t pcInterruptNumber = 0;
    static constexpr auto pc = getBaseAddr<typename MCU::PCInterrupts, pcInterruptNumber>;
    typedef MCU mcu_type;
    typedef PinSet pinset_type;

    typedef AVR::ISR::PcInt<pcInterruptNumber> interrupt_type;
    
    PinChange() = delete;

    static void init() {
        PinSet::template dir<Input>();
        PinSet::allPullup();
        *pc()->pcmsk = PinSet::setMask;
        interrupts()->gifr.template reset<AVR::ATTiny85::Interrupt::GIFlags::pcif>();
        interrupts()->gimsk.template add<AVR::ATTiny85::Interrupt::GIMask::pcie>();
    }
};
#endif

}
