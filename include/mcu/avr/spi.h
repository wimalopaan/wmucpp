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

#include <stdint.h>

#include "config.h"
#include "mcu/avr8.h"
#include "mcu/avr/isr.h"
#include "container/fifo.h"
#include "hal/event.h"
#include "mcu/ports.h"

namespace AVR {

template<typename MCU = DefaultMcuType>
struct SpiMaster final {
    SpiMaster() = delete;
//    static constexpr uint8_t spcr = _BV(SPIE) | _BV(SPE) | _BV(MSTR) | _BV(SPR0) | _BV(SPR1);
    static constexpr auto spcr = MCU::Spi::CR::spie | MCU::Spi::CR::spe | MCU::Spi::CR::mstr | MCU::Spi::CR::spr1 | MCU::Spi::CR::spr0;
    typedef AVR::Output mosi_dir;
    typedef AVR::Input  miso_dir;
    typedef AVR::Output sck_dir;
    typedef AVR::Output ss_dir;
};
template<typename MCU = DefaultMcuType>
struct SpiSlave final {
    SpiSlave() = delete;
//    static constexpr uint8_t spcr = _BV(SPIE) | _BV(SPE);
    static constexpr auto spcr = MCU::Spi::CR::spie | MCU::Spi::CR::spe;
    typedef AVR::Input  mosi_dir;
    typedef AVR::Output miso_dir;
    typedef AVR::Input  sck_dir;
    typedef AVR::Input  ss_dir;
};

template<int N> struct SpiEvent;

template<>
struct SpiEvent<0>{
    SpiEvent() = delete;
    static constexpr EventType event = EventType::Spi0;
};
template<>
struct SpiEvent<1>{
    SpiEvent() = delete;
    static constexpr EventType event = EventType::Spi1;
};

template<int N, typename MCU> struct SpiPort;

template<>
struct SpiPort<0, AVR::ATMega1284P> {
    SpiPort() = delete;
    using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
    using mosi = AVR::Pin<PortB, 5>;
    using miso = AVR::Pin<PortB, 6>;
    using sck  = AVR::Pin<PortB, 7>;
    using ss  = AVR::Pin<PortB, 4>;
};

template<>
struct SpiPort<0, AVR::ATMega328P> {
    SpiPort() = delete;
    using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
    using mosi = AVR::Pin<PortB, 3>;
    using miso = AVR::Pin<PortB, 4>;
    using sck  = AVR::Pin<PortB, 5>;
    using ss  = AVR::Pin<PortB, 2>;
};

template<typename Derived>
struct SpiBase {
    SpiBase() = delete;
};

template<uint8_t N, typename MCU = DefaultMcuType>
class Spi final : public SpiBase<Spi<N, MCU>>, public IsrBaseHandler<typename AVR::ISR::Spi<N>::Stc> {
    static_assert(N < MCU::Spi::count, "wrong spi number");

    friend void ::SPI_STC_vect();

    using spiPort = SpiPort<N, MCU>;
    Spi() = delete;

public:
    typedef MCU mcu_type;
//    static constexpr const uint8_t number = N;

    template<typename Mode>
    static void init() {
        spiPort::mosi::template dir<typename Mode::mosi_dir>();
        spiPort::miso::template dir<typename Mode::miso_dir>();
        spiPort::sck::template dir<typename Mode::sck_dir>();
        spiPort::ss::template dir<typename Mode::ss_dir>();
        spiPort::ss::on();
        getBaseAddr<typename MCU::Spi, N>()->spcr.template set<Mode::spcr>();
    }
    static bool isReady() {
//        return getBaseAddr<typename MCU::Spi, N>()->spsr & _BV(SPIF);
        return getBaseAddr<typename MCU::Spi, N>()->spsr.template is_set<MCU::Spi::SR::spif>();
    }

    static bool put(uint8_t c) {
//        if (getBaseAddr<typename MCU::Spi, N>()->spsr & _BV(SPIF)) {
        if (getBaseAddr<typename MCU::Spi, N>()->spsr.template is_set<MCU::Spi::SR::spif>()) {
            return false;
        }
        // todo: nach dem Transfer wieder auf high()
        spiPort::ss::off();
        *getBaseAddr<typename MCU::Spi, N>()->spdr = c;
        return true;
    }
    
    // todo: mit EventManager doppelt
    static bool leak() {
        bool oBefore = overrun;
        overrun = false;
        return oBefore;
    }
    static void isr() {
        uint8_t c = *getBaseAddr<typename MCU::Spi, N>()->spdr;
        overrun |= !EventManager::enqueueISR({SpiEvent<N>::event, c});
    }
private:
    inline static bool overrun = false;
};

}
