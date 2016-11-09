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
#include "mcu/mcu.h"
#include "mcu/avr/isr.h"
#include "container/fifo.h"
#include "hal/event.h"
#include "mcu/ports.h"

namespace AVR {

struct SpiMaster final {
    SpiMaster() = delete;
    static constexpr uint8_t spcr = _BV(SPIE) | _BV(SPE) | _BV(MSTR) | _BV(SPR0) | _BV(SPR1);
    typedef AVR::Output mosi_dir;
    typedef AVR::Input  miso_dir;
    typedef AVR::Output sck_dir;
    typedef AVR::Output ss_dir;
};
struct SpiSlave final {
    SpiSlave() = delete;
    static constexpr uint8_t spcr = _BV(SPIE) | _BV(SPE);
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
class Spi final : public SpiBase<Spi<N, MCU>> {
    static_assert(N < 2, "wrong spi number");
    friend void ::SPI_STC_vect();

    using spiPort = SpiPort<N, MCU>;
public:
    Spi() = delete;

    template<typename Mode>
    static void init() {
        spiPort::mosi::template dir<typename Mode::mosi_dir>();
        spiPort::miso::template dir<typename Mode::miso_dir>();
        spiPort::sck::template dir<typename Mode::sck_dir>();
        spiPort::ss::template dir<typename Mode::ss_dir>();
        spiPort::ss::on();
        getBaseAddr<typename MCU::Spi, N>()->spcr = Mode::spcr;
    }
    static bool isReady() {
        return getBaseAddr<typename MCU::Spi, N>()->spsr & _BV(SPIF);
    }

    static bool put(uint8_t c) {
        if (getBaseAddr<typename MCU::Spi, N>()->spsr & _BV(SPIF)) {
            return false;
        }
        spiPort::ss::off();
        getBaseAddr<typename MCU::Spi, N>()->spdr = c;
        return true;
    }
    static bool leak() {
        bool oBefore = overrun;
        overrun = false;
        return oBefore;
    }
private:
    static void isr() {
        uint8_t c = getBaseAddr<typename MCU::Spi, N>()->spdr;
        overrun |= !EventManager::enqueueISR({SpiEvent<N>::event, c});
    }
    static bool overrun;
};
template<uint8_t N, typename MCU>
bool Spi<N, MCU>::overrun = false;

}
