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
#include "mcu/avr/isr.h"
#include "std/algorithm.h"
#include "util/bits.h"
#include "units/physical.h"

template<typename W>
concept bool CRWriter() { 
    return std::is_same<W, void>::value || requires (W w) { 
        W::init();
        W::start();
        W::rateProcess();
    };
}

template<typename Pin, uint8_t V>
class TestBitShifter {
public:
    static void rateProcess() {
        if (Util::isSet<Util::MSB>(value)) {
            Pin::high();
        }
        else {
            Pin::low();
        }
        value <<= 1;
    }
    static void start() {
        value = V;
    }
    static void reset() {
        value = V;
    }
    static void init() {
        Pin::template dir<AVR::Output>();
        Pin::low();
    }
private:
    inline static uint8_t value = V;
};

template<typename Buffer, typename Device, typename CounterType = uint16_t, bool disableRx = true>
class ConstanteRateWriter { 
public:
    static void rateProcess() {
        if (!mEnable) return;
        
        if (counter == 0) {
            if constexpr(disableRx) {
                Device::template rxEnable<false>();
            }
        }
        if (counter < Buffer::size()) {
            if (auto data = Buffer::get(counter++)) {
                Device::put(*data);
            }
        }
        else {
            if (!Device::isEmpty()) {
                return;
            }
            else {
                if constexpr(disableRx) {
                    Device::template rxEnable<true>();
                }
            }
        }
    }
    template<bool E>
    static void enable() {
        mEnable = E;
    }

    static void start() {
        Buffer::reset();
        counter = 0;
    }
    static void init() {
        Buffer::init();
    }
private:
    inline static bool mEnable = true;
    inline static CounterType counter = 0;
};

namespace detail {
template<typename T>
struct Mapper {
    static void rateProcess() {
        T::rateProcess();
    }
    static void init() {
        T::init();
    }
    static void start() {
        T::start();
    }
};
template<>
struct Mapper<void> {
    static void rateProcess() {
    }
    static void init() {
    }
    static void start() {
    }
};

} // !detail

template<MCU::Timer Timer, MCU::Interrupt Int, CRWriter... Writers >
class ConstantRateAdapter : public IsrBaseHandler<Int> {
    template<typename... II> friend class IsrRegistrar;
    
public:
    static void periodic() {
        if (tickCounter > 0) {
            tickCounter = 0;
            (detail::Mapper<Writers>::rateProcess(),...);
        }
    }
    
    static constexpr void init() {
        if constexpr(!std::is_same<Timer, void>::value) {
            // todo: wgm2 allein ergibt irgendwie keine Sinn???
            Timer::mcuTimer()->tccrb.template add<Timer::mcu_timer_type::TCCRB::wgm2>();
            Timer::mcuInterrupts()->tifr.template add<Timer::flags_type::ocfa | Timer::flags_type::ocfb>();
            Timer::mcuInterrupts()->timsk.template add<Timer::mask_type::ociea>();
        }
        (detail::Mapper<Writers>::init(),...);
    }
    static void start() {
        (detail::Mapper<Writers>::start(),...);
    }
    static void rateTick() {
        ++tickCounter;
    }
    constexpr static auto isr = rateTick;
private:
    inline static volatile uint8_t tickCounter = 0;
};
