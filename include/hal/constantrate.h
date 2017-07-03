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
#include "mcu/ressource.h"
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

template<typename Buffer, typename Device, typename CounterType = typename Buffer::index_type, bool disableRx = true>
class ConstanteRateWriter { 
public:
    inline static void rateProcess() {
        if (!mEnable) return;
        
        if (counter == 0) {
            if constexpr(disableRx) {
                Device::template rxEnable<false>();
            }
        }
        if (counter < Buffer::size()) {
//            if (counter >= Buffer::cyclesBeforeAnswer) {
//                Device::put(Buffer::getByte(counter - Buffer::cyclesBeforeAnswer));
//            }
//            ++counter;
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
        if constexpr(E) {
            start();
        }
    }

    inline static void start() {
        Buffer::reset();
        counter = 0;
    }
    inline static void init() {
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
class [[deprecated]] ConstantRateAdapterOld : public IsrBaseHandler<Int> {
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

template<typename Reg, uint8_t BitNumber, MCU::Timer Timer, MCU::Interrupt Int, CRWriter... Writers >
class ConstantRateAdapter2 : public IsrBaseHandler<Int> {
    template<typename... II> friend class IsrRegistrar;

    inline static constexpr uint8_t bit_number = BitNumber;
    typedef Reg register_type;

    typedef MCU::Ressource::Type<Reg, Reg::reg_number, BitNumber> ressource_type;

    static_assert(BitNumber < 8, "wrong bit number");
    static inline constexpr typename Reg::type bit_mask{1 << BitNumber};
public:
    static void periodic() {
        if (std::any(Reg::get() & bit_mask)) { // this race condition doesn't harm (AVR lacks a test-and-set-instruction or atomic swap
            Reg::get() &= ~bit_mask;           // race
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
        static_assert(Int::number >= 0, "wrong interrupt number");
        Reg::get() |= bit_mask;;
    }
    constexpr static auto isr = rateTick;
};

template<uint8_t N, MCU::Timer Timer, MCU::Interrupt Interrupt, typename... PP>
using ConstantRateAdapter = ConstantRateAdapter2<AVR::RegisterFlags<DefaultMcuType::GPIOR, 0, std::byte>, N, Timer, Interrupt, PP...>;