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

#include "mcu/avr8.h"
#include "mcu/avr/isr.h"
#include "mcu/avr/util.h"
#include "units/physical.h"
#include "util/concepts.h"

namespace AVR {
    
    enum class TimerMode : uint8_t {Normal = 0, CTC, OverflowInterrupt, CTCNoInt, Icp, IcpNoInt, NumberOfModes};
    
    template<typename MCU, uint8_t N>
    struct TimerBase {
        TimerBase() = delete;
        typedef MCU mcu_type;
    };
    
    template<uint8_t N, typename MCU = DefaultMcuType>
    class Timer8Bit;
    
    template<uint8_t N, typename MCU>
    requires AVR::ATMega_X4<MCU>() || AVR::ATMega_X8<MCU>() || ((N == 0) && (AVR::ATTiny_X4<MCU>()))
    class Timer8Bit<N, MCU> : public TimerBase<MCU, N> {
                                         Timer8Bit() = delete;
    public:
    static constexpr uint8_t number = N;
    static constexpr auto mcuTimer = getBaseAddr<typename MCU::Timer8Bit, N>;
    static constexpr auto mcuInterrupts = getBaseAddr<typename MCU::Timer8Interrupts, N>;
    typedef typename MCU::Timer8Bit mcu_timer_type;
    typedef typename MCU::Timer8Bit::TCCRA tccra_type;
    typedef typename MCU::Timer8Bit::TCCRB tccrb_type;
    typedef typename MCU::Timer8Interrupts::Flags flags_type;
    typedef typename MCU::Timer8Interrupts::Mask mask_type;
    typedef typename MCU::Timer8Interrupts int_type;
    static constexpr auto csBitMask = AVR::csMask10Bit<tccrb_type>;
    typedef uint8_t value_type;
    
    template<int PreScale>
    static void prescale() {
        constexpr tccrb_type bits = AVR::Util::bitsFrom<PreScale>(MCU::Timer8Bit::template PrescalerBits<N>::values);
        static_assert(isset(bits), "wrong prescaler");
        mcuTimer()->tccrb.template set<bits>();
    }
    
    static void off() {
        constexpr auto bits = AVR::Util::bitsFrom<0>(MCU::Timer8Bit::template PrescalerBits<N>::values);
        mcuTimer()->tccrb.template set<bits>();
        *mcuTimer()->tcnt = 0;
    }
    static void reset() {
        *mcuTimer()->tcnt = 0;
    }
    static std::hertz frequency() {
        return Config::fMcu / (uint32_t)prescaler();
    }
    
    static typename AVR::PrescalerPair<tccrb_type>::scale_type prescaler() {
        const auto bits = mcuTimer()->tccrb.template get<csBitMask>();
        return AVR::Util::bitsToPrescale(bits, MCU::Timer8Bit::template PrescalerBits<N>::values);
    }
    
    static void start(){
    }
    
    static void ocra(uint8_t v) {
        *mcuTimer()->ocra = v;
    }
    template<uint8_t V>
    static void ocra() {
        *mcuTimer()->ocra = V;
    }
    
    static inline volatile const uint8_t& counter() {
        return *mcuTimer()->tcnt;
    }
    
    static void mode(TimerMode mode) {
        if (mode == TimerMode::CTC) {
            mcuInterrupts()->timsk.template add<MCU::Timer8Interrupts::Mask::ociea, DisbaleInterrupt<NoDisableEnable>>();
            mcuTimer()->tccra.template set<MCU::Timer8Bit::TCCRA::wgm1>();
        }
        if (mode == TimerMode::CTCNoInt) {
            mcuTimer()->tccra.template set<MCU::Timer8Bit::TCCRA::wgm1>();
        }
        else if (mode == TimerMode::Normal) {
        }
        else if (mode == TimerMode::OverflowInterrupt) {
            mcuInterrupts()->timsk.template add<MCU::Timer8Interrupts::Mask::toie, DisbaleInterrupt<NoDisableEnable>>();
        }
    }
    
    template<flags_type Compare, ::Util::Callable Callable>
    static void periodic(const Callable& f) {
        if (mcuInterrupts()->tifr.template isSet<Compare>()) {
            f();
            mcuInterrupts()->tifr.template reset<Compare>(); // reset
        } 
    }
    
    template<const std::hertz& F>
    static void setup(TimerMode timerMode) {
        constexpr auto t = AVR::Util::calculate<Timer8Bit>(F);
        static_assert(t, "falscher wert für p");
        
        prescale<t.prescaler>();
        ocra<t.ocr - 1>();
        mode(timerMode);
    }
};


template<AVR::ATTiny_X5 MCU>
class Timer8Bit<0, MCU> : public TimerBase<MCU, 0> {
public:
    static constexpr uint8_t N = 0;
    static constexpr uint8_t number = N;
    static constexpr auto mcuTimer = getBaseAddr<typename MCU::Timer8Bit, N>;
    static constexpr auto mcuInterrupts = getBaseAddr<typename MCU::TimerInterrupts>;
    typedef typename MCU::Timer8Bit mcu_timer_type;
    typedef typename MCU::Timer8Bit::TCCRA tccra_type;
    typedef typename MCU::Timer8Bit::TCCRB tccrb_type;
    typedef typename MCU::TimerInterrupts::Flags flags_type;
    typedef typename MCU::TimerInterrupts::Mask mask_type;
    static constexpr auto csBitMask = AVR::csMask10Bit<tccrb_type>;
    typedef uint8_t value_type;
    
    Timer8Bit() = delete;
    
    template<int PreScale>
    static void prescale() {
        constexpr tccrb_type bits = AVR::Util::bitsFrom<PreScale>(MCU::Timer8Bit::template PrescalerBits<N>::values);
        static_assert(isset(bits), "wrong prescaler");
        mcuTimer()->tccrb.template set<bits>();
    }
    
    static std::hertz frequency() {
        return Config::fMcu / (uint32_t)prescaler();
    }
    
    static typename AVR::PrescalerPair<tccrb_type>::scale_type prescaler() {
        const auto bits = mcuTimer()->tccrb.template get<csBitMask>();
        return AVR::Util::bitsToPrescale(bits, MCU::Timer8Bit::template PrescalerBits<N>::values);
    }
    
    static void start(){
    }
    
    static void ocra(uint8_t v) {
        *mcuTimer()->ocra = v;
    }
    template<uint8_t V>
    static void ocra() {
        *mcuTimer()->ocra = V;
    }
    
    static inline volatile const uint8_t& counter() {
        return *mcuTimer()->tcnt;
    }
    
    template<flags_type Compare, ::Util::Callable Callable>
    static void periodic(const Callable& f) {
        if (mcuInterrupts()->tifr.template isSet<Compare>()) {
            f();
            mcuInterrupts()->tifr.template reset<Compare>(); 
        } 
    }
    
    template<const std::hertz& F>
    static void setup(TimerMode timerMode) {
        constexpr auto t = AVR::Util::calculate<Timer8Bit>(F);
        static_assert(t, "falscher wert für p");
        
        prescale<t.prescaler>();
        ocra<t.ocr - 1>();
        mode(timerMode);
    }
    
    static void mode(TimerMode timerMode) {
        if (timerMode == TimerMode::CTC) {
            mcuInterrupts()->timsk.template add<MCU::TimerInterrupts::Mask::ocie0a, DisbaleInterrupt<NoDisableEnable>>();
            mcuTimer()->tccra.template set<MCU::Timer8Bit::TCCRA::wgm1>();
        }
        if (timerMode == TimerMode::CTCNoInt) {
            mcuTimer()->tccra.template set<MCU::Timer8Bit::TCCRA::wgm1>();
        }
        else if (timerMode == TimerMode::Normal) {
        }
        else if (timerMode == TimerMode::OverflowInterrupt) {
            mcuInterrupts()->timsk.template add<MCU::TimerInterrupts::Mask::toie0, DisbaleInterrupt<NoDisableEnable>>();
        }
    }
};

template<AVR::ATTiny_X5 MCU>
class Timer8Bit<1, MCU> : public TimerBase<MCU, 1> {
public:
    static constexpr uint8_t number = 1;
    static constexpr auto mcuTimer = getBaseAddr<typename MCU::Timer8BitHighSpeed, 1>;
    static constexpr auto mcuInterrupts = getBaseAddr<typename MCU::TimerInterrupts>;
    typedef typename MCU::Timer8BitHighSpeed mcu_timer_type;
    typedef typename MCU::Timer8BitHighSpeed::TCCR tccr_type;
    static constexpr auto csBitMask = AVR::csMask14Bit<tccr_type>;
    typedef uint8_t value_type;
    
    Timer8Bit() = delete;
    
    template<int PreScale>
    static void prescale() {
        constexpr tccr_type bits = AVR::Util::bitsFrom<PreScale>(MCU::Timer8BitHighSpeed::template PrescalerBits<1>::values);
        static_assert(isset(bits), "wrong prescaler");
        mcuTimer()->tccr.template set<bits>();
    }
    
    static std::hertz frequency() {
        return Config::fMcu / (uint32_t)prescaler();
    }
    
    static typename AVR::PrescalerPair<tccr_type>::scale_type prescaler() {
        const auto bits = mcuTimer()->tccr.template get<csBitMask>();
        return AVR::Util::bitsToPrescale(bits, MCU::Timer8BitHighSpeed::template PrescalerBits<1>::values);
    }
    
    static void start(){
    }
    
    static void ocra(uint8_t v) {
        *mcuTimer()->ocra = v;
    }   
    template<uint8_t V>
    static void ocra() {
        *mcuTimer()->ocra = V;
    }
    template<uint8_t V>
    static void ocrc() {
        *mcuTimer()->ocrc = V;
    }
    static void ocrc(uint8_t v) {
        *mcuTimer()->ocrc = v;
    }
    
    static inline volatile uint8_t& counter() {
        return *mcuTimer()->tcnt;
    }
    
    static void mode(TimerMode timerMode) {
        if (timerMode == TimerMode::CTC) {
            mcuInterrupts()->timsk.template add<MCU::TimerInterrupts::Mask::ocie1a, DisbaleInterrupt<NoDisableEnable>>();
        }
        if (timerMode == TimerMode::CTCNoInt) {
            mcuTimer()->tccr.template add<MCU::Timer8BitHighSpeed::TCCR::ctc>();
            mcuTimer()->tccr.template add<MCU::Timer8BitHighSpeed::TCCR::pwma>();
            mcuTimer()->tccr.template add<MCU::Timer8BitHighSpeed::TCCR::coma1>();
        }
        else if (timerMode == TimerMode::Normal) {
        }
        else if (timerMode == TimerMode::OverflowInterrupt) {
            mcuInterrupts()->timsk.template add<MCU::TimerInterrupts::Mask::toie1, DisbaleInterrupt<NoDisableEnable>>();
        }
    }
};

template<AVR::ATMega_8 MCU>
struct Timer8Bit<0, MCU> : public TimerBase<MCU, 0> {
    static constexpr uint8_t number = 0;
    static constexpr auto mcuTimer = getBaseAddr<typename MCU::Timer8BitSimple, 0>;
    static constexpr auto mcuInterrupts = getBaseAddr<typename MCU::TimerInterrupts>;
    typedef typename MCU::Timer8BitSimple mcu_timer_type;
    typedef uint8_t value_type;
    
    Timer8Bit() = delete;
    
    template<int PreScale>
    static constexpr void prescale() {
        constexpr auto bits = AVR::Util::bitsFrom<PreScale>(MCU::Timer8BitSimple::template PrescalerBits<0>::values);
        static_assert(bits != 0, "wrong prescaler");
        mcuTimer()->tccr |= bits;
    }
    static void start(){
    }
    
    static void mode(TimerMode timerMode) {
        if (timerMode == TimerMode::CTC) {
        }
        else if (timerMode == TimerMode::Normal) {
        }
    }
};

template<AVR::ATMega_8 MCU>
struct Timer8Bit<2, MCU> : public TimerBase<MCU, 2> {
    static constexpr uint8_t number = 2;
    static constexpr auto mcuTimer = getBaseAddr<typename MCU::Timer8BitSimple2, 2>;
    static constexpr auto mcuInterrupts = getBaseAddr<typename MCU::TimerInterrupts>;
    typedef typename MCU::Timer8BitSimple2 mcu_timer_type;
    typedef uint8_t value_type;
    
    Timer8Bit() = delete;
    
    template<int PreScale>
    static constexpr void prescale() {
        constexpr auto bits = AVR::Util::bitsFrom<PreScale>(MCU::Timer8BitSimple2::template PrescalerBits<2>::values);
        static_assert(bits != 0, "wrong prescaler");
        mcuTimer()->tccr |= bits;
    }
    static void start(){
    }
    
    static void mode(const TimerMode& mode) {
        if (mode == TimerMode::CTC) {
            mcuInterrupts()->timsk.template add<MCU::TimerInterrupts::Mask::ocie2, DisbaleInterrupt<NoDisableEnable>>();
            mcuTimer()->tccr.template set<MCU::Timer8BitSimple2::TCCR::wgm1>();
        }
        else if (mode == TimerMode::Normal) {
        }
    }
};

template<uint8_t N, typename MCU = DefaultMcuType>
struct Timer16Bit;

template<uint8_t N, typename MCU>
requires AVR::ATMega_X4<MCU>() || AVR::ATMega_X8<MCU>() || ((N == 1) && AVR::ATTiny_X4<MCU>())
struct Timer16Bit<N, MCU>: public TimerBase<MCU, N>
{
                                      typedef AVR::ISR::Timer<N> isr_type;
static constexpr uint8_t number = N;
static constexpr auto mcuTimer = getBaseAddr<typename MCU::Timer16Bit, N>;
static constexpr auto mcuInterrupts = getBaseAddr<typename MCU::Timer16Interrupts, N>;
typedef typename MCU::Timer16Bit mcu_timer_type;
typedef typename MCU::Timer16Bit::TCCRA tccra_type;
typedef typename MCU::Timer16Bit::TCCRB tccrb_type;
typedef typename MCU::Timer16Interrupts::Flags flags_type;
typedef typename MCU::Timer16Interrupts::Mask mask_type;
typedef typename MCU::Timer16Interrupts int_type;
static constexpr auto csBitMask = AVR::csMask10Bit<tccrb_type>;
typedef uint16_t value_type;

Timer16Bit() = delete;

template<int PreScale>
static void prescale() {
    constexpr auto bits = AVR::Util::bitsFrom<PreScale>(MCU::Timer16Bit::template PrescalerBits<N>::values);
    static_assert(isset(bits), "wrong prescaler");
    mcuTimer()->tccrb.template set<bits>();
}

static void off() {
    constexpr auto bits = AVR::Util::bitsFrom<0>(MCU::Timer16Bit::template PrescalerBits<N>::values);
    mcuTimer()->tccrb.template set<bits>();
    *mcuTimer()->tcnt = 0;
}

static std::hertz frequency() {
    return Config::fMcu / (uint32_t)prescaler();
}

static typename AVR::PrescalerPair<tccrb_type>::scale_type prescaler() {
    const auto bits = mcuTimer()->tccrb.template get<csBitMask>();
    return AVR::Util::bitsToPrescale(bits, MCU::Timer16Bit::template PrescalerBits<N>::values);
}

inline static void ocra(uint16_t v) {
    *mcuTimer()->ocra = v;
}
template<uint16_t V>
static void ocra() {
    *mcuTimer()->ocra = V;
}

static inline volatile const uint16_t& counter() {
    return *mcuTimer()->tcnt;
}
static inline volatile const uint16_t& icr() {
    return *mcuTimer()->icr;
}

static inline void noiseCancel(bool on = true) {
    if (on) {
        mcuTimer()->tccrb.template add<tccrb_type::icnc>();
    }    
    else {
        mcuTimer()->tccrb.template clear<tccrb_type::icnc>();
    }
}

static inline void captureRising(bool on = true) {
    if (on) {
        mcuTimer()->tccrb.template clear<tccrb_type::ices>();
    }    
    else {
        mcuTimer()->tccrb.template add<tccrb_type::ices>();
    }
}

static void start(){
}

inline static void reset() {
    *mcuTimer()->tcnt = 0;
}

template<flags_type Flag>
inline static void clearFlag() {
    mcuInterrupts()->tifr.template reset<Flag>();
}


template<flags_type Compare, ::Util::Callable Callable>
inline static void periodic(const Callable& f) {
    if (mcuInterrupts()->tifr.template isSet<Compare>()) {
        f();
        mcuInterrupts()->tifr.template reset<Compare>();
    } 
}

static void mode(TimerMode timerMode) {
    if (timerMode == TimerMode::CTC) {
        mcuInterrupts()->timsk.template add<MCU::Timer16Interrupts::Mask::ociea, DisbaleInterrupt<NoDisableEnable>>();
        mcuTimer()->tccrb.template add<MCU::Timer16Bit::TCCRB::wgm2, DisbaleInterrupt<NoDisableEnable>>();
    }
    if (timerMode == TimerMode::CTCNoInt) {
        mcuTimer()->tccrb.template add<MCU::Timer16Bit::TCCRB::wgm2, DisbaleInterrupt<NoDisableEnable>>();
    }
    if (timerMode == TimerMode::IcpNoInt) {
        mcuTimer()->tccrb.template add<MCU::Timer16Bit::TCCRB::icnc | MCU::Timer16Bit::TCCRB::ices, DisbaleInterrupt<NoDisableEnable>>();
    }
}
};

template<uint8_t N, AVR::ATMega_8 MCU>
requires (N == 1)
struct Timer16Bit<N, MCU>: public TimerBase<MCU, N>
{
                                      typedef AVR::ISR::Timer<N> isr_type;
static constexpr uint8_t number = N;
static constexpr auto mcuTimer = getBaseAddr<typename MCU::Timer16Bit, N>;
static constexpr auto mcuInterrupts = getBaseAddr<typename MCU::TimerInterrupts>;
typedef typename MCU::Timer16Bit mcu_timer_type;
typedef typename MCU::Timer16Bit::TCCRA tccra_type;
typedef typename MCU::Timer16Bit::TCCRB tccrb_type;
typedef typename MCU::TimerInterrupts::Flags flags_type;
typedef typename MCU::TimerInterrupts::Mask mask_type;
static constexpr auto csBitMask = AVR::csMask10Bit<tccrb_type>;
typedef uint16_t value_type;

Timer16Bit() = delete;

template<int PreScale>
static void prescale() {
    constexpr auto bits = AVR::Util::bitsFrom<PreScale>(MCU::Timer16Bit::template PrescalerBits<N>::values);
    static_assert(isset(bits), "wrong prescaler");
    mcuTimer()->tccrb.template set<bits>();
}

static std::hertz frequency() {
    return Config::fMcu / (uint32_t)prescaler();
}

static typename AVR::PrescalerPair<tccrb_type>::scale_type prescaler() {
    const auto bits = mcuTimer()->tccrb.template get<csBitMask>();
    return AVR::Util::bitsToPrescale(bits, MCU::Timer16Bit::template PrescalerBits<N>::values);
}

static void ocra(uint16_t v) {
    *mcuTimer()->ocra = v;
}
template<uint16_t V>
static void ocra() {
    *mcuTimer()->ocra = V;
}

static inline volatile const uint16_t& counter() {
    return *mcuTimer()->tcnt;
}

static void start(){
}

static void mode(TimerMode timerMode) {
    if (timerMode == TimerMode::CTC) {
        mcuInterrupts()->timsk.template add<MCU::TimerInterrupts::Mask::ocie1a, DisbaleInterrupt<NoDisableEnable>>();
        mcuTimer()->tccrb.template add<MCU::Timer16Bit::TCCRB::wgm2, DisbaleInterrupt<NoDisableEnable>>();
    }
}
};

}
