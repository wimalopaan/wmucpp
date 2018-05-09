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

#include "config.h"
#include "mcu/avr8.h"
#include "mcu/avr/isr.h"
#include "mcu/avr/util.h"
#include "util/disable.h"
#include "util/types.h"
#include "units/physical.h"

// fixme: flag-Regsiter

template<typename PinChange, typename MCUTimer, auto Channels = 8>
class PpmMultiChannel final : public IsrBaseHandler<AVR::ISR::PcInt<PinChange::pcInterruptNumber>>{
    enum class State : uint8_t {Invalid, Sync1, Data};
public:
    PpmMultiChannel() = delete;
    
    typedef MCUTimer mcu_timer_type;
    typedef typename MCUTimer::value_type  value_type;
    typedef typename MCUTimer::mcu_type mcu_type;
    typedef typename PinChange::pinset_type pinset_type;
    static_assert(pinset_type::size == 1, "use only one pin in pinset");
    
    using pin = Meta::front<typename pinset_type::pinlist>;
    
    inline static constexpr auto mcuTimer = MCUTimer::mcuTimer;
    inline static constexpr uint16_t prescaler = AVR::Util::calculatePpmOutParameter<MCUTimer, value_type>(); 
    inline static constexpr std::hertz timerFrequency = Config::fMcu / (uint32_t)prescaler;
    inline static constexpr value_type ppmMin = 1_ms * timerFrequency;
    inline static constexpr value_type ppmMax = 2_ms * timerFrequency;
    inline static constexpr value_type ppmMaxExtended = (2_ms * timerFrequency) - 100;// + (100_us * timerFrequency);
    
    static_assert(ppmMin >= 10, "wrong prescaler");
    static_assert(ppmMax <= std::numeric_limits<value_type>::max(), "wrong prescaler");
    static_assert(ppmMax > ppmMin, "wrong prescaler");
    //    static_assert(ppmMaxExtended > ppmMax, "wrong prescaler");
    
    inline static constexpr value_type ppmMid = (ppmMax + ppmMin) / 2;
    inline static constexpr value_type ppmDelta = (ppmMax - ppmMin) / 3;
    inline static constexpr value_type ppmMidLow = ppmMid - ppmDelta;
    inline static constexpr value_type ppmMidHigh = ppmMid + ppmDelta;
    inline static constexpr value_type ppmMinHigh = ppmMin + ppmDelta;
    inline static constexpr value_type ppmMaxLow = ppmMax - ppmDelta;
    
    static inline void init() {
        PinChange::init();
        MCUTimer::template prescale<prescaler>();
        MCUTimer::mode(AVR::TimerMode::Normal);
        MCUTimer::start();
    }
    
    static inline value_type value(uint8_t index) {
        Scoped<DisbaleInterrupt<>> di;
        return channels[index];
    }
    template<uint8_t Index>
    static inline value_type value() {
        Scoped<DisbaleInterrupt<>> di;
        return channels[Index];
    }
    
private:
    template<typename T>
    static inline T cdiff(T actual, T start) {
        if (actual >= start) {
            return actual - start;
        }
        else {
            return (std::numeric_limits<T>::module() - start) + actual;
        }
    }
    static inline void isr() {
        if (wasLow && pin::isHigh()) { // rising
            wasLow = false;
            *mcuTimer()->tcnt = 0;
        } 
        else if (!wasLow && !pin::isHigh()){ // falling
            wasLow = true;
            value_type actual = *mcuTimer()->tcnt;
            //            value_type period = cdiff<uint16_t>(actual, mTimerStartValueRising); 
            value_type period = actual; 
            switch (state) {
            case State::Invalid:
                if (period >= ppmMaxExtended) {
                    state = State::Sync1;   
                }
                break;
            case State::Sync1:
                if (period >= ppmMaxExtended) {
                    state = State::Data;   
                    channel = 0;
                }
                break;
            case State::Data:
                if (channel < Channels) {
                    if ((period >= ppmMaxLow) && (period <= ppmMax)) {
                        channels[channel] = period;
                    }
                    else if ((period <= ppmMinHigh) && (period >= ppmMin)) {
                        channels[channel] = period;
                    }
                    else {
                        channels[channel] = period;
                    }
                    ++channel;
                }
                else {
                    state = State::Invalid;
                }
                break;
            default:
                assert(false);
                break;
            }
        }
    }
    
    inline static volatile std::array<value_type, Channels> channels;
    inline static uint8_t channel = 0;
    inline static value_type mTimerStartValueRising = 0;
    inline static volatile bool wasLow = false;
    inline static volatile State state = State::Invalid;
};

template<typename MCUTimer, auto Channels = 8>
class PpmMultiChannelIcp final : public IsrBaseHandler<typename AVR::ISR::Timer<MCUTimer::number>::Capture> {
    enum class State : uint8_t {Invalid, Sync1, Data};
public:
    PpmMultiChannelIcp() = delete;
    
    typedef MCUTimer mcu_timer_type;
    typedef typename MCUTimer::value_type  value_type;
    typedef typename MCUTimer::mcu_type mcu_type;
    
    inline static constexpr auto mcuTimer = MCUTimer::mcuTimer;
    inline static constexpr auto mcuInterrupts = MCUTimer::mcuInterrupts;

    using parameter = PPMParameter<typename AVR::TimerParameter<MCUTimer::number, mcu_type>::timer_type>;
    
    static inline void init() {
        mcuTimer()->tccrb.template add<mcu_type::Timer16Bit::TCCRB::icnc | mcu_type::Timer16Bit::TCCRB::ices, DisbaleInterrupt<NoDisableEnable>>();
        mcuInterrupts()->timsk.template add<mcu_type::Timer16Interrupts::Mask::icie, DisbaleInterrupt<NoDisableEnable>>();
//        MCUTimer::mode(AVR::TimerMode::Normal);
//        MCUTimer::start();
    }
    
    static inline value_type value(uint8_t index) {
        Scoped<DisbaleInterrupt<>> di;
        return channels[index];
    }
    template<uint8_t Index>
    static inline value_type value() {
        Scoped<DisbaleInterrupt<>> di;
        return channels[Index];
    }
    
private:
    template<typename T>
    static inline T cdiff(T actual, T start) {
        if (actual >= start) {
            return actual - start;
        }
        else {
            return (std::numeric_limits<T>::module() - start) + actual;
        }
    }
    //    using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
    //    using testPin1 = AVR::Pin<PortB, 5>;
    
    static inline void isr() {
        mcuTimer()->tccrb.template clear<mcu_type::Timer16Bit::TCCRB::ices, DisbaleInterrupt<NoDisableEnable>>(); // now falling
        
        value_type actual = *mcuTimer()->icr;
        value_type period = cdiff<uint16_t>(actual, mTimerStartValueRising); 
        

        
//        value_type period = actual; 
//        switch (state) {
//        case State::Invalid:
//            if (period >= ppmMaxExtended) {
//                state = State::Sync1;   
//            }
//            break;
//        case State::Sync1:
//            if (period >= ppmMaxExtended) {
//                state = State::Data;   
//                channel = 0;
//            }
//            break;
//        case State::Data:
//            if (channel < Channels) {
//                if ((period >= ppmMaxLow) && (period <= ppmMax)) {
//                    channels[channel] = period;
//                }
//                else if ((period <= ppmMinHigh) && (period >= ppmMin)) {
//                    channels[channel] = period;
//                }
//                else {
//                    channels[channel] = period;
//                }
//                ++channel;
//            }
//            else {
//                state = State::Invalid;
//            }
//            break;
//        default:
//            assert(false);
//            break;
//        }
    }
    
    inline static volatile std::array<value_type, Channels> channels;
    inline static uint8_t channel = 0;
    inline static value_type mTimerStartValueRising = 0;
    inline static State state = State::Invalid;
};
