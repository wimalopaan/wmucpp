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

#include "avr8defs.h"

#pragma pack(push)
#pragma pack(1)

namespace AVR {

struct ATTiny85 final {
    ATTiny85() = delete;
    struct Timer8Bit {
        DataRegister<Timer8Bit, ReadWrite> ocrb;
        DataRegister<Timer8Bit, ReadWrite> ocra;
        enum class TCCRA : uint8_t {
            coma0 = (1 << COM0A0),
            coma1 = (1 << COM0A1),
            comb0 = (1 << COM0B0),
            comb1 = (1 << COM0B1),
            wgm0 = (1 << WGM00),
            wgm1 = (1 << WGM01)
        };
        ControlRegister<Timer8Bit, TCCRA> tccra;
        volatile uint8_t padding[0x32 - 0x2A - 1];
        DataRegister<Timer8Bit, ReadWrite> tcnt;
        enum class TCCRB : uint8_t {
            foca = (1 << FOC0A),
            focb = (1 << FOC0B),
            wgm2 = (1 << WGM02),
            cs2 = (1 << CS02),
            cs1 = (1 << CS01),
            cs0 = (1 << CS00),
        };
        ControlRegister<Timer8Bit, TCCRB> tccrb;
        template<int N> struct Address;
        template<int N> struct PrescalerBits;
    };
    struct Timer8BitHighSpeed {
        DataRegister<Timer8Bit, ReadWrite> ocrb;
        volatile uint8_t gtccr;
        DataRegister<Timer8Bit, ReadWrite> ocrc;
        DataRegister<Timer8Bit, ReadWrite> ocra;
        DataRegister<Timer8Bit, ReadWrite> tcnt;
        enum class TCCR : uint8_t {
            ctc = (1 << CTC1),
            pwma = (1 << PWM1A),
            coma1 = (1 << COM1A1),
            coma0 = (1 << COM1A0),
            cs3 = (1 << CS13),
            cs2 = (1 << CS12),
            cs1 = (1 << CS11),
            cs0 = (1 << CS10),
        };
        ControlRegister<Timer8BitHighSpeed, TCCR> tccr;
        template<int N> struct Address;
        template<int N> struct PrescalerBits;
    };
    
    struct USI {
        enum class USIC : uint8_t {
            sie = (1 << USISIE),
            oie = (1 << USIOIE),
            wm1 = (1 << USIWM1),
            wm0 = (1 << USIWM0),
            cs1 = (1 << USICS1),
            cs0 = (1 << USICS0),
            clk = (1 << USICLK),
            tc =  (1 << USITC)
        };
        ControlRegister<USI, USIC> usicr;
        enum class USIS : uint8_t {
            sif  = (1 << USISIF),            
            oif  = (1 << USIOIF),            
            pf   = (1 << USIPF),            
            dc   = (1 << USIDC),            
            cnt3 = (1 << USICNT3),            
            cnt2 = (1 << USICNT2),            
            cnt1 = (1 << USICNT1),            
            cnt0 = (1 << USICNT0) 
        };
        ControlRegister<USI, USIS> usisr;
        DataRegister<USI, ReadWrite> usidr;
        DataRegister<USI, ReadWrite> usibr;
        template<int N> struct Address;
    };

    struct PortRegister {
        volatile uint8_t in;
        volatile uint8_t ddr;
        volatile uint8_t out;
        template<typename P> struct Address;
    };
    class TimerInterrupts {
    public:
        enum class Flags : uint8_t {
            ocf1a = (1 << OCF1A),
            ocf1b = (1 << OCF1B),
            ocf0a = (1 << OCF0A),
            ocf0b = (1 << OCF0B),
            tov1  = (1 << TOV1),
            tov0  = (1 << TOV0)
        };
        ControlRegister<TimerInterrupts, Flags> tifr;
        enum class Mask : uint8_t {
            ocie1a = (1 << OCIE1A),
            ocie1b = (1 << OCIE1B),
            ocie0a = (1 << OCIE0A),
            ocie0b = (1 << OCIE0B),
            toie1  = (1 << TOIE1),
            toie0  = (1 << TOIE0)
        };
        ControlRegister<TimerInterrupts, Mask> timsk;
        static constexpr uint8_t address = 0x58;
    };
    struct Interrupt {
        enum class GIFlags : uint8_t {
            intf = (1 << INTF0),
            pcif = (1 << PCIF)
        };
        ControlRegister<Interrupt, GIFlags> gifr;
        enum class GIMask : uint8_t {
            ie   = (1 << INT0),
            pcie = (1 << PCIE)
        };
        ControlRegister<Interrupt, GIMask> gimsk;
        static constexpr uint8_t address = 0x5a;
    };
    struct PCInterrupts {
        DataRegister<PCInterrupts, ReadWrite> pcmsk;
        template<int N> struct Address;
    };
};

template<>
struct ATTiny85::PCInterrupts::Address<0> {
    static constexpr uint8_t value = 0x35;
};

template<>
struct ATTiny85::USI::Address<0> {
    static constexpr uint8_t value = 0x2d;
};

template<>
struct ATTiny85::PortRegister::Address<B> {
    static constexpr uint8_t value = 0x36;
};

template<>
struct ATTiny85::Timer8Bit::Address<0> {
    static constexpr uint8_t value = 0x48;
};

}
namespace std {
template<>
struct enable_bitmask_operators<AVR::ATTiny85::USI::USIC> {
    static constexpr bool enable = true;
};
template<>
struct enable_bitmask_operators<AVR::ATTiny85::USI::USIS> {
    static constexpr bool enable = true;
};
template<>
struct enable_bitmask_operators<AVR::ATTiny85::Timer8Bit::TCCRA> {
    static constexpr bool enable = true;
};
template<>
struct enable_bitmask_operators<AVR::ATTiny85::Timer8Bit::TCCRB> {
    static constexpr bool enable = true;
};
template<>
struct enable_bitmask_operators<AVR::ATTiny85::Timer8BitHighSpeed::TCCR> {
    static constexpr bool enable = true;
};
}

namespace AVR {

template<>
struct ATTiny85::Timer8Bit::PrescalerBits<0> {
        static constexpr auto values = prescalerValues10Bit<ATTiny85::Timer8Bit::TCCRB>;
};

template<>
struct ATTiny85::Timer8BitHighSpeed::Address<1> {
    static constexpr uint8_t value = 0x4B;
};
template<>
struct ATTiny85::Timer8BitHighSpeed::PrescalerBits<1> {
    static constexpr auto values = prescalerValues14Bit<ATTiny85::Timer8BitHighSpeed::TCCR>;
};

}
#pragma pack(pop)

