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
    
    struct ATTiny84 final {
        ATTiny84() = delete;
        template<typename T>
        static constexpr bool is_atomic() {return false;}
        struct Timer8Bit {
            static constexpr const uint8_t count = 1;
            enum class TCCRA : uint8_t {
                coma0 = (1 << COM0A0),
                coma1 = (1 << COM0A1),
                comb0 = (1 << COM0B0),
                comb1 = (1 << COM0B1),
                wgm0 = (1 << WGM00),
                wgm1 = (1 << WGM01)
            };
            ControlRegister<Timer8Bit, TCCRA> tccra;
            volatile uint8_t unused1;
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
            volatile uint8_t unused2;
            volatile uint8_t unused3;
            DataRegister<Timer8Bit, ReadWrite> ocra;
            volatile uint8_t padding[0x3c - 0x36 - 1];
            DataRegister<Timer8Bit, ReadWrite> ocrb;
            template<int N> struct Address;
            template<int N> struct PrescalerBits;
        };
        struct Timer16Bit {
            static constexpr const uint8_t count = 1;
            volatile uint8_t tccrc;
            volatile uint8_t reserved;
            DataRegister<Timer16Bit, ReadWrite, uint16_t> icr;
            volatile uint8_t reserved2;
            volatile uint8_t reserved3;
            DataRegister<Timer16Bit, ReadWrite, uint16_t> ocrb;
            DataRegister<Timer16Bit, ReadWrite, uint16_t> ocra;
            DataRegister<Timer16Bit, ReadWrite, uint16_t> tcnt;
            enum class TCCRB : uint8_t {
                icnc = (1 << ICNC1),
                ices = (1 << ICES1),
                wgm3 = (1 << WGM13),
                wgm2 = (1 << WGM12),
                cs2 = (1 << CS12),
                cs1 = (1 << CS11),
                cs0 = (1 << CS10),
            };
            ControlRegister<Timer16Bit, TCCRB> tccrb;
            typedef uint16_t value_type;
            enum class TCCRA : uint8_t {
                coma0 = (1 << COM1A0),
                coma1 = (1 << COM1A1),
                comb0 = (1 << COM1B0),
                comb1 = (1 << COM1B1),
                wgm0 = (1 << WGM10),
                wgm1 = (1 << WGM11)
            };
            ControlRegister<Timer16Bit, TCCRA> tccra;
            
            template<int N> struct Address;
            template<int N> struct PrescalerBits;
        };
        
        struct USI {
            static constexpr const uint8_t count = 1;
            enum class USIC : uint8_t {
                sie = (1 << USISIE),
                oie = (1 << USIOIE),
                wm1 = (1 << USIWM1),
                wm0 = (1 << USIWM0),
                cs1 = (1 << USICS1),
                cs0 = (1 << USICS0),
                clk = (1 << USICLK),
                tc  = (1 << USITC)
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
            DataRegister<PortRegister, ReadWrite, std::byte> in;
            DataRegister<PortRegister, ReadWrite, std::byte> ddr;
            DataRegister<PortRegister, ReadWrite, std::byte> out;
            template<typename P> struct Address;
        };
        struct Timer8Interrupts {
            static constexpr const uint8_t count = 1;
            enum class Flags : uint8_t {
                ocfb = (1 << OCF0B),
                ocfa = (1 << OCF0A),
                tov  = (1 << TOV0)
            };
            ControlRegister<Timer8Interrupts, Flags> tifr;
            enum class Mask : uint8_t {
                ocieb = (1 << OCIE0B),
                ociea = (1 << OCIE0A),
                toie  = (1 << TOIE0)
            };
            ControlRegister<Timer8Interrupts, Mask> timsk;
            template<uint8_t N> struct Address;
        };
        struct Timer16Interrupts {
            static constexpr const uint8_t count = 1;
            enum class Flags : uint8_t {
                icf  = (1 << ICF1),
                ocfb = (1 << OCF1B),
                ocfa = (1 << OCF1A),
                tov  = (1 << TOV1)
            };
            ControlRegister<Timer16Interrupts, Flags> tifr;
            enum class Mask : uint8_t {
                icie  = (1 << ICIE1),
                ocieb = (1 << OCIE1B),
                ociea = (1 << OCIE1A),
                toie  = (1 << TOIE1)
            };
            ControlRegister<Timer16Interrupts, Mask> timsk;
            template<uint8_t N> struct Address;
        };
        struct Interrupt {
            enum class GIFlags : uint8_t {
                intf = (1 << INTF0),
                pcif0 = (1 << PCIF0),
                pcif1 = (1 << PCIF1)
            };
            ControlRegister<Interrupt, GIFlags> gifr;
            enum class GIMask : uint8_t {
                ie   = (1 << INT0),
                pcie0 = (1 << PCIE0),
                pcie1 = (1 << PCIE1),
            };
            ControlRegister<Interrupt, GIMask> gimsk;
            static constexpr uint8_t address = 0x5a;
        };
        struct PCInterrupts {
            static constexpr const uint8_t count = 2;
            DataRegister<PCInterrupts, ReadWrite, std::byte> pcmsk;
            template<int N> struct Address;
        };
        struct Status final {
            enum class Bits : uint8_t {
                globalIntEnable = (1 << 7), 
                bitCopy = (1 << 6) 
            };
            ControlRegister<Status, Bits> value;
            static constexpr uint8_t address = 0x5f;
        };
        struct GPIOR final {
            static constexpr const uint8_t count = 3;
            template<uint8_t N> struct Address; 
        };
    };
    template<>
    constexpr bool ATTiny84::is_atomic<uint8_t>() {return true;}
    
    template<>
    struct ATTiny84::PCInterrupts::Address<0> {
        static constexpr uint8_t value = 0x32;
    };
    template<>
    struct ATTiny84::PCInterrupts::Address<1> {
        static constexpr uint8_t value = 0x40;
    };
    
    template<>
    struct ATTiny84::USI::Address<0> {
        static constexpr uint8_t value = 0x2d;
    };
    
    template<>
    struct ATTiny84::PortRegister::Address<B> {
        static constexpr uint8_t value = 0x36;
    };
    template<>
    struct ATTiny84::PortRegister::Address<A> {
        static constexpr uint8_t value = 0x39;
    };
    
    template<>
    struct ATTiny84::Timer8Bit::Address<0> {
        static constexpr uint8_t value = 0x50;
    };
    template<>
    struct ATTiny84::Timer16Bit::Address<1> {
        static constexpr uint8_t value = 0x42;
    };
}

namespace std {
    template<>
    struct enable_bitmask_operators<AVR::ATTiny84::Status::Bits> {
        static constexpr bool enable = true;
    };
    template<>
    struct enable_bitmask_operators<AVR::ATTiny84::USI::USIC> {
        static constexpr bool enable = true;
    };
    template<>
    struct enable_bitmask_operators<AVR::ATTiny84::USI::USIS> {
        static constexpr bool enable = true;
    };
    template<>
    struct enable_bitmask_operators<AVR::ATTiny84::Timer8Bit::TCCRA> {
        static constexpr bool enable = true;
    };
    template<>
    struct enable_bitmask_operators<AVR::ATTiny84::Timer8Bit::TCCRB> {
        static constexpr bool enable = true;
    };
    template<>
    struct enable_bitmask_operators<AVR::ATTiny84::Timer16Bit::TCCRA> {
        static constexpr bool enable = true;
    };
    template<>
    struct enable_bitmask_operators<AVR::ATTiny84::Timer16Bit::TCCRB> {
        static constexpr bool enable = true;
    };
    template<>
    struct enable_bitmask_operators<AVR::ATTiny84::Timer16Interrupts::Flags> {
        static constexpr bool enable = true;
    };
    template<>
    struct enable_bitmask_operators<AVR::ATTiny84::Timer16Interrupts::Mask> {
        static constexpr bool enable = true;
    };
}

namespace AVR {
    
    template<>
    struct ATTiny84::Timer8Interrupts::Address<0> {
        static constexpr uint8_t value = 0x58;
    };
    template<>
    struct ATTiny84::Timer16Interrupts::Address<1> {
        static constexpr uint8_t value = 0x2b;
    };
    
    template<>
    struct ATTiny84::Timer8Bit::PrescalerBits<0> {
        static constexpr auto values = AVR::prescalerValues10Bit<ATTiny84::Timer8Bit::TCCRB>;
    };
    template<>
    struct ATTiny84::Timer16Bit::PrescalerBits<1> {
        static constexpr auto values = AVR::prescalerValues10Bit<ATTiny84::Timer16Bit::TCCRB>;
    };
    
}
#pragma pack(pop)

