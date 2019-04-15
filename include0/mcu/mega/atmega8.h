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

#pragma pack(push)
#pragma pack(1)

namespace AVR {
    struct ATMega8 final {
        template<typename T>
        static constexpr bool is_atomic() {return false;}
        
        ATMega8() = delete;
        struct Ram {
            inline static constexpr uintptr_t begin = RAMSTART;  
            inline static constexpr uintptr_t end   = RAMEND;  
        };
        struct Usart {
            static constexpr const uint8_t count = 1;
            DataRegister<Usart, ReadWrite, uint8_t> ubbrl;
            enum class SRB : uint8_t {
                rxcie = (1 << RXCIE),
                txcie = (1 << TXCIE),
                udrie = (1 << UDRIE),
                rxen = (1 << RXEN),
                txen = (1 << TXEN),
                ucsz2 = (1 << UCSZ2),
                rxb8 = (1 << RXB8),
                txb8 = (1 << TXB8)
            };
            ControlRegister<Usart, SRB> ucsrb;
            enum class SRA : uint8_t {
                rxc = (1 << RXC),
                txc = (1 << TXC),
                udre = (1 << UDRE),
                fe = (1 << FE),
                dor = (1 << DOR),
                upe = (1 << PE),
                u2x = (1 << U2X),
                mpcm = (1 << MPCM)
            };
            ControlRegister<Usart, SRA> ucsra;
            DataRegister<Usart, ReadWrite, std::byte> udr;
            enum class SRC : uint8_t {
                umsel = (1 << UMSEL),
                ursel = (1 << URSEL),
                upm1 = (1 << UPM1),
                upm0 = (1 << UPM0),
                usbs = (1 << USBS),
                ucsz0 = (1 << UCSZ0),
                ucsz1 = (1 << UCSZ1),
                ucpol = (1 << UCPOL)
            };
            volatile uint8_t padding[0x20 - 0x0c - 1];
            union {
                DataRegister<Usart, ReadWrite, uint8_t> ubbrh;
                ControlRegister<Usart, SRC> ucsrc;
            };
            
            template<int N> struct Address;
        };
        struct Timer8BitSimple {
            static constexpr const uint8_t count = 1;
            typedef uint8_t value_type;
            DataRegister<Timer8BitSimple, ReadWrite> tcnt;
            enum class TCCR : uint8_t {
                cs2 = (1 << CS02),
                cs1 = (1 << CS01),
                cs0 = (1 << CS00),
            };
            ControlRegister<Timer8BitSimple, TCCR> tccr;
            template<int N> struct Address;
            template<int N> struct PrescalerBits;
        };
        struct Timer8BitSimple2 {
            static constexpr const uint8_t count = 1;
            typedef uint8_t value_type;
            volatile uint8_t ocr;
            DataRegister<Timer8BitSimple2, ReadWrite> tcnt;
            enum class TCCR : uint8_t {
                foc = (1 << FOC2),
                wgm0 = (1 << WGM20),
                wgm1 = (1 << WGM21),
                cs2 = (1 << CS22),
                cs1 = (1 << CS21),
                cs0 = (1 << CS20),
            };
            
            ControlRegister<Timer8BitSimple2, TCCR> tccr;
            template<int N> struct Address;
            template<int N> struct PrescalerBits;
        };
        
        struct Timer16Bit {
            static constexpr const uint8_t count = 1;
            DataRegister<Timer16Bit, ReadWrite, uint16_t> icr;
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
            enum class TCCRA : uint8_t {
                coma0 = (1 << COM1A0),
                coma1 = (1 << COM1A1),
                comb0 = (1 << COM1B0),
                comb1 = (1 << COM1B1),
                foca = (1 << FOC1A),
                focb = (1 << FOC1B),
                wgm0 = (1 << WGM10),
                wgm1 = (1 << WGM11)
            };
            ControlRegister<Timer16Bit, TCCRA> tccra;
            template<int N> struct Address;
            template<int N> struct PrescalerBits;
            template<uint8_t N> struct Flags; 
        };
        struct PortRegister {
            DataRegister<PortRegister, ReadWrite, std::byte> in;
            DataRegister<PortRegister, ReadWrite, std::byte> ddr;
            DataRegister<PortRegister, ReadWrite, std::byte> out;
            template<typename P> struct Address;
        };
        struct Interrupt {
            volatile uint8_t tifr;
            volatile uint8_t timsk;
            volatile uint8_t gifr;
            volatile uint8_t gicr;
            static constexpr uint8_t address = 0x58;
        };
        class TimerInterrupts {
        public:
            enum class Flags : uint8_t {
                ocf2  = (1 << OCF2),
                tov2  = (1 << TOV2),
                icf1  = (1 << ICF1),
                ocf1a = (1 << OCF1A),
                ocf1b = (1 << OCF1B),
                tov1  = (1 << TOV1),
                tov0  = (1 << TOV0)
            };
            ControlRegister<TimerInterrupts, Flags> tifr;
            enum class Mask : uint8_t {
                ocie2  = (1 << OCIE2),
                toie2  = (1 << TOIE2),
                ticie1 = (1 << TICIE1),
                ocie1a = (1 << OCIE1A),
                ocie1b = (1 << OCIE1B),
                toie1  = (1 << TOIE1),
                toie0  = (1 << TOIE0)
            };
            ControlRegister<TimerInterrupts, Mask> timsk;
            static constexpr uint8_t address = 0x58;
        };
        struct Status final {
            enum class Bits : uint8_t {
                globalIntEnable = (1 << 7), 
                bitCopy = (1 << 6) 
            };
            ControlRegister<Status, Bits> value;
            static constexpr uint8_t address = 0x5f;
        };
        
    };
    template<>
    constexpr bool ATMega8::is_atomic<uint8_t>() {return true;}
}

namespace std {
    template<>
    struct enable_bitmask_operators<AVR::ATMega8::Usart::SRA>  : std::true_type {};
    template<>
    struct enable_bitmask_operators<AVR::ATMega8::Usart::SRB>  : std::true_type {};
    template<>
    struct enable_bitmask_operators<AVR::ATMega8::Usart::SRC>  : std::true_type {};
    template<>
    struct enable_bitmask_operators<AVR::ATMega8::Status::Bits>  : std::true_type {};
    template<>
    struct enable_bitmask_operators<AVR::ATMega8::Timer8BitSimple::TCCR>  : std::true_type {};
    template<>
    struct enable_bitmask_operators<AVR::ATMega8::Timer8BitSimple2::TCCR>  : std::true_type {};
    template<>
    struct enable_bitmask_operators<AVR::ATMega8::Timer16Bit::TCCRA>  : std::true_type {};
    template<>
    struct enable_bitmask_operators<AVR::ATMega8::Timer16Bit::TCCRB>  : std::true_type {};
}

namespace AVR {
    template<>
    struct ATMega8::PortRegister::Address<B> {
        inline static constexpr uintptr_t value = 0x36;
    };
    template<>
    struct ATMega8::PortRegister::Address<C> {
        inline static constexpr uintptr_t value = 0x33;
    };
    template<>
    struct ATMega8::PortRegister::Address<D> {
        inline static constexpr uintptr_t value = 0x30;
    };
    template<>
    template<>
    struct ATMega8::Timer8BitSimple::Address<0> {
        inline static constexpr uintptr_t value = 0x52;
    };
    template<>
    struct ATMega8::Usart::Address<0> {
        inline static constexpr uintptr_t value = 0x29;
    };
    template<>
    struct ATMega8::Timer8BitSimple::PrescalerBits<0> {
        inline static constexpr auto values = prescalerValues10Bit<ATMega8::Timer8BitSimple::TCCR>;
    };
    template<>
    struct ATMega8::Timer8BitSimple2::Address<2> {
        inline static constexpr uintptr_t value = 0x45;
    };
    template<>
    struct ATMega8::Timer8BitSimple2::PrescalerBits<2> {
        static constexpr auto values = prescalerValues10BitExtended<ATMega8::Timer8BitSimple2::TCCR>;
    };
    template<>
    struct ATMega8::Timer16Bit::Address<1> {
        inline static constexpr uintptr_t value = 0x46;
    };
    template<>
    struct ATMega8::Timer16Bit::PrescalerBits<1> {
        static constexpr auto values = prescalerValues10Bit<ATMega8::Timer16Bit::TCCRB>;
    };
    
}

#pragma pack(pop)
