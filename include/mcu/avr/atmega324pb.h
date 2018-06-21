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

#include <stdint.h>

#include "avr8defs.h"

#pragma pack(push)
#pragma pack(1)

namespace AVR {
    
    struct ATMega324PB final
    {
        template<typename T>
        static constexpr bool is_atomic() {return false;}
        
        ATMega324PB() = delete;
        
        struct Ram {
            inline static constexpr uintptr_t begin = RAMSTART;  
            inline static constexpr uintptr_t end   = RAMEND;  
        };
        
        struct Usart {
            static constexpr const uint8_t count = 3;
            enum class SRA : uint8_t {
                rxc = (1 << RXC0),
                txc = (1 << TXC0),
                udre = (1 << UDRE0),
                fe = (1 << FE0),
                dor = (1 << DOR0),
                upe = (1 << UPE0),
                u2x = (1 << U2X0),
                mpcm = (1 << MPCM0)
            };
            ControlRegister<Usart, SRA> ucsra;
            enum class SRB : uint8_t {
                rxcie = (1 << RXCIE0),
                txcie = (1 << TXCIE0),
                udrie = (1 << UDRIE0),
                rxen = (1 << RXEN0),
                txen = (1 << TXEN0),
                ucsz2 = (1 << UCSZ02),
                rxb8 = (1 << RXB80),
                txb8 = (1 << TXB80)
            };
            ControlRegister<Usart, SRB> ucsrb;
            enum class SRC : uint8_t {
                umsel0 = (1 << UMSEL00),
                umsel1 = (1 << UMSEL01),
                upm1 = (1 << UPM01),
                upm0 = (1 << UPM00),
                usbs = (1 << USBS0),
                ucsz0 = (1 << UCSZ00),
                ucsz1 = (1 << UCSZ01),
                ucpol = (1 << UCPOL0)
            };
            ControlRegister<Usart, SRC> ucsrc;
            volatile uint8_t reserved1;
            DataRegister<Usart, ReadWrite, uint16_t> ubbr;
            DataRegister<Usart, ReadWrite, std::byte> udr;
            volatile uint8_t reserved2;
            template<int N> struct Address;
        };
        struct TWI {
            static constexpr const uint8_t count = 1;
            DataRegister<TWI, ReadWrite> twbr;
            enum class TWS : uint8_t {
                tws7 = (1 << TWS07),
                tws6 = (1 << TWS06),
                tws5 = (1 << TWS05),
                tws4 = (1 << TWS04),
                tws3 = (1 << TWS03),
                twps1 = (1 << TWPS01),
                twps0 = (1 << TWPS00),
                twStart = TW_START,
                twRepStart = TW_REP_START,
                twMtSlaAck = TW_MT_SLA_ACK,
                twMtSlaNack = TW_MT_SLA_NACK,
                twMtDataAck = TW_MT_DATA_ACK,
                twMtDataNack = TW_MT_DATA_NACK,
                twMrSlaAck = TW_MR_SLA_ACK,
                twMrSlaNack = TW_MR_SLA_NACK,
                twSrSlaAck = TW_SR_SLA_ACK,
                twSrDataAck = TW_SR_DATA_ACK,
                twStSlaAck = TW_ST_SLA_ACK,
                twStDataAck = TW_ST_DATA_ACK,
                twSrStop = TW_SR_STOP,
                twStDataNack = TW_ST_DATA_NACK,
                twSrDataNack = TW_SR_DATA_NACK,
                twStLastData = TW_ST_LAST_DATA
            };
            ControlRegister<TWI, TWS> twsr;
            DataRegister<TWI, ReadWrite, std::byte> twar;
            DataRegister<TWI, ReadWrite, std::byte> twdr;
            enum class TWC : uint8_t {
                twint = (1 << TWINT0),
                twea = (1 << TWEA0),
                twsta = (1 << TWSTA0),
                twsto = (1 << TWSTO0),
                twwc = (1 << TWWC0),
                twen = (1 << TWEN0),
                twie = (1 << TWIE0)
            };
            ControlRegister<TWI, TWC> twcr;
            DataRegister<TWI, ReadWrite> twamr;
            template<int N> struct Address;
            template<int N> struct PrescalerRow;
        };
        struct Timer8Bit {
            static constexpr const uint8_t count = 2;
            typedef uint8_t value_type;
            enum class TCCRA : uint8_t {
                coma0 = (1 << COM0A0),
                coma1 = (1 << COM0A1),
                comb0 = (1 << COM0B0),
                comb1 = (1 << COM0B1),
                wgm0 = (1 << WGM00),
                wgm1 = (1 << WGM01)
            };
            ControlRegister<Timer8Bit, TCCRA> tccra;
            enum class TCCRB : uint8_t {
                foca = (1 << FOC0A),
                focb = (1 << FOC0B),
                wgm2 = (1 << WGM02),
                cs2 = (1 << CS02),
                cs1 = (1 << CS01),
                cs0 = (1 << CS00),
            };
            
            ControlRegister<Timer8Bit, TCCRB> tccrb;
            DataRegister<Timer8Bit, ReadWrite> tcnt;
            DataRegister<Timer8Bit, ReadWrite> ocra;
            DataRegister<Timer8Bit, ReadWrite> ocrb;
            template<int N> struct Address;
            template<int N> struct PrescalerBits;
        };
        
        struct Timer16Bit {
            static constexpr const uint8_t count = 3;
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
            volatile uint8_t tccrc;
            volatile uint8_t reserved;
            DataRegister<Timer16Bit, ReadWrite, uint16_t> tcnt;
            DataRegister<Timer16Bit, ReadWrite, uint16_t> icr;
            DataRegister<Timer16Bit, ReadWrite, uint16_t> ocra;
            DataRegister<Timer16Bit, ReadWrite, uint16_t> ocrb;
            template<int N> struct Address;
            template<int N> struct PrescalerBits;
        };
        struct PCInterrupts {
            static constexpr const uint8_t count = 3;
            DataRegister<PCInterrupts, ReadWrite, std::byte> pcmsk;
            template<int N> struct Address;
        };
        struct Timer8Interrupts {
            static constexpr const uint8_t count = 2;
            enum class Flags : uint8_t {
                ocfb = (1 << OCF0B),
                ocfa = (1 << OCF0A),
                tov  = (1 << TOV0)
            };
            FlagRegister<Timer8Interrupts, Flags> tifr;
            volatile uint8_t padding[0x6E - 0x35 - 1];
            enum class Mask : uint8_t {
                ocieb = (1 << OCIE0B),
                ociea = (1 << OCIE0A),
                toie  = (1 << TOIE0)
            };
            ControlRegister<Timer8Interrupts, Mask> timsk;
            template<uint8_t N> struct Address;
        };
        struct Timer16Interrupts {
            static constexpr const uint8_t count = 3;
            enum class Flags : uint8_t {
                icf  = (1 << ICF1),
                ocfb = (1 << OCF1B),
                ocfa = (1 << OCF1A),
                tov  = (1 << TOV1)
            };
            FlagRegister<Timer16Interrupts, Flags> tifr;
            volatile uint8_t padding[0x6E - 0x35 - 1];
            enum class Mask : uint8_t {
                icie  = (1 << ICIE1),
                ocieb = (1 << OCIE1B),
                ociea = (1 << OCIE1A),
                toie  = (1 << TOIE1)
            };
            ControlRegister<Timer16Interrupts, Mask> timsk;
            template<uint8_t N> struct Address;
        };
        struct Spi {
            static constexpr const uint8_t count = 1;
            volatile uint8_t spcr;
            volatile uint8_t spsr;
            volatile uint8_t spdr;
            template<int N> struct Address;
        };
        struct Adc {
            static constexpr const uint8_t count = 1;
            struct AdcRegister {
                DataRegister<Adc, ReadOnly, uint8_t> adcl;
                DataRegister<Adc, ReadOnly, uint8_t> adch;
            };
            union {
                AdcRegister reg;
                DataRegister<Adc, ReadOnly, uint16_t> adc;
            };
            enum class SRA : uint8_t {
                aden = (1 << ADEN),
                adsc = (1 << ADSC),
                adate = (1 << ADATE),
                adif = (1 << ADIF),
                adie = (1 << ADIE),
                adps2 = (1 << ADPS2),
                adps1 = (1 << ADPS1),
                adps0 = (1 << ADPS0)
            };
            ControlRegister<Adc, SRA> adcsra;
            enum class SRB : uint8_t {
                acme = (1 << ACME),
                adts2 = (1 << ADTS2),
                adts1 = (1 << ADTS1),
                adts0 = (1 << ADTS0)
            };
            ControlRegister<Adc, SRB> adcsrb;
            enum class MUX : uint8_t {
                refs1 = (1 << REFS1),
                refs0 = (1 << REFS0),
                adlar = (1 << ADLAR),
                mux4  = (1 << MUX4),            
                mux3  = (1 << MUX3),            
                mux2  = (1 << MUX2),            
                mux1  = (1 << MUX1),            
                mux0  = (1 << MUX0) 
            };
            ControlRegister<Adc, MUX> admux;
            volatile uint8_t reserved;
            DataRegister<Adc, UnUsed> didr0;
            DataRegister<Adc, UnUsed> didr1;
            template<int N> struct Address;
            template<int N> struct Parameter;
        };
        struct PortRegister {
            DataRegister<PortRegister, ReadWrite, std::byte> in;
            DataRegister<PortRegister, ReadWrite, std::byte> ddr;
            DataRegister<PortRegister, ReadWrite, std::byte> out;
            template<typename P> struct Address;
        };
        struct Interrupt {
            enum class PCFlags : uint8_t {
                if2 = (1 << PCIF2),
                if1 = (1 << PCIF1),
                if0 = (1 << PCIF0)
            };
            FlagRegister<Interrupt, PCFlags> pcifr;
            enum class EIFlags : uint8_t {
                int1 = (1 << INTF1),
                int0 = (1 << INTF0)
            };
            FlagRegister<Interrupt, EIFlags> eifr;
            enum class EIMask : uint8_t {
                int1 = (1 << INT1),
                int0 = (1 << INT0)
            };
            ControlRegister<Interrupt, EIMask> eimsk;
            volatile uint8_t padding[0x68 - 0x3b - 1 - 2];
            enum class PCMask : uint8_t {
                ie2 = (1 << PCIE2),
                ie1 = (1 << PCIE1),
                ie0 = (1 << PCIE0)
            };
            ControlRegister<Interrupt, PCMask> pcicr;
            enum class EIControl : uint8_t {
                isc11 = (1 << ISC11),
                isc10 = (1 << ISC10),
                isc01 = (1 << ISC01),
                isc00 = (1 << ISC00)
            };
            ControlRegister<Interrupt, EIControl> eicra;
            static constexpr uint8_t address = 0x3b;
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
    constexpr bool ATMega324PB::is_atomic<uint8_t>() {return true;}
}

namespace std {
    template<>
    struct enable_bitmask_operators<AVR::ATMega324PB::Status::Bits> {
        static constexpr bool enable = true;
    };
    template<>
    struct enable_bitmask_operators<AVR::ATMega324PB::Timer8Interrupts::Flags> {
        static constexpr bool enable = true;
    };
    template<>
    struct enable_bitmask_operators<AVR::ATMega324PB::Timer8Interrupts::Mask> {
        static constexpr bool enable = true;
    };
    template<>
    struct enable_bitmask_operators<AVR::ATMega324PB::Timer16Interrupts::Flags> {
        static constexpr bool enable = true;
    };
    template<>
    struct enable_bitmask_operators<AVR::ATMega324PB::Timer16Interrupts::Mask> {
        static constexpr bool enable = true;
    };
    template<>
    struct enable_bitmask_operators<AVR::ATMega324PB::Usart::SRA> {
        static constexpr bool enable = true;
    };
    template<>
    struct enable_bitmask_operators<AVR::ATMega324PB::Usart::SRB> {
        static constexpr bool enable = true;
    };
    template<>
    struct enable_bitmask_operators<AVR::ATMega324PB::Usart::SRC> {
        static constexpr bool enable = true;
    };
    template<>
    struct enable_bitmask_operators<AVR::ATMega324PB::Timer8Bit::TCCRA> {
        static constexpr bool enable = true;
    };
    template<>
    struct enable_bitmask_operators<AVR::ATMega324PB::Timer8Bit::TCCRB> {
        static constexpr bool enable = true;
    };
    template<>
    struct enable_bitmask_operators<AVR::ATMega324PB::Timer16Bit::TCCRA> {
        static constexpr bool enable = true;
    };
    template<>
    struct enable_bitmask_operators<AVR::ATMega324PB::Timer16Bit::TCCRB> {
        static constexpr bool enable = true;
    };
    template<>
    struct enable_bitmask_operators<AVR::ATMega324PB::TWI::TWS> {
        static constexpr bool enable = true;
    };
    template<>
    struct enable_bitmask_operators<AVR::ATMega324PB::TWI::TWC> {
        static constexpr bool enable = true;
    };
    template<>
    struct enable_bitmask_operators<AVR::ATMega324PB::Adc::SRA> {
        static constexpr bool enable = true;
    };
    template<>
    struct enable_bitmask_operators<AVR::ATMega324PB::Adc::SRB> {
        static constexpr bool enable = true;
    };
    template<>
    struct enable_bitmask_operators<AVR::ATMega324PB::Adc::MUX> {
        static constexpr bool enable = true;
    };
}

namespace AVR {
    template<>
    struct ATMega324PB::GPIOR::Address<0> {
        inline static constexpr uintptr_t value = 0x3e;
    };
    template<>
    struct ATMega324PB::GPIOR::Address<1> {
        inline static constexpr uintptr_t value = 0x4a;
    };
    template<>
    struct ATMega324PB::GPIOR::Address<2> {
        inline static constexpr uintptr_t value = 0x4b;
    };
    
    template<>
    struct ATMega324PB::PortRegister::Address<A> {
        static constexpr uint8_t value = 0x20;
    };
    template<>
    struct ATMega324PB::PortRegister::Address<B> {
        static constexpr uint8_t value = 0x23;
    };
    template<>
    struct ATMega324PB::PortRegister::Address<C> {
        static constexpr uint8_t value = 0x26;
    };
    template<>
    struct ATMega324PB::PortRegister::Address<D> {
        static constexpr uint8_t value = 0x29;
    };
    template<>
    struct ATMega324PB::PortRegister::Address<E> {
        static constexpr uint8_t value = 0x2c;
    };
    
    template<>
    struct ATMega324PB::PCInterrupts::Address<0> {
        static constexpr uint8_t value = 0x6b;
    };
    template<>
    struct ATMega324PB::PCInterrupts::Address<1> {
        static constexpr uint8_t value = 0x6c;
    };
    template<>
    struct ATMega324PB::PCInterrupts::Address<2> {
        static constexpr uint8_t value = 0x6d;
    };
    
    template<>
    struct ATMega324PB::TWI::Address<0> {
        static constexpr uint8_t value = 0xb8;
    };
    template<>
    struct ATMega324PB::TWI::Address<1> {
        static constexpr uint8_t value = 0xd8;
    };
    template<>
    struct ATMega324PB::Spi::Address<0> {
        static constexpr uint8_t value = 0x4c;
    };
    template<>
    struct ATMega324PB::Spi::Address<1> {
        static constexpr uint8_t value = 0xac;
    };
    template<>
    struct ATMega324PB::Usart::Address<0> {
        static constexpr uint8_t value = 0xc0;
    };
    template<>
    struct ATMega324PB::Usart::Address<1> {
        static constexpr uint8_t value = 0xc8;
    };
    template<>
    struct ATMega324PB::Usart::Address<2> {
        static constexpr uint8_t value = 0xd0;
    };
    
    template<>
    struct ATMega324PB::Timer8Interrupts::Address<0> {
        static constexpr uint8_t value = 0x35;
    };
    template<>
    struct ATMega324PB::Timer16Interrupts::Address<1> {
        static constexpr uint8_t value = 0x36;
    };
    template<>
    struct ATMega324PB::Timer8Interrupts::Address<2> {
        static constexpr uint8_t value = 0x37;
    };
    template<>
    struct ATMega324PB::Timer16Interrupts::Address<3> {
        static constexpr uint8_t value = 0x38;
    };
    template<>
    struct ATMega324PB::Timer16Interrupts::Address<4> {
        static constexpr uint8_t value = 0x39;
    };
    
    //Timer0
    template<>
    struct ATMega324PB::Timer8Bit::Address<0> {
        static constexpr uint8_t value = 0x44;
    };
    
    template<>
    struct ATMega324PB::Timer8Bit::PrescalerBits<0> {
        static constexpr auto values = AVR::prescalerValues10Bit<ATMega324PB::Timer8Bit::TCCRB>;
    };
    
    // Timer2
    template<>
    struct ATMega324PB::Timer8Bit::Address<2> {
        static constexpr uint8_t value = 0xb0;
    };
    template<>
    struct ATMega324PB::Timer8Bit::PrescalerBits<2> {
        static constexpr auto values = AVR::prescalerValues10BitExtended<ATMega324PB::Timer8Bit::TCCRB>;
    };
    
    // Timer 1
    template<>
    struct ATMega324PB::Timer16Bit::Address<1> {
        static constexpr uint8_t value = 0x80;
    };
    
    template<>
    struct ATMega324PB::Timer16Bit::PrescalerBits<1> {
        static constexpr auto values = AVR::prescalerValues10Bit<ATMega324PB::Timer16Bit::TCCRB>;
    };
    
    // Timer 3
    template<>
    struct ATMega324PB::Timer16Bit::Address<3> {
        static constexpr uint8_t value = 0x90;
    };
    
    template<>
    struct ATMega324PB::Timer16Bit::PrescalerBits<3> {
        static constexpr auto values = AVR::prescalerValues10Bit<ATMega324PB::Timer16Bit::TCCRB>;
    };
    
    // Timer 4
    template<>
    struct ATMega324PB::Timer16Bit::Address<4> {
        static constexpr uint8_t value = 0xA0;
    };
    
    template<>
    struct ATMega324PB::Timer16Bit::PrescalerBits<4> {
        static constexpr auto values = AVR::prescalerValues10Bit<ATMega324PB::Timer16Bit::TCCRB>;
    };
    template<>
    struct ATMega324PB::Adc::Address<0> {
        static constexpr uint8_t value = 0x78;
    };
    template<>
    struct ATMega324PB::Adc::Parameter<0> {
        static constexpr auto channelMasks = std::make_array(MUX{0}, 
                                                             MUX::mux0,
                                                             MUX::mux1,
                                                             MUX::mux1 | MUX::mux0,
                                                             MUX::mux2,
                                                             MUX::mux2 | MUX::mux0,
                                                             MUX::mux2 | MUX::mux1,
                                                             MUX::mux2 | MUX::mux1 | MUX::mux0,
                                                             MUX::mux3
                                                             );
//        static constexpr double VRef = 1.1;
    };
    
    template<>
    struct ATMega324PB::TWI::PrescalerRow<0> {
        static constexpr auto values = twiPrescalerBit<ATMega324PB::TWI::TWS>;
    };
    template<>
    struct ATMega324PB::TWI::PrescalerRow<1> {
        static constexpr auto values = twiPrescalerBit<ATMega324PB::TWI::TWS>;
    };
    
}
#pragma pack(pop)
