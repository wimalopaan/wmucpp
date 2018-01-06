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

#include <array>
#include "compat/twi.h"
#include "mcu/avr8.h"
#include "mcu/avr/twiaddress.h"
#include "mcu/avr/isr.h"
#include "util/types.h"
#include "util/concepts.h"

namespace TWI {
    template<typename T, bool Enable>
    struct Base;
    template<typename T>
    struct Base<T, false> {
        inline static constexpr uint8_t number_of_flags = 1;
    };
    template<typename T>
    struct Base<T, true> {
        inline static volatile bool mChanged = false;
    };
    
    template<uint8_t N, const TWI::Address& Address, uint16_t Size, typename useInt = MCU::UseInterrupts<true>, 
             typename FlagRegister = void,
             typename MCU = DefaultMcuType>
    class Slave final : public IsrBaseHandler<AVR::ISR::Twi<0>>, // fixme: disable if no Interrupts
            public Base<Slave<N, Address, Size, useInt, FlagRegister, MCU>, std::is_same<FlagRegister, void>::value> {
        Slave() = delete;
        
        inline static constexpr bool useBase = std::is_same<FlagRegister, void>::value;
        
        typedef Base<Slave<N, Address, Size, useInt, FlagRegister, MCU>, std::is_same<FlagRegister, void>::value> baseType;
        
    public:
        typedef MCU                         mcu_type;     
        typedef typename mcu_type::TWI      mcu_twi_type;     
        typedef typename mcu_type::TWI::TWS tws;     
        typedef typename mcu_type::TWI::TWC twc;     
        typedef typename MCU::TWI::template PrescalerRow<N> ps;
        static constexpr uint8_t number = N;
        
        static constexpr const auto mcu_twi = AVR::getBaseAddr<typename MCU::TWI, N>;
        
        static constexpr auto tw_status_mask = tws::tws7 | tws::tws6 | tws::tws5 | tws::tws4 | tws::tws3;
        static constexpr auto twcr_ack = []{
            if constexpr(useInt::value) {
                return twc::twen | twc::twie | twc::twint | twc::twea;
            }
            else {
                return twc::twen | twc::twint | twc::twea;
            }
        }();
        static constexpr auto twcr_nack = []{
            if constexpr(useInt::value) {
                return twc::twen | twc::twie | twc::twint;
            }
            else {
                return twc::twen | twc::twint;
            }
        }();
        static constexpr auto twcr_reset = []{
            if constexpr(useInt::value) {
                return twc::twen | twc::twie | twc::twint | twc::twea | twc::twsto;
            }
            else {
                return twc::twen | twc::twint | twc::twea | twc::twsto;
            }
        }();
        static void init() {
            BusAddress<Write> busAddress{Address};
            *mcu_twi()->twar = busAddress.value();
            mcu_twi()->twcr.template clear<twc::twsta | twc::twsta, DisbaleInterrupt<NoDisableEnable>>();
            if constexpr(useInt::value) {
                mcu_twi()->twcr.template set<twc::twea | twc::twen | twc::twie>();
            }
            else {
                mcu_twi()->twcr.template set<twc::twea | twc::twen>();
            }
            index.setNaN(); // todo: index = NaN;
        }
        static void isr() {
            auto twst = mcu_twi()->twsr.template get<tw_status_mask>();
            switch(twst) {
            case tws::twSrSlaAck:
                mcu_twi()->twcr.template set<twcr_ack>();
                index.setNaN();
                break;
            case tws::twSrDataAck:
            {
                std::byte data = *mcu_twi()->twdr;
                if (!index) {
                    if (std::to_integer<uint8_t>(data) < Size) {
                        index = std::to_integer<uint8_t>(data);
                    }
                    else {
                        index = 0;
                    }
                    mcu_twi()->twcr.template set<twcr_ack>();
                }
                else {
                    if (*index < Size) {
                        mRegisters[*index] = data;
                        if constexpr(useBase) {
                            baseType::mChanged = true;
                        }
                        else {
                            FlagRegister::set();
                        }
                        ++index;
                    }
                    mcu_twi()->twcr.template set<twcr_ack>();
                }
            }
                break;
            case tws::twStSlaAck:
            case tws::twStDataAck:
                if (!index) {
                    index = 0;
                }
                if (*index < Size) {
                    *mcu_twi()->twdr = mRegisters[*index];
                    ++index;
                }
                else {
                    *mcu_twi()->twdr = std::byte{0};
                }
                mcu_twi()->twcr.template set<twcr_ack>();
                break;
            case tws::twSrStop:
                mcu_twi()->twcr.template set<twcr_ack>();
                break;
            case tws::twStDataNack:
            case tws::twSrDataNack:
            case tws::twStLastData:
            default:
                mcu_twi()->twcr.template set<twcr_reset>();
                break;
            }
        }
        
        template<::Util::Callable Callable> 
        static void whenReady(const Callable& f) {
            if (mcu_twi()->twcr.template isSet<twc::twint>()) {
                isr();
                f();
            }
        }
        static bool isChanged() {
            if constexpr(useBase) {
                return baseType::mChanged;
            }   
            else {
                return FlagRegister::isSet();
            }
        }
        static void changed(bool v) {
            if constexpr(useBase) {
                baseType::mChanged = v;
            }   
            else {
                if (v) {
                    FlagRegister::set();
                }
                else {
                    FlagRegister::reset();
                }
            }
        }
        
        static auto& registers() {
            return mRegisters;
        }
        
    private:
        inline static volatile std::array<std::byte, Size> mRegisters;
//        inline static volatile bool mChanged = false;
        inline static volatile uint_NaN<uint8_t> index;
    };
    
}
