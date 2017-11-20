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

namespace TWI {

template<uint8_t N, const TWI::Address& Address, uint16_t Size, typename MCU = DefaultMcuType>
class Slave final : public IsrBaseHandler<AVR::ISR::Twi<0>> {
    Slave() = delete;
    
public:
    typedef MCU                         mcu_type;     
    typedef typename mcu_type::TWI      mcu_twi_type;     
    typedef typename mcu_type::TWI::TWS tws;     
    typedef typename mcu_type::TWI::TWC twc;     
    typedef typename MCU::TWI::template PrescalerRow<N> ps;
    static constexpr uint8_t number = N;
    
    static constexpr const auto mcu_twi = AVR::getBaseAddr<typename MCU::TWI, N>;

    static constexpr auto tw_status_mask = tws::tws7 | tws::tws6 | tws::tws5 | tws::tws4 | tws::tws3;
    static constexpr auto twcr_ack       = twc::twen | twc::twie | twc::twint | twc::twea;
    static constexpr auto twcr_nack      = twc::twen | twc::twie | twc::twint;
    static constexpr auto twcr_reset     = twc::twen | twc::twie | twc::twint | twc::twea | twc::twsto;
    
    static void init() {
        BusAddress<Write> busAddress{Address};
        *mcu_twi()->twar = busAddress.value();
        mcu_twi()->twcr.template clear<twc::twsta | twc::twsta>();
        mcu_twi()->twcr.template set<twc::twea | twc::twen | twc::twie>();
        index.setNaN();
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
                    isChanged() = true;
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
    static auto& isChanged() {
        static volatile bool changed = false;
        return changed;
    }
    static auto& registers() {
        return mRegisters;
    }

    inline static volatile std::array<std::byte, Size> mRegisters;
private:
    inline static volatile uint_NaN<uint8_t> index;
};

}
