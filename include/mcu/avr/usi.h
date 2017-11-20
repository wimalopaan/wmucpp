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
#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/isr.h"
#include "std/type_traits"

namespace AVR {

template<typename MCU = DefaultMcuType>
struct I2C {
    typedef typename MCU::USI::USIC uc;
    typedef typename MCU::USI::USIS us;
    static constexpr auto usicr      = uc::sie     | uc::wm1     | uc::cs1;
    static constexpr auto usisr      = us::sif     | us::oif     | us::pf     | us::dc;
    template<typename SDAPin, typename SCLPin>
    static void init() {
        SCLPin::template dir<Output>();
        SDAPin::template dir<Output>();
        SCLPin::high();
        SDAPin::high();
        
        SDAPin::template dir<Input>();
        SDAPin::pullup();
    }
};
template<typename MCU = DefaultMcuType>
struct SPI {
    typedef typename MCU::USI::USIC uc;
    typedef typename MCU::USI::USIS us;
    static constexpr auto      usicr = uc::wm0     | uc::cs1     | uc::oie;
    static constexpr auto      usisr = us::oif;
    template<typename SDAPin, typename SCLPin>
    static void init() {
        SCLPin::template dir<Input>();
        SCLPin::pullup();
        SDAPin::template dir<Input>();
        SDAPin::pullup();
    }
};

template<uint8_t N, typename MCU = DefaultMcuType> struct UsiPort;

template<>
struct UsiPort<0, AVR::ATTiny84> {
    using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
    typedef AVR::Pin<PortA, 4> clock_pin;
    typedef AVR::Pin<PortA, 5> miso_pin;
    typedef AVR::Pin<PortA, 6> mosi_pin;
};
template<>
struct UsiPort<0, AVR::ATTiny85> {
    using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
    typedef AVR::Pin<PortB, 2> clock_pin;
    typedef AVR::Pin<PortB, 1> miso_pin;
    typedef AVR::Pin<PortB, 0> mosi_pin;
};
template<>
struct UsiPort<0, AVR::ATTiny25> {
    using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
    typedef AVR::Pin<PortB, 2> clock_pin;
    typedef AVR::Pin<PortB, 1> miso_pin;
    typedef AVR::Pin<PortB, 0> mosi_pin;
};

template<uint8_t N, typename SSPin = void, typename Inserter = void, typename MCU = DefaultMcuType>
class Usi final : public IsrBaseHandler<AVR::ISR::Usi<0>::Overflow> {
    Usi() = delete;
public:
    static constexpr auto mcu_usi = getBaseAddr<typename MCU::USI, N>;
    typedef MCU mcu_type;
    typedef typename MCU::USI::USIC uc;
    typedef typename MCU::USI::USIS us;
    typedef typename UsiPort<N, MCU>::mosi_pin mosi_pin;
    using sda_pin = mosi_pin;
    typedef typename UsiPort<N, MCU>::miso_pin miso_pin;
    typedef typename UsiPort<N, MCU>::clock_pin clock_pin;
    using scl_pin = clock_pin;
    
    template<typename Mode = SPI<MCU>>
    static void init() {
        if constexpr(!std::is_same<SSPin, void>::value) {
            SSPin::template dir<Input>();
            SSPin::pullup();
        }

        Mode::template init<mosi_pin, clock_pin>();
        
        if constexpr(std::is_same<miso_pin, SSPin>::value) {
            miso_pin::template dir<Input>();
            miso_pin::pullup();
        }
        else {
            miso_pin::template dir<Output>();
        }
        mcu_usi()->usicr.template set<Mode::usicr>();        
        mcu_usi()->usisr.template set<Mode::usisr>();        
    }

    static bool select() {
        if constexpr(std::is_same<SSPin, void>::value) {
            return true;
        }
        return !SSPin::read();
    }
    
    static void put(std::byte value) {
        mcu_usi()->usidr = value;
    }
    
    static std::byte get() {
        while(!(mcu_usi()->usisr.template isSet<us::oif>()));
        return *mcu_usi()->usibr;
    }
    
    static void isr() {
        mcu_usi()->usisr.template set<us::oif>();        
        if constexpr(!std::is_same<Inserter, void>::value) {
            Inserter::insert(*mcu_usi()->usibr);
        }
    }
    
    static void inline setTwiStartConditionMode() {
        mcu_usi()->usicr.template set<uc::sie | uc::wm1 | uc::cs1>();
        mcu_usi()->usisr.template set<us::oif | us::pf | us::dc>();
    }    
    static void inline setSendAck() {
        *mcu_usi()->usidr = std::byte{0};
        sda_pin::template dir<AVR::Output>();
        mcu_usi()->usisr.template set<us::oif | us::pf | us::dc | us::cnt3 | us::cnt2 | us::cnt1>();
    }
    static void inline setSendData() {
        sda_pin::template dir<AVR::Output>();
        mcu_usi()->usisr.template set<us::oif | us::pf | us::dc>();
    }
    static void inline setReadData() {
        sda_pin::template dir<AVR::Input>();
        mcu_usi()->usisr.template set<us::oif | us::pf | us::dc>();
    }
    static void inline setReadAck() {
        *mcu_usi()->usidr = std::byte{0};
        sda_pin::template dir<AVR::Input>();
        mcu_usi()->usisr.template set<us::oif | us::pf | us::dc | us::cnt3 | us::cnt2 | us::cnt1>();
    }
};

}
