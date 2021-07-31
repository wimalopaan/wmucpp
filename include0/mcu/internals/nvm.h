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
#include <cstring>

#include "../common/concepts.h"

namespace AVR {
    template<typename MCU = DefaultMcuType>
    struct NvmCtrl;
    
    template<AVR::Concepts::AtDxSeries MCU>
    struct NvmCtrl<MCU> {
        using CtrlA_t = MCU::NvmCtrl::CtrlA_t;
        using Status_t = MCU::NvmCtrl::Status_t;
        inline static constexpr auto mcu_nvm = AVR::getBaseAddr<typename MCU::NvmCtrl>;

        using ccp = AVR::Cpu::Ccp<MCU>;
        
        inline static std::byte read_eeprom(const uint16_t offset) {
            const volatile std::byte* const eepromStart = reinterpret_cast<volatile std::byte*>(EEPROM_START);
            return *(eepromStart + offset);
        }

        inline static void read_eeprom(std::byte* const p, const uint16_t offset, uint16_t size) {
            const volatile std::byte* const volatile eepromStart = reinterpret_cast<volatile std::byte*>(EEPROM_START);
            std::memcpy(p, (void*)(eepromStart + offset), size);
        }
        
        inline static void write_eeprom(const std::byte b, const uint16_t offset) {
            volatile std::byte* eepromStart = reinterpret_cast<volatile std::byte*>(EEPROM_START);
        
            ccp::spm([]{
                mcu_nvm()->ctrla.template set<CtrlA_t::eeerwr>();
            });
            
            *(eepromStart + offset) = b;

            ccp::spm([]{
                mcu_nvm()->ctrla.template set<CtrlA_t::nocmd>();
            });
        }

        inline static bool eeprom_ready() {
            return !mcu_nvm()->status.template isSet<Status_t::eebusy>();
        } 
    };
}
