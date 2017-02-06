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

#include "std/array.h"
#include "std/algorithm.h"
#include "util/fixedpoint.h"
#include "external/onewire.h"

// todo: use Single
template<typename OneWireMaster, bool Single = true>
class DS18B20 final : public EventHandler<EventType::OneWireRecvComplete> {
public:
    enum class Resolution {R9bit = 0x1f, R10bit = 0x3f, R11bit = 0x5f, R12bit = 0x7f};
    
    typedef OneWireMaster owmaster_type;
    static constexpr bool single = Single;
    
    static constexpr uint8_t readScratchpadSize = 9;
    static constexpr uint8_t writeScratchpadSize = 3;
    
    typedef std::array<uint8_t, readScratchpadSize> ds18b20_rsp_t;
    typedef std::array<uint8_t, writeScratchpadSize> ds18b20_wsp_t;
    
    DS18B20() = delete;
    
    static void init() {
        OneWireMaster::init();
        OneWireMaster::reset();
    }
    
    template<uint8_t N = readScratchpadSize>
    static void startGet() {
        static_assert(OneWireMaster::isAsync, "async interface shall use async OneWireMaster");
        OneWireMaster::template startGet<N>();
    }
    
    static void reset() {
        OneWireMaster::reset();
    }

    static void command(OneWire::Command cmd) {
        OneWireMaster::put(static_cast<uint8_t>(cmd));
    }

    template<uint8_t N = readScratchpadSize>
    static void startGet(OneWire::ow_rom_t& rom) {
        reset();
        command(OneWire::Command::MatchRom);
        for(uint8_t i = 0; i < rom.size; ++i) {
            OneWireMaster::put(rom[i]);
        }
        command(OneWire::Command::ReadScratchpad);
        startGet<N>();
    }

    static bool convert() {
        if (!OneWireMaster::reset()) {
            return false;
        }
        command(OneWire::Command::SkipRom);
        command(OneWire::Command::Convert);
        return true;
    }
    
    static bool readRom(OneWire::ow_rom_t& rom) {
        static_assert(!OneWireMaster::isAsync, "sync interface shall use sync OneWireMaster");
        if (!OneWireMaster::reset()) {
            return false;
        }
        command(OneWire::Command::ReadRom);
        if constexpr(Single) {
            for(uint8_t i = 0; i < rom.size; ++i) {
                rom[i] = OneWireMaster::get();
            }   
        }
        if (!std::crc8(rom)) {
            return false;
        }
        return true;
    }

    static bool readScratchpad(ds18b20_rsp_t& sp) {
        static_assert(!OneWireMaster::isAsync, "sync interface shall use sync OneWireMaster");
        if (!OneWireMaster::reset()) {
            return false;
        }
        command(OneWire::Command::SkipRom);
        command(OneWire::Command::ReadScratchpad);
        for(uint8_t i = 0; i < sp.size; ++i) {
            sp[i] = OneWireMaster::get();
        }
        return true;
    }

    static bool readScratchpad(const OneWire::ow_rom_t& rom, ds18b20_rsp_t& sp) {
        static_assert(!OneWireMaster::isAsync, "sync interface shall use sync OneWireMaster");
        if (!OneWireMaster::reset()) {
            return false;
        }
        command(OneWire::Command::MatchRom);
        for(uint8_t i = 0; i < rom.size; ++i) {
            OneWireMaster::put(rom[i]);
        }
        command(OneWire::Command::ReadScratchpad);
        for(uint8_t i = 0; i < sp.size; ++i) {
            sp[i] = OneWireMaster::get();
        }
        return true;
    }
    
    static bool writeScratchpad(ds18b20_wsp_t& sp) {
        static_assert(!OneWireMaster::isAsync, "sync interface shall use sync OneWireMaster");
        if (!OneWireMaster::reset()) {
            return false;
        }
        command(OneWire::Command::SkipRom);
        command(OneWire::Command::WriteScratchpad);
        for(uint8_t i = 0; i < sp.size; ++i) {
            OneWireMaster::put(sp[i]);
        }
        return true;
    }

    static FixedPoint<int16_t, 4> temperature() {
        return temperature(scratchPad());
    }

    static FixedPoint<int16_t, 4> temperature(ds18b20_rsp_t& sp) {
        uint16_t valueL = sp[0];
        uint16_t valueH = sp[1];
        
        return FixedPoint<int16_t, 4>::fromRaw(valueH << 8 | valueL);
    }
    
    static bool process(uint8_t) {
        static_assert(OneWireMaster::isAsync, "async interface shall use async OneWireMaster");
        bool ok = true;
        for(uint8_t i = 0; i < scratchPad().size; ++i) {
            if (auto v = OneWireMaster::get()) {
                scratchPad()[i] = *v;
            }
            else {
                ok = false;
            }
        }
        ok &= std::crc8(scratchPad());
        
        if (ok) {
            EventManager::enqueue({EventType::DS18B20Measurement, 0});
        } 
        else {
            EventManager::enqueue({EventType::DS18B20Error, 0});
        }
        return true;
    }
private:
    static ds18b20_rsp_t& scratchPad() {
        static ds18b20_rsp_t mScratchPad;
        return mScratchPad;
    }
};
