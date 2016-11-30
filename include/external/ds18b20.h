/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2013, 2014, 2015, 2016 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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


template<typename OneWireMaster, bool Single = true>
class DS18B20 final : public EventHandler<EventType::OneWireRecvComplete> {
public:
    enum class Resolution {R9bit = 0x1f, R10bit = 0x3f, R11bit = 0x5f, R12bit = 0x7f};
    
    typedef OneWireMaster owmaster_type;
    static constexpr bool single = Single;
    
    static constexpr uint8_t romSize = 8;
    static constexpr uint8_t readScratchpadSize = 9;
    static constexpr uint8_t writeScratchpadSize = 3;
    
    DS18B20() = delete;
    
    static void init() {
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
    static void startGet(std::array<uint8_t, romSize>& rom) {
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
    
    static bool readRom(std::array<uint8_t, romSize>& rom) {
        static_assert(!OneWireMaster::isAsync, "sync interface shall use sync OneWireMaster");
        if (!OneWireMaster::reset()) {
            return false;
        }
        command(OneWire::Command::ReadRom);
        if constexpr(Single) {
            for(uint8_t i = 0; i < romSize; ++i) {
                rom[i] = OneWireMaster::get();
            }   
        }
        if (!std::crc8(rom)) {
            return false;
        }
        return true;
    }

    static bool readScratchpad(std::array<uint8_t, readScratchpadSize>& sp) {
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

    static bool readScratchpad(std::array<uint8_t, romSize>& rom, std::array<uint8_t, readScratchpadSize>& sp) {
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
    
    static bool writeScratchpad(std::array<uint8_t, writeScratchpadSize>& sp) {
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
        union {
            int16_t value;
            struct {
                int8_t valueL;
                int8_t valueH;
            };
        } v;
        v.valueL = scratchPad()[0];
        v.valueH = scratchPad()[1];
        
        return FixedPoint<int16_t, 4>::fromRaw(v.value);
    }

    static void process(uint8_t) {
        static_assert(OneWireMaster::isAsync, "async interface shall use async OneWireMaster");
        bool ok = true;
        for(uint8_t i = 0; i < readScratchpadSize; ++i) {
            if (auto v = OneWireMaster::get()) {
                scratchPad()[i] = *v;
            }
            else {
                ok = false;
            }
        }
        ok &= !std::crc8(scratchPad());
        
        if (ok) {
            EventManager::enqueue({EventType::DS18B20Measurement, 0});
        } 
        EventManager::enqueue({EventType::DS18B20Error, 0});
    }
private:
    static std::array<uint8_t, readScratchpadSize>& scratchPad() {
        static std::array<uint8_t, readScratchpadSize> mScratchPad;
        return mScratchPad;
    }
};
