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
#include "external/onewire.h"

template<typename OneWireMaster, bool Single = true>
class DS18B20 final : public EventHandler<EventType::OneWireRecvComplete> {
public:
  
    typedef OneWireMaster owmaster_type;
    static constexpr bool single = Single;
    
    static constexpr uint8_t romSize = 8;
    static constexpr uint8_t scratchpadSize = 9;
    
    DS18B20() = delete;
    
    static void init() {
        OneWireMaster::reset();
    }
    
    template<uint8_t N>
    static void startGet() {
        OneWireMaster::template startGet<N>();
    }
    
    static void reset() {
        OneWireMaster::reset();
    }

    static void command(OneWire::Command cmd) {
        OneWireMaster::put(static_cast<uint8_t>(cmd));
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
        if (!OneWireMaster::reset()) {
            return false;
        }
        command(OneWire::Command::ReadRom);
        if constexpr(Single) {
            for(uint8_t i = 0; i < romSize; ++i) {
                rom[i] = OneWireMaster::get();
            }   
        }
        return true;
    }
    static bool readScratchpad(std::array<uint8_t, scratchpadSize>& sp) {
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
    static bool writeScratchpad(std::array<uint8_t, 3>& sp) {
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
    static void process(uint8_t) {
        for(uint8_t i = 0; i < scratchpadSize; ++i) {
            if (auto v = OneWireMaster::get()) {
            }
        }
    }
    
};
        