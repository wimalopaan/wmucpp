/*
 * ++C - C++ introduction
 * Copyright (C) 2013, 2014, 2015, 2016, 2017 Wilhelm Meier <wilhelm.meier@hs-kl.de>
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
#include <avr/eeprom.h>

template<typename DataType, uint16_t Offset = 0>
class EEProm;

template<typename T>
class EEPromBase {
    struct Flags {
        uint8_t changed : 1, saving : 1, timeoutExpired : 1;
    } __attribute__((packed));
    friend class EEProm<T>;
    static void startTimeout() {
        mFlags.timeoutExpired = false;
    }
public:
    inline static bool changed() {
        return mFlags.changed || mFlags.saving;
    }
    inline static void change() {
        mFlags.changed = true;
    }
    inline static bool timeout() {
        return mFlags.timeoutExpired;
    }
    inline static void saveStart() {
        mFlags.saving  = true;
        mFlags.changed = false;
    }
    inline static void saveEnd() {
        mFlags.saving = false;
    }
private:
    inline static Flags mFlags = {false, false, false};
};

template<typename DataType, uint16_t Offset>
class EEProm {
public:
    static void init() {
        eeprom_read_block(&mData, static_cast<void*>(Offset), sizeof(DataType));
    }
    static DataType& data() {
        return mData;
    }
    static void saveIfNeeded() {
        if (mData.timeout() && mData.changed()) {
            mData.saveStart();
            if (eeprom_is_ready()) {
                eeprom_update_byte(ePtr(mOffset), rawData(mOffset));
                mOffset = (mOffset + 1) % sizeof(DataType) + Offset;
                if (mOffset == Offset) {
                    mData.saveEnd();
                    mData.startTimeout();
                }
            }
        }
    }
private:
    inline static uint8_t* ePtr(uintptr_t offset) {
        return reinterpret_cast<uint8_t*>(offset);
    }
    inline static uint8_t rawData(uintptr_t offset) {
        return *(reinterpret_cast<uint8_t*>(&mData) + offset);
    }
    inline static uintptr_t mOffset = Offset;
    inline static DataType mData;
};  