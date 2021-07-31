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
#include <avr/eeprom.h>
#include <mcu/common/concepts.h>
#include <mcu/internals/nvm.h> 

namespace EEProm {

    template<typename DataType, uint16_t Offset = 0, typename MCU = DefaultMcuType>
    class Controller;

    template<typename T, bool UseMemberFlags> struct Base;
    
    template<typename T>
    struct Base<T, false> {
        enum class Flags : uint8_t {Changed, Saving, TimeoutExpired, Number};
        inline static constexpr uint8_t number_of_flags = uint8_t(Flags::Number);
    };
    template<typename T>
    struct Base<T, true> {
        struct Flags {
            uint8_t changed : 1, saving : 1, timeoutExpired : 1;
        } __attribute__((packed));
        inline static Flags mFlags = {false, false, false};
    };

    template<typename T, typename FlagRegister = void>
    class DataBase : public Base<DataBase<T, FlagRegister>, std::is_same_v<FlagRegister, void>> {
    public:
        DataBase() = default;
        DataBase(const DataBase&) = delete;
        DataBase& operator=(const DataBase&) = delete;

        using base_type = Base<DataBase<T, FlagRegister>, std::is_same_v<FlagRegister, void>>;

        inline static constexpr bool useBase = std::is_same_v<FlagRegister, void>;

        static void resetTimeout() {
            if constexpr(useBase) {
                base_type::mFlags.timeoutExpired = false;
            }
            else {
                FlagRegister::template reset<uint8_t(base_type::Flags::TimeoutExpired)>();
            }
        }
        inline static bool timeout() {
            if constexpr(useBase) {
                return base_type::mFlags.timeoutExpired;
            }
            else {
                return FlagRegister::template isSet<uint8_t(base_type::Flags::TimeoutExpired)>();
            }
        }
        inline static void saveStart() {
            if constexpr(useBase) {
                base_type::mFlags.saving  = true;
                base_type::mFlags.changed = false;
            }
            else {
                FlagRegister::template set<uint8_t(base_type::Flags::Saving)>();
                FlagRegister::template reset<uint8_t(base_type::Flags::Changed)>();
            }
        }
        inline static void saveEnd() {
            if constexpr(useBase) {
                base_type::mFlags.saving = false;
            }
            else {
                FlagRegister::template reset<uint8_t(base_type::Flags::Saving)>();
            }
        }
        inline static bool changed() {
            if constexpr(useBase) {
                return base_type::mFlags.changed || base_type::mFlags.saving;
            }
            else {
                return FlagRegister::template isSet<uint8_t(base_type::Flags::Changed)>() || 
                        FlagRegister::template isSet<uint8_t(base_type::Flags::Saving)>();
            }
        }
    public:
        static void expire() {
            if constexpr(useBase) {
                base_type::mFlags.timeoutExpired = true;
            }
            else {
                FlagRegister::template set<uint8_t(base_type::Flags::TimeoutExpired)>();
            }
        }
        inline static void change() {
            if constexpr(useBase) {
                base_type::mFlags.changed = true;
            }
            else {
                FlagRegister::template set<uint8_t(base_type::Flags::Changed)>();
            }
        }
    };
   
    template<typename DataType, uint16_t Offset, AVR::Concepts::At01Series MCU>
    class Controller<DataType, Offset, MCU> final {
        Controller() = delete;
    public:
        using data_t = DataType;
        inline static void init() {
            eeprom_read_block(reinterpret_cast<uint8_t*>(&mData), reinterpret_cast<void*>(Offset), sizeof(DataType));
        }
        inline constexpr static DataType& data() {
            return mData;
        }
        inline static bool saveIfNeeded() {
            if (mData.timeout() && mData.changed()) {
                mData.saveStart();
                if (eeprom_is_ready()) {
                    eeprom_update_byte(ePtr(mOffset), rawData(mOffset));
                    if (++mOffset == sizeof(DataType)) {
                        mOffset = 0;
                    }
                    if (mOffset == 0) {
                        mData.saveEnd();
                        mData.resetTimeout();
                        return false; // ready
                    }
                    else {
                        return true; // need to call once more
                    }
                }
                else {
                    return true; // need to call once more
                }
            }
            return true;
        }
        template<typename F>
        inline static void saveIfNeeded(const F& func_when_finished){
            if (!saveIfNeeded()) {
                func_when_finished();
            }
        }
    private:
        inline static uint8_t* ePtr(uintptr_t offset) {
            return reinterpret_cast<uint8_t*>(offset + Offset);
        }
        inline static uint8_t rawData(uintptr_t offset) {
            return *(reinterpret_cast<uint8_t*>(&mData) + offset);
        }
        inline static uintptr_t mOffset = 0;
        inline static DataType mData{};
    };

    template<typename DataType, uint16_t Offset, AVR::Concepts::AtDxSeries MCU>
    class Controller<DataType, Offset, MCU> final {
        Controller() = delete;
    public:
        
        using nvm = AVR::NvmCtrl<MCU>;
        
        using data_t = DataType;
        inline static void init() {
//            std::integral_constant<size_t, sizeof(DataType)>::_;
            nvm::read_eeprom((std::byte*)&mData, Offset, sizeof(DataType));
        }
        inline constexpr static DataType& data() {
            return mData;
        }
        inline static bool saveIfNeeded() {
            if (mData.timeout() && mData.changed()) {
                mData.saveStart();
                if (nvm::eeprom_ready()) {
                    nvm::write_eeprom(rawData(mOffset), mOffset);
                    if (++mOffset == sizeof(DataType)) {
                        mOffset = 0;
                    }
                    if (mOffset == 0) {
                        mData.saveEnd();
                        mData.resetTimeout();
                        return false; // ready
                    }
                    else {
                        return true; // need to call once more
                    }
                }
                else {
                    return true; // need to call once more
                }
            }
            return true;
        }
        template<typename F>
        inline static void saveIfNeeded(const F& func_when_finished){
            if (!saveIfNeeded()) {
                func_when_finished();
            }
        }
    private:
        inline static std::byte* ePtr(uintptr_t offset) {
            return reinterpret_cast<std::byte*>(offset + Offset);
        }
        inline static std::byte rawData(uintptr_t offset) {
            return *(reinterpret_cast<std::byte*>(&mData) + offset);
        }
        inline static uintptr_t mOffset = 0;
        inline static DataType mData{};
    };
    
}
