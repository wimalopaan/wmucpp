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
#include <cstddef>
#include <array>
#include <algorithm>

#include <etl/outfwd.h>

namespace OneWire {
    struct Rom;

    template<etl::Concepts::Stream Stream> 
    inline void out_impl(const OneWire::Rom& rom);
}

namespace OneWire {
    using namespace AVR;
    using namespace AVR::Util;
    using namespace etl;
    using namespace std;
    using namespace std::chrono;
    
    struct Rom {
        template<etl::Concepts::Stream Stream> friend void out_impl(const OneWire::Rom& rom);
        
        inline constexpr uint8_t size() const {
            return 8;
        }
        
        inline std::byte& operator[](uint8_t index) {
            return data[index];
        }
        inline std::byte operator[](uint8_t index) const {
            return data[index];
        }
        
        inline std::byte familiy() const {
            return data[0];
        }
        
        inline explicit operator bool() const {
            return !std::all_of(std::begin(data), std::end(data), [](std::byte v){return v == std::byte{0};}) && etl::crc8(data);
        }
    private:
        std::array<std::byte, 8> data;
    };
    typedef Rom ow_rom_t;
 
    struct Normal {};
    struct OverDrive{};
    
    template<typename Mode>
    struct Parameter;
    
    template<>
    struct Parameter<Normal> {
        static constexpr auto start = 6_us;
        static constexpr auto tA = start;
        static constexpr auto writeOneWait = 64_us;
        static constexpr auto tB = writeOneWait;
        static constexpr auto writeZero = 60_us;
        static constexpr auto tC = writeZero;
        static constexpr auto writeZeroWait = 10_us;
        static constexpr auto tD = writeZeroWait;
        static constexpr auto waitForSampleAfterStart = 9_us;
        static constexpr auto tE = waitForSampleAfterStart;
        static constexpr auto waitAfterSample = 55_us;
        static constexpr auto tF = waitAfterSample;
        static constexpr auto reset = 480_us;
        static constexpr auto tH = reset;
        static constexpr auto waitForPresenceAfterReset= 70_us;
        static constexpr auto tI = waitForPresenceAfterReset;
        static constexpr auto waitAfterPresence = 410_us;
        static constexpr auto tJ = waitAfterPresence;
        
        static constexpr auto recovery = 10_us;
    };
    
    template<>
    struct Parameter<OverDrive> {
        static constexpr centimicroseconds pre{10};
        static constexpr centimicroseconds zeroTotal{75};
        static constexpr centimicroseconds sampleAfterPre{10};
        static constexpr centimicroseconds reset{700};
        static constexpr centimicroseconds presenceAfterReset{85};
    };
    
    enum class Command {ReadRom = 0x33, SkipRom = 0xcc, MatchRom = 0x55, SearchRom = 0xf0, 
                        Convert = 0x44, ReadScratchpad = 0xbe, WriteScratchpad = 0x4e};
    
    template<typename OWMaster, const microseconds& delay, uint16_t QueueSize = 16>
    class MasterAsync final {
        enum class State : uint8_t {Inactive, ResetWait, Sending, Reading};
        
        static_assert(delay >= OneWire::Parameter<OneWire::Normal>::reset, "delay too short");
        
    public:
        typedef typename OWMaster::pin_type pin_type;
        typedef OneWire::Normal mode_type;
        static constexpr bool isAsync = true;
        
        MasterAsync() = delete;
        
        static void rateProcess() { // called every delay us
            static std::byte data{0};
            switch(mState) {
            case State::Inactive:
                if (!mSendQueue.empty()) {
                    mState = State::Sending;
                    mBitCount = 0;
                    data = *mSendQueue.pop_front();
                }
                else if (mBytesToRead > 0) {
                    mBitCount = 0;
                    data = std::byte{0};
                    mState = State::Reading;
                }
                break;
            case State::ResetWait:
            {
                Scoped<DisbaleInterrupt<RestoreState>> di;
                OWMaster::pinTriState();
                AVR::Util::delay(Parameter<mode_type>::waitForPresenceAfterReset);
                mDevicesPresent = !pin_type::read(); // actice low
            }
                mState = State::Inactive;
                break;
            case State::Sending:
                if (mBitCount++ < 8) {
                    OWMaster::template writeBit<false>(std::any(data & std::byte{0x01}));
                    data >>= 1;
                }
                else {
                    mState = State::Inactive;
                }
                break;
            case State::Reading:
                if (mBitCount++ < 8) {
                    data >>= 1;
                    if (OWMaster::template readBit<false>()) {
                        data |= std::byte{0x80};
                    }
                }
                else {
                    [[maybe_unused]] bool ok = mRecvQueue.push_back(data);
                    assert(ok);
                    --mBytesToRead;
                    if (mBytesToRead == 0) {
                        mState = State::Inactive;
                    }
                    else {
                        data = std::byte{0};
                        mBitCount = 0;
                    }
                }
                break;
            default:
                break;
            }        
        } 
        static constexpr auto periodic = rateProcess;
        
        static bool reset() {
            OWMaster::pinLow();
            mState = State::ResetWait;
            return true;
        }
        static void start() {
        }
        static void init() {
            OWMaster::init();
        }
        static bool put(std::byte v) {
            return mSendQueue.push_back(v);
        }
        static std::optional<std::byte> get() {
            return mRecvQueue.pop_front();
        }
        template<uint8_t N>
        static void startGet() {
            assert(mBytesToRead == 0);
            mBytesToRead = N;        
        }
        static bool devivesPresent() {
            return mDevicesPresent;
        }
    private:
        inline static etl::FiFo<std::byte, QueueSize> mRecvQueue;
        inline static etl::FiFo<std::byte, QueueSize> mSendQueue;
        inline static uint8_t mBitCount = 0;
        inline static State mState = State::Inactive;
        inline static bool mDevicesPresent = false;
        inline static uint8_t mBytesToRead = 0;
    };
    
    template<typename Pin, typename Mode, bool InternalPullup = true, bool ParasitePower = false>
    class Master final {
    public:
        Master() = delete;
        
        typedef Pin pin_type;
        typedef Mode mode_type;
        static constexpr bool internal_pullup = InternalPullup;
        static constexpr bool isAsync = false;
        
        static void init() {
            Set<Pin>::output();
            Pin::high();
        }
        static inline void pinLow() {
            Pin::low();
            Set<Pin>::output();
        }
        static inline void pinTriState() {
            Set<Pin>::input();
            if constexpr(InternalPullup) {
                Pin::pullup();      
            }
        }
        static inline void pinHigh() {
            Pin::high();
            Set<Pin>::output();
        }
        
        template<bool UseRecovery = false>
        static void writeBit(bool bit) {
            if constexpr(UseRecovery) {
                Util::delay(Parameter<Mode>::recovery);
            }
            {
                Scoped<DisbaleInterrupt<RestoreState>> di;
                pinLow();
                if (bit) {
                    Util::delay(Parameter<Mode>::start); 
                    pinTriState();
                    Util::delay(Parameter<Mode>::writeOneWait); 
                }
                else {
                    Util::delay(Parameter<Mode>::writeZero); 
                    pinTriState();
                    Util::delay(Parameter<Mode>::writeZeroWait); 
                }
            }
            if constexpr (ParasitePower) {
                pinHigh();
            }
        }    
        
        template<bool UseRecovery = false>
        static bool readBit() {
            if constexpr(UseRecovery) {
                Util::delay(Parameter<Mode>::recovery);
            }
            {
                Scoped<DisbaleInterrupt<RestoreState>> di;
                pinLow();
                Util::delay(Parameter<Mode>::start); 
                pinTriState();
                Util::delay(Parameter<Mode>::waitForSampleAfterStart);
                bool value = Pin::read();
                if constexpr(ParasitePower) {
                    pinHigh();
                }
                Util::delay(Parameter<Mode>::waitAfterSample);
                return value;
            }
        }
        static bool reset() {
            bool presence = false;
            pinLow();
            Util::delay(Parameter<Mode>::reset);       
            {
                Scoped<DisbaleInterrupt<RestoreState>> di;
                pinTriState();
                Util::delay(Parameter<Mode>::waitForPresenceAfterReset);
                presence = !Pin::read(); // actice low
                if constexpr (ParasitePower) {
                    pinHigh();
                }
            }
            Util::delay(Parameter<Mode>::waitAfterPresence);    
            return presence;
        }
        static uint8_t get() {
            uint8_t result = 0;
            for(uint8_t i = 0; i < 8; ++i) {
                result >>= 1;
                if (readBit()) {
                    result |= 0x80;
                }
            }
            return result;
        }
        static void put(uint8_t byte) {
            for(uint8_t i = 0; i < 8; ++i) {
                writeBit(byte & 0x01);
                byte >>= 1;
            }        
        }
        
        enum class Result : uint8_t {
            ok = 0,
            no_presence = 1,
            crc_error = 2,
            scan_error = 3,
            last_code = 4,
            gnd_short = 5
        };
        
        template<uint8_t MaxDevices>
        static uint8_t findDevices(std::array<ow_rom_t, MaxDevices>& devices, std::byte family = 0) {
            Rom rom;
            searchRom2(nullptr);
            for(uint8_t i = 0; i < devices.size(); ++i) {
                auto result = searchRom2(&rom);
                if ((result == Result::ok) || (result == Result::last_code)) {
                    if ((family == std::byte{0}) || (family == rom[0])) {
                        devices[i] = rom;
                        if (result == Result::last_code) {
                            return i + 1;
                        }
                    }
                }
            }
            return MaxDevices;
        }
        
        
        static Result searchRom2(ow_rom_t* rom = nullptr) {
            uint8_t bit = 0;
            uint8_t max_conf_zero = 0;        // last bit conflict that was resolved to zero
            static uint8_t max_conf_old = 64;    // last bit conflict that was resolved to zero in last scan
            uint8_t branch_flag = 0;          // indicate new scan branch, new ROM code found  
            
            if (rom == nullptr) {    // init search
                max_conf_old=64;
                return Result::ok;
            }
            
            if (!reset()) {
                return Result::no_presence;
            }
            
            put(static_cast<uint8_t>(Command::SearchRom));
            
            std::byte rom_tmp = (*rom)[0];
            uint8_t i = 0;
            std::byte mask = std::byte{1};
            
            // scan all 64 ROM bits
            
            for(uint8_t j = 0; j < 64; ++j) {
                bit = 0;
                if (readBit()) {
                    bit = 1;
                }
                if (readBit()) {
                    bit |= 2;
                }
                switch(bit) {
                case 0:     // bit conflict, more than one device with different bit
                    if (j < max_conf_old) {         // below last zero branch conflict level, keep current bit
                        if (std::any(rom_tmp & mask)) {       // last bit was 1
                            bit = 1;
                        } else {                    // last bit was 0
                            bit = 0;
                            max_conf_zero = j;
                            branch_flag = 1;
                        }                        
                    } else if (j == max_conf_old) { // last zero branch conflict, now enter new path
                        bit = 1;
                    } else {                        // above last scan conflict level
                        bit = 0;                    // scan 0 branch first
                        max_conf_zero = j;
                        branch_flag = 1;
                    }
                    break;
                case 1:
                case 2: // no bit conflict
                    // do nothing, just go on with current bit
                    break;
                case 3:     // no response
                    return Result::scan_error;
                    break;
                }
                
                // write bit to OneWire and ROM code
                
                if (bit & 1) {
                    writeBit(true);
                    rom_tmp |= mask;
                } else {
                    writeBit(false);
                    rom_tmp &= ~mask;
                }
                
                mask <<= 1;
                if (mask == std::byte{0}) {
                    mask = std::byte{1};
                    (*rom)[i] = rom_tmp;            // update tmp data
                    i++;
                    if (i<8) {
                        rom_tmp  = (*rom)[i];       // read new data
                    }
                }
            }
            
            max_conf_old = max_conf_zero;
            
            if (branch_flag) {
                return Result::ok;
            } else {
                return Result::last_code;
            }
            
        }
    };
    
    template<etl::Concepts::Stream Stream>
    void out_impl(const OneWire::Rom& rom) {
        etl::out<Stream>("OneWireId["_pgm);
        for(const auto& v : rom.data) {
            etl::out<Stream>(v, etl::Char{','});
        }
        etl::out<Stream>(static_cast<bool>(rom), etl::Char{']'});
    }
}

