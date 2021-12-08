#pragma once

#include "storage.h"

template<typename PPM, typename PA, typename NVM, uint8_t Address = 0, uint8_t Size = 8>
struct FSM {
    inline static constexpr uint8_t address = Address;
    
    enum class SwState : uint8_t {Off, Steady, Blink1, Blink2};
    
    using ppm_value_t = PPM::ranged_type;
    
    using cycle10_t = etl::uint_ranged_circular<uint8_t, 0, 9>; // Robbe / CP mode use only 9 cycles, so we have to increment twice
    using cycle9_t = etl::uint_ranged_circular<uint8_t, 0, 8>; // Robbe / CP mode use only 9 cycles, so we have to increment twice
    using cycle6_t = etl::uint_ranged_circular<uint8_t, 0, 5>; // Robbe / CP mode use only 9 cycles, so we have to increment twice
    using cycle17_t = etl::uint_ranged_circular<uint8_t, 0, 16>; // CP16 mode use 17 cycles
    
    static inline void init() {
        PPM::init();    
    }
    static inline void periodic() {
        PPM::onReload([]{
            update();
        });
    }
    static inline auto& switches() {
        return swStates;
    }
    inline static void update() {
        const uint16_t impulsOffset = NVM::data().mpxOffset(Address);
        const auto mode = NVM::data().mpxMode(Address);
        
        if (mode == Storage::Mode::Graupner8K) {
            if (cycle10 < 2) {
                PPM::ppmRaw(PPM::ocMax + impulsOffset);
            }
            else {
                uint8_t i = cycle10 - 2;
                pulse8(i);
            }
            ++cycle10;
        }
        else if (mode == Storage::Mode::Graupner4K) {
            if (cycle6 < 2) {
                PPM::ppmRaw(PPM::ocMax + impulsOffset);
            }
            else {
                uint8_t i = cycle6 - 2;
                pulse8(i);
            }
            ++cycle6;
        }
        else if (mode == Storage::Mode::Robbe) {
            if (cycle9 < 1) {
                PPM::ppmRaw(PPM::ocMin - impulsOffset);
            }
            else {
                uint8_t i = Size - 1 - (cycle9 - 1); // Robbe counts channels in reverse order
                pulse8(i);
            }
            ++cycle9;
        }
        else if (mode == Storage::Mode::CP8) {
            if (cycle9 < 1) {
                PPM::ppmRaw(PPM::ocMax + impulsOffset);
            }
            else {
                uint8_t i = cycle9 - 1;
                pulse8ShortOnly(i);
            }
            ++cycle9;
        }
        else if (mode == Storage::Mode::CP16) {
            if (cycle17 < 1) {
                PPM::ppmRaw(PPM::ocMax + impulsOffset);
            }
            else {
                uint8_t i = cycle17 - 1;
                pulse16ShortOnly(i);
            }
            ++cycle17;
        }
        else if (mode == Storage::Mode::XXX) {
            if (cycle10 < 2) {
                PPM::ppmRaw(PPM::ocMin - impulsOffset);
            }            
            else {
                uint8_t i = cycle10 - 2;
                pulse8(i);
            }
            ++cycle10;
        }
        else {
            if (cycle10 < 2) {
                PPM::ppmRaw(PPM::ocMax + impulsOffset);
            }
            else {
                uint8_t i = cycle10 - 2;
                pulse8(i);
            }
            ++cycle10;
        }
    }
private:
    static inline void pulse8ShortOnly(uint8_t i) {
        const uint16_t pulseOffset = NVM::data().pulseOffset(Address);
        
        if (swStates[i] == SwState::Off) {
            PPM::ppmRaw(PPM::ocMedium);
        }
        else if (swStates[i] == SwState::Steady) {
            PPM::ppmRaw(PPM::ocMin + pulseOffset);
        }
        else if (swStates[i] == SwState::Blink1) {
            PPM::ppmRaw(PPM::ocMin + pulseOffset);
        }
        else {
            PPM::ppmRaw(PPM::ocMedium);
        }
    }
    static inline void pulse16ShortOnly(uint8_t i) {
        const uint16_t pulseOffset = NVM::data().pulseOffset(Address);
        
        if (i < 8) {
            if (swStates[i] == SwState::Off) {
                PPM::ppmRaw(PPM::ocMedium);
            }
            else if (swStates[i] == SwState::Steady) {
                PPM::ppmRaw(PPM::ocMin + pulseOffset);
            }
            else {
                PPM::ppmRaw(PPM::ocMedium);
            }
        }
        else {
            const uint8_t k = i - 8;
            if (swStates[k] == SwState::Off) {
                PPM::ppmRaw(PPM::ocMedium);
            }
            else if (swStates[k] == SwState::Blink1) {
                PPM::ppmRaw(PPM::ocMin + pulseOffset);
            }
            else {
                PPM::ppmRaw(PPM::ocMedium);
            }            
        }
    }
    static inline void pulse8(uint8_t i) {
        constexpr Storage::ChannelIndex::addr_type adr{Address};
        const Storage::ChannelIndex chi{adr, Storage::ChannelIndex::channel_type{i}};
        
        const uint16_t pulseOffset = NVM::data().pulseOffset(Address);
        
        if (const auto pch = NVM::data().passThru(chi)) {
            auto v = PA::value(pch);
            PPM::ppm_async(v);
        }
        else if (swStates[i] == SwState::Off) {
            PPM::ppmRaw(PPM::ocMedium);
        }
        else if (swStates[i] == SwState::Steady) {
            PPM::ppmRaw(PPM::ocMax - pulseOffset);
        }
        else if (swStates[i] == SwState::Blink1) {
            PPM::ppmRaw(PPM::ocMin + pulseOffset);
        }
        else {
            PPM::ppmRaw(PPM::ocMedium);
        }
    }
    static inline cycle17_t cycle17;
    static inline cycle10_t cycle10;
    static inline cycle9_t cycle9;
    static inline cycle6_t cycle6;
    static inline std::array<SwState, Size> swStates;
};
