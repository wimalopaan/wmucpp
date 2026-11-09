#pragma once

#include <cstring>
#include <span>
#include <type_traits>

#include "eeprom.h"

namespace Multiswitch {
    struct LegacyOutput {
        uint8_t pwm = 0;
        uint8_t pwmDuty = 1;
        uint8_t pwmScale = 0;
        uint8_t blink = 0;
        uint8_t blinkOnTime = 1;
        uint8_t blinkOffTime = 2;
        uint8_t flashCount = 1;
        std::array<char, 16> name{};
    };
    static_assert(sizeof(LegacyOutput) == 23);

    struct LegacyConfiguration {
        uint8_t magic = EEPROM_MAGIC;
        uint8_t pwm1 = 10;
        uint8_t pwm2 = 10;
        uint8_t pwm3 = 10;
        uint8_t pwm4 = 10;
        std::array<uint8_t, 1> addresses{0};
        uint8_t crsf_address = 0xc8;
        uint8_t response_slot = 0;
        uint8_t telemetry = 1;
        std::array<LegacyOutput, 8> outputs{};
        std::array<std::array<char, 16>, 1> morse_texts{};
        uint8_t morse_dit = 3;
        uint8_t morse_dah = 6;
        uint8_t morse_gap = 3;
        uint8_t morse_igap = 3;
        uint8_t morse_wgap = 9;
        uint8_t failsafe_mode = 0;
        std::array<uint8_t, 8> failsafe_pattern{};
    };
    static_assert(sizeof(LegacyConfiguration) == 223);
    static_assert(std::is_trivially_copyable_v<LegacyConfiguration>);

    struct VirtualPatternConfiguration {
        uint8_t magic = EEPROM_MAGIC;
        uint8_t pwm1 = 10;
        uint8_t pwm2 = 10;
        uint8_t pwm3 = 10;
        uint8_t pwm4 = 10;
        std::array<uint8_t, 2> addresses{0, 1};
        uint8_t crsf_address = 0xc8;
        uint8_t response_slot = 0;
        uint8_t telemetry = 1;
        uint8_t use_virtuals = 1;
        std::array<LegacyOutput, 8> outputs{};
        std::array<std::array<char, 16>, 1> morse_texts{};
        uint8_t morse_dit = 3;
        uint8_t morse_dah = 6;
        uint8_t morse_gap = 3;
        uint8_t morse_igap = 3;
        uint8_t morse_wgap = 9;
        std::array<EEProm::Virtual, 4> virtuals{};
        uint8_t enableGlobalDimming_virtual4 = 0;
        std::array<External::Pattern::EEProm, 4> pattern{};
        uint8_t failsafe_mode = 0;
        std::array<uint8_t, 8> failsafe_pattern{};
    };
    static_assert(sizeof(VirtualPatternConfiguration) == 294);
    static_assert(std::is_trivially_copyable_v<VirtualPatternConfiguration>);

    inline EEProm upgradeLegacy(const auto& legacy) {
        EEProm upgraded;
        upgraded.pwm1 = legacy.pwm1;
        upgraded.pwm2 = legacy.pwm2;
        upgraded.pwm3 = legacy.pwm3;
        upgraded.pwm4 = legacy.pwm4;
        upgraded.addresses[0] = legacy.addresses[0];
        if (upgraded.addresses[1] == upgraded.addresses[0]) ++upgraded.addresses[1];
        upgraded.crsf_address = legacy.crsf_address;
        upgraded.response_slot = legacy.response_slot;
        upgraded.telemetry = legacy.telemetry;
        for (size_t i = 0; i < upgraded.outputs.size(); ++i) {
            const auto& old = legacy.outputs[i];
            upgraded.outputs[i] = {
                .pwm = old.pwm, .pwmDuty = old.pwmDuty, .pwmScale = old.pwmScale,
                .blink = old.blink, .blinkOnTime = old.blinkOnTime, .blinkOffTime = old.blinkOffTime,
                .flashCount = old.flashCount, .slaveEnable = 0, .name = old.name};
        }
        upgraded.morse_texts = legacy.morse_texts;
        upgraded.morse_dit = legacy.morse_dit;
        upgraded.morse_dah = legacy.morse_dah;
        upgraded.morse_gap = legacy.morse_gap;
        upgraded.morse_igap = legacy.morse_igap;
        upgraded.morse_wgap = legacy.morse_wgap;
        upgraded.failsafe_mode = legacy.failsafe_mode;
        upgraded.failsafe_pattern = legacy.failsafe_pattern;
        return upgraded;
    }

    inline bool migrateLegacy(const std::span<const uint8_t> bytes, EEProm& config) {
        if (bytes.size() != sizeof(LegacyConfiguration)) return false;
        LegacyConfiguration legacy;
        std::memcpy(&legacy, bytes.data(), bytes.size());
        if (legacy.magic != EEPROM_MAGIC) return false;
        config = upgradeLegacy(legacy);
        return true;
    }

    inline bool migrateVirtualPatterns(const std::span<const uint8_t> bytes, EEProm& config) {
        if (bytes.size() != sizeof(VirtualPatternConfiguration)) return false;
        VirtualPatternConfiguration legacy;
        std::memcpy(&legacy, bytes.data(), bytes.size());
        if (legacy.magic != EEPROM_MAGIC) return false;
        auto upgraded = upgradeLegacy(legacy);
        upgraded.addresses = legacy.addresses;
        upgraded.use_virtuals = legacy.use_virtuals;
        upgraded.virtuals = legacy.virtuals;
        upgraded.enableGlobalDimming_virtual4 = legacy.enableGlobalDimming_virtual4;
        upgraded.pattern = legacy.pattern;
        config = upgraded;
        return true;
    }
}
