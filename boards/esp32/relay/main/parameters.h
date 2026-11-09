// SPDX-License-Identifier: GPL-3.0-or-later
// Parameter table adapted from ../../rcRelay32/crsf_cb.h.
#pragma once

#include <cstdio>

#include "protocol.h"
#include "board.h"
#include "eeprom.h"

namespace Relay {
    class Parameters {
    public:
        using Parameter = Crsf::Parameter<uint8_t>;
        using Type = Parameter::Type;
        enum class ReadResult {Ok, InvalidIndex, InvalidChunk};

        explicit Parameters(EEProm& config) : mConfig{config} {
            add({0, Type::Folder, "root"});
            add({0, Type::Info, "Version(HW/SW)", "4:19"});
            add({0, Type::Info, "Reset reason", mResetReason.data()});
            add({0, Type::U8, "CRSF Address", nullptr, &config.address, 192, 207});
            add({0, Type::U8, "TX Rewrite Address", nullptr, &config.tx_rewrite_address, 192, 207});
            add({0, Type::U8, "RX Rewrite Address", nullptr, &config.rx_rewrite_address, 192, 207});
            add({0, Type::Str, "TX Rewrite Name", nullptr, nullptr, 0, 0, nullptr,
                 0, uint8_t(config.txname.size()), 0, config.txname.data()});
            add({0, Type::Sel, "Rewrite Name", "Off;On", &config.rewrite_name, 0, 1});
            add({0, Type::Sel, "LinkStat", "Off;Forward;Transform;Tunnel", &config.link_stat_mode, 0, 3});
            add({0, Type::Sel, "Failsafe", "Forward;Hold", &config.failsafe_mode, 0, 1});
            add({0, Type::Sel, "Parameter Retry", "Off;On", &config.parameter_retry, 0, 1});
            add({.parent = 0, .type = Type::U8, .name = "Telem. Off Dur.",
                 .value_ptr = &config.tempOffTelemetry, .min = 1, .max = 60, .unitString = "sec"});
            const uint8_t parent = mEntries.size();
            add({0, Type::Folder, "Telemetry forwarding"});
            add({parent, Type::U8, "Forward Rate", nullptr, &config.telemetry_rate, 1, 10});
            for (const auto& entry : telemetry) {
                add({parent, Type::Sel, entry.name, "Off;Forward;Tunnel",
                     &config.telemetryMode[entry.type], 0, 2});
            }
            add({parent, Type::Sel, "ArduPilot", "Off;Forward", &config.ardupilot_mode, 0, 1});
            add({0, Type::Sel, "Half-Duplex", "Off;On", &config.half_duplex, 0, 1});
            add({0, Type::Sel, "Baudrate", "400k;420k;460K;921K;1,8M", &config.baudrate, 0, 4});
        }

        uint8_t size() const { return mEntries.size(); }
        const Parameter& operator[](const uint8_t index) const { return mEntries[index]; }

        const char* name() {
            std::snprintf(mName.data(), mName.size(), "RC-Relay32@%u", mConfig.address);
            return mName.data();
        }

        void resetReason(const unsigned reason) {
            std::snprintf(mResetReason.data(), mResetReason.size(), "%u", reason);
        }

        ReadResult read(const uint8_t destination, const uint8_t index, const uint8_t chunk,
                        ByteBuffer<58>& payload) {
            if (index >= size()) return ReadResult::InvalidIndex;
            ByteBuffer<256> serialized;
            mEntries[index].serialize(serialized, *this, Crsf::Lua::CmdStep::Idle, index);
            constexpr size_t chunkSize = 54;
            const size_t chunks = (serialized.size() + chunkSize - 1) / chunkSize;
            if (chunk >= chunks) return ReadResult::InvalidChunk;
            payload.push_back(destination);
            payload.push_back(mConfig.address);
            payload.push_back(index);
            payload.push_back(uint8_t(chunks - chunk - 1));
            const size_t end = std::min(size_t(serialized.size()), (chunk + 1) * chunkSize);
            for (size_t i = chunk * chunkSize; i < end; ++i) payload.push_back(serialized[i]);
            return ReadResult::Ok;
        }

        bool write(const uint8_t index, const std::span<const uint8_t> data) {
            if (!index || index >= size() || data.empty()) return false;
            auto& p = mEntries[index];
            if (p.type == Type::Str) {
                const size_t count = std::min(data.size(), size_t{p.units - 1u});
                std::copy_n(data.begin(), count, p.stringValue);
                p.stringValue[count] = '\0';
            }
            else if (p.type == Type::U8 || p.type == Type::Sel) {
                if (data[0] < p.min || data[0] > p.max) return false;
                p.value(data[0]);
            }
            else {
                return false;
            }
            return true;
        }

    private:
        void add(const Parameter p) { mEntries.push_back(p); }
        struct TelemetryEntry { uint8_t type; const char* name; };
        static constexpr std::array<TelemetryEntry, 18> telemetry{{
            {0x02, "GPS"}, {0x03, "GPS Time"}, {0x06, "GPS Ext"}, {0x07, "Vario"},
            {0x08, "Battery"}, {0x09, "Barometer"}, {0x0a, "Airspeed"}, {0x0b, "Heartbeat"},
            {0x0c, "RPM"}, {0x0d, "Temperature"}, {0x0e, "Voltages"}, {0x10, "VTX Telemetry"},
            {0x1c, "Link RX"}, {0x1d, "Link TX"}, {0x1e, "Attitude"}, {0x1f, "MAVLink FC"},
            {0x21, "Flight Mode"}, {0x22, "ESP Now"}
        }};
        EEProm& mConfig;
        std::array<char, 32> mName{};
        std::array<char, 16> mResetReason{};
        etl::FixedVector<Parameter, 36> mEntries;
    };
}
