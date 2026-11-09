#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <span>

#include "etl/fixedvector.h"
#include "etl/ranged.h"
#include "rc/rc_2.h"

namespace Multiswitch {
    namespace Crsf = RC::Protokoll::Crsf::V4;

    template<size_t Capacity>
    class ByteBuffer {
    public:
        using value_type = uint8_t;
        void push_back(const uint8_t value) { mData.push_back(value); }
        void push_back(const std::byte value) {
            push_back(uint8_t(value));
        }
        auto begin() const { return mData.begin(); }
        auto size() const { return mData.size(); }
        auto operator[](const size_t index) const { return mData[index]; }
    private:
        etl::FixedVector<uint8_t, Capacity> mData;
    };

    class Parser {
    public:
        void reset() {
            mSize = 0;
        }

        template<typename Callback>
        void process(const uint8_t byte, const uint64_t now, Callback&& callback) {
            if (mSize && now - mLastByte > 5'000) {
                reset();
            }
            mLastByte = now;
            mData[mSize++] = byte;
            while (mSize >= 2) {
                if (mData[0] != uint8_t(Crsf::Address::StartByte) &&
                    mData[0] != uint8_t(Crsf::Address::Handset) &&
                    mData[0] != uint8_t(Crsf::Address::RX)) {
                    discard();
                    continue;
                }
                const size_t size = size_t{mData[1]} + 2;
                if (size < 4 || size > mData.size()) {
                    discard();
                    continue;
                }
                if (mSize < size) {
                    return;
                }
                const std::span<const uint8_t> frame{mData.data(), size};
                if (Crsf::Packet::hasValidPayload(frame) && Crsf::Packet::crcCheck(frame)) {
                    const auto complete = mData;
                    mSize -= size;
                    std::move(mData.begin() + size, mData.begin() + size + mSize, mData.begin());
                    callback(std::span<const uint8_t>{complete.data(), size});
                }
                else {
                    discard();
                }
            }
        }

    private:
        void discard() {
            --mSize;
            std::move(mData.begin() + 1, mData.begin() + 1 + mSize, mData.begin());
        }

        std::array<uint8_t, Crsf::maxMessageSize> mData{};
        size_t mSize = 0;
        uint64_t mLastByte = 0;
    };

    template<typename Set, typename Duty, typename Baud>
    bool command(const std::span<const uint8_t> frame, const uint8_t address,
                 Set set, Duty duty, Baud baud) {
        if (frame.size() < 9) return false;
        const auto source = std::byte{frame[4]};
        const auto realm = std::byte{frame[5]};
        const auto cmd = std::byte{frame[6]};
        if (source == Crsf::Address::RX && realm == Crsf::CommandType::general &&
            cmd == Crsf::GeneralCommand::speed) {
            if (frame.size() < 13) return false;
            const uint32_t speed = (uint32_t{frame[8]} << 24) | (uint32_t{frame[9]} << 16) |
                                   (uint32_t{frame[10]} << 8) | frame[11];
            if (std::find(Crsf::baudrates.begin(), Crsf::baudrates.end(), speed) == Crsf::baudrates.end()) {
                return false;
            }
            baud(speed);
            return true;
        }
        if (source != Crsf::Address::Handset || frame[3] < 0xc0 || frame[3] > 0xcf ||
            realm != Crsf::CommandType::Switch) {
            return false;
        }
        if (cmd == Crsf::SwitchCommand::Set4M) {
            const size_t count = frame[7];
            if (frame.size() < 9 + 3 * count) return false;
            for (size_t i = 0; i < count; ++i) {
                if (frame[8 + 3 * i] == address) {
                    const uint16_t bits = (uint16_t{frame[9 + 3 * i]} << 8) | frame[10 + 3 * i];
                    for (uint8_t k = 0; k < 8; ++k) set(k, ((bits >> (2 * k)) & 3) != 0);
                }
            }
            return true;
        }
        if (frame[7] != address) return false;
        if (cmd == Crsf::SwitchCommand::Set) {
            if (frame.size() < 10) return false;
            for (uint8_t k = 0; k < 8; ++k) set(k, (frame[8] & (1 << k)) != 0);
            return true;
        }
        if (cmd == Crsf::SwitchCommand::Set4 || cmd == Crsf::SwitchCommand::Prop) {
            if (frame.size() < 11) return false;
            if (cmd == Crsf::SwitchCommand::Prop) {
                if (frame[8] >= 8) return false;
                duty(frame[8], std::min(frame[9], uint8_t{100}));
            }
            else {
                const uint16_t bits = (uint16_t{frame[8]} << 8) | frame[9];
                for (uint8_t k = 0; k < 8; ++k) set(k, ((bits >> (2 * k)) & 3) != 0);
            }
            return true;
        }
        return false;
    }
}
