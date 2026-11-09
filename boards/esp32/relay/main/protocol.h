// SPDX-License-Identifier: GPL-3.0-or-later
// CRSF framing adapted from ../multiswitch/main/protocol.h.
#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <span>

#include "etl/fixedvector.h"
#include "etl/ranged.h"
#include "rc/rc_2.h"

namespace Relay {
    namespace Crsf = RC::Protokoll::Crsf::V4;

    template<size_t Capacity>
    class ByteBuffer {
    public:
        using value_type = uint8_t;
        void push_back(const uint8_t value) { mData.push_back(value); }
        void push_back(const std::byte value) { push_back(uint8_t(value)); }
        auto begin() const { return mData.begin(); }
        auto size() const { return mData.size(); }
        auto operator[](const size_t index) const { return mData[index]; }
        std::span<const uint8_t> bytes() const { return {begin(), size()}; }
    private:
        etl::FixedVector<uint8_t, Capacity> mData;
    };

    using Frame = ByteBuffer<Crsf::maxMessageSize>;

    inline void recalculate(const std::span<uint8_t> frame) {
        CRC8 crc;
        for (size_t i = 2; i < frame.size() - 1; ++i) crc += frame[i];
        frame.back() = crc;
    }

    inline Frame makeFrame(const std::byte type, const std::span<const uint8_t> payload) {
        assert(payload.size() <= Crsf::maxMessageSize - 4);
        Frame frame;
        frame.push_back(Crsf::Address::StartByte);
        frame.push_back(uint8_t(payload.size() + 2));
        frame.push_back(type);
        CRC8 crc;
        crc += type;
        for (const auto b : payload) {
            frame.push_back(b);
            crc += b;
        }
        frame.push_back(uint8_t(crc));
        return frame;
    }

    class Parser {
    public:
        void reset() { mSize = 0; }

        template<typename Callback>
        void process(const uint8_t byte, const uint64_t now, Callback&& callback) {
            if (mSize && now - mLastByte > 5'000) reset();
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
                if (mSize < size) return;
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
}
