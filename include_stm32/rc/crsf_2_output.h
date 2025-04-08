/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
#include <limits>
#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <numbers>
#include <chrono>
#include <cstring>
#include <type_traits>

#include "etl/fifo.h"

#include "rc_2.h"

namespace RC {
    namespace Protokoll {
        namespace Crsf {
            namespace V4 {
                using namespace std::literals::chrono_literals;
                using namespace etl::literals;
                template<auto Size, auto ChunkSize, auto AdditionalBytesPerChunk = 4>
                struct ChunkBuffer {
                    using value_type = uint8_t;
                    using size_type = uint16_t;
                    inline void clear() {
                        in = 0;
                    }
                    inline uint8_t chunks() const {
                        return (size() + ChunkSize - AdditionalBytesPerChunk - 1) / (ChunkSize - AdditionalBytesPerChunk);
                    }
                    inline size_type size() const {
                        return in;
                    }
                    inline void push_back(const std::byte item) {
                        push_back((uint8_t)item);
                    }
                    inline void push_back(const uint8_t item) {
                        size_type next = std::min(in + 1, Size - 1);
                        data[in] = item;
                        in = next;
                    }
                    template<typename C>
                    inline int serializeChunk(C& c, const uint8_t n) {
                        const uint16_t start = n * (ChunkSize - AdditionalBytesPerChunk);
                        const int len = std::min((int)size() - start, (int)(ChunkSize - AdditionalBytesPerChunk));
                        for(int i = 0; i < len; ++i) {
                            c.push_back(data[start + i]);
                        }
                        return len;
                    }
                    private:
                    size_type in{0};
                    std::array<value_type, Size> data{};
                };

                template<typename Master>
                struct Output {
                    using messageBuffer = Master::messageBuffer;
                    using systemTimer = Master::systemTimer;
                    using debug = Master::debug;
                    using callback = Master::callback;

                    static inline constexpr uint8_t chunkSize = 58;

                    enum class State : uint8_t {Idle, SendDeviceInfo};
                    enum class Event : uint8_t {None, SendDeviceInfo};

                    static inline void enableReply(const bool b) {
                        mEnableReply = b;
                    }
                    static inline void event(const Event e) {
                        mEvent = e;
                    }
                    static inline void periodic() {
                        switch(mState) {
                        case State::Idle:
                            if (mEnableReply) {
                                mEvent.on(Event::SendDeviceInfo, []{
                                    mState = State::SendDeviceInfo;
                                });
                            }
                            break;
                        case State::SendDeviceInfo:
                            if (mSlotNumber == mActualSlot) {
                                sendDeviceInfo();
                                mActualSlot = 0;
                                mState = State::Idle;
                            }
                            break;
                        }
                    }
                    static inline void ratePeriodic() {
                    }
                    static inline void nextSlot() {
                        ++mActualSlot;
                    }
                    static inline void resetSlot() {
                        mActualSlot = 0;
                    }
                    static inline void setDestination(const std::byte d) {
                        mDest = (uint8_t)d;
                    }
                    static inline void telemetrySlot(const uint8_t s) {
                        mSlotNumber = s;
                    }
                    static inline void sendRadioID() {
                        messageBuffer::create_back((uint8_t)RC::Protokoll::Crsf::V4::Type::RadioID, [&](auto& d){
                            d.push_back(mDest);
                            d.push_back(mSrc);
                            d.push_back(uint8_t{0x10});
                            uint32_t pint = 20 * 10'000; //20ms
                            etl::serializeBE(pint, d);
                            uint32_t pshift= 30030;
                            etl::serializeBE(pshift, d);
                        });
                    }
                    static inline void sendCommandResponse(const uint8_t index, const uint8_t step) {
                        IO::outl<debug>("# CR adr: ", mDest, " src: ", mSrc, " i: ", index, " st: ", step);
                        messageBuffer::create_back((uint8_t)RC::Protokoll::Crsf::V4::Type::ParamEntry, [&](auto& d){
                            d.push_back(mDest);
                            d.push_back(mSrc);
                            d.push_back(index);
                            d.push_back((uint8_t)0); // no chunks follow
                            // callback::parameter(index).serialize(d, Lua::CmdStep(step));
                            callback::serialize(index, d, Lua::CmdStep(step));
                        });
                    }
                    static inline void sendParameterInfo(const uint8_t index, const uint8_t chunk) {
                        IO::outl<debug>("# PI adr: ", mDest, " src: ", mSrc, " i: ", index, " c: ", chunk, " s: ", callback::parameter(index).size());
                        if (chunk == 0) {
                            if (const uint16_t psize = (callback::parameter(index).size() + 4); psize <= chunkSize) {
                                messageBuffer::create_back((uint8_t)RC::Protokoll::Crsf::V4::Type::ParamEntry, [&](auto& d){
                                    d.push_back(mDest);
                                    d.push_back(mSrc);
                                    d.push_back(index);
                                    d.push_back((uint8_t)0); // no chunks follow
                                    // callback::parameter(index).serialize(d);
                                    callback::serialize(index, d);
                                });
                            }
                            else {
                                messageBuffer::create_back((uint8_t)RC::Protokoll::Crsf::V4::Type::ParamEntry, [&](auto& d){
                                    mChunkBuffer.clear();
                                    // callback::parameter(index).serialize(mChunkBuffer);
                                    callback::serialize(index, mChunkBuffer);
                                    uint16_t s = mChunkBuffer.size();
                                    const uint8_t chunksToFollow = mChunkBuffer.chunks() - 1;
                                    d.push_back(mDest);
                                    d.push_back(mSrc);
                                    d.push_back(index);
                                    d.push_back(chunksToFollow);
                                    // mChunkBuffer.serializeChunk(d, 0);
                                    int l = mChunkBuffer.serializeChunk(d, 0);
                                    IO::outl<debug>("# A p: ", index, " c: ", chunk, " cf: ", chunksToFollow, " l: ", l, " s: ", s);;
                                });
                            }
                        }
                        else {
                            messageBuffer::create_back((uint8_t)RC::Protokoll::Crsf::V4::Type::ParamEntry, [&](auto& d){
                                const uint8_t chunksToFollow = mChunkBuffer.chunks() - 1 - chunk;
                                d.push_back(mDest);
                                d.push_back(mSrc);
                                d.push_back(index);
                                d.push_back(chunksToFollow);
                                mChunkBuffer.serializeChunk(d, chunk);
                                // int l = mChunkBuffer.serializeChunk(d, chunk);
                                // IO::outl<debug>("# B p: ", index, " c: ", chunk, " cf: ", chunksToFollow, " l: ", l);;
                            });
                        }
                    }
                    private:
                    static inline void sendDeviceInfo() {
                        IO::outl<debug>("# DI adr: ", mDest, " src: ", mSrc);
                        messageBuffer::create_back((uint8_t)RC::Protokoll::Crsf::V4::Type::Info, [](auto& d){
                            d.push_back(mDest);
                            d.push_back(mSrc);
                            etl::push_back_ntbs_or_emptyString(callback::name(), d);
                            etl::serializeBE(callback::serialNumber(), d);
                            etl::serializeBE(callback::hwVersion(), d);
                            etl::serializeBE(callback::swVersion(), d);
                            etl::serializeBE(callback::numberOfParameters(), d);
                            etl::serializeBE(callback::protocolVersion(), d);
                        });
                    }
                    static inline ChunkBuffer<Master::chunkBufferSize, chunkSize, 4> mChunkBuffer;
                    static inline const auto& mSrc = Master::mAddress;
                    static inline External::Tick<systemTimer> mStateTick;
                    static inline State mState = State::Idle;
                    static inline etl::Event<Event> mEvent;
                    static inline uint8_t mDest = 0;
                    static inline uint8_t mSlotNumber = 0;
                    static inline uint8_t mActualSlot = 0;
                    static inline uint8_t mEnableReply = true;
                };
            }
        }
    }
}




