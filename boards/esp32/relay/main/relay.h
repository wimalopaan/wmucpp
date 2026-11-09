// SPDX-License-Identifier: GPL-3.0-or-later
// Relay processing adapted from ../../rcRelay32/gfsm.h and
// include_stm32/rc/package_relay_rewrite.h, Copyright (C) Wilhelm Meier.
#pragma once

#include "parameters.h"

namespace Relay {
    template<typename Transport>
    class Controller {
    public:
        enum class State {Starting, Init, Disconnected, Connected};

        Controller(EEProm& config, Transport& transport) :
            mConfig{config}, mTransport{transport}, mParameters{config} {}

        void start(const uint64_t now) {
            mStarted = now;
            mNextCheck = now + 300'000;
            mNextChannels = now + 10'000;
        }

        void input(const std::span<const uint8_t> bytes, const uint64_t now) {
            for (const auto b : bytes) {
                mInputParser.process(b, now, [this, now](const auto frame) { fromInput(frame, now); });
            }
        }

        void module(const std::span<const uint8_t> bytes, const uint64_t now) {
            for (const auto b : bytes) {
                mModuleParser.process(b, now, [this, now](const auto frame) { fromModule(frame, now); });
            }
        }

        void resetParser(const bool module) {
            if (module) mModuleParser.reset();
            else mInputParser.reset();
        }

        void moduleFrameError() { mFrameError = true; }
        State state() const { return mState; }
        bool enabled() const { return mEnabled; }
        Parameters& parameters() { return mParameters; }

        void periodic(const uint64_t now) {
            if (mState == State::Starting && now - mStarted >= 500'000) mState = State::Init;
            if (now >= mNextCheck) {
                mNextCheck = now + 300'000;
                if (mState != State::Starting) {
                    if (mChannelsReceived) {
                        mState = State::Connected;
                        mEnabled = true;
                    }
                    else {
                        mState = State::Disconnected;
                        if (!mConfig.failsafe_mode) mEnabled = false;
                    }
                }
                mChannelsReceived = false;
                if (mFrameError) {
                    ByteBuffer<9> payload;
                    payload.push_back(Crsf::Address::Broadcast);
                    payload.push_back(Crsf::Address::RX);
                    payload.push_back(Crsf::CommandType::general);
                    payload.push_back(Crsf::GeneralCommand::speed);
                    payload.push_back(uint8_t{0});
                    etl::serializeBE(Crsf::baudrateHandset, payload);
                    sendModule(Crsf::Type::Command, payload.bytes());
                    mFrameError = false;
                }
            }
            if (now >= mNextChannels) {
                mNextChannels = now + 10'000;
                if (mEnabled) sendModule(Crsf::Type::Channels, mChannelPayload);
            }
            if (mConfig.parameter_retry && mEnabled) retry(now);
        }

    private:
        void sendInput(const std::byte type, const std::span<const uint8_t> payload) {
            const auto frame = makeFrame(type, payload);
            mTransport.input(frame.bytes());
        }

        void sendModule(const std::byte type, const std::span<const uint8_t> payload) {
            const auto frame = makeFrame(type, payload);
            mTransport.module(frame.bytes());
        }

        void suppressTelemetry(const uint64_t now) {
            mTelemetryOffUntil = now + uint64_t{mConfig.tempOffTelemetry} * 1'000'000;
        }

        void deviceInfo(const uint8_t destination) {
            ByteBuffer<60> payload;
            payload.push_back(destination);
            payload.push_back(mConfig.address);
            etl::push_back_ntbs(mParameters.name(), payload);
            etl::serializeBE(uint32_t{1234}, payload);
            etl::serializeBE(Board::hardwareVersion, payload);
            etl::serializeBE(Board::softwareVersion, payload);
            payload.push_back(uint8_t(mParameters.size() - 1));
            payload.push_back(uint8_t{0});
            sendInput(Crsf::Type::Info, payload.bytes());
        }

        void fromInput(const std::span<const uint8_t> frame, const uint64_t now) {
            const auto type = std::byte{frame[2]};
            if (type == Crsf::Type::Channels) {
                std::copy_n(frame.begin() + 3, mChannelPayload.size(), mChannelPayload.begin());
                mChannelsReceived = true;
                return;
            }
            if (type == Crsf::Type::Link) return;
            if (type == Crsf::Type::Ping) {
                suppressTelemetry(now);
                if (frame[3] == mConfig.address || frame[3] == uint8_t(Crsf::Address::Broadcast)) {
                    deviceInfo(frame[4]);
                }
                forward(frame, now);
            }
            else if (type == Crsf::Type::ParamRead) {
                suppressTelemetry(now);
                if (frame[3] != mConfig.address) {
                    forward(frame, now);
                    return;
                }
                ByteBuffer<58> payload;
                const auto result = mParameters.read(frame[4], frame[5], frame[6], payload);
                if (result != Parameters::ReadResult::Ok) {
                    mTransport.error("Invalid parameter index/chunk");
                    return;
                }
                sendInput(Crsf::Type::ParamEntry, payload.bytes());
            }
            else if (type == Crsf::Type::ParamWrite) {
                suppressTelemetry(now);
                if (frame[3] != mConfig.address) {
                    forward(frame, now);
                    return;
                }
                const auto oldBaud = mConfig.baudrate;
                const auto oldDuplex = mConfig.half_duplex;
                const auto oldRetry = mConfig.parameter_retry;
                if (!mParameters.write(frame[5], frame.subspan(6, frame.size() - 7))) {
                    mTransport.error("Invalid parameter write");
                    return;
                }
                if (oldBaud != mConfig.baudrate || oldDuplex != mConfig.half_duplex) {
                    mTransport.configure();
                    mModuleParser.reset();
                    mPending = {};
                }
                if (oldRetry != mConfig.parameter_retry) mPending = {};
                mTransport.changed(now);
            }
            else if (type == Crsf::Type::Command) {
                forward(frame, now);
            }
        }

        void forward(const std::span<const uint8_t> frame, const uint64_t now) {
            if (!mEnabled) return;
            std::array<uint8_t, Crsf::maxMessageSize> copy{};
            std::copy(frame.begin(), frame.end(), copy.begin());
            auto data = std::span{copy}.first(frame.size());
            if (data[3] == mConfig.tx_rewrite_address) data[3] = uint8_t(Crsf::Address::TX);
            else if (data[3] == mConfig.rx_rewrite_address) data[3] = uint8_t(Crsf::Address::RX);
            recalculate(data);
            if (mConfig.parameter_retry) track(data, now);
            mTransport.module(data);
        }

        void fromModule(const std::span<const uint8_t> frame, const uint64_t now) {
            const auto type = std::byte{frame[2]};
            if (type == Crsf::Type::ArduPilot) {
                if (now < mTelemetryOffUntil || !mConfig.ardupilot_mode) return;
                if (shouldSendArduPilot(frame)) mTransport.input(frame);
            }
            else if (frame[2] >= uint8_t(Crsf::Type::Ping)) {
                if (type == Crsf::Type::RadioID) return;
                if (mConfig.parameter_retry && !answer(frame, now)) return;
                std::array<uint8_t, Crsf::maxMessageSize> copy{};
                std::copy(frame.begin(), frame.end(), copy.begin());
                auto data = std::span{copy}.first(frame.size());
                if (data[4] == uint8_t(Crsf::Address::TX)) {
                    data[4] = mConfig.tx_rewrite_address;
                    if (type == Crsf::Type::Info && mConfig.rewrite_name) rewriteName(data);
                }
                else if (data[4] == uint8_t(Crsf::Address::RX)) data[4] = mConfig.rx_rewrite_address;
                if (data[3] == uint8_t(Crsf::Address::TX)) data[3] = uint8_t(Crsf::Address::Handset);
                data[0] = uint8_t(Crsf::Address::StartByte);
                recalculate(data);
                mTransport.input(data);
            }
            else if (now >= mTelemetryOffUntil) {
                const uint8_t mode = type == Crsf::Type::Link ?
                    mConfig.link_stat_mode : mConfig.telemetryMode[frame[2]];
                if (!mode || !due(mTypeCounter[frame[2]])) return;
                if (type == Crsf::Type::Link && mode == 2) {
                    // Same provisional LinkStats mapping as the STM32 implementation.
                    const std::array<uint8_t, 5> rx{frame[10], frame[11], frame[11], frame[12], frame[9]};
                    const std::array<uint8_t, 6> tx{std::min(frame[3], frame[4]), frame[5],
                                                  frame[5], frame[6], 0, frame[8]};
                    sendInput(Crsf::Type::LinkStatsRx, rx);
                    sendInput(Crsf::Type::LinkStatsTx, tx);
                }
                else if ((type == Crsf::Type::Link && mode == 3) ||
                         (type != Crsf::Type::Link && mode == 2)) {
                    tunnel(frame, type == Crsf::Type::Link ?
                        Crsf::PassThru::SubType::LinkStat : Crsf::PassThru::SubType::Telemetry);
                }
                else {
                    mTransport.input(frame);
                }
            }
        }

        bool due(uint8_t& counter) {
            if (++counter < mConfig.telemetry_rate) return false;
            counter = 0;
            return true;
        }

        bool shouldSendArduPilot(const std::span<const uint8_t> frame) {
            size_t index = 17;
            if (frame[3] == 0xf1) return true;
            if (frame[3] == 0xf0 && frame.size() >= 7) index = frame[4] & 0x0f;
            else if (frame[3] == 0xf2) index = 16;
            return due(mArduPilotCounter[index]);
        }

        void tunnel(const std::span<const uint8_t> frame, const std::byte subtype) {
            if (frame.size() > Crsf::maxMessageSize - 6) {
                mTransport.error("Telemetry packet too large for tunnel");
                return;
            }
            ByteBuffer<60> payload;
            payload.push_back(Crsf::Address::Handset);
            payload.push_back(mConfig.address);
            etl::serializeBE(Crsf::PassThru::AppId::Telem, payload);
            payload.push_back(subtype);
            for (size_t i = 2; i < frame.size() - 1; ++i) payload.push_back(frame[i]);
            sendInput(Crsf::Type::PassThru, payload.bytes());
        }

        void rewriteName(const std::span<uint8_t> frame) {
            bool end = false;
            for (size_t i = 0; i < 16 && i + 20 < frame.size(); ++i) {
                auto& b = frame[5 + i];
                if (!b) break;
                if (!end && i < mConfig.txname.size() && mConfig.txname[i]) {
                    b = uint8_t(mConfig.txname[i]);
                }
                else {
                    end = true;
                    b = '.';
                }
            }
        }

        struct Pending {
            std::array<uint8_t, 8> request{};
            uint64_t sentAt = 0;
            uint64_t lastUse = 0;
            uint64_t duplicateUntil = 0;
            uint64_t rtt = 0;
            uint8_t device = 0;
            uint8_t index = 0;
            uint8_t retries = 0;
            uint8_t duplicateIndex = 0;
            uint8_t duplicateRemaining = 0;
            uint8_t duplicates = 0;
            bool used = false;
            bool active = false;
        };

        Pending* find(const uint8_t device) {
            for (auto& p : mPending) if (p.used && p.device == device) return &p;
            return nullptr;
        }

        void track(const std::span<const uint8_t> frame, const uint64_t now) {
            if (frame[3] == uint8_t(Crsf::Address::Broadcast)) return;
            if (std::byte{frame[2]} == Crsf::Type::ParamRead && frame.size() == 8) {
                auto* p = find(frame[3]);
                if (!p) {
                    p = &mPending[0];
                    for (auto& candidate : mPending) {
                        if (!candidate.used) {
                            p = &candidate;
                            break;
                        }
                        if (candidate.lastUse < p->lastUse) p = &candidate;
                    }
                    *p = Pending{};
                    p->device = frame[3];
                    p->used = true;
                }
                std::copy(frame.begin(), frame.end(), p->request.begin());
                p->index = frame[5];
                p->sentAt = p->lastUse = now;
                p->retries = 0;
                p->duplicates = 0;
                p->active = true;
            }
            else if (std::byte{frame[2]} == Crsf::Type::ParamWrite) {
                if (auto* p = find(frame[3])) p->active = false;
            }
        }

        void retry(const uint64_t now) {
            for (auto& p : mPending) {
                if (!p.used || !p.active) continue;
                const bool local = p.device == uint8_t(Crsf::Address::TX);
                const uint64_t rtt = p.rtt ? p.rtt : local ? 20'000 : 150'000;
                const uint64_t timeout = std::clamp<uint64_t>(rtt * 3, local ? 100'000 : 500'000, 1'500'000);
                if (now - p.sentAt < timeout) continue;
                if (p.retries >= 3) {
                    p.active = false;
                    mTransport.error("Parameter read retry limit reached");
                    continue;
                }
                ++p.retries;
                p.sentAt = now;
                mTransport.module(p.request);
            }
        }

        bool answer(const std::span<const uint8_t> frame, const uint64_t now) {
            if (std::byte{frame[2]} != Crsf::Type::ParamEntry || frame.size() < 8) return true;
            auto* p = find(frame[4]);
            if (!p || frame[3] != p->request[4]) return true;
            if (p->duplicates && frame[5] == p->duplicateIndex &&
                frame[6] == p->duplicateRemaining && now < p->duplicateUntil) {
                --p->duplicates;
                return false;
            }
            if (p->active && frame[5] == p->index) {
                if (!p->retries) {
                    const uint64_t rtt = now - p->sentAt;
                    p->rtt = p->rtt ? (p->rtt * 3 + rtt) / 4 : rtt;
                }
                else {
                    p->duplicateIndex = frame[5];
                    p->duplicateRemaining = frame[6];
                    p->duplicates = p->retries;
                    p->duplicateUntil = now + 3'000'000;
                }
                p->active = false;
            }
            return true;
        }

        EEProm& mConfig;
        Transport& mTransport;
        Parameters mParameters;
        Parser mInputParser;
        Parser mModuleParser;
        std::array<uint8_t, 22> mChannelPayload{};
        std::array<uint8_t, uint8_t(Crsf::Type::Ping)> mTypeCounter{};
        std::array<uint8_t, 18> mArduPilotCounter{};
        std::array<Pending, 4> mPending{};
        uint64_t mStarted = 0;
        uint64_t mNextCheck = 0;
        uint64_t mNextChannels = 0;
        uint64_t mTelemetryOffUntil = 0;
        State mState = State::Starting;
        bool mEnabled = false;
        bool mChannelsReceived = false;
        bool mFrameError = false;
    };
}
