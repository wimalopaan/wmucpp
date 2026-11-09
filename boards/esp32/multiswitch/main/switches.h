#pragma once

#include "patterns.h"
#include "protocol.h"

namespace Multiswitch {
    class Switches {
    public:
        Switches(EEProm& config, std::array<Output, 8>& outputs) : mConfig{config}, mOutputs{outputs} {
            reset();
        }

        void reset() {
            for (uint8_t i = 0; i < mPatterns.size(); ++i) configurePattern(i);
            resetDimming();
            mSlaveSending = false;
        }

        void configurePattern(const uint8_t index) {
            mPatterns[index].configure(mConfig.pattern[index], mOutputs);
            mPendingStops &= ~(1 << index);
        }

        void resetDimming() {
            for (auto& output : mOutputs) output.dutyGlobal(100);
        }

        void set(const uint8_t address, const uint8_t index, const bool on, const uint64_t now) {
            if (index >= 8) return;
            if (address == EEProm::AdrIndex::Switch) {
                if ((!mConfig.use_virtuals || !virtualMember(index)) &&
                    !patternMember(index) && !mConfig.outputs[index].slaveEnable) {
                    mOutputs[index].set(on);
                }
            }
            else if (address == EEProm::AdrIndex::Virtual) {
                if (index < mConfig.virtuals.size()) {
                    if (!mConfig.use_virtuals) return;
                    for (const uint8_t member : mConfig.virtuals[index].member) {
                        if (member > 0 && member <= mOutputs.size() && !patternMember(member - 1) &&
                            !mConfig.outputs[member - 1].slaveEnable) {
                            mOutputs[member - 1].set(on);
                        }
                    }
                }
                else if (on) startPattern(index - mConfig.virtuals.size(), now);
                else stopPattern(index - mConfig.virtuals.size());
            }
        }

        bool slaveSet(const uint8_t master, const uint8_t state) {
            if (master != mConfig.master) return false;
            for (size_t i = 0; i < mOutputs.size(); ++i) {
                if (mConfig.outputs[i].slaveEnable) mOutputs[i].set((state & (1 << i)) != 0);
            }
            return true;
        }

        void duty(const uint8_t address, const uint8_t index, const uint8_t value) {
            if (address == EEProm::AdrIndex::Switch && index < mOutputs.size()) {
                mOutputs[index].duty(value);
            }
            else if (address == EEProm::AdrIndex::Virtual && index == 4 &&
                     mConfig.use_virtuals && mConfig.enableGlobalDimming_virtual4) {
                for (auto& output : mOutputs) output.dutyGlobal(value);
            }
        }

        void startPattern(const uint8_t index, const uint64_t now, const bool chain = false) {
            if (index < mPatterns.size()) mPatterns[index].start(now, mOutputs, chain);
        }

        void stopPattern(const uint8_t index, const bool propagate = true) {
            if (index >= mPatterns.size()) return;
            mPatterns[index].stop(mOutputs);
            if (propagate && mConfig.pattern[index].type) mPendingStops |= 1 << index;
            else mPendingStops &= ~(1 << index);
        }

        void stopGroup(const uint8_t group) {
            if (!group) return;
            for (uint8_t i = 0; i < mPatterns.size(); ++i) {
                if (mConfig.pattern[i].group == group) stopPattern(i, false);
            }
        }

        void failsafe() {
            if (!mConfig.failsafe_mode) return;
            for (uint8_t i = 0; i < mPatterns.size(); ++i) stopPattern(i, false);
            for (size_t i = 0; i < mOutputs.size(); ++i) {
                mOutputs[i].failsafe(mConfig.failsafe_mode == 2 && mConfig.failsafe_pattern[i] != 0);
            }
        }

        template<typename Baud>
        bool command(const std::span<const uint8_t> frame, const uint64_t now, Baud baud) {
            return Multiswitch::command(frame, mConfig.addresses,
                [&](const uint8_t address, const uint8_t index, const bool on) { set(address, index, on, now); },
                [&](const uint8_t address, const uint8_t index, const uint8_t value) { duty(address, index, value); },
                baud,
                [&](const uint8_t address, const uint8_t pattern) {
                    if (address == EEProm::AdrIndex::Virtual) startPattern(pattern - 4, now, true);
                },
                [&](const uint8_t group) { stopGroup(group); },
                [&](const uint8_t master, const uint8_t state) { return slaveSet(master, state); });
        }

        template<typename Send>
        std::array<uint8_t, 8> periodic(const uint64_t now, Send send) {
            for (uint8_t i = 0; i < mPatterns.size(); ++i) {
                const auto& config = mConfig.pattern[i];
                const bool local = std::find(mConfig.addresses.begin(), mConfig.addresses.end(),
                                             config.next_address) != mConfig.addresses.end();
                if ((mPendingStops & (1 << i)) && config.group && !local) {
                    const std::array<uint8_t, 5> payload{
                        uint8_t(Crsf::Address::Broadcast), mConfig.crsf_address,
                        uint8_t(Crsf::CommandType::Switch), uint8_t(Crsf::SwitchCommand::InterModulePatternStopAll),
                        config.group};
                    send(std::span<const uint8_t>{payload});
                }
                if (mPatterns[i].periodic(now, mOutputs) && !local) {
                    const std::array<uint8_t, 6> payload{
                        uint8_t(Crsf::Address::Broadcast), mConfig.crsf_address,
                        uint8_t(Crsf::CommandType::Switch), uint8_t(Crsf::SwitchCommand::InterModulePatternStart),
                        config.next_address, static_cast<uint8_t>(i + 4)};
                    send(std::span<const uint8_t>{payload});
                }
            }
            mPendingStops = 0;
            std::array<uint8_t, 8> duties{};
            uint8_t state = 0;
            for (size_t i = 0; i < mOutputs.size(); ++i) {
                duties[i] = mOutputs[i].value(now, mConfig);
                if (duties[i]) state |= 1 << i;
            }
            const std::array<uint8_t, 6> payload{
                uint8_t(Crsf::Address::Broadcast), mConfig.crsf_address,
                uint8_t(Crsf::CommandType::Switch), uint8_t(Crsf::SwitchCommand::InterModuleSlaveSet),
                mConfig.addresses[0], state};
            if (mConfig.slaveSend && (!mSlaveSending || payload != mSlavePayload)) {
                send(std::span<const uint8_t>{payload});
                mSlavePayload = payload;
            }
            mSlaveSending = mConfig.slaveSend != 0;
            return duties;
        }

    private:
        bool virtualMember(const uint8_t index) const {
            for (const auto& group : mConfig.virtuals) {
                for (const uint8_t member : group.member) if (member == index + 1) return true;
            }
            return false;
        }

        bool patternMember(const uint8_t index) const {
            for (const auto& pattern : mConfig.pattern) {
                if (pattern.type) {
                    for (const uint8_t member : pattern.member) if (member == index + 1) return true;
                }
            }
            return false;
        }

        EEProm& mConfig;
        std::array<Output, 8>& mOutputs;
        std::array<Pattern, 4> mPatterns;
        uint8_t mPendingStops = 0;
        std::array<uint8_t, 6> mSlavePayload{};
        bool mSlaveSending = false;
    };
}
