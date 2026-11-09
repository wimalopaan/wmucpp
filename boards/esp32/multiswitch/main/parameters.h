#pragma once

#include <cstdio>
#include <span>

#include "etl/fixedvector.h"
#include "board.h"
#include "outputs.h"
#include "protocol.h"

namespace Multiswitch {
    class Parameters {
    public:
        using Parameter = Crsf::Parameter<uint8_t>;
        using Type = Parameter::Type;
        enum class Action {Store, Output, Test, Reset};
        enum class Result {Invalid, Changed, Transient, Reset};
        enum class ReadResult {Ok, InvalidIndex, InvalidChunk};
        struct Entry {
            Parameter parameter;
            Action action = Action::Store;
            uint8_t output = 0;
        };

        explicit Parameters(EEProm& config, std::array<Output, 8>& outputs) :
            mConfig{config}, mOutputs{outputs} {
            updateName();
            add({0, Type::Folder, ""});
            add({0, Type::Info, "Version(HW/SW)", "13:42"});
            const auto global = folder(0, "Global");
            add({global, Type::U8, "Switch Addr", nullptr, &config.addresses[0], 0, 255});
            add({global, Type::U8, "PWM f (O0..7)", nullptr, &config.pwm1, 1, 10,
                 nullptr, 10, 0, 1, nullptr, "*100Hz"});
            add({global, Type::U8, "CRSF Addr", nullptr, &config.crsf_address, 0xc0, 0xcf});
            add({global, Type::Sel, "Config resp.", "Button;Always on", &config.telemetry, 0, 1});
            for (uint8_t i = 0; i < outputs.size(); ++i) {
                const auto parent = folder(0, names[i]);
                auto& out = config.outputs[i];
                add({parent, Type::Sel, "PWM Mode", "Off;On;Remote;Global/Indiv", &out.pwm, 0, 3}, Action::Output, i);
                add({parent, Type::U8, "PWM Duty", nullptr, &out.pwmDuty, 1, 99,
                     nullptr, 1, 0, 1, nullptr, "%"}, Action::Output, i);
                add({parent, Type::U8, "PWM Expo", nullptr, &out.pwmScale, 0, 100}, Action::Output, i);
                add({parent, Type::Sel, "Intervall Mode", "Off;On;Morse", &out.blink, 0, 2}, Action::Output, i);
                add({parent, Type::U8, "Intervall(on)", nullptr, &out.blinkOnTime, 1, 255,
                     nullptr, 1, 0, 1, nullptr, "*50ms"}, Action::Output, i);
                add({parent, Type::U8, "Intervall(off)", nullptr, &out.blinkOffTime, 1, 255,
                     nullptr, 2, 0, 2, nullptr, "*50ms"}, Action::Output, i);
                add({parent, Type::U8, "Intervall(count)", nullptr, &out.flashCount, 1, 4}, Action::Output, i);
                add({parent, Type::Sel, "Test", "Off;On", nullptr, 0, 1}, Action::Test, i);
            }
            const auto failsafe = folder(0, "Failsafe");
            add({failsafe, Type::Sel, "Mode", "Hold;All-Off;Set", &config.failsafe_mode, 0, 2});
            for (uint8_t i = 0; i < outputs.size(); ++i) {
                add({failsafe, Type::Sel, names[i], "Off;On", &config.failsafe_pattern[i], 0, 1});
            }
            const auto morse = folder(0, "Morse");
            add({morse, Type::Str, "Text1", nullptr, nullptr, 0, 0, nullptr, 0,
                 static_cast<uint8_t>(config.morse_texts[0].size()), 0, config.morse_texts[0].data()});
            add({morse, Type::U8, "Dit duration", nullptr, &config.morse_dit, 1, 10, nullptr, 3, 0, 3, nullptr, "*100ms"});
            add({morse, Type::U8, "Dah duration", nullptr, &config.morse_dah, 1, 10, nullptr, 6, 0, 6, nullptr, "*100ms"});
            add({morse, Type::U8, "Intra S. Gap dur.", nullptr, &config.morse_gap, 1, 10, nullptr, 3, 0, 3, nullptr, "*100ms"});
            add({morse, Type::U8, "Inter S. Gap dur.", nullptr, &config.morse_igap, 1, 10, nullptr, 3, 0, 3, nullptr, "*100ms"});
            add({morse, Type::U8, "Word Gap dur.", nullptr, &config.morse_wgap, 1, 10, nullptr, 9, 0, 9, nullptr, "*100ms"});
            const auto operate = folder(0, "Operate");
            for (uint8_t i = 0; i < outputs.size(); ++i) {
                add({operate, Type::Sel, names[i], "Off;On", nullptr, 0, 1}, Action::Test, i);
            }
            const auto reset = folder(0, "Reset");
            add({reset, Type::Command, "Reset all", "Reset all?"}, Action::Reset);
        }

        uint8_t size() const { return mEntries.size(); }
        const char* name() const { return mName.data(); }
        Parameter& parameter(const uint8_t index) { return mEntries[index].parameter; }
        const Parameter& operator[](const uint8_t index) const { return mEntries[index].parameter; }
        bool isReset(const uint8_t index) const {
            return index > 0 && index < size() && mEntries[index].action == Action::Reset;
        }

        bool valid() const {
            if (mConfig.magic != EEPROM_MAGIC) return false;
            for (const auto& entry : mEntries) {
                const auto& p = entry.parameter;
                if (p.value_ptr && (*p.value_ptr < p.min || *p.value_ptr > p.max)) return false;
                if (p.stringValue && !std::memchr(p.stringValue, '\0', p.units)) return false;
            }
            return true;
        }

        template<typename Buffer>
        void serialize(const uint8_t index, Buffer& buffer) {
            parameter(index).serialize(buffer, *this, Crsf::Lua::CmdStep::Idle, index);
        }

        ReadResult read(const uint8_t destination, const uint8_t index, const uint8_t chunk,
                        ByteBuffer<58>& payload) {
            if (index >= size()) return ReadResult::InvalidIndex;
            ByteBuffer<256> serialized;
            serialize(index, serialized);
            constexpr size_t chunkSize = 54;
            const size_t chunks = (serialized.size() + chunkSize - 1) / chunkSize;
            if (chunk >= chunks) return ReadResult::InvalidChunk;
            payload.push_back(destination);
            payload.push_back(mConfig.crsf_address);
            payload.push_back(index);
            payload.push_back(chunks - chunk - 1);
            const size_t end = std::min(size_t(serialized.size()), (chunk + 1) * chunkSize);
            for (size_t i = chunk * chunkSize; i < end; ++i) payload.push_back(serialized[i]);
            return ReadResult::Ok;
        }

        Result write(const uint8_t index, const std::span<const uint8_t> data) {
            if (!index || index >= size() || data.empty()) return Result::Invalid;
            auto& entry = mEntries[index];
            auto& p = entry.parameter;
            if (entry.action == Action::Reset) {
                if (data[0] != static_cast<uint8_t>(Crsf::Lua::CmdStep::Confirmed)) {
                    return Result::Transient;
                }
                mConfig = EEProm{};
                apply();
                return Result::Reset;
            }
            if (p.type == Type::Str) {
                const size_t count = std::min(data.size(), size_t{p.units - 1u});
                for (size_t i = 0; i < count; ++i) {
                    p.stringValue[i] = static_cast<char>(data[i]);
                }
                p.stringValue[count] = '\0';
            }
            else if (p.type == Type::U8 || p.type == Type::Sel) {
                if (data[0] < p.min || data[0] > p.max) return Result::Invalid;
                p.value(data[0]);
            }
            else {
                return Result::Invalid;
            }
            if (entry.action == Action::Test) {
                mOutputs[entry.output].set(data[0] != 0);
                return Result::Transient;
            }
            if (entry.action == Action::Output) {
                mOutputs[entry.output].configure(mConfig.outputs[entry.output]);
            }
            updateName();
            return Result::Changed;
        }

        void apply() {
            for (size_t i = 0; i < mOutputs.size(); ++i) {
                mOutputs[i].set(false);
                mOutputs[i].configure(mConfig.outputs[i]);
            }
            updateName();
        }

    private:
        void updateName() {
            std::snprintf(mName.data(), mName.size(), "MultiSwitch-E@%u:%u",
                          mConfig.addresses[0], mConfig.crsf_address);
        }
        void add(Parameter p, const Action action = Action::Store, const uint8_t output = 0) {
            mEntries.push_back({p, action, output});
        }
        uint8_t folder(const uint8_t parent, const char* name) {
            add({parent, Type::Folder, name});
            return size() - 1;
        }

        static constexpr std::array<const char*, 8> names{
            "Output 0", "Output 1", "Output 2", "Output 3", "Output 4", "Output 5", "Output 6", "Output 7"};
        EEProm& mConfig;
        std::array<Output, 8>& mOutputs;
        etl::FixedVector<Entry, 128> mEntries;
        std::array<char, 32> mName{};
    };
}
