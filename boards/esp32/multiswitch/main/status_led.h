#pragma once

#include <cstdint>

namespace Multiswitch {
    class StatusLed {
    public:
        bool value(const uint64_t now, const bool connected, const bool telemetry, const bool configReply) {
            mWasConnected |= connected;
            const Mode mode = connected ?
                (telemetry ? Mode::Telemetry : configReply ? Mode::Configuration : Mode::NoTelemetry) :
                (mWasConnected ? Mode::Lost : Mode::Searching);
            if (mode != mMode) {
                mMode = mode;
                mStart = now;
            }
            if (mode == Mode::Configuration) return true;

            constexpr uint64_t pulse = 100'000;
            const uint64_t pause = mode == Mode::Searching ? 1'200'000 :
                                   mode == Mode::Lost ? 210'000 : 5'000'000;
            const uint64_t burst = mode == Mode::Telemetry ? 3 * pulse : pulse;
            const uint64_t elapsed = (now - mStart) % (burst + pause);
            return elapsed < burst && elapsed % (2 * pulse) < pulse;
        }

    private:
        enum class Mode {Undefined, Searching, Telemetry, NoTelemetry, Configuration, Lost};
        Mode mMode = Mode::Undefined;
        uint64_t mStart = 0;
        bool mWasConnected = false;
    };
}
