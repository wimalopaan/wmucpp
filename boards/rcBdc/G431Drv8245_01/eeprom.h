#pragma once

#include <cstdint>
#include <array>

struct EEProm {
    uint8_t telemetry_polepairs = 9;

    uint8_t cutoff_freq = 10; // 100Hz
    uint8_t n_fsample = 4; // n_fsample * cuttoff_freq = sampling_freq
    uint8_t subsampling = 6;

    uint8_t inertia = 1;

    float resistance = 1.0f;
    float inductance = 0.001f;
    float eKm = 1000.0f;

    uint64_t dummy;
};

