#pragma once

#include <cstdint>
#include <array>

struct EEProm {
    uint8_t telemetry_polepairs = 9;
    // info
    // uint16_t max_rpm = 5000; // max_rpm = 60 * cutoff_freq / telemetry_polepairs

    uint8_t cutoff_freq = 10; // 100Hz
    uint8_t n_fsample = 4; // n_fsample * cuttoff_freq = sampling_freq
    uint8_t subsampling = 6;
    // info
    // uint16_t pwm_freq = 24'000; // subsampling * sampling_freq

    uint8_t inertia = 1;

    uint64_t dummy;
};

