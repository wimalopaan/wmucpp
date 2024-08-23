#pragma once

#include <cstdint>
#include <array>

template<typename T = float>
struct Directional {
    T dir1;
    T dir2;
};

struct EEProm {
    uint8_t crsf_channel = 1; // 1...16
    uint8_t prerun_check = 1;
    uint8_t telemetry_polepairs = 9;

    uint8_t cutoff_freq = 10; // 100Hz
    uint8_t n_fsample = 4; // n_fsample * cuttoff_freq = sampling_freq
    uint8_t subsampling = 6;

    uint8_t pwm_calib = 2; // 400Hz

    uint8_t inertia = 1;

    Directional<float> resistance{1.0f, 1.0f};
    Directional<float> inductance{0.001f, 0.001f};
    Directional<uint16_t> eKm{1000, 1000};

    uint8_t calib_ubatt = 100;
    uint8_t temp_filter = 1;
    uint8_t volume = 100;

    uint8_t use_pid = 0;
    uint8_t pid_mode = 0;
    uint8_t pid_p = 90;
    uint8_t pid_i = 10;
    uint8_t pid_d = 10;
};

