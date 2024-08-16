#pragma once

#include <cstdint>
#include <array>

struct EEProm {
    uint8_t crsf_channel = 1; // 1...16
    uint8_t prerun_check = 1;
    uint8_t telemetry_polepairs = 9;

    uint8_t cutoff_freq = 10; // 100Hz
    uint8_t n_fsample = 4; // n_fsample * cuttoff_freq = sampling_freq
    uint8_t subsampling = 6;

    uint8_t inertia = 1;

    float resistance = 1.0f;
    float inductance = 0.001f;
    float eKm = 1000.0f;

    uint8_t calib_ubatt = 100;

    uint8_t use_pid = 0;
    uint8_t pid_p = 90;
    uint8_t pid_i = 10;
    uint8_t pid_d = 10;
};

