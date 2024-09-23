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
    uint8_t prerun_hyst = 30;
#ifdef DEFAULT_POLE_PAIRS
    uint8_t telemetry_polepairs = DEFAULT_POLE_PAIRS;
#else
    uint8_t telemetry_polepairs = 9;
#endif

    uint8_t cutoff_freq = 10; // 1000Hz
    uint8_t n_fsample = 4; // n_fsample * cuttoff_freq = sampling_freq
    uint8_t subsampling = 6;

    uint8_t timeDomainWindow = 1; // 0=None, 1=Blackmann-Nuttall, 2=Hann

#ifdef DEFAULT_RL_CALIB_PWM
    uint8_t pwm_calib = DEFAULT_RL_CALIB_PWM;
#else
    uint8_t pwm_calib = 2; // 400 Hz
#endif

#ifdef DEFAULT_INERTIA
    uint8_t inertia = DEFAULT_INERTIA;
#else
    uint8_t inertia = 1;
#endif

    Directional<float> resistance{1.0f, 1.0f};
    Directional<float> inductance{0.001f, 0.001f};
    Directional<uint16_t> eKm{1000, 1000};

    uint8_t calib_ubatt = 100;
    uint8_t temp_filter = 1;
    uint8_t volume = 100;

    uint8_t current_select = 0; // 0=mean current, 1=peak current
    uint16_t current_offset = 0;

    uint8_t use_pid = 0;
    uint8_t pid_mode = 0;
    uint8_t pid_p = 90;
    uint8_t pid_i = 10;
    uint8_t pid_d = 10;
};

