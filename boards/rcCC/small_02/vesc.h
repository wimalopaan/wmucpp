#pragma once

#include <cstdint>
#include <array>

#ifdef USE_DEVICES3

template<typename VescPA, typename CrsfCallback, typename Storage>
struct VescCallback {
    using crsfTelemetry = CrsfCallback::crsfTelemetry;
    using vesc_pa = VescPA;

    static inline void update() {
        uint16_t temp = vesc_pa::mTemperature;
        temp /= 10;
        crsfTelemetry::mTemp1[1] = std::byte(temp & 0xff);
        crsfTelemetry::mTemp1[0] = std::byte((temp >> 8));

        uint16_t rpm = vesc_pa::mRPM;
        rpm /= Storage::eeprom.telemetry_polepairs;
        crsfTelemetry::mRpm1[1] = std::byte(rpm & 0xff);
        crsfTelemetry::mRpm1[0] = std::byte((rpm >> 8));

        uint16_t volt = vesc_pa::mVoltage;

        crsfTelemetry::mBatt[1] = std::byte(volt & 0xff);
        crsfTelemetry::mBatt[0] = std::byte(volt >> 8);

        uint16_t curr = vesc_pa::mCurrent;
        curr /= 10;
        crsfTelemetry::mBatt[3] = std::byte(curr & 0xff);
        crsfTelemetry::mBatt[2] = std::byte(curr >> 8);
    }
};
#endif

#if defined(USE_DEVICES1) || defined(USE_DEVICES2)
template<typename VescPA, typename CrsfCallback>
struct VescCallback {
    using crsfTelemetry = CrsfCallback::crsfTelemetry;
    using vesc_pa = VescPA;

    static inline void update() {
        uint16_t temp = vesc_pa::mTemperature;
        temp /= 10;
        crsfTelemetry::mTemp1[1] = std::byte(temp & 0xff);
        crsfTelemetry::mTemp1[0] = std::byte((temp >> 8));

        uint16_t rpm = vesc_pa::mRPM;
        rpm /= Storage::eeprom.telemetry_polepairs;
        crsfTelemetry::mRpm1[1] = std::byte(rpm & 0xff);
        crsfTelemetry::mRpm1[0] = std::byte((rpm >> 8));

        uint16_t volt = vesc_pa::mVoltage;

        crsfTelemetry::mBatt[1] = std::byte(volt & 0xff);
        crsfTelemetry::mBatt[0] = std::byte(volt >> 8);

        uint16_t curr = vesc_pa::mCurrent;
        curr /= 10;
        crsfTelemetry::mBatt[3] = std::byte(curr & 0xff);
        crsfTelemetry::mBatt[2] = std::byte(curr >> 8);
    }
};

#endif
