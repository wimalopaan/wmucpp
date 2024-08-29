#pragma once

#include <cstdint>
#include <array>

#ifdef USE_DEVICES3

template<typename VescPA, typename CrsfCallback, typename Storage>
struct VescCallback {
    using crsfCallback = CrsfCallback;
    using crsfTelemetry = CrsfCallback::crsfTelemetry;
    using vesc_pa = VescPA;

    static inline void setFWInfo(const uint8_t fwMajor, const uint8_t fwMinor, const char* const fwName) {
        auto r = std::to_chars(std::begin(crsfCallback::mVescFirmware), std::end(crsfCallback::mVescFirmware), fwMajor);
        *r.ptr++ = ':';
        r  = std::to_chars(r.ptr, std::end(crsfCallback::mVescFirmware), fwMinor);
        *r.ptr = '\0';
        std::strncpy(&crsfCallback::mVescName[0], fwName, crsfCallback::mVescName.size());
    }

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

        uint16_t curr = (Storage::eeprom.telemetry_currentSelect == 0) ? vesc_pa::mCurrentIn : vesc_pa::mCurrent;
        curr /= 10;
        crsfTelemetry::mBatt[3] = std::byte(curr & 0xff);
        crsfTelemetry::mBatt[2] = std::byte(curr >> 8);

        uint32_t mAh = vesc_pa::mConsumption;

        crsfTelemetry::mBatt[6] = std::byte(mAh & 0xff);
        crsfTelemetry::mBatt[5] = std::byte(mAh >> 8);
        crsfTelemetry::mBatt[4] = std::byte(mAh >> 16);
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
