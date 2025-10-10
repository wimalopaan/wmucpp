#pragma once

template<typename Device>
struct MessageBuilder {
    using device = Device;
    explicit MessageBuilder(std::array<std::byte, Crsf::maxMessageSize>& buffer,
                            const std::byte type) : mBuffer(buffer) {
        mBuffer[0] = 0xc8_B;
        mLength = 3;
        mBuffer[1] = std::byte(mLength);
        mBuffer[2] = type;

    }
    ~MessageBuilder() {
        const uint8_t length = mLength + 1; // incl. CRC
        mBuffer[1] = std::byte(length - 2); // without: startbyte and length, including CRC
        CRC8 csum;
        for(int8_t i = 0; i < length - 2 - 1; ++i) { // without CRC
            csum += mBuffer[i + 2];
        }
        mBuffer[length - 1] = csum;
        for(uint8_t i = 0; i < length; ++i) {
            device::put(mBuffer[i]);
        }
    }
    void push_back(const uint8_t d) {
        mBuffer[mLength++] = std::byte(d);
    }
    void push_back(const uint16_t d) {
        mBuffer[mLength++] = std::byte(d >> 8);
        mBuffer[mLength++] = std::byte(d);
    }
    void push_back(const int16_t d) {
        mBuffer[mLength++] = std::byte(d >> 8);
        mBuffer[mLength++] = std::byte(d);
    }
    private:
    uint8_t mLength{};
    std::array<std::byte, Crsf::maxMessageSize>& mBuffer;
};

template<typename Config>
struct Telemetry {
    using debug = Config::debug;
    using crsf = Config::crsf;

    static inline void gotLink() {
        switch(mCounter) {
        case 0:
            sendBits();
            break;
        case 1:
            sendTemperature();
            break;
        case 2:
            sendVoltage();
            break;
        }
        if (++mCounter >= mMax) {
            mCounter = 0;
        }
    }
    private:
    static inline void sendTemperature() {
        etl::outl<debug>("send temp"_pgm);
        MessageBuilder<crsf> b(mMessage, Crsf::Type::Temp);
        b.push_back(mId);
        b.push_back(mTemp);
    }
    static inline void sendBits() {
        etl::outl<debug>("send bits"_pgm);
        MessageBuilder<crsf> b(mMessage, Crsf::Type::PassThru);
        b.push_back(mAppId);
        b.push_back(mBits);
    }
    static inline void sendVoltage() {
        etl::outl<debug>("send voltage"_pgm);
        MessageBuilder<crsf> b(mMessage, Crsf::Type::Cells);
        b.push_back(mId);
        b.push_back(mVoltage);
    }
    static inline uint8_t mCounter{};
    static inline uint8_t mMax{5};

    static inline std::array<std::byte, Crsf::maxMessageSize> mMessage{};
    static inline uint8_t mId{0};
    static inline int16_t mTemp{0};
    static inline uint16_t mVoltage{0};
    static inline uint16_t mAppId{6000};
    static inline uint8_t mBits{0};
};
