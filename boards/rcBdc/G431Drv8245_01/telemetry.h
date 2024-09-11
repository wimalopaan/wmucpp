#pragma once

using namespace std::literals::chrono_literals;


template<typename Out, typename Timer, typename Data>
struct CrsfTelemetry {
    using out = Out;
    static inline constexpr External::Tick<Timer> telemTicks{100ms};

    enum class State : uint8_t {Idle, Gps, Batt, Temp1, Temp2, Rpm1, Rpm2};
    enum class Event : uint8_t {None, SendNext};

    static inline void event(const Event e) {
        mEvent = e;
    }

    static inline void ratePeriodic() {
        // const Event e = std::exchange(mEvent, Event::None);
        (++mTelemTick).on(telemTicks, []{
            switch(mState) {
            case State::Idle:
                mState = State::Batt;
                break;
            case State::Gps:
                out::data(RC::Protokoll::Crsf::Type::Gps, mGps); // crsf gps data
                mState = State::Batt;
                break;
            case State::Batt:
                out::data(RC::Protokoll::Crsf::Type::Battery, mBatt);
                mState = State::Temp1;
                break;
            case State::Temp1:
                out::data(RC::Protokoll::Crsf::Type::Temp1, mTemp1);
                mState = State::Temp2;
                break;
            case State::Temp2:
                out::data(RC::Protokoll::Crsf::Type::Temp2, mTemp2);
                mState = State::Rpm1;
                break;
            case State::Rpm1:
                out::data(RC::Protokoll::Crsf::Type::Rpm1, mRpm1);
                mState = State::Batt;
                break;
            case State::Rpm2:
                out::data(RC::Protokoll::Crsf::Type::Rpm2, mRpm2);
                mState = State::Gps;
                break;
            }
        });
    }
    static inline void curr(const uint16_t c) {
        mBatt[2] = std::byte(c >> 8);
        mBatt[3] = std::byte(c & 0xff);
    }
    static inline void batt(const uint16_t v) {
        mBatt[0] = std::byte(v >> 8);
        mBatt[1] = std::byte(v & 0xff);
    }
    static inline void rpm1(const uint16_t r) {
        mRpm1[0] = std::byte(r >> 8);
        mRpm1[1] = std::byte(r & 0xff);
    }
    static inline void temp1(const uint16_t t) {
        mTemp1[0] = std::byte(t >> 8);
        mTemp1[1] = std::byte(t & 0xff);
    }
    static inline void temp2(const uint16_t t) {
        mTemp2[0] = std::byte(t >> 8);
        mTemp2[1] = std::byte(t & 0xff);
    }
    static inline void sats(const uint8_t n) {
        mGps[14] = std::byte(n);
    }
    static inline void speed(const float s) {
        int16_t st = s * 10;
        mGps[8] = std::byte(st >> 8);
        mGps[9] = std::byte(st & 0xff);
    }
    // private:
    static inline State mState{State::Idle};
    static inline Event mEvent{Event::None};

    static inline std::array<std::byte, 15> mGps{};
    static inline std::array<std::byte, 8> mBatt{}; //volts amps mAh percent
    static inline std::array<std::byte, 2> mTemp1{};
    static inline std::array<std::byte, 2> mTemp2{};
    static inline std::array<std::byte, 2> mRpm1{};
    static inline std::array<std::byte, 2> mRpm2{};
    static inline External::Tick<Timer> mTelemTick;
};

