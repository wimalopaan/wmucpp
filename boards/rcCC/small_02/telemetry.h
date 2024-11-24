#pragma once

#if defined(USE_DEVICES4)

template<typename Out, typename Timer, typename Data>
struct CrsfTelemetry {
    using out = Out;
    static inline constexpr External::Tick<Timer> telemTicks{20ms};

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
                mState = State::Rpm2;
                break;
            case State::Rpm2:
                out::data(RC::Protokoll::Crsf::Type::Rpm2, mRpm2);
                mState = State::Gps;
                break;
            }
        });
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
    static inline State mState{State::Gps};
    static inline Event mEvent{Event::None};

    static inline std::array<std::byte, 15> mGps{};
    static inline std::array<std::byte, 8> mBatt{}; //volts amps mAh percent
    static inline std::array<std::byte, 2> mTemp1{};
    static inline std::array<std::byte, 2> mTemp2{};
    static inline std::array<std::byte, 2> mRpm1{};
    static inline std::array<std::byte, 2> mRpm2{};
    static inline External::Tick<Timer> mTelemTick;
};

#endif


#if defined(USE_DEVICES3)

template<typename Out, typename Timer, typename Data>
struct CrsfTelemetry {
    using out = Out;
    static inline constexpr External::Tick<Timer> telemTicks{20ms};

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
                mState = State::Rpm2;
                break;
            case State::Rpm2:
                out::data(RC::Protokoll::Crsf::Type::Rpm2, mRpm2);
                mState = State::Gps;
                break;
            }
        });
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
    static inline State mState{State::Gps};
    static inline Event mEvent{Event::None};

    static inline std::array<std::byte, 15> mGps{};
    static inline std::array<std::byte, 8> mBatt{}; //volts amps mAh percent
    static inline std::array<std::byte, 2> mTemp1{};
    static inline std::array<std::byte, 2> mTemp2{};
    static inline std::array<std::byte, 2> mRpm1{};
    static inline std::array<std::byte, 2> mRpm2{};
    static inline External::Tick<Timer> mTelemTick;
};

#endif

#if defined(USE_DEVICES1) || defined(USE_DEVICES2)
template<typename Out, typename Timer>
struct CrsfTelemetry {
    using out = Out;
    static inline constexpr External::Tick<Timer> telemTicks{20ms};

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
                mState = State::Rpm2;
                break;
            case State::Rpm2:
                out::data(RC::Protokoll::Crsf::Type::Rpm2, mRpm2);
                mState = State::Gps;
                break;
            }
        });
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
    static inline State mState{State::Gps};
    static inline Event mEvent{Event::None};

    static inline std::array<std::byte, 15> mGps{};
    static inline std::array<std::byte, 8> mBatt{}; //volts amps mAh percent
    static inline std::array<std::byte, 2> mTemp1{};
    static inline std::array<std::byte, 2> mTemp2{};
    static inline std::array<std::byte, 2> mRpm1{};
    static inline std::array<std::byte, 2> mRpm2{};
    static inline External::Tick<Timer> mTelemTick;
};

#endif
