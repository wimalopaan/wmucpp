/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2026 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <numbers>
#include <chrono>

#include "ressources.h"

#include "pid.h"

using namespace std::literals::chrono_literals;

template<uint8_t N, typename In, typename Fb, typename Out, typename systemTimer, typename Debug>
struct Feetech {
    using debug = Debug;

    enum class Event : uint8_t {None, Calibrate, Run};
    enum class State : uint8_t {Start, WaitNoise, Noise,
                                StartPosition, StartPositionRotate,
                                DeadStartP, DeadSetP, DeadTestP,
                                DeadStartN, DeadSetN, DeadTestN,
                                DeadEnd,
                                RangeStart, RangeRun, RangeEnd,
                                Run,
                                Error};

    static inline constexpr External::Tick<systemTimer> noiseMeasureTicks{1000ms};
    static inline constexpr External::Tick<systemTimer> noiseWaitTicks{100ms};
    static inline constexpr External::Tick<systemTimer> deadSettleTicks{100ms};
    static inline constexpr External::Tick<systemTimer> rangeTicks{50ms};
    static inline constexpr External::Tick<systemTimer> initTicks{500ms};
    static inline constexpr External::Tick<systemTimer> debugTicks{250ms};

    static inline void init() {
        IO::outl<debug>("# FT init");
        mState = State::Start;
        mEvent = Event::None;
        mStateTick.reset();

        RessourceCount<Fb>::acquire([]{
            Fb::init();
        });
        RessourceCount<Out>::acquire([]{
            Out::init();
        });

    }
    static inline void reset() {
        IO::outl<debug>("# FT reset");
        RessourceCount<Fb>::release([]{
            Fb::reset(); // only if last
        });
        RessourceCount<Out>::release([]{
            Out::reset(); // only if last
        });
    }
    static inline void speed(const uint16_t) {
    }
    static inline void offset(const uint16_t) {
    }
    static inline std::pair<uint8_t, uint8_t> hwVersion() {
        return {255, 255};
    }
    static inline std::pair<uint8_t, uint8_t> fwVersion() {
        return {255, 255};
    }
    static inline int8_t turns() {
        return 0;
    }
    static inline uint16_t actualPos() {
        return mLastPhi;
    }
    static inline void event(const Event e) {
        mEvent = e;
    }
    static inline void periodic() {
    }
    static inline void ratePeriodic() {
        ++mDebugTick;
        const auto oldState = mState;
        ++mStateTick;
        switch(mState) {
        case State::Start:
            if (const auto e = std::exchange(mEvent, Event::None); e == Event::Calibrate) {
                mState = State::WaitNoise;
            }
            else if (e == Event::Run) {
                mState = State::Run;
            }
            mStateTick.on(initTicks, []{
                mState = State::WaitNoise;
            });
            break;
        case State::WaitNoise:
            mStateTick.on(noiseWaitTicks, []{
                mState = State::Noise;
            });
            break;
        case State::Noise:
        {
            static uint16_t mNoiseMax{0};
            static uint16_t mNoiseMin{std::numeric_limits<uint16_t>::max()};
            const uint16_t v = Fb::read();
            mNoiseMax = std::max(mNoiseMax, v);
            mNoiseMin = std::min(mNoiseMin, v);
            mStateTick.on(noiseMeasureTicks, []{
                mNoiseAmplitude = mNoiseMax - mNoiseMin;
                mState = State::StartPositionRotate;
            });
        }
            break;
        case State::StartPosition:
            if (const uint16_t v = Fb::read(); (v > 1500) && (v < 2500)) {
                mState = State::DeadStartP;
            }
            mStateTick.on(deadSettleTicks, []{
                if (const uint16_t d = std::abs(mLastFb - Fb::read()); d < mNoiseAmplitude) {
                    mLastPwm += 100;
                    if (mLastPwm < 1700) {
                        Out::set(mLastPwm);
                    }
                    else {
                        mState = State::Error;
                    }
                }
            });
            break;
        case State::StartPositionRotate:
            mState = State::StartPosition;
            break;
        case State::DeadStartP:
            mStateTick.on(deadSettleTicks, []{
                mState = State::DeadSetP;
            });
            break;
        case State::DeadSetP:
            mStateTick.on(deadSettleTicks, []{
                mState = State::DeadTestP;
            });
            break;
        case State::DeadTestP:
        {
            const uint16_t fb = Fb::read();
            const uint16_t d = std::abs(fb - mLastFb);
            if (d > (2 * mNoiseAmplitude)) {
                mState = State::DeadStartN;
                mFbInc = (fb > mLastFb);
            }
            else {
                mState = State::DeadSetP;
            }
        }
            break;
        case State::DeadStartN:
            mStateTick.on(deadSettleTicks, []{
                mState = State::DeadSetN;
            });
            break;
        case State::DeadSetN:
            mStateTick.on(deadSettleTicks, []{
                mState = State::DeadTestN;
            });
            break;
        case State::DeadTestN:
        {
            const uint16_t fb = Fb::read();
            const uint16_t d = std::abs(fb - mLastFb);
            if (d > (2 * mNoiseAmplitude)) {
                mState = State::DeadEnd;
            }
            else {
                mState = State::DeadSetN;
            }
        }
            break;
        case State::DeadEnd:
            mStateTick.on(deadSettleTicks, []{
                mState = State::RangeStart;
            });
            break;
        case State::RangeStart:
            mStateTick.on(deadSettleTicks, []{
                mState = State::RangeRun;
            });
            break;
        case State::RangeRun:
        {
            const uint16_t v = Fb::read();
            if (v > mFbMax) {
                mFbMax = v;
            }
            else if (v < mFbMin) {
                mFbMin = v;
            }
            mFbRange = mFbMax - mFbMin;
            mStateTick.on(rangeTicks, [&]{
                const bool increasing = (v > mFbLast);
                mFbLast = v;
                if ((mFbRange > 3000) && (mFbInc != increasing)) {
                    mState = State::RangeEnd;
                }
            });
            mDebugTick.on(debugTicks, [&]{
                IO::outl<Debug>("# RangeRun: fbMin: ", mFbMin, " fbMax: ", mFbMax, " v: ", v);
            });
        }
            break;
        case State::RangeEnd:
            mState = State::Run;
            break;
        case State::Run:
            break;
        case State::Error:
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Start:
                break;
            case State::WaitNoise:
                Out::set(mDeadMid);
                IO::outl<Debug>("# Srv:WaitNoise");
                break;
            case State::Noise:
                IO::outl<Debug>("# Srv:Noise");
                break;
            case State::StartPosition:
                IO::outl<Debug>("# Srv:StartPosition: Noise: ", mNoiseAmplitude);
                break;
            case State::StartPositionRotate:
                IO::outl<Debug>("# Srv:StartPositionRotate");
                mLastFb = Fb::read();
                mLastPwm = 992 + 200;
                Out::set(mLastPwm); // rotate
                break;
            case State::DeadStartP:
                mNoiseAmplitude = std::clamp(mNoiseAmplitude, uint16_t{2}, uint16_t{10});
                mLastFb = Fb::read();
                mDeadMid = Out::pwm::sbusMid;
                Out::set(mDeadMid);
                mDeadP = mDeadMid;
                IO::outl<Debug>("# Srv:DeadStartP NoiseA: ", mNoiseAmplitude);
                break;
            case State::DeadSetP:
                mDeadP += mDeadInc;
                IO::outl<Debug>("# Srv:DeadSetP fb: ", mLastFb, " pwm: ", mDeadP);
                if (mDeadP < (Out::pwm::sbusMid + 200)) {
                    Out::set(mDeadP);
                }
                else {
                    mState = State::Error;
                }
                break;
            case State::DeadTestP:
                IO::outl<Debug>("# Srv:DeadTestP");
                break;
            case State::DeadStartN:
                IO::outl<Debug>("# Srv:DeadStartN NoiseA: ", mNoiseAmplitude);
                mLastFb = Fb::read();
                mDeadMid = Out::pwm::sbusMid;
                Out::set(mDeadMid);
                mDeadN = mDeadMid;
                break;
            case State::DeadSetN:
                mDeadN -= mDeadInc;
                IO::outl<Debug>("# Srv:DeadSetN fb: ", mLastFb, " pwm: ", mDeadN);
                if (mDeadN > (Out::pwm::sbusMid - 200)) {
                    Out::set(mDeadN);
                }
                else {
                    mState = State::Error;
                }
                break;
            case State::DeadTestN:
                IO::outl<Debug>("# Srv:DeadTestN");
                break;
            case State::DeadEnd:
                mDeadMid = (mDeadP + mDeadN) / 2;
                Out::set(mDeadMid);
                IO::outl<Debug>("# Srv:DeadEnd: ", mDeadP, " N: ", mDeadN, " M: ", mDeadMid);
                break;
            case State::RangeStart:
                IO::outl<Debug>("# Srv:RangeStart");
                mFbMax = mFbMin = mFbLast = Fb::read();
                mFbRange = 0;
                Out::set(mDeadP + 100);
                break;
            case State::RangeRun:
                IO::outl<Debug>("# Srv:RangeRun");
                break;
            case State::RangeEnd:
                mFbRange = mFbMax - mFbMin;
                IO::outl<Debug>("# Srv:RangeEnd fbmax: ", mFbMax, " fbmin: ", mFbMin, " fbrange: ", mFbRange);
                break;
            case State::Run:
                Out::set(mDeadMid);
                IO::outl<Debug>("# Srv:Run");
                break;
            case State::Error:
                IO::outl<Debug>("# Srv:Error");
                break;
            }
        }
    }
    static inline void zero() {
    }
    static inline void update_x() {
        if (mState == State::Run) {
            const int e = error();
            const int ec = weight(e);
            const int hyst = mNoiseAmplitude;
            int o = mDeadMid;
            if (ec > hyst) {
                o = mDeadP + ec;
            }
            else if (ec < -hyst) {
                o = mDeadN + ec;
            }
            IO::outl<Debug>("# Srv:update e: ", e, " ec: ", ec, " o: ", o);
            // const volatile uint16_t oc = std::clamp(o, 172, 1811); // sbus
            const uint16_t oc = std::clamp(o, 700, 1200); // sbus
            Out::set(oc);
        }
    }
    static inline void update() {
        if (mState == State::Run) {
            int32_t o = mPid.process(error()) * 4;
            if (o > 0) {
                o += mDeadP;
            }
            else if (o < 0) {
                o += mDeadN;
            }
            else {
                o = mDeadMid;
            }
            IO::outl<Debug>("# Srv:update e: ", o);
            const uint16_t oc = std::clamp(o, int32_t(172), int32_t(1811)); // sbus
            Out::set(oc);
        }
    }

    private:
    static inline uint16_t normalized() {
        if (mFbRange < 10) {
            return 0;
        }
        const int32_t v = std::clamp(Fb::read(), mFbMin, mFbMax);
        return ((v - mFbMin) * 4095) / mFbRange;
    }

    static inline int error() {
        const uint16_t phi = In::phi();
        const uint16_t fb = normalized();
        int e = phi - fb;
        mLastPhi = fb;
        // circular
        if (e > 2048) {
            e -= 4096;
        }
        if (e < -2048) {
            e += 4096;
        }
        return e;
    }
    static inline int weight(const int e) {
        // quadratic + clamp
        const int esqr = e * e;
        const int cl = mSqrMax;
        int ew = 0;
        if (e > 0) {
            ew = esqr / cl;
        }
        else if (e < 0) {
            ew = -esqr / cl;
        }
        return std::clamp(ew, -cl, cl);
    }
    
    static inline PID<int32_t> mPid{800, -800, 1000, 10, 0};
    
    static inline uint16_t mSqrMax{400};
    static inline uint16_t mNoiseAmplitude{2};
    static inline uint16_t mLastPwm{992};
    static inline uint16_t mLastFb{0};
    static inline uint16_t mLastPhi{0};
    static inline uint16_t mDeadMid{913};
    static inline uint16_t mDeadInc{1};
    static inline uint16_t mDeadP{1004};
    static inline uint16_t mDeadN{822};
    static inline uint16_t mFbRange{0};
    static inline uint16_t mFbLast{0};
    static inline uint16_t mFbMax{std::numeric_limits<uint16_t>::max()};
    static inline uint16_t mFbMin{0};
    static inline bool mFbInc = false;
    static inline Event mEvent{Event::None};
    static inline State mState{State::Start};
    static inline External::Tick<systemTimer> mStateTick;
    static inline External::Tick<systemTimer> mDebugTick;
};
