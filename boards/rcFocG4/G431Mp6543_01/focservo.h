/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "etl/event.h"
#include "rc/rc_2.h"
#include "output.h"
#include "dsp.h"

#include "pid.h"

using namespace std::literals::chrono_literals;

template<typename Config>
struct FocServo {
	using debug = Config::debug;
	using timer = Config::timer;
	using out = Config::pwm;
	using sensor = Config::sensor;
	using in = Config::input;
	using adc = Config::adc;
	
	enum class State : uint8_t {Undefined, Init, 
								SetCurrent, ReadOffset, 
								CheckDir,
								MechZero, MechZeroRot,
								FindPolePairs,
								Calculate,
								Calib, Run,
							    Error}; 
	enum class Event : uint8_t {None, Error}; 

	enum class Mode : uint8_t {Servo, Winch, Crane, Custom}; 
	
	using tck_t = External::Tick<timer>;
	
	static inline void init() {
		IO::outl<debug>("# FocServo init");
	}
	static inline void preset(const uint8_t p) {
		if (p == 0) {
			mMode = Mode::Servo;
			mPid = mPidPreServo;
		}
		else if (p == 1) {
			mMode = Mode::Winch;
			mPid = mPidPreWinch;			
		}
	}
	static inline void speed(const uint8_t) {
	}
	static inline void force(const uint8_t) {
	}
	static inline void maxThrow(const uint8_t m) {
		mThrow = m;
	}
	static inline void angle(const uint16_t crsf_value) {
		if (mState == State::Run) {
			const int16_t crsf_centered = (crsf_value - RC::Protokoll::Crsf::V4::mid);
			const float crsf_normalized = mAngleMean.process(crsf_centered) / float(RC::Protokoll::Crsf::V4::span);
			if (mMode == Mode::Servo) {
				const int32_t a = (mPP * mThrow * crsf_normalized * out::pi) / 100.0f;
				mAngle = a + mOffset3;
			}
			else if (mMode == Mode::Winch) {
				const int32_t d = crsf_normalized;
				mAngle += d;				
			}
			else if (mMode == Mode::Crane) {
				const int32_t d = crsf_normalized;
				mAngle += d;				
			}
			else if (mMode == Mode::Custom) {
			}
		}
	}
	static inline void ratePeriodic() {
		const int16_t i1 = adc::template value<0>();
		const int16_t i2 = adc::template value<1>();
		const int16_t i3 = adc::template value<2>();
		const float isum = (i1 + i2 + i3);
		mIMean.process(isum);
		
		const auto oldState = mState;
		++mStateTick;
		switch(mState) {
		case State::Undefined:
			mStateTick.on(tck_t{1ms}, []{
				mState = State::Init;
				// mState = State::Calib;
			});
			break;
		case State::Init:
			mStateTick.on(tck_t{100ms}, []{
				mState = State::SetCurrent;
			});
			break;
		case State::SetCurrent:
			mStateTick.on(tck_t{20ms}, []{
				mPower += 1;
				out::power(mPower);
				out::iangle(mAngle);
				if (mPower >= 30) {
					mState = State::ReadOffset;
				}
			});
			break;
		case State::ReadOffset:
			mStateTick.on(tck_t{20ms}, []{
				mState = State::CheckDir;				
			});
			break;
		case State::CheckDir:
			mStateTick.on(tck_t{10ms}, []{
				const int32_t a = sensor::value();
				if (std::fabs(a - mOffset1) > (sensor::span / 11)) { // max 11 PP
					if (a > mOffset1) {
						mDirPos = true;
					}
					else {
						mDirPos = false;
					}
					mState = State::MechZero;
				}
				else {
					mAngle += mCheckDirSpeed;
					out::iangle(mAngle);
				}
				if (mAngle > out::twopi) {
					mState = State::Error;
				}
			});
			break;
		case State::MechZero:
			mStateTick.on(tck_t{10ms}, []{
				mState = State::MechZeroRot;				
			});
			break;
		case State::MechZeroRot:
			mStateTick.on(tck_t{10ms}, []{
				const int32_t ma = sensor::value();
				if (std::abs(ma) < 20) {
					mOffset2 = mAngle;
					mState = State::FindPolePairs;
				}
				else if (std::abs(ma) < 100) {
					mAngle += (mDelta / 4);
				}
				else {
					mAngle += mDelta;
				}
				out::iangle(mAngle);
			});
			break;
		case State::FindPolePairs:
			mStateTick.on(tck_t{10ms}, []{
				const int32_t ma = sensor::value();
				if (ma > sensor::span) {
					mOffset3 = mAngle;
					mState = State::Calculate;
				}
				mAngle += mDelta;
				out::iangle(mAngle);
			});
			break;
		case State::Calculate:
			if ((mPP >= 5) && (mPP <= 11)) {
				mState = State::Run;
			}
			else {
				mState = State::Error;
			}
			break;
		case State::Calib:
			// read current sensor
			break;
		case State::Run:
		{
			const int32_t a = mPP * correctDir(sensor::value()) + mOffset2;
			const int32_t d = mPid.process(mAngle, a);
			const uint8_t p = std::abs(d);
			out::power(p);
			if (d > 0) {
				out::iangle(a + (out::pi / 2));
			}
			else if (d < 0) {
				out::iangle(a - (out::pi / 2));
			}
		}
			break;
		case State::Error:
			break;
		}
		if (oldState != mState) {
			switch(mState) {
			case State::Undefined:
				break;
			case State::Init:
				IO::outl<debug>("# Foc Init");
				out::polepairs(1);
				out::power(20);
				out::iangle(0);
				mAngle = 0;
				break;
			case State::SetCurrent:
				IO::outl<debug>("# Foc SetC");
				mPower = 0;
				out::power(mPower);
				mAngle = 0;
				out::iangle(mAngle);
				break;
			case State::ReadOffset:
				IO::outl<debug>("# Foc ReadOffset");
				mOffset1 = sensor::value();
				break;
			case State::CheckDir:
				IO::outl<debug>("# Foc CheckDir");
				break;
			case State::MechZero:
				IO::outl<debug>("# Foc MechZero o: ", mOffset1);
				if (sensor::value() > 0) {
					mDelta = -correctDir(mFindZeroSpeed);
				}
				else {
					mDelta = correctDir(mFindZeroSpeed);
				}
				break;
			case State::MechZeroRot:
				IO::outl<debug>("# Foc MechZeroRot d: ", mDelta);
				break;
			case State::FindPolePairs:
				IO::outl<debug>("# Foc FindPP");
				mDelta = correctDir(mFindPPSpeed);
				break;
			case State::Calculate:
				mPP = std::fabs(1.0f * (mOffset3 - mOffset2)) / sensor::span + 0.5f;
				IO::outl<debug>("# Foc Calc PP: ", mPP);
				break;
			case State::Calib:
				IO::outl<debug>("# Foc Calib ");
				out::power(0);
				out::iangle(0);
				break;
			case State::Run:
				IO::outl<debug>("# Foc Run o1: ", mOffset1, " o2: ", mOffset2, " o3: ", mOffset3);
				break;
			case State::Error:
				IO::outl<debug>("# Foc Error");
				mPower = 0;
				out::power(mPower);
				mAngle = 0;
				out::iangle(mAngle);
				break;
			}
		}
	}
	static inline int16_t imean() {
		return mIMean.value();
	}
	private:
	static inline Dsp::V2::ExpMean<float> mAngleMean{0.1f};
	struct ModeParams {
		PID<float> mPid{0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
		bool autoStop{};
	};
	static inline Mode mMode = Mode::Servo;
	static inline std::array<ModeParams, 3> mModes{};
	
	static inline PID<float> mPidPreServo{100.0f, -100.0f, 0.02f, 1e-6f, 0.0f};
	static inline PID<float> mPidPreWinch{20.0f, -20.0f, 0.0005f, 0.0f, 0.0f};
	static inline PID<float> mPidPreCrane{20.0f, -20.0f, 0.0005f, 0.0f, 0.0f};
	static inline PID<float> mPid = mPidPreServo;
	
	static inline Dsp::V2::ExpMean<float> mIMean{0.01f};
	static inline int32_t mThrow = 100;
	
	static inline int32_t correctDir(const int32_t d) {
		return mDirPos ? d : -d;
	}
	static inline bool mDirPos = true;
	
	static inline int32_t mFindPPSpeed = 100;
	static inline int32_t mFindZeroSpeed = 40;
	static inline int32_t mCheckDirSpeed = 60;

	static inline uint8_t mPP = 7;	
	static inline int32_t mDelta = 0;
	static inline int32_t mOffset1 = 0;
	static inline int32_t mOffset2 = 0;
	static inline int32_t mOffset3 = 0;
	static inline int32_t mAngle = 0;
	static inline uint16_t mPower = 0;
	static inline External::Tick<timer> mStateTick;
	static inline etl::Event<Event> mEvent;
	static inline State mState{State::Undefined};
};
