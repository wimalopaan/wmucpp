/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <cstdint>

#include <tick.h>
#include <etl/event.h>
#include <meta.h>
#include <rc/rc_2.h>

namespace External::Pattern {
	using namespace std::literals::chrono_literals;
	struct EEProm {
		uint8_t type = 0;
		std::array<uint8_t, 8> member{1, 2, 3, 4, 5, 6, 7, 8};
		uint8_t onTime = 33; // *10 ms
		uint8_t offTime = 1;
		uint8_t next_address = 3; // next multiswitch's virtual address
		uint8_t group = 1; // 0: no group (no triggering of other multiswitches)
	};
	template<uint8_t Number, typename Config>
	struct Generator {
		using timer = Config::timer;
		using storage = Config::storage;
		using debug = Config::debug;
		using outputs = Config::outputs;
		using messagebuffer = Config::messagebuffer;
	
		static inline constexpr uint8_t size = Meta::size_v<outputs>;
		
		enum class State : uint8_t {Idle, On, Off, Wait};
		enum class Event : uint8_t {None, Start, Stop, Chain};
		enum class Mode   : uint8_t {Single, Double};
	
		static inline void length(const uint8_t l) {
			if ((l > 0) && (l <= size)) {
				mLength = l;
			}
		}
		static inline void outputPosition(const uint8_t out, const uint8_t pos) {
			if (pos < size) {
				mOutsInSequence[pos] = out;		
			}
		}	
		static inline void event(const Event e, const bool propagate = true) {
			if (e == Event::Stop) {
				if (propagate) {
					sendStop();
				}
			}
			if (mActive) {
				mEvent = e;
			}
		}
		static inline void setMode(const uint8_t mode) {
			switch(mode) {
			case 0:
				mActive = false;
				break;
			case 1:
				mActive = true;
				mMode = Mode::Single;
				mUp = true;
				mContinous = true;
				break;
			case 2:
				mActive = true;
				mMode = Mode::Single;
				mUp = false;
				mContinous = true;
				break;
			case 3:
				mActive = true;
				mMode = Mode::Double;
				mUp = true;
				mContinous = true;
				break;
			case 4:
				mActive = true;
				mMode = Mode::Single;
				mUp = true;
				mContinous = false;
				break;
			case 5:
				mActive = true;
				mMode = Mode::Single;
				mUp = false;
				mContinous = false;
				break;
			case 6:
				mActive = true;
				mMode = Mode::Double;
				mUp = true;
				mContinous = false;
				break;
			}
		}
		static inline void ratePeriodic() {
			const auto oldState = mState;
			++mStateTick;
			switch(mState) {
			case State::Idle:
				mEvent.on(Event::Start, []{
					if (mUp) {
						mOutCount = 0;
						mActualCountUp = true;
					}
					else {
						mOutCount = mLength - 1;
						mActualCountUp = false;
					}
					mState = State::On;
				}).thenOn(Event::Chain, []{
					if (mUp) {
						mOutCount = 0;
						mActualCountUp = true;
					}
					else {
						mOutCount = mLength - 1;
						mActualCountUp = false;
					}
					mState = State::On;				
				});
				break;
			case State::Wait:
				mEvent.on(Event::Stop, []{
					mState = State::Idle;
				}).thenOn(Event::Chain, []{
					if (mUp) {
						mOutCount = 0;
						mActualCountUp = true;
					}
					else {
						mOutCount = mLength - 1;
						mActualCountUp = false;
					}
					mState = State::On;				
				});
				break;
			case State::On:
				mStateTick.on(onTicks, []{
					mState = State::Off;
				});			
				break;
			case State::Off:
				mEvent.on(Event::Stop, []{
					mState = State::Idle;
				});
				mStateTick.on(offTicks, []{
					if (mActualCountUp) {
						if (++mOutCount >= mLength) {
							if (mPendingStop) {
								mState = State::Idle;
							}
							else {
								if (mMode == Mode::Single) {
									mOutCount = 0;
								}
								else {
									mOutCount = mLength - 2;
									mActualCountUp = false;
								}
								if (mContinous) {
									mState = State::On;
								}
								else {
									if (mMode == Mode::Single) {
										mState = State::Wait;
										sendTrigger();
									}
									else {
										mState = State::On;
										mPendingStop = true;
									}
								}
							}
						}
						else {
							mState = State::On;
						}
					}
					else {
						if (--mOutCount < 0) {
							if (mPendingStop) {
								mState = State::Idle;
							}
							else {
								if (mMode == Mode::Single) {
									mOutCount = mLength - 1;
								}
								else {
									mOutCount = 1;
									mActualCountUp = true;
								}
								if (mContinous) {
									mState = State::On;
								}
								else {
									if (mMode == Mode::Single) {
										mState = State::Wait;
										sendTrigger();
									}
									else {
										mState = State::On;
										mPendingStop = true;
									}
								}
							}
						}
						else {
							mState = State::On;
						}
					}
				});			
				break;
			}
			if (oldState != mState) {
				mStateTick.reset();
				switch(mState) {
				case State::Idle:
				case State::Wait:
					Meta::visitAt<outputs>(mOutsInSequence[mOutCount], []<typename O>(Meta::Wrapper<O>){
						O::on(false);					
					});
					break;
				case State::On:
					Meta::visitAt<outputs>(mOutsInSequence[mOutCount], []<typename O>(Meta::Wrapper<O>){
						O::on(true);					
					});
					break;
				case State::Off:
					Meta::visitAt<outputs>(mOutsInSequence[mOutCount], []<typename O>(Meta::Wrapper<O>){
						O::on(false);					
					});
					break;
				}			
			}
		}	
		static inline void ontime(const uint16_t t) {
			onTicks = std::chrono::milliseconds(t);
		}
		static inline void offtime(const uint16_t t) {
			offTicks = std::chrono::milliseconds(t);
		}
		private:
		static inline void sendStop() {
#ifdef USE_PATTERNS			
			for(const uint8_t a : storage::eeprom.addresses) {
				if (a == storage::eeprom.pattern[0].next_address) {
					return;
				}
			}
			if (storage::eeprom.pattern[0].group == 0) {
				return;
			}
			using namespace RC::Protokoll::Crsf::V4;
			messagebuffer::create_back((uint8_t)RC::Protokoll::Crsf::V4::Type::Command, [&](auto& d){
				d.push_back(RC::Protokoll::Crsf::V4::Address::Broadcast);
				d.push_back(storage::eeprom.crsf_address);
				d.push_back(RC::Protokoll::Crsf::V4::CommandType::Switch);
				d.push_back(RC::Protokoll::Crsf::V4::SwitchCommand::InterModulePatternStopAll);
				d.push_back(storage::eeprom.pattern[0].group);
			});		
#endif
		}
		static inline void sendTrigger() {
#ifdef USE_PATTERNS
			for(const uint8_t a : storage::eeprom.addresses) {
				if (a == storage::eeprom.pattern[0].next_address) {
					return;
				}
			}
			using namespace RC::Protokoll::Crsf::V4;
			messagebuffer::create_back((uint8_t)RC::Protokoll::Crsf::V4::Type::Command, [&](auto& d){
				d.push_back(RC::Protokoll::Crsf::V4::Address::Broadcast);
				d.push_back(storage::eeprom.crsf_address);
				d.push_back(RC::Protokoll::Crsf::V4::CommandType::Switch);
				d.push_back(RC::Protokoll::Crsf::V4::SwitchCommand::InterModulePatternStart);
				d.push_back(storage::eeprom.pattern[0].next_address);
				d.push_back((uint8_t)4);
			});
#endif
		}
		static inline External::Tick<timer> onTicks{100ms};
		static inline External::Tick<timer> offTicks{10ms};
		static inline bool mActive = false;
		static inline bool mContinous = false;
		static inline bool mUp = true;
		static inline Mode mMode = Mode::Single;
		static inline int8_t mOutCount = 0;
		static inline bool mActualCountUp = mUp;
		static inline bool mPendingStop = false;
		static inline uint8_t mLength = size;
		static inline std::array<uint8_t, size> mOutsInSequence{};
		static inline State mState = State::Idle;
		static inline etl::Event<Event> mEvent;
		static inline External::Tick<timer> mStateTick;
	};
	
}
