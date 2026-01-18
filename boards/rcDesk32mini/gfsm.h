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

#include "rc/rc_2.h"
#include "etl/event.h"
#include <tick.h>

using namespace std::literals::chrono_literals;

template<typename Devices>
struct GFSM {
    using devs = Devices;
    using debug = devs::debug;
    using storage = devs::storage;
    using systemTimer = devs::systemTimer;
    using led = devs::ledBlinker;
    using btn = devs::btn;
    using adc = devs::adc;
	using adap = devs::adcAdapter;
	using digitals = devs::digitals;

    using led1 = devs::ledBlinker1;
    using led2 = devs::ledBlinker2;

	using hwext = devs::hwext;
	using sbus = devs::sbus;
	using modcom = devs::modcom;
	using crsf = devs::crsf;
    using sumdv1 = devs::sumdv1;
    using sumdv3 = devs::sumdv3;

    enum class State : uint8_t {Undefined, Init, Run, CheckSerial, Calibration1, Calibration2, BootPress, BootPressRelease};
    enum class Event : uint8_t {None, ButtonPress, HeartBeat, StartCalib, StopCalib, StartNormal};

    static inline constexpr External::Tick<systemTimer> initTicks{100ms};
    static inline constexpr External::Tick<systemTimer> debugTicks{500ms};
    static inline constexpr External::Tick<systemTimer> telemTicks{10ms};
	static inline constexpr External::Tick<systemTimer> waitTicks{5000ms};

	static inline void heartbeat(){
		mEvent = Event::HeartBeat;
	}
	static inline void startNormal(){
		mEvent = Event::StartNormal;
	}
	static inline void startCalib(){
		mEvent = Event::StartCalib;
	}
	static inline void stopCalib(){
		mEvent = Event::StopCalib;		
	}
    static inline void init() {
        devs::init();
    }
    static inline void event(const Event e) {
        mEvent = e;
    }
    static inline void periodic() {
		devs::periodic();
    }
    static inline void ratePeriodic() {
		devs::ratePeriodic();
        checkButton();

        ++mStateTick;
        const auto oldState = mState;
        switch(mState) {
        case State::Undefined:
            mStateTick.on(initTicks, []{
                mState = State::Init;
            });
            break;
        case State::Init:
            if constexpr(requires{devs::bootPress();}) {
                if (devs::bootPress()) {
                    mState = State::BootPress;
                }
                else {
                    mStateTick.on(initTicks, []{
                        mState = State::CheckSerial;
                    });
                }
            }
            else {
                mStateTick.on(initTicks, []{
                    mState = State::CheckSerial;
                });
            }
            break;
        case State::BootPress:
            if constexpr(requires{devs::bootPress();}) {
                if (!devs::bootPress()) {
                    mState = State::BootPressRelease;
                }
            }
            else {
                mState = State::CheckSerial;
            }
            break;
        case State::BootPressRelease:
            if constexpr(requires{devs::bootPress();}) {
                if (devs::bootRelease()) {
                    mState = State::Calibration1;
                }
            }
            else {
                mState = State::Calibration1;
            }
            break;
		case State::CheckSerial:
			mEvent.on(Event::HeartBeat, []{
				mWaitTick.reset();
			}).thenOn(Event::StartCalib, []{
                mState = State::Calibration1;
			}).thenOn(Event::StartNormal, []{
				save();
				mState = State::Run;
			});
			(++mWaitTick).on(waitTicks, []{
				mState = State::Run;				
			});
			update();
			(++mDebugTick).on(debugTicks, []{
				print();
			});
			break;
        case State::Calibration1:
            if constexpr(requires{devs::press1();}) {
                if (devs::press1()) {
                    storeMid();
                    mState = State::Calibration2;
                }
                else {
                    mEvent.on(Event::ButtonPress, []{
                              storeMid();
                              mState = State::Calibration2;
                          }).thenOn(Event::StartNormal, []{
                            storeMid();
                            mState = State::Calibration2;
                        }).thenOn(Event::StopCalib, []{
                            mState = State::CheckSerial;
                        });
                }
            }
            else {
                mEvent.on(Event::ButtonPress, []{
                          storeMid();
                          mState = State::Calibration2;
                      }).thenOn(Event::StartNormal, []{
                        storeMid();
                        mState = State::Calibration2;
                    }).thenOn(Event::StopCalib, []{
                        mState = State::CheckSerial;
                    });
            }
            break;
        case State::Calibration2:
            if constexpr(requires{devs::press1();}) {
                if (devs::press1()) {
                    save();
                    mState = State::Run;
                }
                else {
                    mEvent.on(Event::ButtonPress, []{
                              save();
                              mState = State::Run;
                          }).thenOn(Event::StartNormal, []{
                            save();
                            mState = State::Run;
                        }).thenOn(Event::StopCalib, []{
                            mState = State::CheckSerial;
                        });
                }
            }
            else {
                mEvent.on(Event::ButtonPress, []{
                          save();
                          mState = State::Run;
                      }).thenOn(Event::StartNormal, []{
                        save();
                        mState = State::Run;
                    }).thenOn(Event::StopCalib, []{
                        mState = State::CheckSerial;
                    });
            }
			calibration();
			update();
			(++mDebugTick).on(debugTicks, []{
				print();
			});
			break;
        case State::Run:
			mEvent.on(Event::ButtonPress, []{
                mState = State::Calibration1;
			});
			update();
			(++mDebugTick).on(debugTicks, []{
				print();
			});
            break;
        }
        if (oldState != mState) {
            mStateTick.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                IO::outl<debug>("# Init eep magic: ", storage::eeprom.magic);
                break;
            case State::BootPress:
                IO::outl<debug>("# BootPress");
                if constexpr(!std::is_same_v<led1, void>) {
                    led1::event(led1::Event::Steady);
                    led2::event(led2::Event::Steady);
                }
                break;
            case State::BootPressRelease:
                IO::outl<debug>("# BootPressRelease");
                led::event(led::Event::Off);
                break;
            case State::CheckSerial:
                IO::outl<debug>("# CheckSerial");
				led::event(led::Event::Steady);
				if constexpr(!std::is_same_v<modcom, void>) {
					modcom::init();
					modcom::setStatus(1);
				}
                break;
            case State::Run:
                IO::outl<debug>("# Run");
				led::count(2);
				led::event(led::Event::Slow);
                if constexpr(!std::is_same_v<led1, void>) {
                    led1::count(1);
                    led1::event(led1::Event::Medium);
                    led2::count(1);
                    led2::event(led2::Event::Medium);
                }
                if constexpr(!std::is_same_v<modcom, void>) {
					modcom::reset();
				}
                if constexpr(!std::is_same_v<sbus, void>) {
                    static_assert(std::is_same_v<crsf, void>);
                    static_assert(std::is_same_v<hwext, void>);
                    sbus::init();
#ifdef SBUS_SERIAL_INVERT
                    sbus::invert(true);
#else
                    sbus::invert(false); // TX16s can only read inverted SBUS (normal polarity)
#endif
				}
                if constexpr(!std::is_same_v<crsf, void>) {
                    static_assert(std::is_same_v<sbus, void>);
                    static_assert(std::is_same_v<hwext, void>);
                    crsf::init();
					crsf::baud(RC::Protokoll::Crsf::V4::baudrateHandset);
					crsf::activateSource(true);
				}
                if constexpr(!std::is_same_v<sumdv3, void>) {
                    static_assert(std::is_same_v<sbus, void>);
                    static_assert(std::is_same_v<hwext, void>);
                    sumdv3::init();
                }
                if constexpr(!std::is_same_v<sumdv1, void>) {
                    static_assert(std::is_same_v<sbus, void>);
                    static_assert(std::is_same_v<hwext, void>);
                    sumdv1::init();
                }
                break;
            case State::Calibration1:
                IO::outl<debug>("# Calib1");
				led::count(1);
                led::event(led::Event::Fast);
                if constexpr(!std::is_same_v<led1, void>) {
                    led1::count(2);
                    led1::event(led1::Event::Medium);
                    led2::count(2);
                    led2::event(led2::Event::Medium);
                }
                if constexpr(!std::is_same_v<modcom, void>) {
					modcom::setStatus(2);
				}
                if constexpr(!std::is_same_v<sbus, void>) {
                    static_assert(std::is_same_v<crsf, void>);
                    static_assert(std::is_same_v<hwext, void>);
                    sbus::init();
#ifdef SBUS_SERIAL_INVERT
                    sbus::invert(true);
#else
                    sbus::invert(false); // TX16s can only read inverted SBUS (normal polarity)
#endif
                }
                startCalibration();
				break;
            case State::Calibration2:
                IO::outl<debug>("# Calib2");
                led::count(2);
                led::event(led::Event::Fast);
                if constexpr(!std::is_same_v<led1, void>) {
                    led1::count(3);
                    led2::count(3);
                }
                break;
            }
        }
    }
    private:
	static inline void save() {
        if (const auto [ok, err] = Mcu::Stm32::savecfg(storage::eeprom, storage::eeprom_flash); ok) {
            IO::outl<debug>("# EEPROM OK");
        }
        else {
            IO::outl<debug>("# EEPROM NOK: ", err);
        }
    }
	static inline void startCalibration() {
        const uint16_t delta = 160;
		const uint16_t mid   = 2048;
		for(auto& c : storage::eeprom.calibration) {
			c.max = mid + delta;
			c.min = mid - delta;
			c.mid = mid;
            // c.span = 2 * delta;
		}
	}
    static inline void storeMid() {
        for(uint8_t i = 0; const auto& v : adc::values()) {
            storage::eeprom.calibration[i].mid = v;
            ++i;
        }
    }
	static inline void calibration() {
		for(uint8_t i = 0; const auto& v : adc::values()) {
			if (v > storage::eeprom.calibration[i].max) {
				storage::eeprom.calibration[i].max = v;
			}			
			if (v < storage::eeprom.calibration[i].min) {
				storage::eeprom.calibration[i].min = v;
			}			
            // storage::eeprom.calibration[i].span = storage::eeprom.calibration[i].max - storage::eeprom.calibration[i].min;
            // storage::eeprom.calibration[i].mid = (storage::eeprom.calibration[i].max + storage::eeprom.calibration[i].min) / 2;
			++i;
		}
	}
	static inline void print() {
		IO::out<debug>("adc:");
		for(const auto& v: adc::values()) {
			IO::out<debug>(" ", v);
		}
		IO::outl<debug>();		
		IO::out<debug>("sbus:");
		for(const auto& v: adap::values()) {
			IO::out<debug>(" ", v);
		}
		IO::outl<debug>();		
	}
	static inline void update() {
		mStateTick.on(telemTicks, []{
			for(uint8_t i = 0; const auto& v : adc::values()) {
				if constexpr(!std::is_same_v<crsf, void>) {
				}
				if constexpr(!std::is_same_v<modcom, void>) {
					modcom::setValue(i, v);
				}
				if constexpr(!std::is_same_v<hwext, void>) {
					hwext::setChannel(i++, v);
				}
				if constexpr(!std::is_same_v<sbus, void>) {
					const uint16_t scaled = adcToSbus(i, v);
					sbus::setChannel(i++, scaled);
				}
                if constexpr(!std::is_same_v<sumdv3, void>) {
                    const uint16_t scaled = adcToSbus(i, v);
                    sumdv3::set(i++, scaled);
                }
                if constexpr(!std::is_same_v<sumdv1, void>) {
                    const uint16_t scaled = adcToSbus(i, v);
                    sumdv1::set(i++, scaled);
                }
            }
			uint8_t i = 0;
			uint8_t newSwitchState = 0;
			Meta::visit<digitals>([&]<typename DI>(Meta::Wrapper<DI>){
									  const bool in = DI::read();
									  if constexpr(!std::is_same_v<adap, void>) {
										  adap::injectChannel(INJECT_DIGITAL_START + i, in ? RC::Protokoll::SBus::V2::max : RC::Protokoll::SBus::V2::min);						  										  
									  }
									  if constexpr(!std::is_same_v<crsf, void>) {
										  newSwitchState |= (!in ? (0b01 << i) : 0b00);
									  }
									  if constexpr(!std::is_same_v<modcom, void>) {
										  modcom::setSwitch(i, in);
									  }
									  if constexpr(!std::is_same_v<hwext, void>) {
											hwext::setSwitch(i, in);						  
									  }
									  if constexpr(!std::is_same_v<sbus, void>) {
											sbus::setChannel(INJECT_DIGITAL_START + i, in ? RC::Protokoll::SBus::V2::max : RC::Protokoll::SBus::V2::min);						  
									  }
                                      if constexpr(!std::is_same_v<sumdv3, void>) {
                                          sumdv3::set(INJECT_DIGITAL_START + i, in ? RC::Protokoll::SBus::V2::max : RC::Protokoll::SBus::V2::min);
                                          sumdv3::setSwitch(i, in ? 0 : 1);
                                      }
                                      ++i;
								  });
			if constexpr(!std::is_same_v<crsf, void>) {
				if (newSwitchState != mSwState) {
					IO::outl<debug>("switch: ", newSwitchState);
					uint8_t i = 1;
					mSwitchPacket[i++] = 0;
					mSwitchPacket[i++] = (uint8_t)RC::Protokoll::Crsf::V4::Type::Command;
					mSwitchPacket[i++] = (uint8_t)CRSF_SWITCH_COMMAND_ADDRESS;
					mSwitchPacket[i++] = (uint8_t)RC::Protokoll::Crsf::V4::Address::Handset;
					mSwitchPacket[i++] = (uint8_t)RC::Protokoll::Crsf::V4::CommandType::Switch;
					mSwitchPacket[i++] = (uint8_t)RC::Protokoll::Crsf::V4::SwitchCommand::Set;
					mSwitchPacket[i++] = SWITCH_ADDRESS; 
					mSwitchPacket[i++] = newSwitchState;
					
					CRC8BA csum8ba;
					for(uint8_t k = 2; k < i; ++k) {
						csum8ba += mSwitchPacket[k];
					}
					
					mSwitchPacket[i++] = csum8ba; // crc_ba
					mSwitchPacket[i++] = 0; // crc
					mSwitchPacket[1] = i - 2;
					mSwState = newSwitchState;
					crsf::forwardPacket(&mSwitchPacket[0], i);
				}
			}
		});		
	}
    static inline uint16_t adcToSbus(const uint8_t input, const uint16_t v) {
        const int32_t v1 = (v - storage::eeprom.calibration[input].mid);

        int32_t sb = 0;
        if (v1 >= 0) {
            sb = (v1 * RC::Protokoll::SBus::V2::span) / (storage::eeprom.calibration[input].max - storage::eeprom.calibration[input].mid) + RC::Protokoll::SBus::V2::mid;
        }
        else {
            sb = (v1 * RC::Protokoll::SBus::V2::span) / (storage::eeprom.calibration[input].mid - storage::eeprom.calibration[input].min) + RC::Protokoll::SBus::V2::mid;
        }
        // const int32_t sb = (v1 * RC::Protokoll::SBus::V2::amp) / storage::eeprom.calibration[input].span + RC::Protokoll::SBus::V2::mid;
        return std::clamp(sb, (int32_t)RC::Protokoll::SBus::V2::min, (int32_t)RC::Protokoll::SBus::V2::max);
    }
    static inline void checkButton() {
        if constexpr(!std::is_same_v<btn, void>) {
            if (const auto e = btn::event(); e == btn::Press::Long) {
                event(Event::ButtonPress);
            }
        }
    }
	static inline uint8_t mSwState = 0;
	static inline std::array<uint8_t, 16> mSwitchPacket{(uint8_t)RC::Protokoll::Crsf::V4::Address::StartByte};
    static inline etl::Event<Event> mEvent;
    static inline External::Tick<systemTimer> mStateTick;
    static inline External::Tick<systemTimer> mDebugTick;
	static inline External::Tick<systemTimer> mWaitTick;
    static inline State mState = State::Undefined;
};
