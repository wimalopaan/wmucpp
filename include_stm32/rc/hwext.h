/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2016 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include <etl/event.h>
#include <etl/algorithm.h>

#include "rc/rc_2.h"

namespace External::EdgeTx {
    using namespace std::literals::chrono_literals;

	template<typename T = uint8_t>
	struct CheckSum {
		T operator+=(const T b) {
			mValue += b;
			return b;
		}
		void reset() {
			mValue = T{0};
		}
		operator T() const {
			return mValue;
		}
	private:
		T mValue{};
	};
	
	template<uint8_t N, typename Config, typename MCU = DefaultMcu>
    struct ModuleCom {
		static inline constexpr uint8_t number = N;
		using clock = Config::clock;
		using systemTimer = Config::systemTimer;
		using debug = Config::debug;
		using rxpin = Config::rxpin;
		using txpin = Config::txpin;
		using tp = Config::tp;
		using callback = Config::callback;
		using storage = Config::storage;
		using value_t = uint8_t;

		struct UartConfig {
			using Clock = clock;
			using ValueType = ModuleCom::value_t;
			static inline constexpr auto mode = Mcu::Stm::Uarts::Mode::FullDuplex;
			static inline constexpr uint32_t baudrate = 115'200;
			struct Rx {
				using DmaChComponent = Config::dmaChRead;
				static inline constexpr bool enable = true;
				static inline constexpr size_t size = 16;
				static inline constexpr size_t idleMinSize = 4;
			};
			struct Tx {
				using DmaChComponent = Config::dmaChWrite;
				static inline constexpr bool singleBuffer = true;
				static inline constexpr bool enable = true;
				static inline constexpr size_t size = 64;
			};
			struct Isr {
				static inline constexpr bool idle = true;
				static inline constexpr bool txComplete = true;
			};
			using tp = Config::tp;
		};
		using uart = Mcu::Stm::V4::Uart<N, UartConfig, MCU>;

		static inline void setSwitch(const uint8_t i, const bool s) {
			const uint8_t mask = (1 << i);
			if (s) {
				mSwitch |= mask;
			}			
			else {
				mSwitch &= ~mask;
			}
		}
		static inline void setValue(const uint8_t i, const uint16_t v) {
			if (i < mValues.size()) {
				mValues[i] = v;
			}
		}		
		static inline void setStatus(const uint8_t s) {
			mStatus = s;
		}
		static inline void init() {
			IO::outl<debug>("# ModCom init");
			Mcu::Arm::Atomic::access([]{
				uart::init();
				mActive = true;
				mState = State::Init;
				mRxEvent = Event::None;
			});
			static constexpr uint8_t rxaf = Mcu::Stm::AlternateFunctions::mapper_v<rxpin, uart, Mcu::Stm::AlternateFunctions::RX>;
			rxpin::afunction(rxaf);
			rxpin::template pullup<true>();
			static constexpr uint8_t txaf = Mcu::Stm::AlternateFunctions::mapper_v<txpin, uart, Mcu::Stm::AlternateFunctions::TX>;
			txpin::afunction(txaf);
		}
		static inline void reset() {
			IO::outl<debug>("# ModCom reset");
			Mcu::Arm::Atomic::access([]{
				mActive = false;
				uart::reset();
			});
			rxpin::analog();
			txpin::analog();
		}
		struct Isr {
			static inline void onTransferComplete(const auto f) {
				if (mActive) {
					const auto fEnable = [&]{
						f();
					};
					uart::Isr::onTransferComplete(fEnable);
				}
			}
			static inline void onIdle(const auto f) {
				if (mActive) {
					const auto f2 = [&](const volatile uint8_t* const data, const uint16_t size){
						f();
						if (validityCheck(data, size)) {
							mRxEvent = Event::ReceiveComplete;
							return true;
						}
						return false;
					};
					uart::Isr::onIdle(f2);
				}
			}
		};

		static inline External::Tick<systemTimer> initTicks{100ms};
		static inline External::Tick<systemTimer> updateTicks{100ms};
		
		enum class State : uint8_t {Init, Run};
		enum class Event : uint8_t {None, ReceiveComplete};

		static inline void periodic() {
			if (!mActive) return;
			if (mRxEvent.is(Event::ReceiveComplete)) {
				readReply();
			}
		}
		static inline void ratePeriodic() {
			if (!mActive) return;
			const auto oldState = mState;
			++mStateTick;
			switch(mState) {
			case State::Init:
				mStateTick.on(initTicks, []{
					mState = State::Run;
				});
				break;
			case State::Run:
				mStateTick.on(updateTicks, []{
					sendData();
				});
				break;
			}
			if (oldState != mState) {
				mStateTick.reset();
				switch(mState) {
				case State::Init:
					break;
				case State::Run:
					break;
				}
			}
		}
		private:
		static inline void sendData() {
			uart::fillSendBuffer([](auto& data){
				CheckSum<uint8_t> csum;
				uint8_t i = 0;
				data[i++] = 0xaa;
				data[i++] = 0; // length of payload
				data[i++] = (csum += 0x01); // type calib data
				data[i++] = (csum += mStatus); // status bits
				static_assert(storage::eeprom.calibration.size() == mValues.size());
				data[i++] = (csum += u_int8_t(storage::eeprom.calibration.size())); // number of 16bit values
				for(const auto& c : storage::eeprom.calibration) {
					data[i++] = (csum += (uint8_t)etl::nth_byte<0>(c.max));
					data[i++] = (csum += (uint8_t)etl::nth_byte<1>(c.max));
					data[i++] = (csum += (uint8_t)etl::nth_byte<0>(c.min));
					data[i++] = (csum += (uint8_t)etl::nth_byte<1>(c.min));
				}
				for(const auto& v : mValues) {
					data[i++] = (csum += (uint8_t)etl::nth_byte<0>(v));
					data[i++] = (csum += (uint8_t)etl::nth_byte<1>(v));
				}
				data[i++] = (csum += mSwitch); // switches
				data[i++] = csum;
				data[1] = i - 2; // set length
				return i;
			});			
		}
		static inline bool validityCheck(const volatile uint8_t* const data, const uint16_t length) {
			if (data[0] != 0xaa) {
				return false;
			}
			if (length < 5) {
				return false;
			}
			const uint8_t paylength = data[1];
			return crcCheck(data, paylength);
		}
		static inline bool crcCheck(auto data, const uint8_t paylength) {
			CheckSum<uint8_t> csum;
			for(int8_t i = 0; i < paylength - 1; ++i) {
				csum += data[i + 2];
			}
			if (csum == data[paylength - 1 + 2]) {
				return true;
			}
			return false;
		}
		static inline void readReply() {
			uart::readBuffer([&](const auto& data){
				if (data[2] == 0x02) { // command
					if (data[3] == 0x01) { // heartbeat
						callback::heartbeat();
					}					
					else if (data[3] == 0x02) { // start calib
						callback::startCalib();
					}					
					else if (data[3] == 0x03) { // stop calib
						callback::stopCalib();
					}					
					else if (data[3] == 0x04) { // normal
						callback::startNormal();
					}					
				}
			});
		}
		static inline std::array<uint16_t, 8> mValues{};
		static inline volatile bool mActive = false;
		static inline uint8_t mSwitch = 0;
		static inline uint8_t mStatus = 0;
		static inline volatile etl::Event<Event> mRxEvent;
		static inline volatile State mState = State::Init;
		static inline External::Tick<systemTimer> mStateTick;
    };
	
	
    template<uint8_t N, typename Config, typename MCU = DefaultMcu>
    struct HwExtension {
        using clock = Config::clock;
        using systemTimer = Config::systemTimer;
        using debug = Config::debug;
        using dmaChComponent = Config::dmaChComponent;
        // using input = Config::input;
        using storage = Config::storage;
        using pin = Config::pin;
        using tp = Config::tp;

        struct UartConfig {
            using Clock = clock;
            using ValueType = uint8_t;
            using DmaChComponent = dmaChComponent;
            static inline constexpr Mcu::Stm::Uarts::Mode mode = Mcu::Stm::Uarts::Mode::HalfDuplex;
            static inline constexpr uint32_t baudrate = 115'200;
            struct Rx {
                static inline constexpr size_t size = 64;
                static inline constexpr size_t idleMinSize = 4;
            };
            struct Tx {
                static inline constexpr bool singleBuffer = true;
                static inline constexpr bool enable = true;
                static inline constexpr size_t size = 64;
            };
            struct Isr {
                static inline constexpr bool idle = true;
                static inline constexpr bool txComplete = true;
            };
            using tp = Config::tp;
        };

        using uart = Mcu::Stm::V4::Uart<N, UartConfig, MCU>;

        static inline void init() {
            static constexpr uint8_t af = Mcu::Stm::AlternateFunctions::mapper_v<pin, uart, Mcu::Stm::AlternateFunctions::TX>;
            IO::outl<debug>("# HwExt init");
            for(auto& v : mValues) {
                v = RC::Protokoll::Crsf::V4::mid;
            }
            Mcu::Arm::Atomic::access([]{
                mState = State::Init;
                mErrorCount = 0;
                mEvent = Event::None;
                mActive = true;
                uart::init();
            });
            pin::afunction(af);
            pin::template pullup<true>();
        }
        static inline void reset() {
            IO::outl<debug>("# HwExt reset");
            Mcu::Arm::Atomic::access([]{
                uart::reset();
                mActive = false;
            });
            pin::analog();
        }

        static inline constexpr auto& eeprom = storage::eeprom;

        enum class State : uint8_t {Init, Run};
        enum class Event : uint8_t {None, ReceiveComplete, Send};

        static inline void event(const Event e) {
            mEvent = e;
        }
        static inline void periodic() {
            switch(mState) {
            case State::Init:
                break;
            case State::Run:
                if (mEvent.is(Event::ReceiveComplete)) {
                    readReply();
                }
                else if (mEvent.is(Event::Send)) {
                    send();
                }
                break;
            }
        }

        static inline constexpr External::Tick<systemTimer> initTicks{1000ms};
        static inline constexpr External::Tick<systemTimer> masterTicks{100ms};

        static inline void ratePeriodic() {
            const auto oldState = mState;
            ++mStateTick;
            switch(mState) {
            case State::Init:
                mStateTick.on(initTicks, []{
                    mState = State::Run;
                });
                break;
            case State::Run:
                if (eeprom.controllerNumber == 0) {
                    mStateTick.on(masterTicks, []{
                        send();
                    });
                }
                break;
            }
            if (oldState != mState) {
                mStateTick.reset();
                switch(mState) {
                case State::Init:
                    break;
                case State::Run:
                    break;
                }
            }
        }
        struct Isr {
            static inline void onIdle(const auto f) {
                if (mActive) {
                    const auto f2 = [&](const volatile uint8_t* const data, const uint16_t size){
                        f();
                        if (validityCheck(data, size)) {
                            event(Event::ReceiveComplete);
                            return true;
                        }
                        return false;
                    };
                    uart::Isr::onIdle(f2);
                }
            }
            static inline void onTransferComplete(const auto f) {
                if (mActive) {
                    const auto fEnable = [&]{
                        f();
                        uart::template rxEnable<true>();
                    };
                    uart::Isr::onTransferComplete(fEnable);
                }
            }
        };
        static inline auto errorCount() {
            return mErrorCount;
        }
        static inline void setSw(const uint64_t sw) {
            IO::outl<debug>("# set: ", sw);
            mSwitches = sw;
        }
        static inline void setSw(const uint8_t sw, const bool on) {
            // IO::outl<debug>("# set2: ", sw, " ", (uint8_t)on);
            const uint64_t mask = (uint64_t{1} << sw);
            mSwitches = (mSwitches & ~mask) | (on ? mask : 0);
        }
        static inline void setChannel(const uint8_t i, const uint16_t v) {
            mValues[i] = v;
        }
        private:
        static inline void send() {
            static bool sendSwitches = true;
            if (sendSwitches) {
                uart::fillSendBuffer([](auto& data){
                    data[0] = 0xaa;
                    data[1] = eeprom.controllerNumber;
                    data[2] = 0x00; // type
                    data[3] = 0x08; // length

                    uint8_t cs = 0;
                    for(uint8_t i = 0; i < 8; ++i) {
                        data[4 + i] = (mSwitches >> (i * 8));
                        cs += data[4 + i];
                    }
                    data[4 + 8] = cs;
                    return 4 + 8 + 1;
                });
            }
            else {
                if (eeprom.prop8mode) {
                    uart::fillSendBuffer([](auto& data){
                        data[0] = 0xaa;
                        data[1] = eeprom.controllerNumber;
                        data[2] = 0x01; // type
                        data[3] = 16; // 16 8-bit values

                        uint8_t cs = 0;
                        for(uint8_t i = 0; i < 16; ++i) {
                            const int vc = mValues[i] - RC::Protokoll::Crsf::V4::mid;
                            const int vcn = (vc + vc / 8 + vc / 32);
                            const int v = std::clamp(vcn + 128, 0, 255);
                            const uint8_t lsb = v;
                            data[i + 4] = lsb;
                            cs += lsb;
                        }
                        data[4 + 16] = cs;
                        return 4 + 16 + 1;
                    });
                }
                else {
                    uart::fillSendBuffer([](auto& data){
                        data[0] = 0xaa;
                        data[1] = eeprom.controllerNumber;
                        data[2] = 0x02; // type
                        data[3] = 32; // 16 16-bit values

                        uint8_t cs = 0;
                        for(uint8_t i = 0; i < 16; ++i) {
                            const int vc = mValues[i] - RC::Protokoll::Crsf::V4::mid;
                            const int vcn = (vc + vc / 4); // * 1.25
                            const int v = std::clamp(vcn + 1024, 0, 2048);
                            const uint8_t lsb = v;
                            const uint8_t msb = v >> 8;
                            data[2 * i + 4] = lsb;
                            cs += lsb;
                            data[2 * i + 1 + 4] = msb;
                            cs += msb;
                        }
                        data[4 + 32] = cs;
                        return 4 + 32 + 1;
                    });
                }
            }
            sendSwitches = !sendSwitches;
        }
        static inline bool validityCheck(const volatile uint8_t* const data, const uint16_t size) {
            if (size < 6) {
                return false;
            }
            if (data[0] != 0xaa) {
                return false;
            }
            const uint8_t l = data[3];
            uint8_t cs = 0;
            for(uint8_t i = 0; i < l; ++i) {
                cs += data[4 + i];
            }
            const uint8_t ics = data[4 + l];
            if (cs != ics) {
                ++mErrorCount;
                return false;
            }
            return true;
        }
        static inline void readReply() {
            uart::readBuffer([&](const auto& data){
                const uint8_t inController = data[1];
                if ((inController + 1) == eeprom.controllerNumber) {
                    mEvent = Event::Send;
                }
            });
        }
        static inline uint64_t mSwitches = 0;
        static inline std::array<uint16_t, 16> mValues{};
        static inline uint16_t mErrorCount = 0;
        static inline etl::Event<Event> mEvent;
        static inline volatile State mState = State::Init;
        static inline External::Tick<systemTimer> mStateTick;
        static inline volatile bool mActive = false;
    };
}

