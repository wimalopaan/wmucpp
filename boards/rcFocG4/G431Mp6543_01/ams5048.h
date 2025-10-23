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

#include "mcu/alternate.h"
#include "etl/fifo.h"
#include "spi.h"
#include "tick.h"

namespace External {
    template<uint8_t N, typename Config, typename MCU = DefaultMcu>
    struct AMS5048 {
        using systemTimer = Config::systemTimer;
        using mosi = Config::mosi;
        using miso = Config::miso;
        using clk  = Config::clk;
        using cs   = Config::cs;

        using debug = Config::debug;
        using txDmaComponent = Config::txDmaComponent;
		using rxDmaComponent = Config::rxDmaComponent;

		using tp = Config::tp;

		static inline constexpr uint16_t span = (1 << 14);
		
        struct SpiConfig {
            using debug = Config::debug;
            static inline constexpr size_t clockDivider = 32; // 5.5 MHz (max. 10MHz)
            using value_type = uint16_t;
            static inline constexpr size_t size = 1;
			static inline constexpr bool useNSS = false; // can't use hw nss with falling edge
			static inline constexpr bool useFallingEdge = true;
            struct Isr {
                static inline constexpr bool txComplete = false;
				static inline constexpr bool rxComplete = false;
            };
            using txDmaComponent = AMS5048::txDmaComponent;
			using rxDmaComponent = AMS5048::rxDmaComponent;
        };

        using spi = Mcu::Stm::V2::Spi<N, SpiConfig, MCU>;

        enum class State : uint8_t {Idle, Write, Wait};

        static inline constexpr External::Tick<systemTimer> initTicks{100ms};

        static inline void init() {
            IO::outl<debug>("# AMS5048 init");
            Mcu::Arm::Atomic::access([]{
                mActive = true;
                mState = State::Idle;
                spi::init();
            });
            static constexpr uint8_t miso_af = Mcu::Stm::AlternateFunctions::mapper_v<miso, spi, Mcu::Stm::AlternateFunctions::MISO>;
            miso::afunction(miso_af);
            static constexpr uint8_t mosi_af = Mcu::Stm::AlternateFunctions::mapper_v<mosi, spi, Mcu::Stm::AlternateFunctions::MOSI>;
            mosi::afunction(mosi_af);
            static constexpr uint8_t clk_af = Mcu::Stm::AlternateFunctions::mapper_v<clk, spi, Mcu::Stm::AlternateFunctions::SCLK>;
            clk::afunction(clk_af);
			// static constexpr uint8_t cs_af = Mcu::Stm::AlternateFunctions::mapper_v<cs, spi, Mcu::Stm::AlternateFunctions::CS>;
   //          cs::afunction(cs_af);
			cs::template dir<Mcu::Output>();
			cs::set();
        }
        static inline void reset() {
            IO::outl<debug>("# AMS5048 reset");
            Mcu::Arm::Atomic::access([]{
                mActive = false;
                spi::reset();
            });
            miso::analog();
            mosi::analog();
            clk::analog();
            cs::analog();
        }
		static inline int32_t value() {
			const int16_t v = 0x3fff & spi::rxData()[0];
			if (v < (mLastValue - (span / 2))) {
				++mRotations;
			}
			else if (v > (mLastValue + (span / 2))) {
				--mRotations;
			}
			mLastValue = v;
			return (mRotations * span) + v;			
		}
        static inline void periodic() {
			switch(mState) {
			case State::Idle:
				cs::reset();
				spi::fillSendBuffer([](auto& data){
					data[0] = 0xffff;
					return 1;
				});
				mState = State::Write;
				break;
			case State::Write:
				// the AMS5048 uses falling clock polarity, but in this mode the STM32G4xx SPI
				// is not able to do the NSS (CS) pin management: this has to be done in software.
				// Therefore, we can't use DMA circular mode (no interrupts). Instead we have 
				// to do the CS pin by software -> polling here
				if (!spi::busy()) {
					cs::set();
					mState = State::Wait;
				}
				break;
			case State::Wait: 
				// one extra wait state (deassert time: min 350ns)
				mState = State::Idle;
			}
        }
        private:
		static inline uint16_t mLastValue = 0;
		static inline int16_t mRotations = 0;
        static inline bool mActive = false;
        static inline State mState = State::Idle;
        static inline External::Tick<systemTimer> mStateTick;
    };
}
