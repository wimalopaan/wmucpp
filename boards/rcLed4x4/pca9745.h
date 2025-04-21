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

#include "spi.h"
#include "mcu/alternate.h"

namespace External {

    template<uint8_t N, typename Config, typename MCU = DefaultMcu>
    struct PCA9745 {
        using mosi = Config::mosi;
        using miso = Config::miso;
        using clk  = Config::clk;
        using cs   = Config::cs;
        using debug = Config::debug;
        using rxDmaComponent = Config::rxDmaComponent;
        using txDmaComponent = Config::txDmaComponent;

        struct SpiConfig {
            using value_type = uint16_t;
            static inline constexpr size_t size = 16;
            struct Isr {
                static inline constexpr bool txComplete = true;
            };
            using rxDmaComponent = PCA9745::rxDmaComponent;
            using txDmaComponent = PCA9745::txDmaComponent;
        };

        using spi = Mcu::Stm::V2::Spi<N, SpiConfig, MCU>;

        static inline void init() {
            static constexpr uint8_t miso_af = Mcu::Stm::AlternateFunctions::mapper_v<miso, spi, Mcu::Stm::AlternateFunctions::MISO>;
            miso::afunction(miso_af);
            static constexpr uint8_t mosi_af = Mcu::Stm::AlternateFunctions::mapper_v<mosi, spi, Mcu::Stm::AlternateFunctions::MOSI>;
            mosi::afunction(mosi_af);
            static constexpr uint8_t clk_af = Mcu::Stm::AlternateFunctions::mapper_v<clk, spi, Mcu::Stm::AlternateFunctions::SCLK>;
            clk::afunction(clk_af);
            static constexpr uint8_t cs_af = Mcu::Stm::AlternateFunctions::mapper_v<cs, spi, Mcu::Stm::AlternateFunctions::CS>;
            cs::afunction(cs_af);

            spi::init();
        }

    };
}
