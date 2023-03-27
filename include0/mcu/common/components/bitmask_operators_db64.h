/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017, 2018 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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
#include <std/utility>

namespace AVR {
    namespace Series0 {
    }
}

namespace std {
    template<> struct enable_bitmask_operators<AVR::SeriesDb::PortRegister::PinCtrl_t> : std::true_type {};
    template<> struct enable_bitmask_operators<AVR::SeriesDa::Adc::CtrlC_t> : std::true_type {};
    template<> struct enable_bitmask_operators<AVR::Series0::AdComparator::CtrlA1_t> : std::true_type {};
    template<> struct enable_bitmask_operators<AVR::SeriesDb::Usart::CtrlA_t> : std::true_type {};
    template<> struct enable_bitmask_operators<AVR::SeriesDb::Usart::CtrlB_t> : std::true_type {};
    template<> struct enable_bitmask_operators<AVR::SeriesDb::Usart::CtrlC_t> : std::true_type {};
    template<> struct enable_bitmask_operators<AVR::Series0::TCA::CtrlA_t> : std::true_type {};
    template<> struct enable_bitmask_operators<AVR::Series0::TCA::CtrlB_t> : std::true_type {};
    template<> struct enable_bitmask_operators<AVR::Series0::TCA::Intctrl_t> : std::true_type {};
    template<> struct enable_bitmask_operators<AVR::SeriesDa::TCB::CtrlA_t> : std::true_type {};
    template<> struct enable_bitmask_operators<AVR::SeriesDb::Rtc::CtrlA_t> : std::true_type {};
    template<> struct enable_bitmask_operators<AVR::SeriesDb::Rtc::PitCtrlA_t> : std::true_type {};
    template<> struct enable_bitmask_operators<AVR::Series0::Cpu::SReg_t> : std::true_type {};
    template<> struct enable_bitmask_operators<AVR::SeriesDb::Portmux::UsartRouteA_t> : std::true_type {};
    template<> struct enable_bitmask_operators<AVR::SeriesDb::Portmux::UsartRouteB_t> : std::true_type {};
    template<> struct enable_bitmask_operators<AVR::SeriesDb::Portmux::CclRoute_t> : std::true_type {};
    template<> struct enable_bitmask_operators<AVR::SeriesDb::Portmux::TcbRoute_t> : std::true_type {};
    template<> struct enable_bitmask_operators<AVR::SeriesDa::Ccl::CtrlA_t> : std::true_type {};
    template<> struct enable_bitmask_operators<AVR::SeriesDa::Ccl::Lut0CtrlA_t> : std::true_type {};

    template<> struct enable_bitmask_operators<AVR::SeriesDb::Dac::CtrlA_t> : std::true_type {};
    template<> struct enable_bitmask_operators<AVR::Series0::Sleep::CtrlA_t> : std::true_type {};

    template<> struct enable_bitmask_operators<AVR::Series0::Spi::CtrlA1_t> : std::true_type {};
    template<> struct enable_bitmask_operators<AVR::Series0::Spi::CtrlB1_t> : std::true_type {};

    template<> struct enable_bitmask_operators<AVR::SeriesDa::Twi::MStatus_t> : std::true_type {};
    template<> struct enable_bitmask_operators<AVR::SeriesDa::Twi::MCtrlA_t> : std::true_type {};
    template<> struct enable_bitmask_operators<AVR::SeriesDa::Twi::MCtrlB_t> : std::true_type {};
}

