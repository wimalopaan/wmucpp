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
    namespace SeriesDb {
        struct Rtc final {
            template<typename T>
            using pp_t = AVR::Util::PrescalerPair<T>;

            enum class CtrlA_t : uint8_t {
                standby = RTC_RUNSTDBY_bm,
                correction = RTC_CORREN_bm,
                enable = RTC_RTCEN_bm,
            };
            static inline constexpr std::array<pp_t<CtrlA_t>, 16> prescalerValues {
                pp_t<CtrlA_t>{CtrlA_t((0x00 << 3)), 1},
                pp_t<CtrlA_t>{CtrlA_t((0x01 << 3)), 2},
                pp_t<CtrlA_t>{CtrlA_t((0x02 << 3)), 4},
                pp_t<CtrlA_t>{CtrlA_t((0x03 << 3)), 8},
                pp_t<CtrlA_t>{CtrlA_t((0x04 << 3)), 16},
                pp_t<CtrlA_t>{CtrlA_t((0x05 << 3)), 32},
                pp_t<CtrlA_t>{CtrlA_t((0x06 << 3)), 64},
                pp_t<CtrlA_t>{CtrlA_t((0x07 << 3)), 128},
                pp_t<CtrlA_t>{CtrlA_t((0x08 << 3)), 256},
                pp_t<CtrlA_t>{CtrlA_t((0x09 << 3)), 512},
                pp_t<CtrlA_t>{CtrlA_t((0x01 << 3)), 1024},
                pp_t<CtrlA_t>{CtrlA_t((0x0b << 3)), 2048},
                pp_t<CtrlA_t>{CtrlA_t((0x0c << 3)), 4096},
                pp_t<CtrlA_t>{CtrlA_t((0x0d << 3)), 8192},
                pp_t<CtrlA_t>{CtrlA_t((0x0e << 3)), 16384},
                pp_t<CtrlA_t>{CtrlA_t((0x0f << 3)), 32768},
            };
            ControlRegister<Rtc, CtrlA_t> ctrla;
            
            enum class Status_t : uint8_t {
                cmpbusy = RTC_CMPBUSY_bm,
                perbusy = RTC_PERBUSY_bm,
                cntbusy = RTC_CNTBUSY_bm,
                ctrlabusy = RTC_CTRLABUSY_bm,
            };
            FlagRegister<Rtc, Status_t, ReadOnly> status;
            
            enum class IntCtrl_t : uint8_t {
                cmp = RTC_CMP_bm,
                ovf = RTC_OVF_bm,
            };
            ControlRegister<Rtc, IntCtrl_t> intctrl;

            enum class IntFlags_t : uint8_t {
                cmp = RTC_CMP_bm,
                ovf = RTC_OVF_bm,
            };
            FlagRegister<Rtc, IntFlags_t, ReadWrite> intflags;
            
            DataRegister<Rtc, ReadWrite, std::byte> temp;

            enum class DbgCtrl_t : uint8_t {
                DBG = RTC_DBGRUN_bm,
            };
            ControlRegister<Rtc, DbgCtrl_t> dbgctrl;

            DataRegister<Rtc, ReadWrite, std::byte> calib;
            
            enum class ClockSel_t : uint8_t {
                cs1 = RTC_CLKSEL_1_bm,
                cs0 = RTC_CLKSEL_0_bm,
            };
            ControlRegister<Rtc, ClockSel_t> clksel;

            DataRegister<Rtc, ReadWrite, uint16_t> cnt;
            DataRegister<Rtc, ReadWrite, uint16_t> per;
            DataRegister<Rtc, ReadWrite, uint16_t> cmp;

            volatile uint8_t padding[0x0f - 0x0e + 1];
            
            enum class PitCtrlA_t : uint8_t {
                enable = RTC_PITEN_bm,
            };
            static inline constexpr std::array<pp_t<PitCtrlA_t>, 15> pitPrescalerValues {
                pp_t<PitCtrlA_t>{PitCtrlA_t((0x00 << 3)), 0},
                pp_t<PitCtrlA_t>{PitCtrlA_t((0x01 << 3)), 4},
                pp_t<PitCtrlA_t>{PitCtrlA_t((0x02 << 3)), 8},
                pp_t<PitCtrlA_t>{PitCtrlA_t((0x03 << 3)), 16},
                pp_t<PitCtrlA_t>{PitCtrlA_t((0x04 << 3)), 32},
                pp_t<PitCtrlA_t>{PitCtrlA_t((0x05 << 3)), 64},
                pp_t<PitCtrlA_t>{PitCtrlA_t((0x06 << 3)), 128},
                pp_t<PitCtrlA_t>{PitCtrlA_t((0x07 << 3)), 256},
                pp_t<PitCtrlA_t>{PitCtrlA_t((0x08 << 3)), 512},
                pp_t<PitCtrlA_t>{PitCtrlA_t((0x09 << 3)), 1024},
                pp_t<PitCtrlA_t>{PitCtrlA_t((0x0a << 3)), 2048},
                pp_t<PitCtrlA_t>{PitCtrlA_t((0x0b << 3)), 4096},
                pp_t<PitCtrlA_t>{PitCtrlA_t((0x0c << 3)), 8192},
                pp_t<PitCtrlA_t>{PitCtrlA_t((0x0d << 3)), 16384},
                pp_t<PitCtrlA_t>{PitCtrlA_t((0x0e << 3)), 32768},
            };
            ControlRegister<Rtc, PitCtrlA_t> pitctrla;
            
            enum class PitStatus_t : uint8_t {
                ctrlbusy = RTC_CTRLBUSY_bm,
            };
            FlagRegister<Rtc, PitStatus_t, ReadOnly> pitstatus;

            enum class PitIntCtrl_t : uint8_t {
                pi = RTC_PI_bm,
            };
            ControlRegister<Rtc, PitIntCtrl_t> pitintctrl;

            enum class PitIntFlags_t : uint8_t {
                pi = RTC_PI_bm,
            };
            FlagRegister<Rtc, PitIntFlags_t, ReadWrite> pitintflags;

            volatile uint8_t padding1;

            enum class PitDbgFlags_t : uint8_t {
                dbgrun = RTC_DBGRUN_bm,
            };
            FlagRegister<Rtc, PitDbgFlags_t, ReadOnly> pitdbgctrl;

            static inline constexpr uintptr_t address = 0x0140;
        };
        
        static_assert(sizeof(Rtc) == 0x16);

    }
}
