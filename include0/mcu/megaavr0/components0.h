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

#include "../common/components.h"

//namespace AVR {
//    namespace Series0 {
//        struct Usart {
//            static constexpr const uint8_t count = 4;
            
//            DataRegister<Usart, ReadOnly, std::byte> rxd;
            
//            enum class RxDataH_t : uint8_t {
//                rxcif = USART_RXCIF_bm,
//                bufovl = USART_BUFOVF_bm,
//                ferr = USART_FERR_bm,
//                perr = USART_PERR_bm,
//                data8 = USART_DATA8_bm,
//            };
//            ControlRegister<Usart, RxDataH_t> rxdh;
            
//            DataRegister<Usart, ReadWrite, std::byte> txd;
//            DataRegister<Usart, ReadWrite, std::byte> txdh;

//            enum class Status_t : uint8_t {
//                rxc = USART_RXCIF_bm,
//                txc = USART_TXCIF_bm,
//                dre = USART_DREIF_bm,
//                rxs  = USART_RXSIF_bm,
//                isf = USART_ISFIF_bm,
//                bdf = USART_BDF_bm,
//                wfb = USART_WFB_bm,
//            };
//            FlagRegister<Usart, Status_t, ReadWrite> status;

//            enum class CtrlA_t : uint8_t {
//                rxcie = USART_RXCIE_bm,
//                txcie = USART_TXCIE_bm,
//                dreie = USART_DREIE_bm,
//                rxsie = USART_RXSIE_bm,
//                lbme  = USART_LBME_bm,
//                abeie  = USART_ABEIE_bm,
//                rs485_1 = USART_RS4851_bm,
//                rs485_0 = USART_RS4850_bm,
//            };
//            ControlRegister<Usart, CtrlA_t> ctrla;

//            enum class CtrlB_t : uint8_t {
//                rxen = USART_RXEN_bm,
//                txen = USART_TXEN_bm,
//                sfden = USART_SFDEN_bm,
//                odme = USART_ODME_bm,
//                rxm0= USART_RXMODE0_bm,
//                rxm1= USART_RXMODE1_bm,
//                mpcm= USART_MPCM_bm,
//            };
//            ControlRegister<Usart, CtrlB_t> ctrlb;

//            enum class CtrlC_t : uint8_t {
//                cmode1 = USART_CMODE1_bm,
//                cmode0 = USART_CMODE0_bm,
//                pmode1 = USART_PMODE1_bm,
//                pmode0 = USART_PMODE0_bm,
//                sbmode = USART_SBMODE_bm,
//                chsize2 = USART_CHSIZE2_bm,
//                chsize1 = USART_CHSIZE1_bm,
//                chsize0 = USART_CHSIZE0_bm,
//            };
//            ControlRegister<Usart, CtrlC_t> ctrlc;

//            DataRegister<Usart, ReadWrite, uint16_t> baud;
            
//            volatile uint8_t reserved;
//            template<int N> struct Address;
//        };
        
//        struct Rtc final {
//            template<typename T>
//            using pp_t = AVR::Util::PrescalerPair<T>;

//            enum class CtrlA_t : uint8_t {
//                standby = RTC_RUNSTDBY_bm,
////                correction = RTC_CORREN_bm, // fehlt?
//                enable = RTC_RTCEN_bm,
//            };
//            static inline constexpr std::array<pp_t<CtrlA_t>, 16> prescalerValues {
//                pp_t<CtrlA_t>{CtrlA_t((0x00 << 3)), 1},
//                pp_t<CtrlA_t>{CtrlA_t((0x01 << 3)), 2},
//                pp_t<CtrlA_t>{CtrlA_t((0x02 << 3)), 4},
//                pp_t<CtrlA_t>{CtrlA_t((0x03 << 3)), 8},
//                pp_t<CtrlA_t>{CtrlA_t((0x04 << 3)), 16},
//                pp_t<CtrlA_t>{CtrlA_t((0x05 << 3)), 32},
//                pp_t<CtrlA_t>{CtrlA_t((0x06 << 3)), 64},
//                pp_t<CtrlA_t>{CtrlA_t((0x07 << 3)), 128},
//                pp_t<CtrlA_t>{CtrlA_t((0x08 << 3)), 256},
//                pp_t<CtrlA_t>{CtrlA_t((0x09 << 3)), 512},
//                pp_t<CtrlA_t>{CtrlA_t((0x01 << 3)), 1024},
//                pp_t<CtrlA_t>{CtrlA_t((0x0b << 3)), 2048},
//                pp_t<CtrlA_t>{CtrlA_t((0x0c << 3)), 4096},
//                pp_t<CtrlA_t>{CtrlA_t((0x0d << 3)), 8192},
//                pp_t<CtrlA_t>{CtrlA_t((0x0e << 3)), 16384},
//                pp_t<CtrlA_t>{CtrlA_t((0x0f << 3)), 32768},
//            };
//            ControlRegister<Rtc, CtrlA_t> ctrla;
            
//            enum class Status_t : uint8_t {
//                cmpbusy = RTC_CMPBUSY_bm,
//                perbusy = RTC_PERBUSY_bm,
//                cntbusy = RTC_CNTBUSY_bm,
//                ctrlabusy = RTC_CTRLABUSY_bm,
//            };
//            FlagRegister<Rtc, Status_t, ReadOnly> status;
            
//            enum class IntCtrl_t : uint8_t {
//                cmp = RTC_CMP_bm,
//                ovf = RTC_OVF_bm,
//            };
//            ControlRegister<Rtc, IntCtrl_t> intctrl;

//            enum class IntFlags_t : uint8_t {
//                cmp = RTC_CMP_bm,
//                ovf = RTC_OVF_bm,
//            };
//            FlagRegister<Rtc, IntFlags_t, ReadWrite> intflags;
            
//            DataRegister<Rtc, ReadWrite, std::byte> temp;

//            enum class DbgCtrl_t : uint8_t {
//                DBG = RTC_DBGRUN_bm,
//            };
//            ControlRegister<Rtc, DbgCtrl_t> dbgctrl;

//            DataRegister<Rtc, ReadWrite, std::byte> calib;
            
//            enum class ClockSel_t : uint8_t {
//                cs1 = RTC_CLKSEL1_bm,
//                cs0 = RTC_CLKSEL0_bm,
//            };
//            ControlRegister<Rtc, ClockSel_t> clksel;

//            struct {
//                DataRegister<Rtc, ReadWrite, std::byte> cntl;
//                DataRegister<Rtc, ReadWrite, std::byte> cnth;
//            } cnt;
            
//            struct {
//                DataRegister<Rtc, ReadWrite, std::byte> perl;
//                DataRegister<Rtc, ReadWrite, std::byte> perh;
//            } per;
            
//            struct {
//                DataRegister<Rtc, ReadWrite, std::byte> cmpl;
//                DataRegister<Rtc, ReadWrite, std::byte> cmph;
//            } cmp;

//            volatile uint8_t padding[0x0f - 0x0e + 1];
            
//            enum class PitCtrlA_t : uint8_t {
//                enable = RTC_PITEN_bm,
//            };
//            static inline constexpr std::array<pp_t<PitCtrlA_t>, 15> pitPrescalerValues {
//                pp_t<PitCtrlA_t>{PitCtrlA_t((0x00 << 3)), 0},
//                pp_t<PitCtrlA_t>{PitCtrlA_t((0x01 << 3)), 4},
//                pp_t<PitCtrlA_t>{PitCtrlA_t((0x02 << 3)), 8},
//                pp_t<PitCtrlA_t>{PitCtrlA_t((0x03 << 3)), 16},
//                pp_t<PitCtrlA_t>{PitCtrlA_t((0x04 << 3)), 32},
//                pp_t<PitCtrlA_t>{PitCtrlA_t((0x05 << 3)), 64},
//                pp_t<PitCtrlA_t>{PitCtrlA_t((0x06 << 3)), 128},
//                pp_t<PitCtrlA_t>{PitCtrlA_t((0x07 << 3)), 256},
//                pp_t<PitCtrlA_t>{PitCtrlA_t((0x08 << 3)), 512},
//                pp_t<PitCtrlA_t>{PitCtrlA_t((0x09 << 3)), 1024},
//                pp_t<PitCtrlA_t>{PitCtrlA_t((0x0a << 3)), 2048},
//                pp_t<PitCtrlA_t>{PitCtrlA_t((0x0b << 3)), 4096},
//                pp_t<PitCtrlA_t>{PitCtrlA_t((0x0c << 3)), 8192},
//                pp_t<PitCtrlA_t>{PitCtrlA_t((0x0d << 3)), 16384},
//                pp_t<PitCtrlA_t>{PitCtrlA_t((0x0e << 3)), 32768},
//            };
//            ControlRegister<Rtc, PitCtrlA_t> pitctrla;
            
//            enum class PitStatus_t : uint8_t {
//                ctrlbusy = RTC_CTRLBUSY_bm,
//            };
//            FlagRegister<Rtc, PitStatus_t, ReadOnly> pitstatus;

//            enum class PitIntCtrl_t : uint8_t {
//                pi = RTC_PI_bm,
//            };
//            ControlRegister<Rtc, PitIntCtrl_t> pitintctrl;

//            enum class PitIntFlags_t : uint8_t {
//                pi = RTC_PI_bm,
//            };
//            FlagRegister<Rtc, PitIntFlags_t, ReadWrite> pitintflags;

//            volatile uint8_t padding1;

//            enum class PitDbgFlags_t : uint8_t {
//                dbgrun = RTC_DBGRUN_bm,
//            };
//            FlagRegister<Rtc, PitDbgFlags_t, ReadOnly> pitdbgctrl;

//            static inline constexpr uintptr_t address = 0x0140;
//        };
        
//        static_assert(sizeof(Rtc) == 22);
        
//        struct Cpu final {
//            volatile uint8_t padding[4];
//            DataRegister<Cpu, ReadWrite, std::byte> ccp;
            
//            enum class SReg_t: uint8_t {
//                globalIntEnable = (1 << 7), 
//                bitCopy = (1 << 6) 
//            };
//            ControlRegister<Cpu, SReg_t> sreg;
            
//            static inline constexpr uintptr_t address = 0x0030;
//        };
        
//        struct Clock final {
//            enum class MClkCtrlA_t : uint8_t {
//                clkout = CLKCTRL_CLKOUT_bm,
//                clksel1 = CLKCTRL_CLKSEL1_bm,
//                clksel0 = CLKCTRL_CLKSEL0_bm
//            };
//            ControlRegister<Clock, MClkCtrlA_t> mclkctrla;

//            enum class MClkCtrlB_t : uint8_t {
//                pdiv3 = CLKCTRL_PDIV3_bm,
//                pdiv2 = CLKCTRL_PDIV2_bm,
//                pdiv1 = CLKCTRL_PDIV1_bm,
//                pdiv0 = CLKCTRL_PDIV0_bm,
//                pen = CLKCTRL_PEN_bm,
//            };
            
//            template<typename T>
//            using pp_t = AVR::Util::PrescalerPair<T>;
            
//            static inline constexpr std::array<pp_t<MClkCtrlB_t>, 12> prescalerValues {
//                pp_t<MClkCtrlB_t>{MClkCtrlB_t((0x00 << 1) | 0x00), 1},
//                pp_t<MClkCtrlB_t>{MClkCtrlB_t((0x00 << 1) | 0x01), 2},
//                pp_t<MClkCtrlB_t>{MClkCtrlB_t((0x01 << 1) | 0x01), 4},
//                pp_t<MClkCtrlB_t>{MClkCtrlB_t((0x02 << 1) | 0x01), 8},
//                pp_t<MClkCtrlB_t>{MClkCtrlB_t((0x03 << 1) | 0x01), 16},
//                pp_t<MClkCtrlB_t>{MClkCtrlB_t((0x04 << 1) | 0x01), 32},
//                pp_t<MClkCtrlB_t>{MClkCtrlB_t((0x05 << 1) | 0x01), 64},
//                pp_t<MClkCtrlB_t>{MClkCtrlB_t((0x08 << 1) | 0x01), 6},
//                pp_t<MClkCtrlB_t>{MClkCtrlB_t((0x09 << 1) | 0x01), 10},
//                pp_t<MClkCtrlB_t>{MClkCtrlB_t((0x0a << 1) | 0x01), 12},
//                pp_t<MClkCtrlB_t>{MClkCtrlB_t((0x0b << 1) | 0x01), 24},
//                pp_t<MClkCtrlB_t>{MClkCtrlB_t((0x0c << 1) | 0x01), 48},
//            };
//            ControlRegister<Clock, MClkCtrlB_t> mclkctrlb;

//            enum class MClkLock_t : uint8_t {
//                lock = CLKCTRL_LOCKEN_bm,
//            };
//            ControlRegister<Clock, MClkLock_t> mclklock;
            
//            static inline constexpr uintptr_t address = 0x0060;
//        };
        
//        struct PortRegister final {
//            DataRegister<PortRegister, ReadWrite, std::byte> dir;
//            DataRegister<PortRegister, ReadWrite, std::byte> dirset;
//            DataRegister<PortRegister, ReadWrite, std::byte> dirclr;
//            DataRegister<PortRegister, ReadWrite, std::byte> dirtgl;
//            DataRegister<PortRegister, ReadWrite, std::byte> out;
//            DataRegister<PortRegister, ReadWrite, std::byte> outset;
//            DataRegister<PortRegister, ReadWrite, std::byte> outclr;
//            DataRegister<PortRegister, ReadWrite, std::byte> outtgl;
//            DataRegister<PortRegister, ReadWrite, std::byte> in;
//            DataRegister<PortRegister, ReadWrite, std::byte> inflags;
//            DataRegister<PortRegister, ReadWrite, std::byte> portctrl;

//            volatile uint8_t padding[0x0f - 0x0b + 1];
            
//            DataRegister<PortRegister, ReadWrite, std::byte> pin0ctrl;
//            DataRegister<PortRegister, ReadWrite, std::byte> pin1ctrl;
//            DataRegister<PortRegister, ReadWrite, std::byte> pin2ctrl;
//            DataRegister<PortRegister, ReadWrite, std::byte> pin3ctrl;
//            DataRegister<PortRegister, ReadWrite, std::byte> pin4ctrl;
//            DataRegister<PortRegister, ReadWrite, std::byte> pin5ctrl;
//            DataRegister<PortRegister, ReadWrite, std::byte> pin6ctrl;
//            DataRegister<PortRegister, ReadWrite, std::byte> pin7ctrl;

//            template<typename P> struct Address;
//        };
////        std::integral_constant<uint8_t, sizeof(PortRegister)>::_;
//        static_assert(sizeof(PortRegister) == 0x18);
        
//        struct TCA {
//            template<typename T>
//            using pp_t = AVR::Util::PrescalerPair<T>;
            
//            static constexpr const uint8_t count = 1;

//            enum class CtrlA_t : uint8_t {
//                clksel2 = TCA_SINGLE_CLKSEL2_bm,
//                clksel1 = TCA_SINGLE_CLKSEL1_bm,
//                clksel0 = TCA_SINGLE_CLKSEL0_bm,
//                enable = TCA_SINGLE_ENABLE_bm,
//            };
//            static inline constexpr std::array<pp_t<CtrlA_t>, 8> prescalerValues {
//                pp_t<CtrlA_t>{CtrlA_t((0x00 << 1)), 1},
//                pp_t<CtrlA_t>{CtrlA_t((0x01 << 1)), 2},
//                pp_t<CtrlA_t>{CtrlA_t((0x02 << 1)), 4},
//                pp_t<CtrlA_t>{CtrlA_t((0x03 << 1)), 8},
//                pp_t<CtrlA_t>{CtrlA_t((0x04 << 1)), 16},
//                pp_t<CtrlA_t>{CtrlA_t((0x05 << 1)), 64},
//                pp_t<CtrlA_t>{CtrlA_t((0x06 << 1)), 256},
//                pp_t<CtrlA_t>{CtrlA_t((0x07 << 1)), 1024},
//            };
//            ControlRegister<TCA, CtrlA_t> ctrla;

//            enum class Ctrlb_t : uint8_t {
//                cmp2en = TCA_SINGLE_CMP2EN_bm,
//                cmp1en = TCA_SINGLE_CMP1EN_bm,
//                cmp0en = TCA_SINGLE_CMP0EN_bm,
//                alupd  = TCA_SINGLE_ALUPD_bm,
//                wgm2   = TCA_SINGLE_WGMODE2_bm,
//                wgm1   = TCA_SINGLE_WGMODE1_bm,
//                wgm0   = TCA_SINGLE_WGMODE0_bm,
//            };
//            ControlRegister<TCA, Ctrlb_t> ctrlb;
            
//            enum class Ctrlc_t : uint8_t {
//                cov2 = TCA_SINGLE_CMP2OV_bm,
//                cov1 = TCA_SINGLE_CMP1OV_bm,
//                cov0 = TCA_SINGLE_CMP0OV_bm,
//            };
//            ControlRegister<TCA, Ctrlc_t> ctrlc;
            
//            enum class Ctrld_t : uint8_t {
//            };
//            ControlRegister<TCA, Ctrld_t> ctrld;
            
//            enum class Ctrle_t : uint8_t {
//            };
//            FlagRegister<TCA, Ctrle_t, WriteOnly> ctrleclr;
//            FlagRegister<TCA, Ctrle_t, WriteOnly> ctrleset;

//            enum class Ctrlf_t : uint8_t {
//            };
//            FlagRegister<TCA, Ctrlf_t, WriteOnly> ctrlfclr;
//            FlagRegister<TCA, Ctrlf_t, WriteOnly> ctrlfset;
            
//            volatile uint8_t padding1;
            
//            enum class Evctrl_t : uint8_t {
//            };
//            ControlRegister<TCA, Evctrl_t> evctrl;
            
//            enum class Intctrl_t : uint8_t {
//            };
//            ControlRegister<TCA, Intctrl_t> intctrl;
            
//            enum class Intflags_t : uint8_t {
//                ovf = TCA_SINGLE_OVF_bm
//            };
//            FlagRegister<TCA, Intflags_t, ReadWrite> intflags;
            
//            volatile uint8_t padding2[2];
            
//            enum class Dbgctrl_t : uint8_t {
//            };
//            ControlRegister<TCA, Dbgctrl_t> dbgctrl;

//            DataRegister<TCA, ReadWrite, std::byte> temp;

//            volatile uint8_t padding3[0x1f - 0x10 + 1];
            
//            struct {
//                DataRegister<TCA, ReadWrite, std::byte> cntl;
//                DataRegister<TCA, ReadWrite, std::byte> cnth;
//            } cnt;

//            volatile uint8_t padding4[0x25 - 0x22 + 1];
            
//            struct {
//                DataRegister<TCA, ReadWrite, std::byte> perl;
//                DataRegister<TCA, ReadWrite, std::byte> perh;
//            } per;
            
//            struct {
//                DataRegister<TCA, ReadWrite, std::byte> cmp0l;
//                DataRegister<TCA, ReadWrite, std::byte> cmp0h;
//            } cmp0;
//            struct {
//                DataRegister<TCA, ReadWrite, std::byte> cmp1l;
//                DataRegister<TCA, ReadWrite, std::byte> cmp1h;
//            } cmp1;
//            struct {
//                DataRegister<TCA, ReadWrite, std::byte> cmp2l;
//                DataRegister<TCA, ReadWrite, std::byte> cmp2h;
//            } cmp3;

//            volatile uint8_t padding5[0x35 - 0x2e + 1];

//            struct {
//                DataRegister<TCA, ReadWrite, std::byte> perl;
//                DataRegister<TCA, ReadWrite, std::byte> perh;
//            } perbuf;
            
//            struct {
//                DataRegister<TCA, ReadWrite, std::byte> cmp0l;
//                DataRegister<TCA, ReadWrite, std::byte> cmp0h;
//            } cmp0buf;
//            struct {
//                DataRegister<TCA, ReadWrite, std::byte> cmp1l;
//                DataRegister<TCA, ReadWrite, std::byte> cmp1h;
//            } cmp1buf;
//            struct {
//                DataRegister<TCA, ReadWrite, std::byte> cmp2l;
//                DataRegister<TCA, ReadWrite, std::byte> cmp2h;
//            } cmp3buf;
            
            
//            template<int N> struct Address;
//        };

////        std::integral_constant<uint8_t, sizeof(TCA)>::_;
//        static_assert(sizeof(TCA) == 0x3e);
        
//    }
//}
