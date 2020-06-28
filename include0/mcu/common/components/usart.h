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
        struct Usart {
//            static constexpr const uint8_t count = 4;
            
            DataRegister<Usart, ReadOnly, std::byte> rxd;
            
            enum class RxDataH_t : uint8_t {
                rxcif = USART_RXCIF_bm,
                bufovl = USART_BUFOVF_bm,
                ferr = USART_FERR_bm,
                perr = USART_PERR_bm,
                data8 = USART_DATA8_bm,
            };
            ControlRegister<Usart, RxDataH_t> rxdh;
            
            DataRegister<Usart, ReadWrite, std::byte> txd;
            DataRegister<Usart, ReadWrite, std::byte> txdh;

            enum class Status_t : uint8_t {
                rxc = USART_RXCIF_bm,
                txc = USART_TXCIF_bm,
                dre = USART_DREIF_bm,
                rxs  = USART_RXSIF_bm,
                isf = USART_ISFIF_bm,
                bdf = USART_BDF_bm,
                wfb = USART_WFB_bm,
            };
            FlagRegister<Usart, Status_t, ReadWrite> status;

            enum class CtrlA_t : uint8_t {
                rxcie = USART_RXCIE_bm,
                txcie = USART_TXCIE_bm,
                dreie = USART_DREIE_bm,
                rxsie = USART_RXSIE_bm,
                lbme  = USART_LBME_bm,
                abeie  = USART_ABEIE_bm,
                rs485_1 = USART_RS4851_bm,
                rs485_0 = USART_RS4850_bm,
            };
            ControlRegister<Usart, CtrlA_t> ctrla;

            enum class CtrlB_t : uint8_t {
                rxen = USART_RXEN_bm,
                txen = USART_TXEN_bm,
                sfden = USART_SFDEN_bm,
                odme = USART_ODME_bm,
                rxm0= USART_RXMODE0_bm,
                rxm1= USART_RXMODE1_bm,
                mpcm= USART_MPCM_bm,
            };
            ControlRegister<Usart, CtrlB_t> ctrlb;

            enum class CtrlC_t : uint8_t {
                cmode1 = USART_CMODE1_bm,
                cmode0 = USART_CMODE0_bm,
                pmode1 = USART_PMODE1_bm,
                pmode0 = USART_PMODE0_bm,
                sbmode = USART_SBMODE_bm,
                xfer1stopbit = 0x00,
                xfer2stopbit = 0x01 << 3,
                chsize2 = USART_CHSIZE2_bm,
                chsize1 = USART_CHSIZE1_bm,
                chsize0 = USART_CHSIZE0_bm,
                xfer8bit = USART_CHSIZE1_bm | USART_CHSIZE0_bm,
                noparity = 0x00,
                evenparity = (0x02) << 4,
            };
            ControlRegister<Usart, CtrlC_t> ctrlc;

            DataRegister<Usart, ReadWrite, uint16_t> baud;
            
            volatile uint8_t reserved;

            template<int N> struct Address;
        };
    }
}
