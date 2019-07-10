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
    namespace Series1 {
        struct Ccl {
            enum class CtrlA_t : uint8_t {
                standby = CCL_RUNSTDBY_bm,
                enable  = CCL_ENABLE_bm,
            };  
            ControlRegister<Ccl, CtrlA_t> ctrla;
            
            enum class SeqCtrl0_t : uint8_t {
                disable = CCL_SEQSEL_DISABLE_gc,
                dff     = CCL_SEQSEL_DFF_gc,
                jk      = CCL_SEQSEL_JK_gc,
                latch   = CCL_SEQSEL_LATCH_gc,
                rs      = CCL_SEQSEL_RS_gc,
            };  
            ControlRegister<Ccl, SeqCtrl0_t> seqctrl0;
            
            volatile uint8_t reserved0;
            volatile uint8_t reserved1;
            volatile uint8_t reserved2;
            
            enum class Lut0CtrlA_t : uint8_t {
                edge = CCL_EDGEDET_bm,
                outenable = CCL_OUTEN_bm,
                enable = CCL_ENABLE_bm,
                filter_disable = CCL_FILTSEL_DISABLE_gc,
                filter_synch = CCL_FILTSEL_SYNCH_gc,
                filter_filter = CCL_FILTSEL_FILTER_gc,
                clk_per = (0 << 6),
                clk_in2 = (1 << 6),
                
            };  
            
            union Luts {
                struct Raw {
                    ControlRegister<Ccl, Lut0CtrlA_t> ctrla;
                    volatile std::byte ctrlb;
                    volatile std::byte ctrlc;
                    volatile std::byte truth;
                };
                std::array<Raw, 2> raw;
                
                struct Typed {
                    ControlRegister<Ccl, Lut0CtrlA_t> lut0ctrla;
                    
                    enum class Lut0CtrlB_t : uint8_t {
                        in1_mask = CCL_INSEL1_MASK_gc,
                        in1_feedback = CCL_INSEL1_FEEDBACK_gc,
                        in1_link = CCL_INSEL1_LINK_gc,
                        in1_event0 = CCL_INSEL1_EVENT0_gc,
                        in1_event1  = CCL_INSEL1_EVENT1_gc,
                        in1_io = CCL_INSEL1_IO_gc,
                        in1_ac0 = CCL_INSEL1_AC0_gc,
                        in1_usart0 = CCL_INSEL1_USART0_gc,
                        in1_spi0 = CCL_INSEL1_SPI0_gc,
                        in1_tca0 = CCL_INSEL1_TCA0_gc,
                        in1_tcb0 = CCL_INSEL1_TCB0_gc,
                        in1_tcd0 = CCL_INSEL1_TCD0_gc,
                        
                        in0_mask = CCL_INSEL0_MASK_gc,
                        in0_feedback = CCL_INSEL0_FEEDBACK_gc,
                        in0_link = CCL_INSEL0_LINK_gc,
                        in0_event0 = CCL_INSEL0_EVENT0_gc,
                        in0_event1  = CCL_INSEL0_EVENT1_gc,
                        in0_io = CCL_INSEL0_IO_gc,
                        in0_ac0 = CCL_INSEL0_AC0_gc,
                        in0_usart0 = CCL_INSEL0_USART0_gc,
                        in0_spi0 = CCL_INSEL0_SPI0_gc,
                        in0_tca0 = CCL_INSEL0_TCA0_gc,
                        in0_tcb0 = CCL_INSEL0_TCB0_gc,
                        in0_tcd0 = CCL_INSEL0_TCD0_gc,
                    };  
                    ControlRegister<Ccl, Lut0CtrlB_t> lut0ctrlb;
                    
                    enum class Lut0CtrlC_t : uint8_t {
                        in2_mask = CCL_INSEL2_MASK_gc,
                        in2_feedback = CCL_INSEL2_FEEDBACK_gc,
                        in2_link = CCL_INSEL2_LINK_gc,
                        in2_event0 = CCL_INSEL2_EVENT0_gc,
                        in2_event1  = CCL_INSEL2_EVENT1_gc,
                        in2_io = CCL_INSEL2_IO_gc,
                        in2_ac0 = CCL_INSEL2_AC0_gc,
                        in2_spi0 = CCL_INSEL2_SPI0_gc,
                        in2_tca0 = CCL_INSEL2_TCA0_gc,
                        in2_tcb0 = CCL_INSEL2_TCB0_gc,
                        in2_tcd0 = CCL_INSEL2_TCD0_gc,
                    };  
                    ControlRegister<Ccl, Lut0CtrlC_t> lut0ctrlc;
                    
                    DataRegister<Ccl, ReadWrite, std::byte> truth0;
                    
                    enum class Lut1CtrlA_t : uint8_t {
                        edge = CCL_EDGEDET_bm,
                        outenable = CCL_OUTEN_bm,
                        filter_disable = CCL_FILTSEL_DISABLE_gc,
                        filter_synch = CCL_FILTSEL_SYNCH_gc,
                        filter_filter = CCL_FILTSEL_FILTER_gc,
                        clk_per = (0 << 6),
                        clk_in2 = (1 << 6),
                    };  
                    ControlRegister<Ccl, Lut1CtrlA_t> lut1ctrla;
                    
                    enum class Lut1CtrlB_t : uint8_t {
                        in1_mask = CCL_INSEL1_MASK_gc,
                        in1_feedback = CCL_INSEL1_FEEDBACK_gc,
                        in1_link = CCL_INSEL1_LINK_gc,
                        in1_event0 = CCL_INSEL1_EVENT0_gc,
                        in1_event1  = CCL_INSEL1_EVENT1_gc,
                        in1_io = CCL_INSEL1_IO_gc,
                        in1_ac0 = CCL_INSEL1_AC0_gc,
                        in1_usart0 = CCL_INSEL1_USART0_gc,
                        in1_spi0 = CCL_INSEL1_SPI0_gc,
                        in1_tca0 = CCL_INSEL1_TCA0_gc,
                        in1_tcb0 = CCL_INSEL1_TCB0_gc,
                        in1_tcd0 = CCL_INSEL1_TCD0_gc,
                        
                        in0_mask = CCL_INSEL0_MASK_gc,
                        in0_feedback = CCL_INSEL0_FEEDBACK_gc,
                        in0_link = CCL_INSEL0_LINK_gc,
                        in0_event0 = CCL_INSEL0_EVENT0_gc,
                        in0_event1  = CCL_INSEL0_EVENT1_gc,
                        in0_io = CCL_INSEL0_IO_gc,
                        in0_ac0 = CCL_INSEL0_AC0_gc,
                        in0_usart0 = CCL_INSEL0_USART0_gc,
                        in0_spi0 = CCL_INSEL0_SPI0_gc,
                        in0_tca0 = CCL_INSEL0_TCA0_gc,
                        in0_tcb0 = CCL_INSEL0_TCB0_gc,
                        in0_tcd0 = CCL_INSEL0_TCD0_gc,
                    };  
                    ControlRegister<Ccl, Lut1CtrlB_t> lut1ctrlb;
                    
                    enum class Lut1CtrlC_t : uint8_t {
                        in2_mask = CCL_INSEL2_MASK_gc,
                        in2_feedback = CCL_INSEL2_FEEDBACK_gc,
                        in2_link = CCL_INSEL2_LINK_gc,
                        in2_event0 = CCL_INSEL2_EVENT0_gc,
                        in2_event1  = CCL_INSEL2_EVENT1_gc,
                        in2_io = CCL_INSEL2_IO_gc,
                        in2_ac0 = CCL_INSEL2_AC0_gc,
                        in2_spi0 = CCL_INSEL2_SPI0_gc,
                        in2_tca0 = CCL_INSEL2_TCA0_gc,
                        in2_tcb0 = CCL_INSEL2_TCB0_gc,
                        in2_tcd0 = CCL_INSEL2_TCD0_gc,
                    };  
                    ControlRegister<Ccl, Lut1CtrlC_t> lut1ctrlc;
                    
                    DataRegister<Ccl, ReadWrite, std::byte> truth1;
                    
                };
                Typed typed;
            };
            
            Luts luts;
            
            static inline constexpr uintptr_t address = 0x01c0;
        };

//        std::integral_constant<uint16_t, sizeof(Ccl)>::_;
        
        static_assert(sizeof(Ccl) == 0x0d);
        
    }
}
