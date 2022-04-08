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
        struct Ccl {
            enum class CtrlA_t : uint8_t {
                standby = CCL_RUNSTDBY_bm,
                enable  = CCL_ENABLE_bm,
            };  
            ControlRegister<Ccl, CtrlA_t> ctrla;
            
            enum class SeqCtrl0_t : uint8_t {
                disable = CCL_SEQSEL0_DISABLE_gc,
                dff     = CCL_SEQSEL0_DFF_gc,
                jk      = CCL_SEQSEL0_JK_gc,
                latch   = CCL_SEQSEL0_LATCH_gc,
                rs      = CCL_SEQSEL0_RS_gc,
            };  
            ControlRegister<Ccl, SeqCtrl0_t> seqctrl0;
            
            enum class SeqCtrl1_t : uint8_t {
                disable = CCL_SEQSEL1_DISABLE_gc,
                dff     = CCL_SEQSEL1_DFF_gc,
                jk      = CCL_SEQSEL1_JK_gc,
                latch   = CCL_SEQSEL1_LATCH_gc,
                rs      = CCL_SEQSEL1_RS_gc,
            };  
            ControlRegister<Ccl, SeqCtrl1_t> seqctrl1;
            
            volatile uint8_t reserved0;
            volatile uint8_t reserved1;
            
            enum class IntCtrl0_t : uint8_t {
                disable0 = CCL_INTMODE0_INTDISABLE_gc,
                rising0 = CCL_INTMODE0_RISING_gc,
                falling0 = CCL_INTMODE0_FALLING_gc,
                both0 = CCL_INTMODE0_BOTH_gc,
                
                disable1 = CCL_INTMODE1_INTDISABLE_gc,
                rising1  = CCL_INTMODE1_RISING_gc,
                falling1 = CCL_INTMODE1_FALLING_gc,
                both1    = CCL_INTMODE1_BOTH_gc,
                
                disable2 = CCL_INTMODE2_INTDISABLE_gc,
                rising2 = CCL_INTMODE2_RISING_gc,
                falling2 = CCL_INTMODE2_FALLING_gc,
                both2 = CCL_INTMODE2_BOTH_gc,
                
                disable3 = CCL_INTMODE3_INTDISABLE_gc,
                rising3 = CCL_INTMODE3_RISING_gc,
                falling3 = CCL_INTMODE3_FALLING_gc,
                both3 = CCL_INTMODE3_BOTH_gc,
            };  
            ControlRegister<Ccl, IntCtrl0_t> intctrl0;
            
            volatile uint8_t reserved2;
            
            enum class IntFlags_t : uint8_t {
                int0 = CCL_INT0_bm,
                int1 = CCL_INT1_bm,
                int2 = CCL_INT2_bm,
                int3 = CCL_INT3_bm,
            };
            FlagRegister<Ccl, IntFlags_t, ReadWrite> intflags;
            
            enum class Lut0CtrlA_t : uint8_t {
                edge = CCL_EDGEDET_bm,
                outenable = CCL_OUTEN_bm,
                enable = CCL_ENABLE_bm,
                filter_disable = CCL_FILTSEL_DISABLE_gc,
                filter_synch = CCL_FILTSEL_SYNCH_gc,
                filter_filter = CCL_FILTSEL_FILTER_gc,
                clk_per = CCL_CLKSRC_CLKPER_gc,
                clk_in2 = CCL_CLKSRC_IN2_gc,
                clk_20M = CCL_CLKSRC_OSC20M_gc,
                clk_32K = CCL_CLKSRC_OSCULP32K_gc,
                clk_1K = CCL_CLKSRC_OSCULP1K_gc,
                
            };  
            
            union Luts {
                struct Raw {
                    ControlRegister<Ccl, Lut0CtrlA_t> ctrla;
                    volatile std::byte ctrlb;
                    volatile std::byte ctrlc;
                    volatile std::byte truth;
                };
                std::array<Raw, 4> raw;
                
                struct Typed {
                    ControlRegister<Ccl, Lut0CtrlA_t> lut0ctrla;
                    
                    enum class Lut0CtrlB_t : uint8_t {
                        in1_mask = CCL_INSEL1_MASK_gc,
                        in1_feedback = CCL_INSEL1_FEEDBACK_gc,
                        in1_link = CCL_INSEL1_LINK_gc,
                        in1_eventa = CCL_INSEL1_EVENTA_gc,
                        in1_eventb  = CCL_INSEL1_EVENTB_gc,
                        in1_io = CCL_INSEL1_IO_gc,
                        in1_ac0 = CCL_INSEL1_AC0_gc,
                        in1_usart1 = CCL_INSEL1_USART1_gc,
                        in1_spi0 = CCL_INSEL1_SPI0_gc,
                        in1_tca0 = CCL_INSEL1_TCA0_gc,
                        in1_tcb1 = CCL_INSEL1_TCB1_gc,
                        
                        in0_mask = CCL_INSEL0_MASK_gc,
                        in0_feedback = CCL_INSEL0_FEEDBACK_gc,
                        in0_link = CCL_INSEL0_LINK_gc,
                        in0_eventa = CCL_INSEL0_EVENTA_gc,
                        in0_eventb  = CCL_INSEL0_EVENTB_gc,
                        in0_io = CCL_INSEL0_IO_gc,
                        in0_ac0 = CCL_INSEL0_AC0_gc,
                        in0_usart0 = CCL_INSEL0_USART0_gc,
                        in0_spi0 = CCL_INSEL0_SPI0_gc,
                        in0_tca0 = CCL_INSEL0_TCA0_gc,
                        in0_tcb0 = CCL_INSEL0_TCB0_gc,
                    };  
                    ControlRegister<Ccl, Lut0CtrlB_t> lut0ctrlb;
                    
                    enum class Lut0CtrlC_t : uint8_t {
                        in2_mask = CCL_INSEL2_MASK_gc,
                        in2_feedback = CCL_INSEL2_FEEDBACK_gc,
                        in2_link = CCL_INSEL2_LINK_gc,
                        in2_eventa = CCL_INSEL2_EVENTA_gc,
                        in2_eventb  = CCL_INSEL2_EVENTB_gc,
                        in2_io = CCL_INSEL2_IO_gc,
                        in2_ac0 = CCL_INSEL2_AC0_gc,
                        in2_usart2 = CCL_INSEL2_USART2_gc,
                        in2_spi0 = CCL_INSEL2_SPI0_gc,
                        in2_tca0 = CCL_INSEL2_TCA0_gc,
                        in2_tcb2 = CCL_INSEL2_TCB2_gc,
                    };  
                    ControlRegister<Ccl, Lut0CtrlC_t> lut0ctrlc;
                    
                    DataRegister<Ccl, ReadWrite, std::byte> truth0;
                    
                    enum class Lut1CtrlA_t : uint8_t {
                        edge = CCL_EDGEDET_bm,
                        outenable = CCL_OUTEN_bm,
                        filter_disable = CCL_FILTSEL_DISABLE_gc,
                        filter_synch = CCL_FILTSEL_SYNCH_gc,
                        filter_filter = CCL_FILTSEL_FILTER_gc,
                        clk_per = CCL_CLKSRC_CLKPER_gc,
                        clk_in2 = CCL_CLKSRC_IN2_gc,
                        clk_20M = CCL_CLKSRC_OSC20M_gc,
                        clk_32K = CCL_CLKSRC_OSCULP32K_gc,
                        clk_1K = CCL_CLKSRC_OSCULP1K_gc,
                    };  
                    ControlRegister<Ccl, Lut1CtrlA_t> lut1ctrla;
                    
                    enum class Lut1CtrlB_t : uint8_t {
                        in1_mask = CCL_INSEL1_MASK_gc,
                        in1_feedback = CCL_INSEL1_FEEDBACK_gc,
                        in1_link = CCL_INSEL1_LINK_gc,
                        in1_eventa = CCL_INSEL1_EVENTA_gc,
                        in1_eventb  = CCL_INSEL1_EVENTB_gc,
                        in1_io = CCL_INSEL1_IO_gc,
                        in1_ac0 = CCL_INSEL1_AC0_gc,
                        in1_usart1 = CCL_INSEL1_USART1_gc,
                        in1_spi0 = CCL_INSEL1_SPI0_gc,
                        in1_tca0 = CCL_INSEL1_TCA0_gc,
                        in1_tcb1 = CCL_INSEL1_TCB1_gc,
                        
                        in0_mask = CCL_INSEL0_MASK_gc,
                        in0_feedback = CCL_INSEL0_FEEDBACK_gc,
                        in0_link = CCL_INSEL0_LINK_gc,
                        in0_eventa = CCL_INSEL0_EVENTA_gc,
                        in0_eventb  = CCL_INSEL0_EVENTB_gc,
                        in0_io = CCL_INSEL0_IO_gc,
                        in0_ac0 = CCL_INSEL0_AC0_gc,
                        in0_usart0 = CCL_INSEL0_USART0_gc,
                        in0_spi0 = CCL_INSEL0_SPI0_gc,
                        in0_tca0 = CCL_INSEL0_TCA0_gc,
                        in0_tcb0 = CCL_INSEL0_TCB0_gc,
                    };  
                    ControlRegister<Ccl, Lut1CtrlB_t> lut1ctrlb;
                    
                    enum class Lut1CtrlC_t : uint8_t {
                        in2_mask = CCL_INSEL2_MASK_gc,
                        in2_feedback = CCL_INSEL2_FEEDBACK_gc,
                        in2_link = CCL_INSEL2_LINK_gc,
                        in2_eventa = CCL_INSEL2_EVENTA_gc,
                        in2_eventb  = CCL_INSEL2_EVENTB_gc,
                        in2_io = CCL_INSEL2_IO_gc,
                        in2_ac0 = CCL_INSEL2_AC0_gc,
                        in2_usart2 = CCL_INSEL2_USART2_gc,
                        in2_spi0 = CCL_INSEL2_SPI0_gc,
                        in2_tca0 = CCL_INSEL2_TCA0_gc,
                        in2_tcb2 = CCL_INSEL2_TCB2_gc,
                    };  
                    ControlRegister<Ccl, Lut1CtrlC_t> lut1ctrlc;
                    
                    DataRegister<Ccl, ReadWrite, std::byte> truth1;
                    
                    enum class Lut2CtrlA_t : uint8_t {
                        edge = CCL_EDGEDET_bm,
                        outenable = CCL_OUTEN_bm,
                        filter_disable = CCL_FILTSEL_DISABLE_gc,
                        filter_synch = CCL_FILTSEL_SYNCH_gc,
                        filter_filter = CCL_FILTSEL_FILTER_gc,
                        clk_per = CCL_CLKSRC_CLKPER_gc,
                        clk_in2 = CCL_CLKSRC_IN2_gc,
                        clk_20M = CCL_CLKSRC_OSC20M_gc,
                        clk_32K = CCL_CLKSRC_OSCULP32K_gc,
                        clk_1K = CCL_CLKSRC_OSCULP1K_gc,
                    };  
                    ControlRegister<Ccl, Lut2CtrlA_t> lut2ctrla;
                    
                    enum class Lut2CtrlB_t : uint8_t {
                        in1_mask = CCL_INSEL1_MASK_gc,
                        in1_feedback = CCL_INSEL1_FEEDBACK_gc,
                        in1_link = CCL_INSEL1_LINK_gc,
                        in1_eventa = CCL_INSEL1_EVENTA_gc,
                        in1_eventb  = CCL_INSEL1_EVENTB_gc,
                        in1_io = CCL_INSEL1_IO_gc,
                        in1_ac0 = CCL_INSEL1_AC0_gc,
                        in1_usart1 = CCL_INSEL1_USART1_gc,
                        in1_spi0 = CCL_INSEL1_SPI0_gc,
                        in1_tca0 = CCL_INSEL1_TCA0_gc,
                        in1_tcb1 = CCL_INSEL1_TCB1_gc,
                        
                        in0_mask = CCL_INSEL0_MASK_gc,
                        in0_feedback = CCL_INSEL0_FEEDBACK_gc,
                        in0_link = CCL_INSEL0_LINK_gc,
                        in0_eventa = CCL_INSEL0_EVENTA_gc,
                        in0_eventb  = CCL_INSEL0_EVENTB_gc,
                        in0_io = CCL_INSEL0_IO_gc,
                        in0_ac0 = CCL_INSEL0_AC0_gc,
                        in0_usart0 = CCL_INSEL0_USART0_gc,
                        in0_spi0 = CCL_INSEL0_SPI0_gc,
                        in0_tca0 = CCL_INSEL0_TCA0_gc,
                        in0_tcb0 = CCL_INSEL0_TCB0_gc,
                    };  
                    ControlRegister<Ccl, Lut2CtrlB_t> lut2ctrlb;
                    
                    enum class Lut2CtrlC_t : uint8_t {
                        in2_mask = CCL_INSEL2_MASK_gc,
                        in2_feedback = CCL_INSEL2_FEEDBACK_gc,
                        in2_link = CCL_INSEL2_LINK_gc,
                        in2_eventa = CCL_INSEL2_EVENTA_gc,
                        in2_eventb  = CCL_INSEL2_EVENTB_gc,
                        in2_io = CCL_INSEL2_IO_gc,
                        in2_ac0 = CCL_INSEL2_AC0_gc,
                        in2_usart2 = CCL_INSEL2_USART2_gc,
                        in2_spi0 = CCL_INSEL2_SPI0_gc,
                        in2_tca0 = CCL_INSEL2_TCA0_gc,
                        in2_tcb2 = CCL_INSEL2_TCB2_gc,
                    };  
                    ControlRegister<Ccl, Lut2CtrlC_t> lut2ctrlc;
                    
                    DataRegister<Ccl, ReadWrite, std::byte> truth2;
                    
                    enum class Lut3CtrlA_t : uint8_t {
                        edge = CCL_EDGEDET_bm,
                        outenable = CCL_OUTEN_bm,
                        filter_disable = CCL_FILTSEL_DISABLE_gc,
                        filter_synch = CCL_FILTSEL_SYNCH_gc,
                        filter_filter = CCL_FILTSEL_FILTER_gc,
                        clk_per = CCL_CLKSRC_CLKPER_gc,
                        clk_in2 = CCL_CLKSRC_IN2_gc,
                        clk_20M = CCL_CLKSRC_OSC20M_gc,
                        clk_32K = CCL_CLKSRC_OSCULP32K_gc,
                        clk_1K = CCL_CLKSRC_OSCULP1K_gc,
                    };  
                    ControlRegister<Ccl, Lut3CtrlA_t> lut3ctrla;
                    
                    enum class Lut3CtrlB_t : uint8_t {
                        in1_mask = CCL_INSEL1_MASK_gc,
                        in1_feedback = CCL_INSEL1_FEEDBACK_gc,
                        in1_link = CCL_INSEL1_LINK_gc,
                        in1_eventa = CCL_INSEL1_EVENTA_gc,
                        in1_eventb  = CCL_INSEL1_EVENTB_gc,
                        in1_io = CCL_INSEL1_IO_gc,
                        in1_ac0 = CCL_INSEL1_AC0_gc,
                        in1_usart1 = CCL_INSEL1_USART1_gc,
                        in1_spi0 = CCL_INSEL1_SPI0_gc,
                        in1_tca0 = CCL_INSEL1_TCA0_gc,
                        in1_tcb1 = CCL_INSEL1_TCB1_gc,
                        
                        in0_mask = CCL_INSEL0_MASK_gc,
                        in0_feedback = CCL_INSEL0_FEEDBACK_gc,
                        in0_link = CCL_INSEL0_LINK_gc,
                        in0_eventa = CCL_INSEL0_EVENTA_gc,
                        in0_eventb  = CCL_INSEL0_EVENTB_gc,
                        in0_io = CCL_INSEL0_IO_gc,
                        in0_ac0 = CCL_INSEL0_AC0_gc,
                        in0_usart0 = CCL_INSEL0_USART0_gc,
                        in0_spi0 = CCL_INSEL0_SPI0_gc,
                        in0_tca0 = CCL_INSEL0_TCA0_gc,
                        in0_tcb0 = CCL_INSEL0_TCB0_gc,
                    };  
                    ControlRegister<Ccl, Lut3CtrlB_t> lut3ctrlb;
                    
                    enum class Lut3CtrlC_t : uint8_t {
                        in2_mask = CCL_INSEL2_MASK_gc,
                        in2_feedback = CCL_INSEL2_FEEDBACK_gc,
                        in2_link = CCL_INSEL2_LINK_gc,
                        in2_eventa = CCL_INSEL2_EVENTA_gc,
                        in2_eventb  = CCL_INSEL2_EVENTB_gc,
                        in2_io = CCL_INSEL2_IO_gc,
                        in2_ac0 = CCL_INSEL2_AC0_gc,
                        in2_usart2 = CCL_INSEL2_USART2_gc,
                        in2_spi0 = CCL_INSEL2_SPI0_gc,
                        in2_tca0 = CCL_INSEL2_TCA0_gc,
                        in2_tcb2 = CCL_INSEL2_TCB2_gc,
                    };  
                    ControlRegister<Ccl, Lut3CtrlC_t> lut3ctrlc;
                    
                    DataRegister<Ccl, ReadWrite, std::byte> truth3;
                };
                Typed typed;
            };
            
            Luts luts;
            
            static inline constexpr uintptr_t address = 0x01c0;
        };

//        std::integral_constant<uint16_t, sizeof(Ccl)>::_;
        
        static_assert(sizeof(Ccl) == 0x18);
        
    }
}
