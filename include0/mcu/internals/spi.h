#pragma once

#include <cstdint>
#include <array>
#include <etl/fifo.h>
#include <etl/types.h>
#include <etl/rational.h>
#include <etl/type_traits.h>

#include "../common/concepts.h"
#include "../common/isr.h"

#include "portmux.h"

namespace AVR {
    
    template<auto N>
    using QueueLength = etl::NamedConstant<N>;
    
    template<AVR::Concepts::ComponentPosition CP,
             etl::Concepts::NamedConstant Size = etl::NamedConstant<16>,
             etl::Concepts::NamedFlag useISR = etl::NamedFlag<false>, 
             typename MCU = DefaultMcuType> struct Spi;
    
    template<AVR::Concepts::ComponentPosition CP, etl::Concepts::NamedConstant Size, 
             etl::Concepts::NamedFlag useISR, AVR::Concepts::At01Series MCU>
    struct Spi<CP, Size, useISR, MCU> final {
        static inline constexpr auto N = CP::component_type::value;

        static constexpr auto mcu_spi = getBaseAddr<typename MCU::Spi, N>;
        static_assert(N < AVR::Component::Count<typename MCU::Spi>::value, "wrong number of spi");

        using mcu_spi_t = typename MCU::Spi;
        using ctrla1_t = typename MCU::Spi::CtrlA1_t;
        using ctrla2_t = typename MCU::Spi::CtrlA2_t;

        using ctrlb1_t = typename MCU::Spi::CtrlB1_t;
        using ctrlb2_t = typename MCU::Spi::CtrlB2_t;
        
        using intflags_t = typename MCU::Spi::IntFlags_t;
        
        inline static constexpr auto size{Size::value};
        
        using fifo_type = std::conditional_t<useISR::value, 
                                             volatile etl::FiFo<std::byte, size>, etl::FiFo<std::byte, size> >; 
        
        using mosipin = AVR::Portmux::Map<CP, MCU>::mosipin;
        using sckpin = AVR::Portmux::Map<CP, MCU>::sckpin;

//        mosipin::_;
//        sckpin::_;

        inline static void init() {
            mosipin::template dir<Output>();
            sckpin::template dir<Output>();
            
            mcu_spi()->ctrlb.template set<ctrlb1_t::ssd | ctrlb1_t::bufen>();
//            mcu_spi()->ctrlb.template add<ctrlb2_t::mode3>();
//            mcu_spi()->ctrla.template set<ctrla2_t::div4>();
            mcu_spi()->ctrla.template add<ctrla1_t::enable | ctrla1_t::master>();
        }
        
        inline static bool put(std::byte b) {
            return mData.push_back(b);
        }
        
        inline static bool empty() {
            return mData.empty();
        }

        inline static void periodic() {
            if (mcu_spi()->intflags.template isSet<intflags_t::dreif>()) {
                if (std::byte b; mData.pop_front(b)) {
//                    mcu_spi()->data;
                    *mcu_spi()->data = b;
//                    ++counter;
                }
            }
        }
//        inline static uint16_t counter{};
    private:
        inline static fifo_type mData;
    };
}

