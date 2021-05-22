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
    
    // ohne SS Pin
    template<AVR::Concepts::ComponentPosition CP,
             etl::Concepts::NamedConstant Size = etl::NamedConstant<16>,
             etl::Concepts::NamedFlag useISR = etl::NamedFlag<false>, 
             typename MCU = DefaultMcuType> struct Spi;

    // mit SS Pin (jeweils ein Tuple wird byteweise ausgebenen, danach wird SS wieder (kurz) inaktiviert
    template<AVR::Concepts::ComponentPosition CP,
             etl::Concepts::Container Tuple, 
             etl::Concepts::NamedConstant Size = etl::NamedConstant<16>,
             typename SSPin = void,
             typename MCU = DefaultMcuType> struct SpiSS;
    
    template<AVR::Concepts::ComponentPosition CP, etl::Concepts::NamedConstant Size, 
             etl::Concepts::NamedFlag useISR, AVR::Concepts::AtDxSeries MCU>
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
            mcu_spi()->ctrla.template add<ctrla1_t::enable | ctrla1_t::master>();
        }
        
        inline static bool put(std::byte b) {
            return mData.push_back(b);
        }
        
        inline static bool empty() {
            return mData.empty();
        }

        inline static void flush() {
            while(!empty()) {
                periodic();
            }
        }
        
        inline static void periodic() requires(!useISR::value){
            if (mcu_spi()->intflags.template isSet<intflags_t::dreif>()) {
                if (std::byte b; mData.pop_front(b)) {
                    *mcu_spi()->data = b;
#ifndef NDEBUG
                    ++counter;
#endif
                }
            }
        }
#ifndef NDEBUG
        inline static auto count() {
            return counter;
        }
#endif
    private:
#ifndef NDEBUG
        inline static uint16_t counter{};
#endif
        inline static fifo_type mData;
    };

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
        using sspin = AVR::Portmux::Map<CP, MCU>::sspin;

//        mosipin::_;
//        sckpin::_;

        inline static void init() {
            mosipin::template dir<Output>();
            sckpin::template dir<Output>();
            
            mcu_spi()->ctrlb.template set<ctrlb1_t::ssd | ctrlb1_t::bufen>();
//            mcu_spi()->ctrlb.template add<ctrlb2_t::mode3>();
//            mcu_spi()->ctrla.template set<ctrla2_t::div128>();
            mcu_spi()->ctrla.template add<ctrla1_t::enable | ctrla1_t::master>();
        }
        
        inline static bool put(std::byte b) {
            return mData.push_back(b);
        }
        
        inline static bool empty() {
            return mData.empty();
        }
        inline static bool idle() {
            return mcu_spi()->intflags.template isSet<intflags_t::txcif>();
        }

        inline static void flush() {
            while(!empty()) {
                periodic();
            }
        }

        inline static void periodic() requires(!useISR::value){
            if (mcu_spi()->intflags.template isSet<intflags_t::dreif>()) {
                if (std::byte b; mData.pop_front(b)) {
                    *mcu_spi()->data = b;
                    mcu_spi()->intflags.template reset<intflags_t::txcif>();
                }
                ++counter;
            }
        }
        inline static auto count() {
            return counter;
        }
    private:
        inline static uint16_t counter{};
        inline static fifo_type mData;
    };

    template<AVR::Concepts::ComponentPosition CP, etl::Concepts::Container Tuple, 
             etl::Concepts::NamedConstant Size, typename SSPin, AVR::Concepts::At01Series MCU>
    struct SpiSS<CP, Tuple, Size, SSPin, MCU> final {
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
        
        using fifo_type = etl::FiFo<Tuple, size>; 
        
        using mosipin = AVR::Portmux::Map<CP, MCU>::mosipin;
        using sckpin = AVR::Portmux::Map<CP, MCU>::sckpin;

        using sspin = std::conditional_t<std::is_same_v<SSPin, void>, typename AVR::Portmux::Map<CP, MCU>::sspin, SSPin>;
        using ss = AVR::ActiveLow<sspin, AVR::Output>;
        
//        mosipin::_;
//        sckpin::_;
//        sspin::_;

        enum class State : uint8_t {Undefined, Idle, WaitIdle, Transfer};
        
        inline static void init() {
            mosipin::template dir<Output>();
            sckpin::template dir<Output>();
            ss::init();
            mcu_spi()->ctrlb.template set<ctrlb1_t::ssd | ctrlb1_t::bufen>();
//            mcu_spi()->ctrlb.template add<ctrlb2_t::mode3>();
            mcu_spi()->ctrla.template set<ctrla2_t::div128>();
            mcu_spi()->ctrla.template add<ctrla1_t::enable | ctrla1_t::master>();
            mState = State::Undefined;
        }
        
        inline static bool put(const Tuple& b) {
            return mData.push_back(b);
        }
        
        inline static bool empty() {
            return mData.empty();
        }

        inline static void flush() {
            while(!empty()) {
                periodic();
            }
        }
        
        inline static void periodic() {
            const auto oldState = mState;
            switch(mState) {
            case State::Undefined:
                mState = State::Idle;
                break;
            case State::Idle:
                if (!mData.empty()) {
                    mState = State::Transfer;
                }
                break;
            case State::WaitIdle:
                mcu_spi()->intflags.template testAndReset<intflags_t::txcif>([]{
                    mState = State::Idle;
                });
                break;
            case State::Transfer:
                transfer();
                break;
            }
            if (oldState != mState) {
                switch(mState) {
                case State::Undefined:
                    break;
                case State::WaitIdle:
                    break;
                case State::Idle:
                    ss::inactivate();
                    break;
                case State::Transfer:
                    ss::activate();
                    mByteCounter.setToBottom();
                }                
            }    
        }
        inline static auto count() {
            return counter;
        }
    private:
        static inline void transfer() {
            const std::byte& b = mData.front()[mByteCounter];
            if (mcu_spi()->intflags.template isSet<intflags_t::dreif>()) {
                *mcu_spi()->data = b;
                ++counter;
                if (mByteCounter.isTop()) {
                    mData.pop_front();
                    mState = State::WaitIdle;                                        
                }
                ++mByteCounter;
            }
        }
        static inline etl::uint_ranged<uint8_t, 0, Tuple::size() - 1> mByteCounter;
        static inline State mState{State::Undefined};
        inline static uint16_t counter{};
        inline static fifo_type mData;
    };
}

