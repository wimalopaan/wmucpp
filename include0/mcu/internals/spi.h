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

    template<AVR::Concepts::ComponentPosition CP,
             etl::Concepts::Container Command, 
             etl::Concepts::Container Result, 
             etl::Concepts::NamedConstant Size = etl::NamedConstant<16>,
             typename SSPin = void,
             typename MCU = DefaultMcuType> struct SpiBiSS;

    template<AVR::Concepts::ComponentPosition CP,
             etl::Concepts::Container Command, 
             etl::Concepts::Container Result, 
             etl::Concepts::NamedConstant Size = etl::NamedConstant<16>,
             typename SSPin = void,
             typename MCU = DefaultMcuType> struct SpiBiSS2;

    template<AVR::Concepts::ComponentPosition CP,
             etl::Concepts::Container Command, 
             etl::Concepts::Container Result, 
             etl::Concepts::NamedConstant Size = etl::NamedConstant<2>,
             typename SSPin = void,
             typename MCU = DefaultMcuType> struct SpiSync;
    
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
                ++mTxByteCounter;
            }
        }
        inline static auto count() {
            return mTxByteCounter;
        }
    private:
        inline static uint16_t mTxByteCounter{};
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
                    mTxCounter.setToBottom();
                }                
            }    
        }
        inline static auto count() {
            return mTxByteCounter;
        }
    private:
        static inline void transfer() {
            if (mcu_spi()->intflags.template isSet<intflags_t::dreif>()) {
                const std::byte& b = mData.front()[mTxCounter];
                *mcu_spi()->data = b;
                ++mTxByteCounter;
                if (mTxCounter.isTop()) {
                    mData.pop_front();
                    mState = State::WaitIdle;                                        
                }
                ++mTxCounter;
            }
        }
        static inline etl::uint_ranged<uint8_t, 0, Tuple::size() - 1> mTxCounter;
        static inline State mState{State::Undefined};
        inline static uint16_t mTxByteCounter{};
        inline static fifo_type mData;
    };

    template<AVR::Concepts::ComponentPosition CP, etl::Concepts::Container Command, etl::Concepts::Container Result, 
             etl::Concepts::NamedConstant Size, typename SSPin, AVR::Concepts::At01DxSeries MCU>
    struct SpiBiSS<CP, Command, Result, Size, SSPin, MCU> final {
        static inline constexpr auto N = CP::component_type::value;
        static constexpr auto mcu_spi = getBaseAddr<typename MCU::Spi, N>;
        static_assert(N < AVR::Component::Count<typename MCU::Spi>::value, "wrong number of spi");

        static_assert(Command::size() == Result::size());
        
        using mcu_spi_t = typename MCU::Spi;
        using ctrla1_t = typename MCU::Spi::CtrlA1_t;
        using ctrla2_t = typename MCU::Spi::CtrlA2_t;

        using ctrlb1_t = typename MCU::Spi::CtrlB1_t;
        using ctrlb2_t = typename MCU::Spi::CtrlB2_t;
        
        using intflags_t = typename MCU::Spi::IntFlags_t;
        
        inline static constexpr auto size{Size::value};
        
        using txfifo_type = etl::FiFo<Command, size>; 
        using rxfifo_type = etl::FiFo<Result, size>; 
        
        using mosipin = AVR::Portmux::Map<CP, MCU>::mosipin;
        using sckpin = AVR::Portmux::Map<CP, MCU>::sckpin;

        using sspin = std::conditional_t<std::is_same_v<SSPin, void>, typename AVR::Portmux::Map<CP, MCU>::sspin, SSPin>;
        using ss = AVR::ActiveLow<sspin, AVR::Output>;
        
//        mosipin::_;
//        sckpin::_;
//        sspin::_;

        enum class State : uint8_t {Undefined, Idle, WaitIdle, WaitIdle2, Transfer};
        
        inline static void init() {
            mosipin::template dir<Output>();
            sckpin::template dir<Output>();
            ss::init();
            mcu_spi()->ctrlb.template set<ctrlb1_t::ssd | ctrlb1_t::bufen>();
//            mcu_spi()->ctrlb.template add<ctrlb2_t::mode3>();
            mcu_spi()->ctrla.template set<ctrla2_t::div64>();
            mcu_spi()->ctrla.template add<ctrla1_t::enable | ctrla1_t::master>();
            mState = State::Undefined;
        }
        
        inline static void mode1() {
            mcu_spi()->ctrlb.template add<ctrlb2_t::mode1>();
        }
        
        inline static bool put(const Command& b) {
            return mTxData.push_back(b);
        }
        
        inline static bool empty() {
            return mTxData.empty();
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
                if (!mTxData.empty()) {
                    mState = State::Transfer;
                }
                break;
            case State::WaitIdle:
                readBack();
                mcu_spi()->intflags.template testAndReset<intflags_t::txcif>([]{
                    mState = State::WaitIdle2;
                });
                break;
            case State::WaitIdle2:
                if (!readBack()) {
                    mState = State::Idle;
                }
                break;
            case State::Transfer:
                readBack();
                transfer();
                break;
            }
            if (oldState != mState) {
                switch(mState) {
                case State::Undefined:
                    break;
                case State::WaitIdle:
                case State::WaitIdle2:
                    break;
                case State::Idle:
                    ss::inactivate();
                    break;
                case State::Transfer:
                    ss::activate();
                    mTxCounter.setToBottom();
                    mRxCounter.setToBottom();
                }                
            }    
        }
        inline static auto txByteCount() {
            return mTxByteCounter;
        }
        inline static auto rxByteCount() {
            return mRxByteCounter;
        }
        inline static auto txCount() {
            return mTxCounter;
        }
        inline static auto rxCount() {
            return mRxCounter;
        }
        
        inline static auto& rxQueue() {
            return mRxData;
        }
    private:
        static inline bool readBack() {
            if (mcu_spi()->intflags.template isSet<intflags_t::rxcif>()) {
                mActualRxData[mRxCounter] = *mcu_spi()->data;
                ++mRxByteCounter;
                if (mRxCounter.isTop()) {
                    mRxData.push_back(mActualRxData);
                }
                else {
                    ++mRxCounter;
                }
                return true;
            }
            return false;
        }
        static inline void transfer() {
            if (mcu_spi()->intflags.template isSet<intflags_t::dreif>()) {
                const std::byte& b = mTxData.front()[mTxCounter];
                *mcu_spi()->data = b;
                ++mTxByteCounter;
                if (mTxCounter.isTop()) {
                    mTxData.pop_front();
                    mState = State::WaitIdle;                                        
                }
                else {
                    ++mTxCounter;
                }
            }
        }
        static inline etl::uint_ranged<uint8_t, 0, Command::size() - 1> mTxCounter;
        static inline etl::uint_ranged<uint8_t, 0, Result::size() - 1>  mRxCounter;
        static inline State mState{State::Undefined};
        inline static uint16_t mTxByteCounter{};
        inline static uint16_t mRxByteCounter{};
        inline static txfifo_type mTxData;
        inline static rxfifo_type mRxData;
        inline static Result mActualRxData;
    };



    template<AVR::Concepts::ComponentPosition CP, etl::Concepts::Container Command, etl::Concepts::Container Result, 
             etl::Concepts::NamedConstant Size, typename SSPin, AVR::Concepts::At01DxSeries MCU>
    struct SpiBiSS2<CP, Command, Result, Size, SSPin, MCU> final {
        static inline constexpr auto N = CP::component_type::value;
        static constexpr auto mcu_spi = getBaseAddr<typename MCU::Spi, N>;
        static_assert(N < AVR::Component::Count<typename MCU::Spi>::value, "wrong number of spi");

        static_assert(Command::size() == Result::size());
        
        using mcu_spi_t = typename MCU::Spi;
        using ctrla1_t = typename MCU::Spi::CtrlA1_t;
        using ctrla2_t = typename MCU::Spi::CtrlA2_t;

        using ctrlb1_t = typename MCU::Spi::CtrlB1_t;
        using ctrlb2_t = typename MCU::Spi::CtrlB2_t;
        
        using intflags_t = typename MCU::Spi::IntFlags_t;
        
        inline static constexpr auto size{Size::value};

//        Size::_;
        
        static inline std::array<Command, size> txData;        
        
        using mosipin = AVR::Portmux::Map<CP, MCU>::mosipin;
        using sckpin = AVR::Portmux::Map<CP, MCU>::sckpin;

        using sspin = std::conditional_t<std::is_same_v<SSPin, void>, typename AVR::Portmux::Map<CP, MCU>::sspin, SSPin>;
        using ss = AVR::ActiveLow<sspin, AVR::Output>;
        
//        mosipin::_;
//        sckpin::_;
//        sspin::_;

        enum class State : uint8_t {Undefined, Idle, WaitIdle, WaitIdle2, Transfer, TransferWait, TransferWaitComplete};
        
        inline static void init() {
            mosipin::template dir<Output>();
            sckpin::template dir<Output>();
            ss::init();
            mcu_spi()->ctrlb.template set<ctrlb1_t::ssd | ctrlb1_t::bufen>();
//            mcu_spi()->ctrlb.template add<ctrlb2_t::mode3>();
            mcu_spi()->ctrla.template set<ctrla2_t::div4>();
            mcu_spi()->ctrla.template add<ctrla1_t::enable | ctrla1_t::master>();
            mState = State::Undefined;
        }
        
        inline static void mode1() {
            mcu_spi()->ctrlb.template add<ctrlb2_t::mode1>();
        }

        template<typename... C> 
        requires(Meta::all_same_front_v<Meta::List<C...>> && std::is_same_v<Command, Meta::front<Meta::List<C...>>> && (size == sizeof...(C)))
        inline static bool put(const C&... b) {
            if (mState == State::Idle) {
                uint8_t index{0};
                ((txData[index++] = b), ...);
                startTransfer();
                return true;
            }
            return false;
        }
        
        inline static void periodic() {
            const auto oldState = mState;
            switch(mState) {
            case State::Undefined:
                mState = State::Idle;
                break;
            case State::Idle:
                break;
            case State::WaitIdle:
                mcu_spi()->intflags.template testAndReset<intflags_t::txcif>([]{
                    mState = State::WaitIdle2;
                });
                break;
            case State::WaitIdle2:
                if (!readBack()) {
                    mLastRxData = mActualRxData;
                    mState = State::Idle;
                }
                break;
            case State::Transfer:
                transferWord();
                transferWord();
                break;
            case State::TransferWaitComplete:
                mcu_spi()->intflags.template testAndReset<intflags_t::txcif>([]{
                    mState = State::TransferWait;
                });
                break;
            case State::TransferWait:
                if (!readBack()) {
                    mState = State::Transfer;
                }
                break;
            }
            if (oldState != mState) {
                switch(mState) {
                case State::Undefined:
                    break;
                case State::WaitIdle:
                case State::WaitIdle2:
                    break;
                case State::Idle:
                    ss::inactivate();
                    break;
                case State::Transfer:
                    ss::activate();
                    mTxByteCounter.setToBottom();
                    mRxByteCounter.setToBottom();
                    break;
                case State::TransferWait:
                    ss::inactivate();
                    break;
                case State::TransferWaitComplete:
                    break;
                }                
            }    
        }
        inline static auto txTotalByteCount() {
            return mTxTotalByteCounter;
        }
        inline static auto rxTotalByteCount() {
            return mRxTotalByteCounter;
        }
        
        inline static const Result& result() {
            return mLastRxData;
        }
        
        inline static bool isIdle() {
            return mState == State::Idle;
        }
        
    private:
        inline static void startTransfer() {
            mcu_spi()->intflags.template reset<intflags_t::rxcif>();
            mTxByteCounter.setToBottom();
            mRxByteCounter.setToBottom();
            mTxWordCounter.setToBottom();
            ss::activate();
            mState = State::Transfer;
        }
        
        static inline bool readBack() {
            if (mcu_spi()->intflags.template isSet<intflags_t::rxcif>()) {
                mActualRxData[mRxByteCounter] = *mcu_spi()->data;
                ++mRxByteCounter;
                ++mRxTotalByteCounter;
                return true;
            }
            return false;
        }
        static inline void transferWord() {
            if (mcu_spi()->intflags.template isSet<intflags_t::dreif>()) {
                const std::byte& b = txData[mTxWordCounter][mTxByteCounter];
                *mcu_spi()->data = b;
                ++mTxTotalByteCounter;
                if (mTxByteCounter.isTop()) {
                    if (mTxWordCounter.isTop()) {
                        mState = State::WaitIdle;                                        
                    }
                    else {
                        mState = State::TransferWaitComplete;    
                        ++mTxWordCounter;
                        mTxByteCounter.setToBottom();
                    }
                }
                else {
                    ++mTxByteCounter;
                }
            }
        }

        static inline State mState{State::Undefined};
        static inline etl::uint_ranged<uint8_t, 0, size - 1> mTxWordCounter;
        inline static etl::uint_ranged<uint8_t, 0, Command::size() - 1> mTxByteCounter;
        inline static etl::uint_ranged<uint8_t, 0, Result::size() - 1> mRxByteCounter;
        
        inline static uint16_t mTxTotalByteCounter{};
        inline static uint16_t mRxTotalByteCounter{};
        inline static Result mActualRxData;
        inline static Result mLastRxData;
    };

    template<AVR::Concepts::ComponentPosition CP, etl::Concepts::Container Command, etl::Concepts::Container Result, 
             etl::Concepts::NamedConstant Size, typename SSPin, AVR::Concepts::At01DxSeries MCU>
    struct SpiSync<CP, Command, Result, Size, SSPin, MCU> final {
        static inline constexpr auto N = CP::component_type::value;
        static constexpr auto mcu_spi = getBaseAddr<typename MCU::Spi, N>;
        static_assert(N < AVR::Component::Count<typename MCU::Spi>::value, "wrong number of spi");

        static_assert(Command::size() == Result::size());
        
        using mcu_spi_t = typename MCU::Spi;
        using ctrla1_t = typename MCU::Spi::CtrlA1_t;
        using ctrla2_t = typename MCU::Spi::CtrlA2_t;

        using ctrlb1_t = typename MCU::Spi::CtrlB1_t;
        using ctrlb2_t = typename MCU::Spi::CtrlB2_t;
        
        using intflags_t = typename MCU::Spi::IntFlags_t;
        
        inline static constexpr auto size{Size::value};

//        Size::_;
        
        static inline std::array<Command, size> txData;        
        
        using mosipin = AVR::Portmux::Map<CP, MCU>::mosipin;
        using sckpin = AVR::Portmux::Map<CP, MCU>::sckpin;

        using sspin = std::conditional_t<std::is_same_v<SSPin, void>, typename AVR::Portmux::Map<CP, MCU>::sspin, SSPin>;
        using ss = AVR::ActiveLow<sspin, AVR::Output>;
        
//        mosipin::_;
//        sckpin::_;
//        sspin::_;

        enum class State : uint8_t {Undefined, Idle, WaitIdle, WaitIdle2, Transfer, TransferWait, TransferWaitComplete};
        
        inline static void init() {
            mosipin::template dir<Output>();
            sckpin::template dir<Output>();
            ss::init();
            mcu_spi()->ctrlb.template set<ctrlb1_t::ssd | ctrlb1_t::bufen>();
            mcu_spi()->ctrla.template set<ctrla2_t::div4>();
            mcu_spi()->ctrla.template add<ctrla1_t::enable | ctrla1_t::master>();
        }
        
        inline static void mode1() {
            mcu_spi()->ctrlb.template add<ctrlb2_t::mode1>();
        }

        template<bool increaseTime = false, typename... C> 
        requires(Meta::all_same_front_v<Meta::List<C...>> && std::is_same_v<Command, Meta::front<Meta::List<C...>>> && (size == sizeof...(C)))
        inline static void put(const C&... b) {
            (putSingle<increaseTime>(b, std::make_index_sequence<Command::size()>{}), ...);
        }
        
        template<bool increaseTime = false, size_t... II>
        inline static void putSingle(const Command& b, std::index_sequence<II...>) {
            ss::activate();
            (put<II>(b[II]), ...);
            ss::inactivate();
            if constexpr(increaseTime) {
                ss::inactivate();
//                ss::inactivate();
//                ss::inactivate();
            }
        }
        
        template<size_t II>
        inline static void put(const std::byte b) {
            mcu_spi()->intflags.template waitFor<intflags_t::dreif>();
            *mcu_spi()->data = b;
            mcu_spi()->intflags.template waitFor<intflags_t::rxcif>();
            mRxData[II] = *mcu_spi()->data;
        }
        
        inline static const Result& result() {
            return mRxData;
        }
    private:
        inline static Result mRxData;
    };
}

