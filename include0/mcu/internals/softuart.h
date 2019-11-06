#pragma once

#include <cstdint>

#include "../common/concepts.h"
#include "../common/groups.h"

#include <etl/fifo.h>
#include <etl/concepts.h>

// todo: 

namespace AVR {
    
    template<bool B>
    using UseRx = etl::NamedFlag<B>;
    
    namespace SoftDevices {
        
        template<etl::Concepts::NamedConstant IcpTimerNumber, 
                 etl::Concepts::NamedConstant Baud,
                 AVR::Concepts::Pin TxPin, 
                 etl::Concepts::NamedFlag UseRX = etl::NamedFlag<false>, 
                 etl::Concepts::NamedConstant RxQueueLength = etl::NamedConstant<64>, 
                 etl::Concepts::NamedConstant TxQueueLength = etl::NamedConstant<128>, 
                 typename MCU = DefaultMcuType>
        class IcpUart;
        
        template<etl::Concepts::NamedConstant TimerNumber, 
                 etl::Concepts::NamedConstant Baud,
                 AVR::Concepts::Pin TxPin, 
                 etl::Concepts::NamedFlag UseRX = etl::NamedFlag<false>, 
                 etl::Concepts::NamedFlag UseInts = etl::NamedFlag<true>, 
                 etl::Concepts::NamedConstant IntNumber = AVR::ExternalInterruptNumber<0>,
                 etl::Concepts::NamedConstant RxQueueLength = etl::NamedConstant<64>, 
                 etl::Concepts::NamedConstant TxQueueLength = etl::NamedConstant<128>, 
                 typename MCU = DefaultMcuType>
        class IntUart;
        
        template<etl::Concepts::NamedConstant TimerNumber,
                 etl::Concepts::NamedConstant Baud,
                 AVR::Concepts::Pin TxPin, 
                 etl::Concepts::NamedFlag UseRX, 
                 etl::Concepts::NamedFlag UseInts, 
                 etl::Concepts::NamedConstant IntNumber,
                 etl::Concepts::NamedConstant RxQueueLength, 
                 etl::Concepts::NamedConstant TxQueueLength, typename MCU>
        requires (AVR::Concepts::AtMega_X8<MCU> && ((TimerNumber::value == 0) || (TimerNumber::value == 2)))
        class IntUart<TimerNumber, Baud, TxPin, UseRX, UseInts, IntNumber, RxQueueLength, TxQueueLength, MCU> final {
            
            static inline constexpr auto timer_number = TimerNumber::value;
            
            using mcu_timer_type = mcu_timer_t<timer_number>;
            using value_type  = typename TimerParameter<timer_number, MCU>::value_type;        
            using ta = typename TimerParameter<timer_number, MCU>::ta;        
            using tb = typename TimerParameter<timer_number, MCU>::tb;        
            
            using flags_type = mcu_timer_interrupts_flags_t<timer_number>;
            using mask_type = mcu_timer_interrupts_mask_t<timer_number>;
            
            static constexpr auto mcu_timer = TimerParameter<timer_number, MCU>::mcu_timer;
            static constexpr auto mcu_timer_interrupts = TimerParameter<timer_number, MCU>::mcu_timer_interrupts;
            
            static constexpr auto external_interrupts = AVR::getBaseAddr<typename MCU::Interrupt>;
            
            using param_type = AVR::Util::SoftUart::ExternalInterrupt<IntNumber::value>;
            
            IntUart() = delete;
            
            using noop = etl::DisbaleInterrupt<etl::NoDisableEnable>;
            
            static inline void isr_impl() {
                if (outframe != 0x0001) {
                    if (outframe & 0x0001) {
                        TxPin::on();
                    }
                    else {
                        TxPin::off();
                    }
                    outframe = (outframe >> 1);
                }
                else {
                    if (auto c = sendQueue.pop_front()) {
                        outframe = (3 << 9) | ((uint8_t)*c << 1);
                    }
                    else {
                        outframe = 0x0001;
                        mcu_timer_interrupts()->timsk.template clear<mask_type::ociea, noop>();
                    }
                }
            }
            
            public:
            
            template<bool visible = !UseInts::value>
            static inline 
            std::enable_if_t<visible> periodic() {
                if(mcu_timer_interrupts()->tifr.template isSet<flags_type::ocfa>()) {
                    isr_impl();
                    mcu_timer_interrupts()->tifr.template reset<flags_type::ocfa>();
                }
            }
            
            struct TxHandler : public IsrBaseHandler<typename AVR::ISR::Timer<timer_number>::CompareA> {
                template<bool visible = UseInts::value>
                static inline
                std::enable_if_t<visible> isr() {
                    isr_impl();
                }
            };
            struct StartBitHandler : public IsrBaseHandler<typename AVR::ISR::Int<IntNumber::value>> {
                template<bool visible = UseRX::value>
                static inline std::enable_if_t<visible> isr() {
                    uint8_t ocrb = (*mcu_timer()->tcnt - (tsd.ocr / 10) + tsd.ocr / 2);
                    if (ocrb >= tsd.ocr) {
                        ocrb -= tsd.ocr;
                    }
                    *mcu_timer()->ocrb = ocrb;
                    
                    mcu_timer_interrupts()->tifr.template reset<flags_type::Flags::ocfb>();
                    mcu_timer_interrupts()->timsk.template add<mask_type::Mask::ocieb, noop>();
                    
                    external_interrupts()->eimsk.template clear<param_type::mask, noop>(); // disbale
                    external_interrupts()->eifr.template reset<param_type::flag>(); // clear
                    
                    inframe = 0;
                    inbits = 0;
                }
            };
            struct RxHandler : public IsrBaseHandler<typename AVR::ISR::Timer<timer_number>::CompareB> {
                template<bool visible = UseRX::value>
                static inline std::enable_if_t<visible> isr() {
                    inframe >>= 1;
                    if (param_type::rx::read()) {
                        inframe |= (1 << 8);
                    }
                    ++inbits;
                    if (inbits == 9) {
                        uint8_t c = 0xff & (inframe >> 1);
                        //                        if constexpr(!std::is_same<PA, void>::value) {
                        //                            PA::process(std::byte{c});
                        //                        }
                        //                        else {
                        //                            //                        EventManager::enqueueISR({SWUsartEventMapper<N>::event, std::byte{c}});
                        //                        }
                        mcu_timer_interrupts()->timsk.template clear<mask_type::Mask::ocieb, noop>();
                        external_interrupts()->eimsk.template add<param_type::mask, noop>(); // enable
                        external_interrupts()->eifr.template reset<param_type::flag>(); // clear
                        
                        inframe = 0;
                        inbits = 0;
                    }
                }
            };
            
            static inline void init() {
                
                constexpr auto bits = bitsFrom<tsd.prescaler>(prescaler_bits_v<timer_number>);            
                static_assert(isset(bits), "wrong prescaler");
                mcu_timer()->tccrb.template set<bits>();
                
                *mcu_timer()->ocra= tsd.ocr;
                mcu_timer()->tccra.template add<mcu_timer_type::TCCRA::wgm1, noop>();
                
                mcu_timer_interrupts()->tifr.template reset<flags_type::ocfb | flags_type::ocfa>();
                if constexpr(UseRX::value) {
                    external_interrupts()->eicra.template add<param_type::edge_falling, noop>(); // falling
                    external_interrupts()->eimsk.template add<param_type::mask, noop>(); // enable
                    external_interrupts()->eifr.template reset<param_type::flag>(); // clear
                } 
                TxPin::template dir<AVR::Output>();
                TxPin::on();
                
                if constexpr(UseRX::value) {
                    param_type::pin::template dir<AVR::Input>();
                    param_type::pin::pullup();
                }
                
            }
            static inline bool put(std::byte item) {
                static_assert(TxQueueLength::value > 0);
                if (sendQueue.push_back(item)) {
                    if constexpr(UseInts::value) {
                        mcu_timer_interrupts()->timsk.template add<mask_type::ociea>();
                    }
                    return true;
                }
                return false;
            }
            template<bool enable, bool visible = UseRX::value>
            static inline std::enable_if_t<visible> rxEnable() {
                mcu_timer_interrupts()->tifr.template add<flags_type::Flags::icf>();
                if constexpr (enable) {
                    mcu_timer_interrupts()->timsk.template add<mask_type::Mask::ocieb>();
                    external_interrupts()->eimsk.template add<MCU::Interrupt::EIMask::int2, noop>(); // ???????????????????????? int2
                }
                else {
                    mcu_timer_interrupts()->timsk.template clear<mask_type::Mask::ociea>();
                    external_interrupts()->eimsk.template clear<MCU::Interrupt::EIMask::int2, noop>(); // ?????????????????? int2
                }
            }
            private:
            
            using fifo_type = std::conditional_t<UseInts::value, volatile etl::FiFo<std::byte, TxQueueLength::value>, etl::FiFo<std::byte, TxQueueLength::value>>;
            using inframe_type = std::conditional_t<UseInts::value, volatile uint16_t, uint16_t>;
            using inbits_type = std::conditional_t<UseInts::value, volatile uint8_t, uint8_t>;
            
            inline static fifo_type sendQueue;
            inline static volatile uint16_t outframe = 0x0001;
            inline static inframe_type inframe = 0;
            inline static inbits_type inbits = 0;
            
            inline static constexpr auto tsd = AVR::Util::Timer::calculate<TimerNumber::value>(External::Units::hertz{Baud::value});
            static_assert(Baud::value >= 2400, "SWUSART should use a valid baud rate >= 2400");
            static_assert(tsd, "can't calculate timer setup");
        };
        
        template<etl::Concepts::NamedConstant IcpTimerNumber, 
                 etl::Concepts::NamedConstant Baud, 
                 AVR::Concepts::Pin TxPin, 
                 etl::Concepts::NamedFlag UseRX, 
                 etl::Concepts::NamedConstant RxQueueLength, 
                 etl::Concepts::NamedConstant TxQueueLength, typename MCU>
        requires AVR::Concepts::AtMega_X8<MCU> || ((IcpTimerNumber::value == 0) && AVR::Concepts::AtTiny_X4<MCU>)
        class IcpUart<IcpTimerNumber, Baud, TxPin, UseRX, RxQueueLength, TxQueueLength, MCU> final {
            static_assert(IcpTimerNumber::value < 2, "wrong swusart number");
            
            static constexpr auto mcu_timer = AVR::getBaseAddr<typename MCU::Timer16Bit, IcpTimerNumber>;
            static constexpr auto mcu_timer_interrupts = AVR::getBaseAddr<typename MCU::Timer16Interrupts, IcpTimerNumber>;
            
            typedef MCU mcu_type;
            typedef typename MCU::Timer16Bit mcu_timer_type;
            typedef typename MCU::Timer16Interrupts int_type;
            
            IcpUart() = delete;
            
            using noop = etl::DisbaleInterrupt<etl::NoDisableEnable>;
            
            public:
            struct TransmitBitHandler : public IsrBaseHandler<typename AVR::ISR::Timer<IcpTimerNumber::value>::CompareA> {
                static inline void isr() {
                    if (outframe != 0x0001) {
                        if (outframe & 0x0001) {
                            TxPin::on();
                        }
                        else {
                            TxPin::off();
                        }
                        outframe = (outframe >> 1);
                    }
                    else {
                        if (auto c = sendQueue.pop_front()) {
                            outframe = (3 << 9) | ((uint8_t)*c << 1);
                        }
                        else {
                            outframe = 0x0001;
                            mcu_timer_interrupts()->timsk.template clear<int_type::Mask::ociea, noop>();
                        }
                    }
                }
            };
            
            struct StartBitHandler : public IsrBaseHandler<typename AVR::ISR::Timer<IcpTimerNumber::value>::Capture> {
                template<bool visible = UseRX::value>
                static inline std::enable_if_t<visible> isr() {
                    uint16_t ocra = *mcu_timer()->ocra;
                    *mcu_timer()->ocrb = (*mcu_timer()->icr + ocra / 2) % ocra;
                    
                    mcu_timer_interrupts()->tifr.template reset<int_type::Flags::ocfb>();
                    mcu_timer_interrupts()->timsk.template clear<int_type::Mask::icie, noop>();
                    mcu_timer_interrupts()->timsk.template add<int_type::Mask::ocieb, noop>();
                    inframe = 0;
                    inbits = 0;
                }
            };
            struct ReceiveBitHandler : public IsrBaseHandler<typename AVR::ISR::Timer<IcpTimerNumber::value>::CompareB> {
                template<bool visible = UseRX::value>
                static inline std::enable_if_t<visible> isr() {
//                    inframe >>= 1;
                    inframe = inframe >> 1;
                    if (Util::SoftUart::Icp<IcpTimerNumber::value>::rx::read()) {
//                        inframe |= (1 << 8);
                        inframe = inframe | (1 << 8);
                    }
//                    ++inbits;
                    inbits = inbits + 1;
                    if (inbits == 9) {
                        uint8_t c = 0xff & (inframe >> 1);
                        mcu_timer_interrupts()->timsk.template clear<int_type::Mask::ocieb, noop>();
                        mcu_timer_interrupts()->timsk.template add<int_type::Mask::icie, noop>();
                        mcu_timer_interrupts()->tifr.template reset<int_type::Flags::icf>();
                        inframe = 0;
                        inbits = 0;
                    }
                }
            };
            
            static inline void init() {
                constexpr auto bits = bitsFrom<tsd.prescaler>(mcu_timer_type::template PrescalerBits<IcpTimerNumber>::values);            
                static_assert(isset(bits), "wrong prescaler");
                mcu_timer()->tccrb.template set<bits>();
                
                *mcu_timer()->ocra= tsd.ocr;
                
                mcu_timer()->tccrb.template add<mcu_timer_type::TCCRB::wgm2, noop>();
                mcu_timer()->tccrb.template add<mcu_timer_type::TCCRB::icnc, noop>();
                mcu_timer()->tccrb.template clear<mcu_timer_type::TCCRB::ices, noop>();
                
                mcu_timer_interrupts()->timsk.template add<int_type::Mask::icie, noop>();
                mcu_timer_interrupts()->tifr.template reset<int_type::Flags::icf | int_type::Flags::ocfb | int_type::Flags::ocfa>();
                
                TxPin::template dir<AVR::Output>();
                TxPin::on();
                
                if constexpr(UseRX::value) {
                    Util::SoftUart::Icp<IcpTimerNumber::value>::rx::template dir<AVR::Input>();
                    Util::SoftUart::Icp<IcpTimerNumber::value>::rx::on();
                }
            }
            static inline bool put(std::byte item) {
                if (sendQueue.push_back(item)) {
                    mcu_timer_interrupts()->timsk.template add<int_type::Mask::ociea>();
                    return true;
                }
                return false;
            }
            template<bool enable, bool visible = UseRX::value>
            static inline std::enable_if_t<visible> rxEnable() {
                mcu_timer_interrupts()->tifr.template add<int_type::Flags::icf>();
                if constexpr (enable) {
                    mcu_timer_interrupts()->timsk.template clear<int_type::Mask::ocieb | int_type::Mask::icie>();
                }
                else {
                    mcu_timer_interrupts()->timsk.template clear<int_type::Mask::ociea | int_type::Mask::icie>();
                }
            }
            private:
            inline static etl::FiFo<std::byte, TxQueueLength::value> sendQueue;
            inline static volatile uint16_t outframe = 0x0001;
            inline static volatile uint16_t inframe = 0;
            inline static volatile uint8_t inbits = 0;
            
            static_assert(Baud::value >= 2400, "SWUSART should use a valid baud rate >= 2400");
            
            inline static constexpr auto tsd = AVR::Util::Timer::calculate<IcpTimerNumber>(External::Units::hertz{Baud::value});
            static_assert(tsd, "can't calculate timer setup");
        };
        
        
        // todo: TxPin konfigurierbar machen
        //    template<uint8_t N, AVR::Concepts::AtTiny_X5 MCU>
        //    class SoftUart<N, MCU> final {
        //        static_assert(N < 2, "wrong swusart number");
        
        //        static constexpr uint8_t mcu_timer_number = N;
        
        //        using mcu_timer_type = typename std::conditional<(N == 0), typename MCU::Timer8Bit, typename MCU::Timer8BitHighSpeed>::type;
        
        //        static constexpr auto timer = AVR::getBaseAddr<mcu_timer_type, mcu_timer_number>;
        //        static constexpr auto mcuInterrupts = AVR::getBaseAddr<typename MCU::TimerInterrupts>;
        
        //        typedef typename MCU::TimerInterrupts int_type;
        //    public:
        //        SWUsart() = delete;
        
        //        template<uint16_t Baud>
        //        static void init() {
        //            static_assert(Baud >= 2400, "SWUSART should use a valid baud rate >= 2400");
        
        //            constexpr auto tsd = AVR::Util::calculate<AVR::Timer8Bit<mcu_timer_number>>(std::hertz{Baud});
        //            static_assert(tsd, "can't calculate timer setup");
        
        //            AVR::Timer8Bit<mcu_timer_number>::template prescale<tsd.prescaler>();
        //            *timer()->ocra = tsd.ocr;
        
        //            timer()->tccra.template add<mcu_timer_type::TCCRA::wgm1, DisbaleInterrupt<NoDisableEnable>>();
        
        //            if constexpr(N == 0) {
        //                mcuInterrupts()->tifr.template reset<int_type::Flags::ocf0a>();
        //            }
        //            else {
        //                mcuInterrupts()->tifr.template add<int_type::Flags::ocf1a, DisbaleInterrupt<NoDisableEnable>>();
        //            }
        //            SWUsartRxTx<N>::tx::template dir<AVR::Output>();
        //            SWUsartRxTx<N>::tx::on();
        //        }
        //        static bool put(std::byte item) {
        //            if (sendQueue.push_back(item)) {
        //                if constexpr(N == 0) {
        //                    mcuInterrupts()->timsk.template add<int_type::Mask::ocie0a>();
        //                }
        //                else {
        //                    mcuInterrupts()->timsk.template add<int_type::Mask::ocie1a>();
        //                }
        //                return true;
        //            }
        //            return false;
        //        }
        //        struct TxHandler : public IsrBaseHandler<typename AVR::ISR::Timer<mcu_timer_number>::CompareA> {
        //            static void isr() {
        //                if (outframe != 0x0001) {
        //                    if (outframe & 0x0001) {
        //                        SWUsartRxTx<N>::tx::on();
        //                    }
        //                    else {
        //                        SWUsartRxTx<N>::tx::off();
        //                    }
        //                    outframe = (outframe >> 1);
        //                }
        //                else {
        //                    if (auto c = sendQueue.pop_front()) {
        //                        outframe = (3 << 9) | ((uint8_t)*c << 1);
        //                    }
        //                    else {
        //                        outframe = 0x0001;
        //                        if constexpr(N == 0) {
        //                            mcuInterrupts()->timsk.template clear<int_type::Mask::ocie0a>();
        //                        }
        //                        else {
        //                            mcuInterrupts()->timsk.template clear<int_type::Mask::ocie1a>();
        //                        }
        //                    }
        //                }
        //            }
        //        };
        //    private:    
        //        inline static std::FiFo<std::byte, Config::Usart::SendQueueLength> sendQueue;
        //        inline static volatile uint16_t outframe = 0x0001;
        //    };
        
    }
    
}
