/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "mcu/avr8.h"
#include "mcu/avr/mcutimer.h"

#include "util/meta.h"

#include "hal/alarmtimer.h"
#include "hal/event.h"

#include "simavr/simavrdebugconsole.h"
#include "console.h"

namespace ISR {
    namespace detail {
        template<typename T>
        concept bool PeriodicCallback() {
            return requires (T p){
                T::init();                
                T::periodic();                
            };
        }
        template<typename T>
        concept bool PeriodicInterruptCallback() {
            return requires (T p){
                typename T::picb;                
            };
        }
        template<typename W>
        concept bool CRWriter() { 
            return requires (W w) { 
                W::init();
                W::start();
                W::rateProcess();
            };
        }
        template<typename T>
        concept bool ConstantRateAdapter() {
            return requires (T a){
                typename T::crad;                
            };
        }
    }
    template<MCU::Interrupt Interrupt, detail::PeriodicCallback... CBs>
    struct PeriodicInterruptCallback  {
        typedef PeriodicInterruptCallback picb;
        typedef Interrupt interrupt_type;
        static void init() {
            (CBs::init(), ...);
        }
        static void periodic() {
            (CBs::periodic(), ...);
        }
    };
    template<MCU::Timer Timer, MCU::Interrupt Interrupt, detail::CRWriter... CRs>
    struct ConstantRateAdapter {
        typedef ConstantRateAdapter crad;
        typedef Interrupt interrupt_type;
        static void init() {
            (CRs::init(), ...);
        }
        static void start() {
            (CRs::start(),...);
        }
        static void periodic() {
            (CRs::rateProcess(), ...);
        }
    };
    
    namespace detail {
        template<uint8_t N = 0>
        using flags = AVR::RegisterFlags<DefaultMcuType::GPIOR, N, std::byte>;
        
        template<typename Flags, uint8_t N, typename PIC>
        struct NumberedInterruptCallback : public IsrBaseHandler<typename PIC::interrupt_type> {
            typedef NumberedInterruptCallback isr_type;
            
            inline static constexpr uint8_t interrupt_number = PIC::interrupt_type::number;
            static_assert(interrupt_number >= 0, "wrong interrupt number");
            
            inline static constexpr uint8_t number = N;
            inline static constexpr uint8_t BitNumber = N;
            static_assert(BitNumber < 8, "wrong bit number");
            static inline constexpr typename Flags::type bit_mask{1 << BitNumber};
            
            static void init() {
                PIC::init();
            }
            static void periodic() {
                if (std::any(Flags::get() & bit_mask)) {
                    Flags::get() &= ~bit_mask;
                    PIC::periodic();
                }
            }
            static void isr() {
                Flags::get() |= bit_mask;
            }
        };
        
        template<typename Flags, uint8_t N, typename  T>
        struct TypeMapper;

        template<typename Flags, uint8_t N, typename T>
        requires detail::PeriodicInterruptCallback<T>() || detail::ConstantRateAdapter<T>()
        struct TypeMapper<Flags, N, T> {
            typedef detail::NumberedInterruptCallback<Flags, N, T> type;
            typedef T isr_type;
        };
        template<typename Flags, uint8_t N, MCU::IServiceR ISR>
        struct TypeMapper<Flags, N, ISR> {
            template<typename I>
            struct IsrOnly {
                typedef ISR isr_type;
                static void init() {}
                static void periodic() {}
            };
            typedef IsrOnly<ISR> type;            
        };
        template<typename Flags, uint8_t N>
        struct TypeMapper<Flags, N, void> {
            struct Empty {
                typedef void isr_type;
                static void init() {}
                static void periodic() {}
            };
            typedef Empty type;
        };
        template<typename Flags, uint8_t N, typename T>
        using mapped_type = typename TypeMapper<Flags, N, T>::type;
        
        template<typename Flags, typename... PICBs> 
        requires ((MCU::IServiceR<PICBs>() || detail::ConstantRateAdapter<PICBs>() || detail::PeriodicInterruptCallback<PICBs>()) && ...)
        struct Controller {
            inline static constexpr uint8_t size = sizeof...(PICBs);
            using Indexes = std::make_index_sequence<sizeof...(PICBs)>;
            
            template<typename... T>
            struct TypeList {
                using isrReg = IsrRegistrar<typename T::isr_type...>;
                static void check() {
                    isrReg::init();
                }
                static void init() {
                    (T::init(),...);
                }
                static void periodic() {
                    (T::periodic(),...);
                }
                template<MCU::Interrupt Int>
                static void isr() {
                    isrReg::template isr<Int>();
                }
            };
        
            template<typename>
            struct XXX;
            template<size_t... Is>
            struct XXX<std::index_sequence<Is...>> {
                using tl = TypeList<mapped_type<Flags, Is, Util::nth_element<Is, PICBs...>>...>;
                
            };
            
            using xxx = XXX<Indexes>;
            
            static void init() {
                init(Indexes{});
            }            
            template<size_t... Is>
            static void init(std::index_sequence<Is...>) {
                using tl = TypeList<mapped_type<Flags, Is, Util::nth_element<Is, PICBs...>>...>;
                tl::check();
                tl::init();
            }
            static void periodic() {
                periodic(Indexes{});
            }
            template<size_t... Is>
            static void periodic(std::index_sequence<Is...>) {
                using tl = TypeList<mapped_type<Flags, Is, Util::nth_element<Is, PICBs...>>...>;
                tl::periodic();
            }
            template<MCU::Interrupt INT>
            static void isr() {
                isr<INT>(Indexes{});
            }            
            template<MCU::Interrupt INT, size_t... Is>
            static void isr(std::index_sequence<Is...>) {
                using tl = TypeList<mapped_type<Flags, Is, Util::nth_element<Is, PICBs...>>...>;
                tl::template isr<INT>();
            }            
        };
    }
    template<typename... PICBs>
    using Controller = detail::Controller<detail::flags<0>, PICBs...>;
}

namespace  {
    constexpr bool useTerminal = false;
}

struct TestCallback {
    static void init() {}
    static void periodic() {
        ++x;
    }
    inline static volatile uint8_t x = 0; 
};

struct TestIsr : public IsrBaseHandler<AVR::ISR::PcInt<0>> {
    static void isr() {
        --y;
    }
    inline static volatile uint8_t y = 0; 
};

struct TestCRWriter {
    static void init() {}
    static void start() {}
    static void rateProcess() {
        z++;
    }
    inline static volatile uint8_t z = 0; 
};

using systemTimer = AVR::Timer8Bit<0>;

struct LocalConfig {
    static constexpr AVR::Util::TimerSetupData tsd = AVR::Util::caculateForExactFrequencyAbove<systemTimer>(1000_Hz);
    static_assert(tsd, "wrong timer parameter");
    static constexpr std::milliseconds reso = std::duration_cast<std::milliseconds>(1 / tsd.f);
    static constexpr std::hertz exactFrequency = tsd.f;
};

using terminalDevice = SimAVRDebugConsole;
using terminal = std::conditional<useTerminal, std::basic_ostream<terminalDevice>, void>::type;

using t1 = ISR::PeriodicInterruptCallback<AVR::ISR::Timer<0>::CompareA, TestCallback>;
using r1 = ISR::ConstantRateAdapter<AVR::Timer8Bit<2>, AVR::ISR::Timer<2>::CompareA, TestCRWriter>;
using isrController = ISR::Controller<t1, r1, TestIsr, void>; 

using allEventHandler = EventHandlerGroup<>; 

namespace detail {
    template<typename Terminal>
    void main() {
        isrController::init();
        if constexpr(!std::is_same<Terminal, void>::value) {
            Terminal::device_type::template init<2400>();
            std::outl<Terminal>("ConstantRate"_pgm);
        }
        
        systemTimer::template prescale<LocalConfig::tsd.prescaler>();
        systemTimer::template ocra<LocalConfig::tsd.ocr - 1>();
        systemTimer::mode(AVR::TimerMode::CTC);
        systemTimer::start();
        
        {
            Scoped<EnableInterrupt<>> ei;
            while(true)  {
                isrController::periodic();
            }
        }
    }
}

int main() {
    detail::main<terminal>();
}

ISR(TIMER0_COMPA_vect) {
    isrController::isr<AVR::ISR::Timer<0>::CompareA>();
}
ISR(TIMER1_COMPA_vect) {
    isrController::isr<AVR::ISR::Timer<2>::CompareA>();
}
ISR(PCINT0_vect) {
    isrController::isr<AVR::ISR::PcInt<0>>();
}
