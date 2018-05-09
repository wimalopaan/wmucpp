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

#include "mcu/avr8.h"
#include "mcu/avr/mcutimer.h"
#include "util/meta.h"
#include "hal/alarmtimer.h"
#include "hal/event.h"
#include "simavr/simavrdebugconsole.h"
#include "console.h"

struct TestIsr;

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
        template<typename T> struct isIsr : public std::false_type{};
        template<MCU::IServiceRNonVoid T> struct isIsr<T> : public std::true_type{};
        
        template<typename T> struct isCallback : public std::false_type{};
        template<typename T> requires (ConstantRateAdapter<T>() || PeriodicInterruptCallback<T>()) struct isCallback<T> : public std::true_type{};

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
        
        template<typename Flags, uint8_t BitNumber, typename Callback>
        struct NumberedInterruptCallback : public IsrBaseHandler<typename Callback::interrupt_type> {
            static_assert(BitNumber < 8, "wrong bit number");
            static inline constexpr typename Flags::type bit_mask{1 << BitNumber};
            
            static void init() {
                Callback::init();
            }
            static void periodic() {
                if (std::any(Flags::get() & bit_mask)) {
                    Flags::get() &= ~bit_mask;
                    Callback::periodic();
                }
            }
            static void isr() {
                Flags::get() |= bit_mask;
            }
        };

        template<typename Flags, typename... PICBs> 
        requires ((MCU::IServiceR<PICBs>() || detail::ConstantRateAdapter<PICBs>() || detail::PeriodicInterruptCallback<PICBs>()) && ...)
        struct Controller {
            using simpleISRs = Meta::filter<detail::isIsr, Meta::List<PICBs...>>;
            using callbacks = Meta::filter<detail::isCallback, Meta::List<PICBs...>>;

            template<typename CB, size_t N>
            using makeNumberedCallback = detail::NumberedInterruptCallback<Flags, N, CB>;

            using numberedCallbacks = Meta::transformN<makeNumberedCallback, callbacks>;
         
            using isrReg = Meta::apply<IsrRegistrar, Meta::concat<simpleISRs, numberedCallbacks>>;
            
            template<typename... T> struct Distributor {
                typedef Distributor<T...> type;
                static void init() {
                    (T::init(),...);
                }
                static void periodic() {
                    (T::periodic(),...);
                }
            };     
            
            using distributor = Meta::apply<Distributor, numberedCallbacks>;
            
            static void init() {
                isrReg::init();   
                distributor::init();
            }
            
            static void periodic() {
                distributor::periodic();
            }
            
            template<MCU::Interrupt INT>
            static void isr() {
                isrReg::template isr<INT>();
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
        ++z;
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
using isrController = ISR::Controller<t1, void, void, r1, TestIsr, void>; 

static_assert(Meta::size<isrController::simpleISRs>::value == 1);
static_assert(Meta::size<isrController::callbacks>::value  == 2);
static_assert(Meta::size<isrController::numberedCallbacks>::value  == 2);

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
