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

#define NDEBUG

#include "ledbuttons.h"
#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/usart.h"
#include "units/duration.h"
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

using namespace AVR;

using PortB = Port<DefaultMcuType::PortRegister, B>;
using PortC = Port<DefaultMcuType::PortRegister, C>;

template<typename Button, typename Led>
class ButtonLed {
public:
    typedef Button button;    
    typedef Led led;    
    static void init() {
        Led::template dir<Output>();
        Button::template dir<Input>();
        Button::pullup();
    }
    static void update(std::milliseconds timeStamp) {
        if (Button::isHigh() && ((timeStamp - mStartTime) > mDelay) ) {
            Led::off();
        }
        if (!Button::isHigh()) {
            mStartTime = timeStamp;
            Led::on();
        }
    }
    template<typename Terminal>
    static void print() {
        std::outl<Terminal>("TasterPin: "_pgm, Button::isHigh(), " LedPin: "_pgm, Led::get(), " Startzeit: "_pgm, mStartTime);
    }

private:
    inline static constexpr auto mDelay = 1000_ms;
    inline static auto mStartTime = 0_ms;
};

template<typename... BTs>
class ButtonLedsController {
public:
    static void init() {
        (BTs::init(), ...);
    }
    static void periodic() {
        ++mTimeStamp;
        (BTs::update(mTimeStamp), ...);
    }
    template<typename Terminal>
    static void print() {
        (BTs::template print<Terminal>(), ...);
    }  
private:
    static inline auto mTimeStamp = 0_ms;
};

template<typename ...> struct makeController;
template<typename LedPort, typename ButtonPort, size_t... Is>
struct makeController<LedPort, ButtonPort, std::index_sequence<Is...>> {
    typedef ButtonLedsController<ButtonLed<Pin<LedPort, Is>, Pin<ButtonPort, Is> >... > type;
};

using controller = typename makeController<PortB, PortC, std::make_index_sequence<2>>::type;

using systemTimer = AVR::Timer8Bit<0>;

using terminalDevice = Usart<0>;
using terminal = std::basic_ostream<terminalDevice>;

using pic = ::ISR::PeriodicInterruptCallback<AVR::ISR::Timer<0>::CompareA, controller>;
using isrController = ::ISR::Controller<pic, terminalDevice::RxHandler, terminalDevice::TxHandler>; 

template<typename Timer, const std::hertz& f>
struct LocalConfig {
    static constexpr AVR::Util::TimerSetupData tsd = AVR::Util::caculateForExactFrequencyAbove<Timer>(1000_Hz);
    static_assert(tsd, "wrong timer parameter");
    static void init() {
        Timer::template prescale<tsd.prescaler>();
        Timer::template ocra<tsd.ocr - 1>();
        Timer::mode(AVR::TimerMode::CTC);
    }
};

static constexpr auto f = 1000_Hz;

int main() {
    controller::init();
    terminalDevice::init<9600>();

    LocalConfig<systemTimer, f>::init();
    
    {
        Scoped<EnableInterrupt<>> ei;
        while(true) {
            isrController::periodic();
            if (auto c = terminalDevice::get()) {
                if (*c == std::byte{'p'}) {
                    controller::print<terminal>();    
                }
            }
        }
    }
}

ISR(TIMER0_COMPA_vect) {
    isrController::isr<AVR::ISR::Timer<0>::CompareA>();
}

ISR(USART_RX_vect) {
    isrController::isr<AVR::ISR::Usart<0>::RX>();
}
ISR(USART_UDRE_vect){
    isrController::isr<AVR::ISR::Usart<0>::UDREmpty>();
}

