#define NDEBUG

#include <sam.h>
#include <type_traits>
#include <util/meta.h>

namespace ARM {
    namespace Sam {
        namespace PortGroup {
            struct PA {};
            struct PB {};
        }
//        namespace Function {
//            struct A {};
//            struct B {};
//            struct C {};
//            struct D {};
//            struct E {};
//            struct F {};
//            struct G {};
//            struct H {};
//            struct I {};
//        }
        
        namespace Options {
            template<typename...> struct Name {};
            
            template<typename... Options> struct Input {};
            struct Sampled {};
            struct Buffered {};
            
            template<typename Kind> struct Pull {};
            struct Up {};
            struct Down {};
            struct Off {};
            
            template<typename F> struct Alternate {};
            struct ExtInt {};
            struct Ref {};
            struct Adc {};
            struct Ac0 {};
            struct Ac1 {};
            struct Dac {};
            
            template<typename T>
            struct Uart {};
            struct RxD {};
            struct TxD {};
            
            template<typename T>
            struct Spi {};
            struct Miso {};
            struct Mosi {};
            struct Sck {};
            struct SSel {};
            
            template<typename T>
            struct I2C {};
            struct Sda {}; // Sda s.o.
            
            struct Output {};
        }
        
        template<typename> struct GroupIndex;
        template<> struct GroupIndex<PortGroup::PA> : std::integral_constant<uint8_t, 0> {};
        template<> struct GroupIndex<PortGroup::PB> : std::integral_constant<uint8_t, 1> {};
        
        template<typename MCU = DefaultMcuType>
        struct PowerManager {
            typedef MCU mcu_type;
        };
        
        template<typename Letter, typename MCU = DefaultMcuType>
        struct GpioPort {
            typedef MCU mcu_type;
            typedef PowerManager<DefaultMcuType> pm_type;
            
            inline static constexpr auto pgroup = []{return &PORT->Group[GroupIndex<Letter>::value];};
            
            static inline constexpr void init() {
                
            }
        };
        
        template<typename Port, auto Number, typename Option, typename Name = Options::Name<void>>
        struct GpioPin {
            typedef Port port_type;
            typedef std::integral_constant<uint8_t, Number> number_type;
            typedef Option option_type;
            typedef Name name_type;
            
            inline static constexpr auto pgroup = []{return port_type::pgroup();};
            
            static inline constexpr void init() {
            }
            
        };
        
        namespace Devices {
            template<typename RxPin, typename TxPin>
            struct Uart {
                static_assert(std::is_same_v<typename RxPin::name_type, typename TxPin::name_type>);
                
                static inline void init() {
                    
                }
            };
            
        }
        
    }
}

namespace App {
    using namespace ARM::Sam;
    using namespace ARM::Sam::Options;
    using namespace ARM::Sam::PortGroup;
    
    using porta = GpioPort<PA>;
    using portb = GpioPort<PB>;
    
    using pin0 = GpioPin<porta, 0, Output>;
    using pin1 = GpioPin<porta, 1, Input<Sampled, Buffered, Pull<Up>>>;
    
    using pin2 = GpioPin<porta, 2, Alternate<ExtInt>>;
    
    struct DebugUart;
    using pinrx0 = GpioPin<porta, 3, Alternate<Uart<RxD>>, Name<DebugUart>>;
    using pintx0 = GpioPin<porta, 4, Alternate<Uart<TxD>>, Name<DebugUart>>;
    
    struct GpsUart;
    using pinrx1 = GpioPin<porta, 5, Alternate<Uart<RxD>>, Name<GpsUart>>;
    using pintx1 = GpioPin<porta, 6, Alternate<Uart<TxD>>, Name<GpsUart>>;
    
    using uart0 = Devices::Uart<pinrx0, pintx0>;
    
}

int main() {
    using namespace App;
    
    //    SYSCTRL->DPLLCTRLA.bit.ENABLE = true;
    //    SYSCTRL->OSC8M.bit.PRESC = 0; 
    
    //    PM->APBBMASK.bit.PORT_ = true;
    
    App::uart0::init();
    
    pin0::init();
    pin1::init();
    
    while(true) {
        //        pin0::toggle();
    }
}
