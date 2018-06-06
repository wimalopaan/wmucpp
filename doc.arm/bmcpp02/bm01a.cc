#define NDEBUG

#include <sam.h>
#include <type_traits>
#include <util/meta.h>

namespace ARM {
    namespace Sam {
        struct A {};
        struct B {};
        struct C {};
        struct D {};
        struct E {};
        struct F {};
        struct G {};
        struct H {};
        struct I {};
        
        struct Input {};
        struct Output {};
        struct Bidirectional {};
        struct Sampling {};
        
        template<typename> struct index;
        template<> struct index<A> : std::integral_constant<uint8_t, 0> {};
        template<> struct index<B> : std::integral_constant<uint8_t, 1> {};
        
        template<typename MCU = DefaultMcuType>
        struct PowerManager {
            typedef MCU mcu_type;
            
            
        };

        template<typename Letter, typename MCU = DefaultMcuType>
        struct GpioPort {
            typedef MCU mcu_type;
            typedef PowerManager<DefaultMcuType> pm_type;
            
            inline static constexpr auto pgroup = []{return &PORT->Group[index<Letter>::value];};
            
            static inline constexpr void init() {
                
            }
        };
        
        template<typename Port, auto Number, typename Direction>
        struct GpioPin {
            typedef Port port_type;
            typedef std::integral_constant<uint8_t, Number> number_type;
            
            typedef Direction direction_type;
            
            inline static constexpr auto pgroup = []{return port_type::pgroup();};
            
            static inline constexpr void init() {
                Port::init();
                if constexpr(!std::is_same_v<Direction, Bidirectional>) {
                    dir<true>(Direction{});
                }
            }
            template<typename Dir, bool visible = std::is_same_v<Direction, Bidirectional>, typename = std::enable_if_t<visible>>
            static inline constexpr void dir() {
                dir(Dir{});
            }
            template<bool visible = std::is_same_v<Direction, Bidirectional>, typename = std::enable_if_t<visible>>
            static inline constexpr void dir(const Input&) {
                pgroup()->DIRCLR.reg = (1 << Number);
                pgroup()->PINCFG[Number].bit.INEN = true;
            }
            template<bool visible = std::is_same_v<Direction, Bidirectional>, typename = std::enable_if_t<visible>>
            static inline constexpr void dir(const Output&) {
                pgroup()->DIRSET.reg = (1 << Number);
            }
            template<bool visible = std::is_same_v<Direction, Output>, typename = std::enable_if_t<visible>>
            static inline constexpr void toggle() {
                pgroup()->OUTTGL.reg = (1 << Number);
            }
            template<bool visible = std::is_same_v<Direction, Input>, typename = std::enable_if_t<visible>>
            static inline constexpr void pullup(bool on) {
                pgroup()->PINCFG[Number].bit.PULLEN = true;
            }
        };
        
    }
}

using namespace ARM::Sam;

using porta = GpioPort<A>;
using portb = GpioPort<B>;

using pin0 = GpioPin<porta, 0, Output>;
using pin1 = GpioPin<porta, 1, Input>;

int main() {
    SYSCTRL->DPLLCTRLA.bit.ENABLE = true;
    SYSCTRL->OSC8M.bit.PRESC = 0; 
    
    PM->APBBMASK.bit.PORT_ = true;
    
    //    pin0::dir<Output>();    
    //    pin1::dir<Input>();    
    
    pin0::init();
    pin1::init();
    
    while(true) {
        pin0::toggle();
    }
}
