#include "mcu/mcu.h"
#include "units.h"
#include "concepts.h"
#include "mcu/mcu_traits.h"
#include "mcu/alternate.h"
#include "meta.h"

#include <type_traits>
#include <concepts>
#include <array>

namespace Mcu::Stm {
    using namespace Units::literals;

#ifdef USE_MCU_STM_V2
    inline 
#endif
    namespace V2 {

        namespace Pwm {
                
//            template<typename Pin, uint8_t Channel>
//            struct Output {
//                using pin_t = Pin;
//                static inline constexpr uint8_t channel = Channel;
//            };

            template<typename Timer, typename OutList> struct Servo;
            
            template<typename Timer, typename... Outs>
            struct Servo<Timer, Meta::List<Outs...>> {
//                static inline constexpr uint8_t size = sizeof...(Outs);
                static inline constexpr std::integral_constant<std::size_t, sizeof...(Outs)> size{};
                
                static void init() {
                    Timer::init();
                }  
                static void duty(const std::array<uint16_t, size> d) {
                }
            };
            
        }
        
    }
}
