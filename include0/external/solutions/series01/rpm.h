
#include <std/chrono>

#include "mcu/common/concepts.h"
#include "mcu/internals/port.h"
#include "mcu/common/isr.h"

namespace External {
    namespace Rpm {
        template<typename Pin, typename Timer, typename MCU = DefaultMcuType>
        struct RpmGpio {
            using port = Pin::port;
            using port_name = port::name_type;
            
            using isr =  typename AVR::ISR::Port<port_name>;

            inline static void init() {
                Pin::template dir<AVR::Input>();
                Pin::template attributes<typename AVR::Attributes::Interrupt<AVR::Attributes::OnRising>>();
            }
            
            struct ImpulsIsr : public AVR::IsrBaseHandler<isr> {
                inline static void isr() {
                    auto value = Timer::actual();
                    mDiff = value - mLastValue;
                    mLastValue = value;
                }
            };
            
            inline static const auto& diff() {
                return mDiff;
            }
            
        private:
            inline static uint16_t mLastValue = 0;
            inline static uint16_t mDiff = 0;
        };
    }
    
}
