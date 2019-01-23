#pragma once

#include <etl/concepts.h>
#include <etl/types.h>
#include <etl/meta.h>

namespace External {
    namespace Hal {
        template<Device... DD>
        struct DeviceMapper {
            using device_list = Meta::List<DD...>;
            inline static constexpr uint8_t Size = Meta::size<device_list>::value;
            
            using use_isr_type = typename Meta::front<device_list>::use_isr_type;
            static_assert((DD::use_isr_type::value && ...) == use_isr_type::value);
            
            inline static void select(etl::uint_ranged<uint8_t, 0, Size-1>) {
            }
            
            template<auto... Baud>
            inline static void init() {
                
            }
            
            inline static void periodic() {
            }
            
        private:
            
        };
        
    }
}
