#include <mcu/avr.h>

int main() {
    using mcu_timer_t = typename DefaultMcuType::TCD; 

    constexpr auto mcu_tcd = AVR::getBaseAddr<mcu_timer_t, 0>;

    auto& sr = mcu_tcd()->status;    
    
//    waitFor<mcu_timer_t::Status_t::enready>(mcu_tcd()->status);
    sr.waitFor<mcu_timer_t::Status_t::enready>();
}
