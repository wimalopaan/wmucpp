#include <mcu/avr.h>
#include <cstdint>
#undef TSM
#include <sml.h>

namespace sml = boost::sml;

volatile uint8_t r = 0;
volatile uint8_t x = 0;

namespace {
    struct re {
        const uint8_t v{};
    };
    
    using namespace sml;
    
    struct machine {
        inline auto operator()() const {
            return sml::make_transition_table(
                        *"off"_s + event<re> [ ([](auto v) {return v.v == 1;}) ] = "start"_s,
                        "start"_s + event<re> [ ([](auto v) {return v.v == 2;}) ]  = "running"_s,
                        "start"_s + sml::on_entry<_> / []{x = 43;},
                        "running"_s + event<re> [ ([](auto v) {return v.v == 3;}) ]= "off"_s,
                        "running"_s + sml::on_entry<_> / []{x = 44;},
                        "running"_s + event<re> [ ([](auto v) {return v.v > 100;}) ] = "error"_s
                                                     );
        }
    };
    struct FSM {
        static inline void periodic() {
            fsm.process_event(re{r});
        }
    private:
        inline static sml::sm<machine> fsm{};
    };
} 

using fsm = FSM;

int main() {
    while(true) {
        fsm::periodic();
    }
}

