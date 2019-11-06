//#include <mcu/avr.h>
//#undef TSM
#include <cstdint>
#include <sml.h>

namespace sml = boost::sml;

volatile uint8_t r = 0;
volatile uint8_t x = 0;

namespace {
    struct start {};
    struct run {};
    struct off {};
    struct error {};
    
    using namespace sml;
    
    struct machine {
        inline auto operator()() const noexcept {
            return sml::make_transition_table(
                        *"off"_s + event<start> = "start"_s,
                        "start"_s + event<run>  = "running"_s,
                        "start"_s + sml::on_entry<_> / []{x = 43;},
                        "running"_s + event<off> = "off"_s,
                        "running"_s + sml::on_entry<_> / []{x = 44;},
                        "running"_s + event<error> = "error"_s
                                                     );
        }
    };
} 

struct FSM {
    void periodic() {
        toEvent(r);
    }
    void toEvent(uint8_t value) {
        if (value == 1) {
            fsm.process_event(start{});
        }
        else if (value == 2) {
            fsm.process_event(run{});
        }
        else if (value == 3) {
            fsm.process_event(off{});
        }
        else if (value > 100) {
            fsm.process_event(error{});
        }
    }
private:
    sml::sm<machine> fsm{};
};

int main() {
    FSM fsm;    
    while(true) {
        fsm.periodic();
    }
}
