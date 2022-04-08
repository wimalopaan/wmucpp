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
        "test0"_s + sml::on_entry<_> / []{x = 40;},
        "test0"_s + event<re> [ ([](auto v) {return v.v == 0;}) ]  = "test1"_s,
        "test1"_s + sml::on_entry<_> / []{x = 41;},
        "test1"_s + event<re> [ ([](auto v) {return v.v == 1;}) ]  = "test2"_s,
        "test2"_s + sml::on_entry<_> / []{x = 42;},
        "test2"_s + event<re> [ ([](auto v) {return v.v == 2;}) ]  = "test3"_s,
        "test3"_s + sml::on_entry<_> / []{x = 43;},
        "test3"_s + event<re> [ ([](auto v) {return v.v == 3;}) ]  = "test4"_s,
        "test4"_s + sml::on_entry<_> / []{x = 44;},
        "test4"_s + event<re> [ ([](auto v) {return v.v == 4;}) ]  = "test5"_s,
        "test5"_s + sml::on_entry<_> / []{x = 45;},
        "test5"_s + event<re> [ ([](auto v) {return v.v == 5;}) ]  = "test6"_s,
        "test6"_s + sml::on_entry<_> / []{x = 46;},
        "test6"_s + event<re> [ ([](auto v) {return v.v == 6;}) ]  = "test7"_s,
        "test7"_s + sml::on_entry<_> / []{x = 47;},
        "test7"_s + event<re> [ ([](auto v) {return v.v == 7;}) ]  = "test8"_s,
        "test8"_s + sml::on_entry<_> / []{x = 48;},
        "test8"_s + event<re> [ ([](auto v) {return v.v == 8;}) ]  = "test9"_s,
        "test9"_s + sml::on_entry<_> / []{x = 49;},
        "test9"_s + event<re> [ ([](auto v) {return v.v == 9;}) ]  = "running"_s,
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

