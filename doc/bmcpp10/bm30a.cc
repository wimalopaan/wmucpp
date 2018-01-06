#define NDEBUG

#include "mcu/avr8.h"
#include "mcu/avr/usart.h"
#include "mcu/avr/mcutimer.h"
#include "std/literals.h"

using namespace std::literals::physical;

using uart0 = AVR::Usart<0>;
using timer0 = AVR::Timer8Bit<0>;

constexpr auto f = 100_Hz;

int main() {
//    timer0::setup<f>(AVR::TimerMode::CTCNoInt);    
//    timer0::periodic<timer0::flags_type::ocfa>([](){});    
    
    uart0::init<9600>();
    uart0::put(std::byte{0});
}
