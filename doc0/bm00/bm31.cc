#include <mcu/avr.h>

#include <cstdint>

using namespace AVR;

using p1 = Pin<Port<B>, 0>;

int main() {
    
    p1::template dir<Output>();
    
    p1::on();
    
//    VPORTA.DIR |= 0x01;
}
