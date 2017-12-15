#include "mcu/avr8.h"
#include "mcu/ports.h"

using namespace AVR;

using PortB = Port<AVR::ATMega328P::PortRegister, B>;
using PortD = Port<AVR::ATMega328P::PortRegister, D>;

using motor = Pin<PortB, 1>;
using beleuchtung = Pin<PortB, 2>;
using startknopf = Pin<PortD, 1>;
using bremse = Pin<PortD, 2>;
using gang = Pin<PortD, 3>;

int main() {
    motor::dir<Output>();
    beleuchtung::dir<Output>();
    startknopf::dir<Input>();
    bremse::dir<Input>();
    gang::dir<Input>();

    while(true) {
         if (startknopf::isHigh()) {
             beleuchtung::on();
             if (gang::isHigh() and bremse::isHigh()) {
                 motor::on();
             }
         } else {
             motor::off();
             beleuchtung::off();
         }
     }
}
