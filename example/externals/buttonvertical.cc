#define NDEBUG
//#define USE_INTERRUPTS
#define USE_8_BUTTONS
#define USE_DIFF_PORT

// Vergleich (avr-gcc-7.3.1)                    text/data/bss
//                                              (immer 7 Bit)           (immer 2 Bit)
//                                                                      (immer ISR)
// # of B | Ports | Interrupt | Counter Bits |  button.cc               buttonmacro.cc     buttonvertical.cc
//   3        B        yes         2            442/0/3                 478/2/2            468/0/4
//   8        B        yes         2            786/0/8                 518/2/2            734/0/4
//   3        B        no          2            376/0/3                   -                316/0/4
//   8        B        no          2            682/0/8                   -                390/0/4
//   3       B/C       no          2            376/0/3                   -                344/0/4
//   8       B/C       no          2            682/0/8                   -                518/0/4
//   3       B/C       no          4              (7)                     -                344/0/4
//   8       B/C       no          4              (7)                     -                378/0/6

// Vergleich (avr-gcc-8.1.0)                    text/data/bss
//                                              (immer 7 Bit)           (immer 2 Bit)
//                                                                      (immer ISR)
// # of B | Ports | Interrupt | Counter Bits |  button.cc               buttonmacro.cc     buttonvertical.cc
//   3        B        yes         2            430/0/3                 460/2/2            468/0/4
//   8        B        yes         2            780/0/8                 552/2/2            728/0/4
//   3        B        no          2            376/0/3                   -                322/0/4
//   8        B        no          2            682/0/8                   -                394/0/4
//   3       B/C       no          2            376/0/3                   -                350/0/4
//   8       B/C       no          2            682/0/8                   -                530/0/4
//   3        B        no          4               -                      -                346/0/4
//   8        B        no          4               -                      -                418/0/6
//   3       B/C       no          4               -                      -                378/0/4
//   8       B/C       no          4               -                      -                554/0/6


#include <stdlib.h>

#include "mcu/avr8.h"
#include "mcu/ports.h"
#include "mcu/avr/mcutimer.h"
#include "hal/alarmtimer.h"
#include "hal/button_v.h"
#include "simavr/simavrdebugconsole.h"
#include "console.h"

using terminalDevice = SimAVRDebugConsole;
using terminal = std::basic_ostream<terminalDevice>;

using PortA = AVR::Port<DefaultMcuType::PortRegister, AVR::A>;
using PortB = AVR::Port<DefaultMcuType::PortRegister, AVR::B>;
using PortC = AVR::Port<DefaultMcuType::PortRegister, AVR::C>;
using PortD = AVR::Port<DefaultMcuType::PortRegister, AVR::D>;

using buttonPin0 = AVR::Pin<PortB, 0>;
using buttonPin1 = AVR::Pin<PortB, 1>;
#ifdef USE_DIFF_PORT
using buttonPin2 = AVR::Pin<PortC, 2>;
#else
using buttonPin2 = AVR::Pin<PortB, 2>;
#endif
using buttonPin3 = AVR::Pin<PortB, 3>;
using buttonPin4 = AVR::Pin<PortB, 4>;
using buttonPin5 = AVR::Pin<PortB, 5>;
using buttonPin6 = AVR::Pin<PortB, 6>;
using buttonPin7 = AVR::Pin<PortB, 7>;
using button0 = Button<0, buttonPin0, UseEvents<false>>;
using button1 = Button<1, buttonPin1, UseEvents<false>>;
using button2 = Button<2, buttonPin2, UseEvents<false>>;
using button3 = Button<3, buttonPin3, UseEvents<false>>;
using button4 = Button<4, buttonPin4, UseEvents<false>>;
using button5 = Button<5, buttonPin5, UseEvents<false>>;
using button6 = Button<6, buttonPin6, UseEvents<false>>;
using button7 = Button<7, buttonPin7, UseEvents<false>>;

using ledPort = PortA;
using led0 = AVR::Pin<PortA, 0>;
using led1 = AVR::Pin<PortA, 1>;
using led2 = AVR::Pin<PortA, 2>;

using systemTimer = AVR::Timer8Bit<0>;
    
using buttonController = ButtonControllerVertical<4, 
#ifdef USE_INTERRUPTS
    AVR::ISR::Timer<0>::CompareA
#else
    void
#endif
    , ActiveLow<true>
    , button0, button1, button2
#ifdef USE_8_BUTTONS
    , button3
    , button4
    , button5
    , button6
    , button7
#endif
    >;

#ifdef USE_INTERRUPTS
using isrRegistrar = IsrRegistrar<buttonController>;
#endif
    
static constexpr auto f = 100_Hz;

int main() {
#ifdef USE_INTERRUPTS
    isrRegistrar::init();
#endif
    buttonController::init();
    led0::dir<AVR::Output>();
    led1::dir<AVR::Output>();
    led2::dir<AVR::Output>();
    
#ifdef USE_INTERRUPTS
    systemTimer::setup<f>(AVR::TimerMode::CTC);
    {
        Scoped<EnableInterrupt<>> ei;
        while(true) {
            if (buttonController::isPressed<button0>()) {
                led0::toggle();
            }    
            if (buttonController::isPressed<button1>()) {
                led1::toggle();
            }    
            if (buttonController::isPressed<button2>()) {
                led2::toggle();
            } 
#ifdef USE_8_BUTTONS
            if (buttonController::isPressed<button3>()) {
                led0::toggle();
            }    
            if (buttonController::isPressed<button4>()) {
                led1::toggle();
            }    
            if (buttonController::isPressed<button5>()) {
                led2::toggle();
            }    
            if (buttonController::isPressed<button6>()) {
                led0::toggle();
            }    
            if (buttonController::isPressed<button7>()) {
                led1::toggle();
            }    
#endif
        }
    }
#else
    systemTimer::setup<f>(AVR::TimerMode::CTCNoInt);
    while(true) {
        systemTimer::periodic<systemTimer::flags_type::ocfa>([](){
            buttonController::periodic(
                        [](){
                            led0::toggle();
                        },
                        [](){
                            led1::toggle();
                        },
                        [](){
                            led2::toggle();
                        }
#ifdef USE_8_BUTTONS
            ,[](){
                led2::toggle();
            }
            ,[](){
                led1::toggle();
            }
            ,[](){
                led2::toggle();
            }
            ,[](){
                led0::toggle();
            }
            ,[](){
                led1::toggle();
            }
#endif
            );
//            buttonController::periodic([](buttonController::DataType b){
//                if (b & 0x01) {
//                    led0::toggle();
//                }
//                else if (b & 0x02) {
//                    led1::toggle();
//                }
//                else if (b & 0x04) {
//                    led2::toggle();
//                }
//            });
        });
    }
#endif    
}
#ifdef USE_INTERRUPTS
ISR(TIMER0_COMPA_vect) {
    isrRegistrar::isr<AVR::ISR::Timer<0>::CompareA>();
}
#endif
