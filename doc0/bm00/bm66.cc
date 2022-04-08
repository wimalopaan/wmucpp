#include <mcu/avr.h>
#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/adc.h>

#include <external/hal/adccontroller.h>
#include <external/hal/alarmtimer.h>
#include <external/bluetooth/roboremo.h>
#include <external/bluetooth/qtrobo.h>
#include <external/sbus/sbus.h>

#include <external/solutions/button.h>
#include <external/solutions/rotaryencoder.h>

#include <std/chrono>

#include <etl/output.h>
#include <etl/meta.h>

#include <etl/fsm.h>

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

volatile uint8_t r;
volatile uint8_t x;

namespace {
    constexpr auto fRtc = 500_Hz; // 14ms    
    using systemTimer = SystemTimer<AVR::Component::Rtc<0>, fRtc>;
    
    template<typename Timer>
    struct Off;
    template<typename Timer>
    struct Start;
    template<typename Timer>
    struct Running;
    template<typename Timer>
    struct Error;
    
    using off = Off<systemTimer>;
    using running = Running<systemTimer>;
    using start = Start<systemTimer>;
    using error = Error<systemTimer>;
    
    template<typename Timer>
    struct Off {
        using timer = Timer;
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 1) {
                S::template toState<start>();                    
            }
        }            
        static inline constexpr void onEnter() {
        }            
        static inline constexpr void onExit() {
        }            
    };
    template<typename Timer>
    struct Start {
        using timer = Timer;
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 3) {
                S::template toState<running>();                    
            }
            S::on(timeout, [](){
                S::template toState<error>();                                    
            });
        }            
        static inline constexpr void onEnter() {
            x = 43;
        }            
        static inline constexpr void onExit() {
        }            
    private:
        static inline constexpr External::Tick<Timer> timeout{500_ms};        
    };
    template<typename Timer>
    struct Running {
        using timer = Timer;
        template<typename S>
        static inline constexpr void process(const uint8_t b) {
            if (b == 3) {
                S::template toState<off>();                    
            }
            else if (b > 100) {
                S::template toState<error>();                    
            }
        }            
        static inline constexpr void onEnter() {
            x = 44;
        }            
        static inline constexpr void onExit() {
        }            
    };
    template<typename Timer>
    struct Error {
        using timer = Timer;
        template<typename S>
        static inline constexpr void process(const uint8_t) {
        }            
        static inline constexpr void onEnter() {
        }            
        static inline constexpr void onExit() {
        }            
    };
    
    using sp = FSM::Timed::StateProcessor<off, Meta::List<off, running, start, error>, uint8_t>;
    
}

int main() {
    while(true) {
        sp::ratePeriodic();
        uint8_t b = r;
        sp::process(b);
    }
}

