//#define NDEBUG

#define SCHALTMODUL

template<typename S1, typename S2>
[[noreturn]] void assertFunction([[maybe_unused]] const S1& expr, [[maybe_unused]] const S2& file, [[maybe_unused]] unsigned int line) {
    while(true) {
    }    
}

#include "board.h"

using terminal = etl::basic_ostream<rcUsart>;

using qtrobo = etl::basic_ostream<qtroboUsart, etl::lineTerminator<etl::LF>>;

using log0 = External::QtRobo::Logging<qtrobo, 0>;
using gauge0 = External::QtRobo::Gauge<qtrobo, 0>;
using plot0 = External::QtRobo::ValuePlot<qtrobo, 0>;
using toggle0 = External::QtRobo::Toggle<qtrobo, 0>;
using pong = External::QtRobo::Pong<qtrobo>;

template<typename MCU = DefaultMcuType>
struct FSM final {
    enum class State : uint8_t {Idle, Busy};
    
    inline constexpr static void periodic_slow() {
        qtroboPA::whenChanged([]{
        });
        qtroboPA::whenPinged([]{
        });
    }
    inline constexpr static void periodic() {
        switch (mState) {
        case State::Idle:
            if (qtroboPA::toggleValues[0]) {
                mState = State::Busy;
                mEdgeCount = 2;
            }
            if (qtroboPA::toggleValues[1]) {
                mState = State::Busy;
                mEdgeCount = 2;
            }
            if (qtroboPA::toggleValues[2]) {
                mState = State::Busy;
                mEdgeCount = 3;
            }
            if (qtroboPA::toggleValues[3]) {
                mState = State::Busy;
                mEdgeCount = 4;
            }
            break;
        case State::Busy:
            break;
        default:
            break;
        }
    }
    inline static void edge() {
        static bool rising = false;
        if (mState == State::Busy) {
            if (rising = !rising; rising) {
                risingEdge();
            }
            else {
                fallingEdge();
            }
        }
    }
    inline static void risingEdge() {
        ppmInPin::on();
        etl::uint_ranged_NaN<uint8_t, 0, 100> vv{99};
//        cppm::ppm(0, vv);
        toggle0::put(true);
    }
    inline static void fallingEdge() {
        ppmInPin::off();
        etl::uint_ranged_NaN<uint8_t, 0, 100> vv{50};
//        cppm::ppm(0, vv);
        toggle0::put(false);
        if (--mEdgeCount == 0) {
            mState = State::Idle;
            qtroboPA::toggleValues[0] = false;
        }
    }
    
private:
    FSM() = delete;
    
    static inline State mState = State::Idle;
    static inline uint8_t mEdgeCount = 0;
    
};


using fsm = FSM<>;

using components = AVR::Components<rcUsart, qtroboUsart, cppm>;

int main() {
    using namespace etl;
    using namespace std;
    using namespace AVR;
    
    systemClock::init();
    cppm::init();
    
    rxSelect::dir<Output>();
    rxSelect::off();
    
    paired::dir<Input>();
    ppmInPin::dir<Output>(); // debug
    
    rcUsart::init<AVR::BaudRate<115200>>();
    qtroboUsart::init<AVR::BaudRate<9600>>();
    
    const auto st = alarmTimer::create(3000_ms, External::Hal::AlarmFlags::Periodic);
    const auto t = alarmTimer::create(1000_ms, External::Hal::AlarmFlags::Periodic);
    const auto edgeTimer = alarmTimer::create(75_ms, External::Hal::AlarmFlags::Periodic);
    
    {
        Scoped<EnableInterrupt<>> ei;
        
        log0::outl("Schaltmodul 01"_pgm);    
        
        while(true) {
            ppmInPin::toggle();
            components::periodic();
            fsm::periodic();
            systemClock::periodic([&](){
                fsm::periodic_slow();
                alarmTimer::periodic([&](const alarmTimer::index_type timer){
                    if (timer == t) {
                        log0::outl("v "_pgm, qtroboPA::propValues[0], Char{','}, qtroboPA::toggleValues[0], Char{','});    
                        etl::outl<terminal>("v "_pgm, qtroboPA::propValues[0], Char{','}, qtroboPA::toggleValues[0], Char{','});    
                    }
                    if (timer == edgeTimer) {
                        fsm::edge();                        
                    }
                    if (timer == st) {
                        qtroboPA::toggleValues[0] = !qtroboPA::toggleValues[0];
                    }
                });
            });
        }    
    }
}

