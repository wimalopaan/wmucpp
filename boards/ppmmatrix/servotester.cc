//#define NDEBUG

#define SERVOTESTER

template<typename S1, typename S2>
[[noreturn]] void assertFunction([[maybe_unused]] const S1& expr, [[maybe_unused]] const S2& file, [[maybe_unused]] unsigned int line) {
    while(true) {
    }    
}

#include "board.h"

using qtrobo = etl::basic_ostream<qtroboUsart, etl::lineTerminator<etl::LF>>;

using log0 = External::QtRobo::Logging<qtrobo, 0>;
using gauge0 = External::QtRobo::Gauge<qtrobo, 0>;
using plot0 = External::QtRobo::ValuePlot<qtrobo, 0>;
using toggle0 = External::QtRobo::Toggle<qtrobo, 0>;
using pong = External::QtRobo::Pong<qtrobo>;

template<typename MCU = DefaultMcuType>
struct FSM final {
    enum class State : uint8_t {Direct, AutoSweep, Receiver};
    
    inline constexpr static void periodic_slow() {
        qtroboPA::whenChanged([]{
            gauge0::put(qtroboPA::propValues[0]);    
            toggle0::put(qtroboPA::toggleValues[0]);
        });
        qtroboPA::whenPinged([]{
            pong::put();
        });
    }

    inline constexpr static void periodic() {
        switch(updateCycle.toInt()) {
        case 0:
            update_00();
            break;
        case 1:
            update_01();
            break;
        case 2:
            update_02();
            break;
        case 3:
            update_03();
            break;
        case 4:
            update_04();
            break;
        case 5:
            update_05();
            break;
        case 6:
            update_06();
            break;
        case 7:
            update_07();
            break;
        case 8:
            update_08();
            break;
        }
        ++updateCycle;
    }
private:
    inline static constexpr void update_00() {
        auto v = qtroboPA::propValues[0];
        etl::uint_ranged_NaN<uint8_t, 0, 100> vv{v};
        
        switch (state) {
        case State::Direct:
            cppm::ppm(outputChannel, vv);
            break;
        case State::Receiver:
        {
            auto v = sumd::value(inputChannel);
            cppm::ppm(outputChannel, v);
        }
            break;
        case State::AutoSweep:
            break;
        default:
            break;
        }
    }        
    inline static constexpr void update_01() {
        auto v = qtroboPA::propValues[1];
        etl::uint_ranged_NaN<uint8_t, 0, 100> vv{v};
        cppm::ppm(1, vv);
    }        
    inline static constexpr void update_02() {
        auto v = qtroboPA::propValues[2];
        etl::uint_ranged_NaN<uint8_t, 0, 100> vv{v};
        cppm::ppm(2, vv);
    }        
    inline static constexpr void update_03() {
        auto v = qtroboPA::propValues[3];
        etl::uint_ranged_NaN<uint8_t, 0, 100> vv{v};
        cppm::ppm(3, vv);
    }        
    inline static constexpr void update_04() {
        auto v = qtroboPA::propValues[4];
        etl::uint_ranged_NaN<uint8_t, 0, 100> vv{v};
        cppm::ppm(4, vv);
    }        
    inline static constexpr void update_05() {
        auto v = qtroboPA::propValues[5];
        etl::uint_ranged_NaN<uint8_t, 0, 100> vv{v};
        cppm::ppm(5, vv);
    }        
    inline static constexpr void update_06() {
        auto v = qtroboPA::propValues[6];
        etl::uint_ranged_NaN<uint8_t, 0, 100> vv{v};
        cppm::ppm(6, vv);
    }        
    inline static constexpr void update_07() {
        auto v = qtroboPA::propValues[7];
        etl::uint_ranged_NaN<uint8_t, 0, 100> vv{v};
        cppm::ppm(7, vv);
    }        
    inline static constexpr void update_08() {
        if (qtroboPA::toggleValues[1]) {
            state = State::Receiver;
        }
        else {
            if (qtroboPA::toggleValues[0]) {
                state = State::AutoSweep;
            }
            else {
                state = State::Direct;
            }
        }
    }        
    
    inline static constexpr etl::uint_ranged<uint8_t, 0, 7> inputChannel{0};

    inline static etl::uint_ranged<uint8_t, 0, 7> outputChannel;

    inline static State state = State::Direct;
    inline static etl::uint_ranged_circular<uint8_t, 0, 8> updateCycle;
    FSM() = delete;
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
    
    rcUsart::init<115200>();
    qtroboUsart::init<9600>();
    
    const auto t = alarmTimer::create(1000_ms, External::Hal::AlarmFlags::Periodic);

    {
        Scoped<EnableInterrupt<>> ei;

        log0::outl("ServoTester 01"_pgm);    
        
        while(true) {
            ppmInPin::toggle();
            components::periodic();
            fsm::periodic();
            systemClock::periodic([&](){
                fsm::periodic_slow();
                alarmTimer::periodic([&](alarmTimer::index_type timer){
                    if (timer == t) {
                        log0::outl("v "_pgm, qtroboPA::propValues[0], Char{','}, qtroboPA::toggleValues[0], Char{','});    
                    }
                });
            });
        }    
    }
}

