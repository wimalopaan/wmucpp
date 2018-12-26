#define NDEBUG

template<typename S1, typename S2>
[[noreturn]] void assertFunction([[maybe_unused]] const S1& expr, [[maybe_unused]] const S2& file, [[maybe_unused]] unsigned int line) {
    while(true) {
    }    
}

#include "board.h"

//using isrRegistrar = AVR::IsrRegistrar<rcUsart::RxHandler, rcUsart::TxHandler, btUsart::RxHandler, btUsart::TxHandler>;
//using isrRegistrar = AVR::IsrRegistrar<btUsart::RxHandler, btUsart::TxHandler>;

using terminal = etl::basic_ostream<rcUsart>;
using robo = etl::basic_ostream<btAndSensorUsart>;

template<typename V, auto Size = 8, typename MCU = DefaultMcuType>
struct Matrix final {
    inline static constexpr V map(etl::uint_ranged<uint8_t, 0, Size - 1> channel) {
        
    }
private:
    Matrix() = delete;
    
    std::array<uint8_t, Size> factors;
    std::array<bool, Size>    inverter;
    
};

template<typename MCU = DefaultMcuType>
struct FSM final {
    inline constexpr static void periodic() {
        switch(state.toInt()) {
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
        ++state;
    }
private:
    inline static constexpr void update_00() {
        cppm::ppm(0, sumd::value(0));
    }        
    inline static constexpr void update_01() {
        cppm::ppm(1, sumd::value(1).invert());
    }        
    inline static constexpr void update_02() {
        cppm::ppm(2, sumd::value(2));
    }        
    inline static constexpr void update_03() {
        cppm::ppm(3, sumd::value(3));
    }        
    inline static constexpr void update_04() {
        cppm::ppm(4, sumd::value(4));
    }        
    inline static constexpr void update_05() {
        cppm::ppm(5, sumd::value(5));
    }        
    inline static constexpr void update_06() {
        cppm::ppm(6, sumd::value(6));
    }        
    inline static constexpr void update_07() {
        cppm::ppm(7, sumd::value(7));
    }        
    inline static constexpr void update_08() {
        if (paired::isHigh()) {
            
        }        
    }        
    inline static etl::uint_ranged_circular<uint8_t, 0, 8> state;

    FSM() = delete;
};


using fsm = FSM<>;

using components = AVR::Components<btAndSensorUsart, rcUsart, cppm, fsm>;

struct AsciiHandler {
    static void start() {
        crWriterSensorText::enable<true>();
    }    
    static void stop() {
        // not: das disable sollte automatisch laufen
        //        crWriterSensorText::enable<false>();
    }    
    static void process(std::byte ) {
    }
};
struct BinaryHandler {
    static void start() {
        crWriterSensorBinary::enable<true>();
    }    
    static void stop() {
        // not: das disable sollte automatisch laufen
        //        crWriterSensorBinary::enable<false>();
    }    
};
struct BCastHandler {
    static void start() {
#ifdef OUTPUT
        std::outl<terminal>("hbr start"_pgm);
#endif
        crWriterSensorBinary::enable<true>();
    }    
    static void stop() {
    }    
};


int main() {
    using namespace etl;
    using namespace std;
    using namespace AVR;
    
    systemClock::init();
    cppm::init();
    
    rxSelect::dir<Output>();
    paired::dir<Input>();
    
    ppmInPin::dir<Output>(); // debug
    
    crWriterSensorBinary::init();
    crWriterSensorText::init();

    btAndSensorUsart::init<9600>();
    rcUsart::init<115200>();
    
    uint8_t counter = 0;
    const auto t = alarmTimer::create(1000_ms, External::Hal::AlarmFlags::Periodic);

    {
        Scoped<EnableInterrupt<>> ei;
        
        while(true) {
            ppmInPin::toggle();
            components::periodic();
            systemClock::periodic([&](){
                crWriterSensorBinary::rateProcess();
                crWriterSensorText::rateProcess();
                alarmTimer::periodic([&](alarmTimer::index_type timer){
                    if (timer == t) {
                        outl<terminal>("c0: "_pgm, sumd::value(0).toInt());    
                        outl<robo>("l0: "_pgm, counter);    
                        counter++;
                    }
                });
            });
        }    
    }
}

//// BT
//ISR(USART0_RX_vect) {
//    isrRegistrar::isr<AVR::ISR::Usart<0>::RX>();
//}
//ISR(USART0_UDRE_vect){
//    isrRegistrar::isr<AVR::ISR::Usart<0>::UDREmpty>();
//}

//// SumD
//ISR(USART1_RX_vect) {
//    isrRegistrar::isr<AVR::ISR::Usart<1>::RX>();
//}
//ISR(USART1_UDRE_vect){
//    isrRegistrar::isr<AVR::ISR::Usart<1>::UDREmpty>();
//}
