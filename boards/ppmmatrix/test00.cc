#define NDEBUG

#define MATRIX

template<typename S1, typename S2>
[[noreturn]] void assertFunction([[maybe_unused]] const S1& expr, [[maybe_unused]] const S2& file, [[maybe_unused]] unsigned int line) {
    while(true) {
    }    
}

#include "board.h"

//using isrRegistrar = AVR::IsrRegistrar<rcUsart::RxHandler, rcUsart::TxHandler, btUsart::RxHandler, btUsart::TxHandler>;
//using isrRegistrar = AVR::IsrRegistrar<btUsart::RxHandler, btUsart::TxHandler>;

//using terminal = etl::basic_ostream<rcUsart>;
using terminal = etl::basic_ostream<btUsart>;
//using robo = etl::basic_ostream<btUsart>;

template<typename V, auto Size = 8, typename MCU = DefaultMcuType>
struct Matrix final {
    struct Linear {
        etl::uint_NaN<uint8_t> in = 0;
        etl::uint_NaN<uint8_t> out = 0;
        inline void update() {
            
        }
    };
    struct Sum {
        etl::uint_NaN<uint8_t> in = 0;
        etl::uint_NaN<uint8_t> out = 0;
        inline void update() {
            
        }
        
    };
    struct Mul {
        etl::uint_NaN<uint8_t> in = 0;
        etl::uint_NaN<uint8_t> out = 0;
        inline void update() {
            
        }
        
    };
    struct Step {
        etl::uint_NaN<uint8_t> in = 0;
        etl::uint_NaN<uint8_t> out = 0;
        inline void update() {
            
        }
        
    };
    struct SHold {
        etl::uint_NaN<uint8_t> in = 0;
        etl::uint_NaN<uint8_t> out = 0;
        inline void update() {
            
        }
        
    };

private:
    Matrix() = delete;
    inline static std::array<Linear, 8> linears;
    inline static std::array<Sum, 8> summers;
    inline static std::array<Mul, 8> muls;
    inline static std::array<Step, 8> steps;
    inline static std::array<SHold, 8> holds;
    inline static std::array<typename sumd::value_type, 64> values;
};

template<typename MCU = DefaultMcuType>
struct CommFSM {
    inline static void periodic() {
        if (paired::isHigh()) {
            if (!btInitialized) {
                rxSelect::off();
                btUsart::init<AVR::BaudRate<9600>>();
                btInitialized = true;
                sensorInitialized = false;
            }
            else {
                btUsart::periodic();
            }
        }
        else {
            if (!sensorInitialized) {
                rxSelect::on();
                sensorUsart::init<AVR::BaudRate<19200>>();
                sensorInitialized = true;
                btInitialized = false;
            }
            else {
                sensorUsart::periodic();
            }
        }
    }
private:
    inline static bool btInitialized = false;
    inline static bool sensorInitialized = false;
};

using commFsm = CommFSM<>;

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
//    inline static uint16_t test1;
//    inline static uint16_t test2;
//    inline static uint16_t test3;
//    inline static uint16_t test4;
    
private:
    inline static constexpr void update_00() {
        auto v = sumd::value(1);
        if (v) {
            int16_t x = v.toInt();
            x += 10 * (roboremoPA::propValues[0] - 128);
            constexpr uint16_t m = (decltype(v)::Upper - decltype(v)::Lower) / 2 + decltype(v)::Lower;
            
            int32_t nv = ((int32_t)x - m) * roboremoPA::propValues[1];
            nv /= 256;
            nv += m;
            v = nv;
            cppm::ppm(0, v);
        }
    }        
    inline static constexpr void update_01() {
        auto v = sumd::value(1).invert();
        if (v) {
            int16_t x = v.toInt();
            x += 10 * (roboremoPA::propValues[2] - 128);
            constexpr uint16_t m = (decltype(v)::Upper - decltype(v)::Lower) / 2 + decltype(v)::Lower;
            
            int32_t nv = ((int32_t)x - m) * roboremoPA::propValues[3];
            nv /= 256;
            nv += m;
            v = nv;
            cppm::ppm(1, v);
        }
    }        
    inline static constexpr void update_02() {
        cppm::ppm(2, sumd::value(1));
    }        
    inline static constexpr void update_03() {
        cppm::ppm(3, sumd::value(1).invert());
    }        
    inline static constexpr void update_04() {
        cppm::ppm(4, sumd::value(1));
    }        
    inline static constexpr void update_05() {
        cppm::ppm(5, sumd::value(1).invert());
    }        
    inline static constexpr void update_06() {
        cppm::ppm(6, sumd::value(1));
    }        
    inline static constexpr void update_07() {
        cppm::ppm(7, sumd::value(1).invert());
    }        
    inline static constexpr void update_08() {
    }        
    inline static etl::uint_ranged_circular<uint8_t, 0, 8> state;

    FSM() = delete;
};


using fsm = FSM<>;

using components = AVR::Components<rcUsart, cppm, fsm, commFsm>;

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

    rcUsart::init<AVR::BaudRate<115200>>();
    
    const auto t = alarmTimer::create(1000_ms, External::Hal::AlarmFlags::Periodic);

    {
        Scoped<EnableInterrupt<>> ei;
        
        while(true) {
//            ppmInPin::toggle();
            components::periodic();
            systemClock::periodic([&](){
                crWriterSensorBinary::rateProcess();
                crWriterSensorText::rateProcess();
                alarmTimer::periodic([&](alarmTimer::index_type timer){
                    if (timer == t) {
                        outl<terminal>("c1: "_pgm, sumd::value(1).toInt());    
                        outl<terminal>("p00: "_pgm, roboremoPA::propValues[0]);    
                        outl<terminal>("p01: "_pgm, roboremoPA::propValues[1]);    
                        outl<terminal>("p02: "_pgm, roboremoPA::propValues[2]);    
                        outl<terminal>("p03: "_pgm, roboremoPA::propValues[3]);    
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
