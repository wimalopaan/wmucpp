//#include <mcu/avr.h>
//#include <mcu/pgm/pgmstring.h>
//#include <mcu/pgm/pgmarray.h>
//#include <mcu/internals/pwm.h>
//#include <mcu/internals/portmux.h>

//#include <external/units/music.h>

//#include <std/chrono>
//#include <etl/output.h>

//using namespace AVR;

//using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::AltA>;
//using tonePwm = PWM::DynamicPwm<tcaPosition>;

//namespace {
//    using namespace External::Music;
    
//    using note = Note<tonePwm>;
    
//    constexpr note c_ii_4 {Pitch{Letter::c}, Length{Base::quarter}};
//    constexpr note c_s_ii_4 {Pitch{Letter::c, Octave::ii, Accidential::sharp}, Length{Base::quarter}};
//    constexpr note d_ii_4 {Pitch{Letter::d}, Length{Base::quarter}};
//    constexpr note d_s_ii_4 {Pitch{Letter::d, Octave::ii, Accidential::sharp}, Length{Base::quarter}};
    
//    //    std::integral_constant<uint32_t, note::convert(Pitch{Letter::c, Octave::i, Accidential::flat}).value>::_;
//    //    std::integral_constant<uint16_t, note::convert(Length{Base::quarter})>::_;
    
//    constexpr auto melody = AVR::Pgm::Array<note, c_ii_4, c_s_ii_4, d_ii_4, d_s_ii_4, d_s_ii_4, c_ii_4, c_s_ii_4, d_ii_4, d_s_ii_4, d_s_ii_4, c_ii_4, c_s_ii_4, d_ii_4, d_s_ii_4, d_s_ii_4, c_ii_4, c_s_ii_4, d_ii_4, d_s_ii_4, d_s_ii_4>{};
//}


//volatile uint16_t r1;
//volatile uint16_t r2;

//int main() {
//    etl::uint_ranged_circular<uint8_t, 0, melody.size() - 1> index;
    
//    while(true) {
//        const auto n = melody[index.toInt()];
//        r1 = n.pitch;
//        r2 = n.length;
//        ++index;
//    }
    
    
////    while(true) {
////        for(const auto& n : melody) {
////            r1 = n.pitch;
////            r2 = n.length;
////        }
////    }
    
//}
