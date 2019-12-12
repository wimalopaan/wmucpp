#include <mcu/avr.h>
#include <mcu/pgm/pgmstring.h>
#include <mcu/pgm/pgmarray.h>
#include <mcu/internals/pwm.h>
#include <mcu/internals/portmux.h>

#include <external/units/music.h>

#include <std/chrono>
#include <etl/output.h>
#include <etl/vector.h>
//#include <etl/bitfield.h>


using namespace AVR;

using tcaPosition = Portmux::Position<Component::Tca<0>, Portmux::AltA>;
using tonePwm = PWM::DynamicPwm<tcaPosition>;

namespace External::Music2 {
    enum class Letter : uint8_t {c = 1, d = 3, e = 5, f = 6, g = 8, a = 10, h = 12, pause, _number};
    
    enum class Accidential : int8_t {flat = -1, natural = 0, sharp = 1};
    enum class Octave : uint8_t {i = 0, ii, iii, iiii,  _number};
    
    enum class Base : uint8_t {whole = 0, half = 3, quarter = 6, eigth = 9, doublenote = 12, sixteenth = 13, thirteenth = 14};
    enum class Augment : uint8_t {normal = 0, dot, dotdot, _number};
    
    struct BasePitch {
        inline constexpr BasePitch(Letter l = Letter::c, Accidential a = Accidential::natural) : letter{l}, accidential{a} {}    
        Letter letter;
        Accidential accidential;
    };
    
    struct Pitch {
        inline constexpr Pitch(Letter l, Octave o = Octave::ii, Accidential a = Accidential::natural) : basePitch{l, a}, octave{o} {}  
        const BasePitch basePitch;
        const Octave octave;
    };
    
    struct Length {
        inline constexpr Length(Base b, Augment a = Augment::normal) : base{b}, augment{a} {}
        const Base base{Base::quarter};
        const Augment augment{Augment::normal};
    };

    struct Note {
        inline constexpr Note(Pitch p, Length l) : pitch{p}, length{l} {}
        const Pitch pitch;
        const Length length;
    };

    
    enum class Key : uint8_t {C = 0, G, D, A, E, H, Fs, F, B, Eb, Ab, Db, _number};

    constexpr auto keys = []{
        std::array<std::array<BasePitch, 7>, (uint8_t)Key::_number> k;
        k[(uint8_t)Key::C] = std::array<BasePitch, 7>{Letter::c, Letter::d, Letter::e, Letter::f, Letter::g, Letter::a, Letter::h};
        k[(uint8_t)Key::G] = std::array<BasePitch, 7>{Letter::g, Letter::a, Letter::h, Letter::c, Letter::d, Letter::e, BasePitch(Letter::f, Accidential::sharp)};
        k[(uint8_t)Key::D] = std::array<BasePitch, 7>{Letter::d, Letter::e, BasePitch(Letter::f, Accidential::sharp), Letter::g, Letter::a, Letter::h, BasePitch(Letter::c, Accidential::sharp)};
        return k;
    }();
    
    namespace Representation {
        struct Tone final {
            inline static constexpr uint8_t discriminator_bits{1};
            inline static constexpr uint8_t scaletone_bits{3};
            inline static constexpr uint8_t length_bits{4};
            
            struct ScaleTone;
            struct Length;
            struct Discriminator;
            
            inline static constexpr std::pair<Tone, bool> noteToScaleTone(const Note& note, Key key) {
                const auto& s = keys[uint8_t(key)];
                for(decltype(s.size()) i{0}; i < s.size(); ++i) {
                    if (s[i].letter == note.pitch.basePitch.letter) {
                        if (note.pitch.basePitch.accidential == Accidential::sharp) {
                            return std::pair{Tone{i, noteToLength(note)}, true};                            
                        }
                        if (note.pitch.basePitch.accidential == Accidential::flat) {
                            if (i > 0) {
                                return std::pair{Tone{uint8_t(i - 1), noteToLength(note)}, true};                            
                            }
                        }
                        return std::pair{Tone{i, noteToLength(note)}, false};
                    }
                }
                assert(false);
//                return std::pair{Tone{0, 0}, false};
            }
            inline static constexpr uint8_t noteToLength(const Note& note) {
                return uint8_t(note.length.base) + uint8_t(note.length.augment);
            }
        private:
            constexpr Tone(uint8_t s, uint8_t l) : scaleTone{s}, length{l}{}
            uint8_t discriminator : discriminator_bits = 0;
            uint8_t scaleTone : scaletone_bits = 0;
            uint8_t length : length_bits = 0;
        };
        
        struct Control final {
            inline static constexpr uint8_t discriminator_bits{1};
            inline static constexpr uint8_t octave_bits{2};
            inline static constexpr uint8_t sharp_bits{1};
            inline static constexpr uint8_t legato_bits{1};
            
            struct Legato;
            struct Discriminator;
            
            inline constexpr Control(Octave o, bool s) :
                octave{uint8_t(o)}, mSharp{uint8_t(s)}    
            {} 
            inline void constexpr set(Octave o) {
                octave = uint8_t(o);
            }
            inline void constexpr sharp(bool s) {
                mSharp = s;
            }
        private:
            uint8_t discriminator : discriminator_bits = 1;
            uint8_t octave : octave_bits = 0;
            uint8_t mSharp: sharp_bits = 0;
            uint8_t legato : legato_bits = 0;
        };

        union Storage {
            constexpr Storage() : b{0}{}
            Tone t;
            Control c;
            std::byte b;
        };
        
        using representation_list = Meta::List<Tone, Control>;
        template<typename T> struct get_size {using type = std::integral_constant<uint8_t, sizeof(T)>;};
        using rep_size_list = Meta::transform_type<get_size, representation_list>;
        using byte_size_type = std::integral_constant<uint8_t, sizeof(std::byte)>;
        static_assert(Meta::all_same_v<byte_size_type, rep_size_list>);
        
        static_assert([]{
//            Tone t{Note{Letter::c, Length{Base::doublenote}}, Key::C};
            // tbd
            return true;
        }());
    }

    template<Key k>
    struct Converter {
        template<auto N>
        inline static constexpr auto compress(const auto& notes) {
            std::array<Representation::Storage, N> a;
            for(auto it = std::begin(a); const auto& b : notes) {
                *it++ = b;
            }
            return a;
        }
        inline static constexpr auto makeRepresentation(const auto& notes) {
            etl::Vector<Representation::Storage, 2 * notes.size()> r; // worst case
            
            Representation::Control control{notes[0].pitch.octave, false};
            Representation::Storage s;
            s.c = control;
            r.push_back(s);

            bool needControl = false;
            for(Octave o = notes[0].pitch.octave; const auto& note : notes) {
                auto [nr, sharp] = Representation::Tone::noteToScaleTone(note, k); 
                if (o != note.pitch.octave) {
                    o = note.pitch.octave;
                    control.set(o);
                    needControl = true;
                }
                if (sharp) {
                    needControl = true;
                    control.sharp(true);
                }
                if (needControl) {
                    s.c = control;
                    r.push_back(s);
                }
                s.t = nr;
                r.push_back(s);
                needControl = false;
            }
            return r;
        }
    };    
}


namespace {
    using namespace External::Music2;
    
    using note = Note;
    
    constexpr note c_ii_4 {Pitch{Letter::c}, Length{Base::quarter}};
    constexpr note c_s_ii_4 {Pitch{Letter::c, Octave::ii, Accidential::sharp}, Length{Base::quarter}};
    constexpr note d_ii_4 {Pitch{Letter::d}, Length{Base::quarter}};
    constexpr note d_s_ii_4 {Pitch{Letter::d, Octave::ii, Accidential::sharp}, Length{Base::quarter}};
    constexpr note e_ii_4 {Pitch{Letter::e}, Length{Base::quarter}};
    constexpr note f_ii_4 {Pitch{Letter::f}, Length{Base::quarter}};
    
    constexpr note c_i_4 {Pitch{Letter::c, Octave::i}, Length{Base::quarter}};
    
    template<typename E, typename... Tail>
    constexpr auto make_array1(E& f, Tail&... tail) {
        return std::array<E, sizeof...(Tail) + 1>{E(f), E(tail)...};
    }
    constexpr auto melody = make_array1(c_ii_4, d_ii_4, e_ii_4, f_ii_4, c_s_ii_4);
//    constexpr auto melody = make_array1(c_ii_4, c_i_4, c_s_ii_4, d_ii_4, d_s_ii_4, d_s_ii_4, c_ii_4, c_s_ii_4, d_ii_4, d_s_ii_4, d_s_ii_4, c_ii_4, c_s_ii_4, d_ii_4, d_s_ii_4, d_s_ii_4, c_ii_4, c_s_ii_4, d_ii_4, d_s_ii_4, d_s_ii_4);
    
    constexpr auto melodyRep = Converter<Key::C>::makeRepresentation(melody);
    constexpr auto melodyRepCompress  = Converter<Key::C>::compress<melodyRep.size()>(melodyRep);
    
}

volatile uint16_t r1;
volatile uint16_t r2;

inline static void foo(const auto& m) {
    while(true) {
        for(const auto& n : m) {
            r1 = (uint16_t)n.b;
        }
    }
}

int main() {
    foo(melodyRepCompress);
}
