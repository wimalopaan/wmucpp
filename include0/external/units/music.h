#pragma once

#include <std/cmath>

#include "physical.h"

namespace External {
    
    namespace Music {
        using namespace External::Units::literals;
        
        enum class Letter : uint8_t {c = 0, d = 2, e = 4, f = 5, g = 7, a = 9, h = 11, pause};
        enum class Accidential : uint8_t {sharp, flat, natural};
        enum class Octave : uint8_t {i = 1, ii = 2, iii = 4, iiii = 8};
        
        enum class Base : uint8_t {doublenote = 1, whole, half, quarter, eigth, sixteenth, thirteenth};
        enum class Augment : uint8_t {normal = 1, dot, dotdot};
        
        struct Pitch {
            inline constexpr Pitch(Letter l, Octave o = Octave::ii, Accidential a = Accidential::natural) : letter{l}, accidential{a}, octave{o} {}  
            const Letter letter{Letter::a};
            const Accidential accidential{Accidential::natural};
            const Octave octave{Octave::ii};
        };
        
        struct Length {
            inline constexpr Length(Base b, Augment a = Augment::normal) : base{b}, augment{a} {}
            const Base base{Base::quarter};
            const Augment augment{Augment::normal};
        };
        
        template<typename PWM, typename L = std::integral_constant<uint16_t, 40>> struct Note;
        
        template<typename PWM, uint16_t base_ticks>
        struct Note<PWM, std::integral_constant<uint16_t, base_ticks>> {
            inline static constexpr double r12 = pow(2.0, 1.0/12.0);
            
            inline static constexpr External::Units::hertz convert(Pitch p) {
                constexpr auto a_tone = 440_Hz;
                if (p.letter == Letter::pause) {
                    return {std::numeric_limits<External::Units::hertz::value_type>::max()};
                }
                const double diff = int8_t(p.letter) - int8_t(Letter::a);
                const double rel = pow(r12, diff);
                double acci = 1.0;
                if (p.accidential == Accidential::sharp) {
                    acci = r12;
                }
                if (p.accidential == Accidential::flat) {
                    acci = 1.0/ r12;
                }
                const double octave = int8_t(p.octave);
                const uint32_t f(a_tone.value * rel * acci * octave);
                return External::Units::hertz{f};
            }
            inline static constexpr uint16_t convert(Length l) {
                double diff = int8_t(Base::quarter) - int8_t(l.base);
                double rel = pow(2.0, diff);
                double augm = 1.0;
                if (l.augment == Augment::dot) {
                    augm = 1.5;
                }
                if (l.augment == Augment::dotdot) {
                    augm = 1.75;
                }
                return base_ticks * rel * augm;
            }
            inline constexpr Note(Pitch p, Length l) : pitch(PWM::f_timer / convert(p)), length{convert(l)} {}
            explicit inline Note(const AVR::Pgm::Ptr<Note>& note_pgm) : pitch(pgm_read_word(&note_pgm->pitch)), length(pgm_read_word(&note_pgm->length)) {}
            const uint16_t pitch;
            const uint16_t length;
        private:
            inline constexpr Note(uint16_t p, uint16_t l) : pitch(p), length(l) {}
        };
        
        template<typename PWM, typename BT = std::integral_constant<uint16_t, 40>, typename SizeType = uint8_t>
        struct Generator {
            enum class State : uint8_t {Off, SingleNote, Melody, MelodyRepeat};
            
            inline static void init() {
                PWM::init();
            }
            inline static void play(const Note<PWM, BT>& note) {
                set(note);
                state = State::SingleNote;
            }
            inline static void play(const AVR::Pgm::ArrayView<Note<PWM, BT>, SizeType>& m, bool repeat = false) {
                melody = m;
                noteIndex = 0;
                if (melody) {
                    set(melody[0]);
                    if (repeat) {
                        state = State::MelodyRepeat;    
                    }
                    else {
                        state = State::Melody;    
                    }
                }
            }
            inline static bool busy() {
                return (state != State::Off);
            }
            inline static void off() {
                PWM::off();
                state = State::Off;
            }
            inline static void periodic() {
                switch(state) {
                case State::Off:
                    break;
                case State::SingleNote:
                    if (ticksLeft > 0) {
                        --ticksLeft;
                    }
                    else {
                        PWM::off();
                        state = State::Off;
                    }
                    break;
                case State::Melody:
                    if (ticksLeft > 0) {
                        --ticksLeft;
                    }
                    else {
                        ++noteIndex;
                        if (noteIndex < melody.size()) {
                            set(melody[noteIndex]);
                        }
                        else {
                            off();
                        }
                    }
                    break;
                case State::MelodyRepeat:
                    if (ticksLeft > 0) {
                        --ticksLeft;
                    }
                    else {
                        ++noteIndex;
                        if (noteIndex < melody.size()) {
                            set(melody[noteIndex]);
                        }
                        else {
                            set(melody[noteIndex = 0]);
                        }
                    }
                    break;
                }
            }
        private:
            inline static void set(const Note<PWM, BT>& note) {
               if (note.pitch > 0) {
                   PWM::frequency(note.pitch);
                   PWM::duty(note.pitch / 2);
                   PWM::on();
               }
               else {
                   PWM::off();
               }
                ticksLeft = note.length;
            }
            inline static AVR::Pgm::ArrayView<Note<PWM, BT>, SizeType> melody;
            inline static SizeType noteIndex{0};
            
            inline static State state{State::Off};
            inline static uint16_t ticksLeft{0};
        };
    }
}
