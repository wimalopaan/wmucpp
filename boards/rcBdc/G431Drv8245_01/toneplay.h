#pragma once

#include <cstdint>
#include <type_traits>
#include <output.h>
#include <mcu/alternate.h>

#include "tick.h"

namespace External {
    using namespace std::literals::chrono_literals;

    template<typename Pwm, typename Timer, typename Storage, typename Debug = void>
    struct TonePlayer {
        using pwm = Pwm;
        using timer = Timer;
        using storage = Storage;

        struct Note {
            std::chrono::milliseconds length;
            uint16_t frequency;
        };

        enum class State : uint8_t {Off, Play};
        enum class Event : uint8_t {None, Off, PlayTune1, PlayTune2, PlayTune3, PlayTune4};

        static inline void event(const Event e) {
            IO::outl<Debug>("# Tone E: ", (uint8_t)e);
            mEvent = e;
        }
        static inline bool isOff() {
            return mState == State::Off;
        }
        static inline void ratePeriodic() {
            const Event e = std::exchange(mEvent, Event::None);
            const auto oldState = mState;
            ++mStateTick;
            switch(mState) {
            case State::Off:
                if (e == Event::PlayTune1) {
                    mState = State::Play;
                    mTuneIndex = 0;
                }
                else if (e == Event::PlayTune2) {
                    mState = State::Play;
                    mTuneIndex = 1;
                }
                else if (e == Event::PlayTune3) {
                    mState = State::Play;
                    mTuneIndex = 2;
                }
                else if (e == Event::PlayTune4) {
                    mState = State::Play;
                    mTuneIndex = 3;
                }
                break;
            case State::Play:
                if (e == Event::Off) {
                    mState = State::Off;
                }
                (++noteTick).on(External::Tick<Timer>{mTunes[mTuneIndex][mNoteIndex].length}, []{
                    if (mNoteIndex < (mTunes[mTuneIndex].size() - 1)) {
                        ++mNoteIndex;
                        nextNote();
                    }
                    else {
                        mState = State::Off;
                    }
                });
                break;
            }
            if (oldState != mState) {
                mStateTick.reset();
                switch(mState) {
                case State::Off:
                    IO::outl<Debug>("# Tone Off");
                    pwm::duty(0);
                    break;
                case State::Play:
                    IO::outl<Debug>("# Tone Play");
                    pwm::setToneMode();
                    pwm::duty(storage::eeprom.volume);
                    mNoteIndex = 0;
                    nextNote();
                    break;
                }
            }
        }
        private:
        static inline void nextNote() {
            const uint16_t f = mTunes[mTuneIndex][mNoteIndex].frequency;
            IO::outl<Debug>("# NextNote: ", f);
            pwm::pwm(f);
            noteTick.reset();
        }
        static inline uint8_t mNoteIndex = 0;
        static inline uint8_t mTuneIndex = 0;
        static inline auto mTunes = []{
            std::array<std::array<Note, 4>, 4> tunes;
            tunes[0] = {Note{200ms, 440}, Note{200ms, 554}, Note{200ms, 659}, Note{200ms, 880}};
            tunes[1] = {Note{200ms, 880}, Note{200ms, 659}, Note{200ms, 523}, Note{200ms, 440}};
            tunes[2] = {Note{100ms, 880}, Note{100ms, 0}, Note{100ms, 932}, Note{100ms, 0}};
            tunes[3] = {Note{400ms, 440}, Note{400ms, 0}, Note{400ms, 440}, Note{400ms, 0}};
            return tunes;
        }();

        static inline Event mEvent{Event::None};
        static inline State mState{State::Off};
        static inline External::Tick<Timer> noteTick;
        static inline External::Tick<Timer> mStateTick;
    };

}
