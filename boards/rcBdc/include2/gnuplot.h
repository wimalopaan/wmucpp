#pragma once

#include <meta.h>

#include <algorithm>
#include <array>
#include <cstdint>

// echo -e "\$data0 << EOD\n1\n2\n3\nEOD\n\$data1 << EOD\n2\n1\n0\nEOD\nplot \$data0 w lines, \$data1 w circles" | gnuplot -persist

namespace Graphics {

    template<typename Device, typename SetList> struct Gnuplot;

    template<typename Device, typename... Sets>
    struct Gnuplot<Device, Meta::List<Sets...>> {
        using dev = Device;

        enum class State : uint8_t {Init, Title, StartSet, EndSet, PlotCmd};

        static inline void periodic() {
            if (!dev::isTxQueueEmpty()) return;

            const State oldState = state;
            switch(state) {
            case State::Init:
                state = State::Title;
                break;
            case State::Title:
                IO::outl<dev>("set title 'ESC'");
                state = State::StartSet;
                break;
            case State::StartSet:
                Meta::visitAt<Meta::List<Sets...>>(setNumber, []<typename Set>(const Meta::Wrapper<Set>){
                                                       if (indexInSet >= Set::data.size()) {
                                                           state = State::EndSet;
                                                       }
                                                       else {
                                                           const auto v = Set::data[indexInSet];
                                                           using DT = std::remove_cv_t<decltype(v)>;
                                                           if constexpr(std::is_same_v<DT, float>) {
                                                               IO::outl<dev>((uint32_t)v);
                                                           }
                                                           else if constexpr(std::is_same_v<DT, uint16_t>) {
                                                               IO::outl<dev>(v);
                                                           }
                                                           else if constexpr(requires{v.first; v.second;}){
                                                               IO::outl<dev>(v.first, ' ', (uint32_t)v.second);
                                                           }
                                                           else {
                                                               static_assert(false, "wrong value type");
                                                           }
                                                           indexInSet++;
                                                       }
                                                   });
                break;
            case State::EndSet:
                indexInSet = 0;
                ++setNumber;
                if (setNumber == sizeof...(Sets)) {
                    state = State::PlotCmd;
                }
                else {
                    state = State::StartSet;
                }
                break;
            case State::PlotCmd:
                setNumber = 0;
                state = State::StartSet;
                break;
            }
            if (oldState != state){
                switch(state) {
                case State::Init:
                    break;
                case State::Title:
                    break;
                case State::StartSet:
                    IO::outl<dev>("$data", setNumber, " << EOD");
                    break;
                case State::EndSet:
                    IO::outl<dev>("EOD");
                    break;
                case State::PlotCmd:
                    // IO::outl<dev>("plot $data0 w lines");
                    IO::outl<dev>(plotCmd);
                    break;
                }
            }
        }
        private:
        static inline const std::array styles{"lines", "circles"};
        static inline const std::array colors{"'blue'", "'red'", "'green'", "'black'", "'cyan'", "'violet'", "'orange'", "'yellow'"};
        static inline State state = State::Init;
        static inline uint8_t setNumber = 0;
        static inline uint16_t indexInSet = 0;
        static_assert(sizeof...(Sets) < 10);
        static inline const auto plotCmd = []{
            std::array<char, 2> ds{};
            std::array<char, 20 * 10 + 5> cc{};
            strcpy(&cc[0], "plot ");
            for(uint8_t i = 0; i < sizeof...(Sets); ++i) {
                strcat(&cc[0], "$data");
                ds[0] = '0' + i;
                strcat(&cc[0], &ds[0]);
                strcat(&cc[0], " w lines lc ");
                strcat(&cc[0], colors[i % colors.size()]);
                Meta::visitAt<Meta::List<Sets...>>(i, [&]<typename Set>(const Meta::Wrapper<Set>){
                        if constexpr(requires{Set::title;}) {
                                                           strcat(&cc[0], " t \"");
                                                           strcat(&cc[0], Set::title);
                                                           strcat(&cc[0], "\"");
                                                       }
                                                   });
                if (i != (sizeof...(Sets) - 1)) {
                    strcat(&cc[0], ", ");
                }
            }
            return cc;
        }();
    };
}
