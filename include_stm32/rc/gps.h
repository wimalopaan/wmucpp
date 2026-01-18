/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2026 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <array>
#include <algorithm>

#include <etl/algorithm.h>
// #include <etl/stringbuffer.h>

template<auto... II, typename A, typename B>
inline static constexpr bool compareElements(const A& a, const B& b) {
    static_assert(((II < std::size(a)) && ...));
    static_assert(((II < b.size()) && ...));
    return ((a[II] == b[II]) && ...);
}

namespace detail {
    template<typename A, typename B, auto... II>
    inline static constexpr bool compareFirstN(const A& a, const B& b, std::index_sequence<II...>) {
        return compareElements<II...>(a, b);
    }
}
template<size_t N, typename A, typename B>
inline static constexpr bool compareFirstN(const A& a, const B& b) {
    return detail::compareFirstN(a, b, std::make_index_sequence<N>{});
}



// sentence: $GPSSS,....*CC (SSS: sentencetyp, CC: checksum: exor über alles zwischen $ und *)

// VTG: $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48
// terme:       0    1  2    3  4    5  6    7
// #6: speed in km/h

// RMC: $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
// terme:        0    1    2     3    4      5   6    7      8      9  10
// #0: time
// #8: date

// GSV: $GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00*74
// terme:      0 1  2  3  3   5  6  7  8   9 10 11 12 13  14 15 16 18  18

// Formate:
// latitude / longitude: DDDMM.MMMM (D = degree, M = minute) (DDD kann fehlen, MM.MM ist mindestens da)
// decimal: dddd.ddd (.ddd muss nicht da sein)
// time: hhmmss.cc (.cc muss nicht da sein)
// date: ddmmyy

namespace External::GPS {
    using namespace etl;
    using namespace std;

    struct Sentence {
        inline static constexpr uint8_t MsgLength = 80;
        inline static constexpr std::byte StartSymbol{'$'};
        inline static constexpr std::byte EndSymbol{'*'};
        inline static constexpr std::byte FieldSep{','};
        inline static constexpr std::byte CR{'\r'};
        inline static constexpr std::byte NL{'\n'};
        inline static constexpr uint8_t DegreeMaxWidth = 10;
        inline static constexpr uint8_t DecimalMaxWidth = 7;
        inline static constexpr uint8_t IntegerMaxWidth = 3;
        inline static constexpr uint8_t TimeMaxWidth = 9;
        inline static constexpr uint8_t DateMaxWidth = 6;
        inline static constexpr uint8_t SentenceTypeMaxWidth = 3;
        inline static constexpr std::byte GpsChar1 = std::byte{'G'};
//        inline static constexpr std::byte GpsChar2 = std::byte{'N'}; // verschiedene provider moeglich
        //        inline static constexpr std::byte GpsChar2 = std::byte{'P'};
    };

    template<typename UseInts = Mcu::UseInterrupts<false>>
    struct GSV {
        inline static constexpr bool useISR = UseInts::value;
        inline static constexpr char prefix[] = "GSV";
        inline static void process(const std::byte b, const uint8_t index, const uint8_t field) {
            ++receivedBytes;
            if (field == SatelliteNumberFieldNumber) {
                if (index < std::size(sNumber)) {
                    if (index == 0) {
                        mReceivedPackages = mReceivedPackages + 1;
                        copy(raw, sNumber);
                        std::fill(std::begin(raw), std::end(raw), char{'\0'});
                        // std::fill(std::begin(raw), std::end(raw), char{' '});
                        // fill(raw, std::byte{'\0'});
                    }
                    raw[index] = (char)b;
                }
            }
        }
        inline static uint16_t receivedPackages() {
            if constexpr(useISR) {
                // Scoped<DisbaleInterrupt<RestoreState>> di;
            }
            return mReceivedPackages;
        }
        inline static void numberRaw(std::array<char, Sentence::DecimalMaxWidth>& result) {
            if constexpr(useISR) {
                // Scoped<DisbaleInterrupt<RestoreState>> di;
            }
            etl::copy(sNumber, result);
        }
    // private:
        using count_t = std::conditional_t<useISR, volatile uint16_t, uint16_t>;
        inline static count_t mReceivedPackages{0};
        inline static constexpr uint8_t SatelliteNumberFieldNumber = 2;

        using arr_t = std::conditional_t<useISR, volatile std::array<char, Sentence::IntegerMaxWidth>, std::array<char, Sentence::IntegerMaxWidth>>;
        inline static arr_t raw;
        inline static arr_t sNumber;
        inline static uint16_t receivedBytes = 0;
    };

    template<typename UseInts = Mcu::UseInterrupts<false>>
    struct VTG {
        inline static constexpr bool useISR = UseInts::value;
        inline static constexpr char prefix[] = "VTG";
        inline static void process(const std::byte b, const uint8_t index, const uint8_t field) {
            if (field == SpeedFieldNumber) {
                if (index < std::size(speed)) {
                    if (index == 0) {
                        mReceivedPackages = mReceivedPackages + 1;
                        copy(raw, speed);
                        // std::fill(std::begin(raw), std::end(raw), char{'0'});
                        std::fill(std::begin(raw), std::end(raw), char{'\0'});
                        // fill(raw, std::byte{'\0'});
                    }
                    raw[index] = (char)b;
                }
            }
        }
        inline static uint16_t receivedPackages() {
            if constexpr(useISR) {
                // Scoped<DisbaleInterrupt<RestoreState>> di;
            }
            return mReceivedPackages;
        }
        inline static void speedRaw(std::array<char, Sentence::DecimalMaxWidth>& result) {
            if constexpr(useISR) {
                // Scoped<DisbaleInterrupt<RestoreState>> di;
            }
            etl::copy(speed, result);
        }
    private:
        using count_t = std::conditional_t<useISR, volatile uint16_t, uint16_t>;
        inline static count_t mReceivedPackages{0};
        inline static constexpr uint8_t SpeedFieldNumber = 6;

        using arr_t = std::conditional_t<useISR, volatile std::array<char, Sentence::DecimalMaxWidth>, std::array<char, Sentence::DecimalMaxWidth>>;
        inline static arr_t raw;
        inline static arr_t speed;
    };

    template<typename UseInts = Mcu::UseInterrupts<false>>
    struct RMC {
        inline static constexpr bool useISR = UseInts::value;
        inline static constexpr char prefix[] = "RMC";
        inline static void process(const std::byte b, const uint8_t index, const uint8_t field) {
            if (field == TimeFieldNumber) {
                if (index < std::size(time)) {
                    if (index == 0) {
                        mReceivedPackages = mReceivedPackages + 1;
                        copy(rawtime, time);
                        std::fill(std::begin(rawtime), std::end(rawtime), std::byte{' '});
                        // std::fill(std::begin(rawtime), std::end(rawtime), std::byte{'\0'});
                        // fill(rawtime, std::byte{'\0'});
                    }
                    rawtime[index] = b;
                }
            }
            else if (field == DateFieldNumber) {
                if (index < std::size(date)) {
                    if (index == 0) {
                        mReceivedPackages = mReceivedPackages + 1;
                        copy(date, rawdate);
                        std::fill(std::begin(rawdate), std::end(rawdate), std::byte{' '});
                        // std::fill(std::begin(rawdate), std::end(rawdate), std::byte{'\0'});
                        // fill(rawdate, std::byte{'\0'});
                    }
                    rawdate[index] = b;
                }
            }
        }
        inline static uint16_t receivedPackages() {
            if constexpr(useISR) {
                // Scoped<DisbaleInterrupt<RestoreState>> di;
            }
            return mReceivedPackages;
        }
        inline static void timeRaw(std::array<char, Sentence::TimeMaxWidth>& result) {
            if constexpr(useISR) {
                // Scoped<DisbaleInterrupt<RestoreState>> di;
            }
            copy(time, result);
        }
        inline static void dateRaw(std::array<char, Sentence::DateMaxWidth>& result) {
            if constexpr(useISR) {
                // Scoped<DisbaleInterrupt<RestoreState>> di;
            }
            copy(date, result);
        }
    private:
        using count_t = std::conditional_t<useISR, volatile uint16_t, uint16_t>;
        inline static count_t mReceivedPackages{};
        inline static constexpr uint8_t TimeFieldNumber = 0;
        inline static constexpr uint8_t DateFieldNumber = 8;

        using time_array_t = std::conditional_t<useISR, volatile std::array<std::byte, Sentence::TimeMaxWidth>, std::array<std::byte, Sentence::TimeMaxWidth>>;
        inline static time_array_t rawtime{};
        inline static time_array_t time{};
        using date_array_t = std::conditional_t<useISR, volatile std::array<std::byte, Sentence::DateMaxWidth>, std::array<std::byte, Sentence::DateMaxWidth>>;
        inline static date_array_t rawdate{};
        inline static date_array_t date{};
    };

    template<uint8_t N, typename DecoderList, typename UseIsr = Mcu::UseInterrupts<false>>
    struct GpsProtocollAdapter;

    template<uint8_t N, typename... SentenceDecoder, bool useISR>
    struct GpsProtocollAdapter<N, Meta::List<SentenceDecoder...>, Mcu::UseInterrupts<useISR>> {
        enum class State : uint8_t {Undefined, Prefix1, Prefix2, Type, FieldData, Checksum};

        using decoders = Meta::List<SentenceDecoder...>;

        inline static uint16_t receivedBytes() {
            return mReceivedBytes;
        }

        inline static bool process(const std::byte b) {
            mReceivedBytes = mReceivedBytes + 1;
            switch(state) {
            case State::Undefined:
                if (b == Sentence::StartSymbol) {
                    state = State::Prefix1;
                }
                break;
            case State::Prefix1:
                if (b == Sentence::GpsChar1) {
                    state = State::Prefix2;
                }
                else {
                    state = State::Undefined;
                }
                break;
            case State::Prefix2:
                if (((char)b >= 'A') && ((char)b <= 'z')) {
                    state = State::Type;
                    index = 0;
                }
                else {
                    state = State::Undefined;
                }
                break;
            case State::Type:
                ++mDecoder;

                if (b == Sentence::FieldSep) {
                    state = State::FieldData;
                    field = 0;
                    index = 0;
                    decoder = Meta::find<decoders>([]<typename type>(Meta::Wrapper<type>){
                        return compareFirstN<3>(type::prefix, sentenceType);
                    });
                }
                else {
                    if (index < std::size(sentenceType)) {
                        sentenceType[index++] = char(b);
                    }
                }
                break;
            case State::FieldData:
                // todo: neuer State: wenn decoder gültig FieldDataForDecoder
                if (b == Sentence::FieldSep) {
                    ++field;
                    index = 0;
                }
                else if (b == Sentence::EndSymbol) {
                    state = State::Checksum;
                }
                else {
                    if (decoder < Meta::size<decoders>::value) {
                        Meta::visitAt<decoders>(decoder, [&]<typename Decoder>(Meta::Wrapper<Decoder>){
                            Decoder::process(b, index++, field);
                        });
                    }
                }
                break;
            case State::Checksum:
                if (b == Sentence::StartSymbol) {
                    state = State::Prefix1;
                    index = 0;
                }
                break;
            default:
                assert(false);
                break;
            }
            return true;
        }
        inline static void ratePeriodic() {}
    // private:
        using count_t = std::conditional_t<useISR, volatile uint16_t, uint16_t>;
        inline static count_t mReceivedBytes{0};
        inline static count_t mDecoder{0};
        inline static State state{State::Undefined};
        inline static uint8_t index{0};
        inline static uint8_t field{0};
        inline static uint8_t decoder{std::numeric_limits<uint8_t>::max()};
        inline static std::array<char, Sentence::SentenceTypeMaxWidth> sentenceType;
    };
}
