#pragma once

#include <cstdint>
#include <cstddef>
#include <array>
#include <algorithm>

#include <etl/algorithm.h>
#include <etl/stringbuffer.h>

// sentence: $GPSSS,....*CC (SSS: sentencetyp, CC: checksum: exor über alles zwischen $ und *)

// VTG: $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48
// terme:       0    1  2    3  4    5  6    7
// #6: speed in km/h

// RMC: $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
// terme:        0    1    2     3    4      5   6    7      8      9  10
// #0: time
// #8: date

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
        inline static constexpr uint8_t TimeMaxWidth = 9;
        inline static constexpr uint8_t DateMaxWidth = 6;
        inline static constexpr uint8_t SentenceTypeMaxWidth = 3;
        inline static constexpr std::byte GpsChar1 = std::byte{'G'};
//        inline static constexpr std::byte GpsChar2 = std::byte{'N'}; // verschiedene provider moeglich
    };
    
    struct VTG {
        inline static constexpr const char prefix[] = "VTG";
        inline static void process(std::byte b, uint8_t index, uint8_t field) {
            if (field == SpeedFieldNumber) {
                if (index < std::size(speed)) {
                    speed[index] = b;
                }
            }
        }  
        inline static void speedRaw(StringBuffer<Sentence::DecimalMaxWidth>& result) {
            Scoped<DisbaleInterrupt<RestoreState>> di;
            etl::copy(result, speed);
        }
        inline static constexpr uint8_t SpeedFieldNumber = 6;
        inline static std::array<std::byte, Sentence::DecimalMaxWidth> speed;
    };
    struct RMC {
        inline static constexpr const char prefix[] = "RMC";
        inline static void process(std::byte b, uint8_t index, uint8_t field) {
            if (field == TimeFieldNumber) {
                if (index < std::size(time)) {
                    time[index] = b;
                }
            }
            else if (field == DateFieldNumber) {
                if (index < std::size(date)) {
                    date[index] = b;
                }
            } 
        }  
        inline static void timeRaw(StringBuffer<Sentence::TimeMaxWidth>& result) {
            Scoped<DisbaleInterrupt<RestoreState>> di;
            copy(result, time);
        }
        inline static void dateRaw(StringBuffer<Sentence::DateMaxWidth>& result) {
            Scoped<DisbaleInterrupt<RestoreState>> di;
            copy(result, date);
        }
        inline static constexpr uint8_t TimeFieldNumber = 0;
        inline static constexpr uint8_t DateFieldNumber = 8;
        inline static std::array<std::byte, Sentence::TimeMaxWidth> time;
        inline static std::array<std::byte, Sentence::DateMaxWidth> date;
    };
    template<uint8_t N, typename... SentenceDecoder>
    struct GpsProtocollAdapter {
        enum class State : uint8_t {Undefined, Prefix1, Prefix2, Type, FieldData, Checksum};

        using decoders = Meta::List<SentenceDecoder...>;
        
        inline static bool process(std::byte b) { // from isr only
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
                if (b == Sentence::FieldSep) {
                    state = State::FieldData;                    
                    field = 0;
                    index = 0;
                    decoder = Meta::find<decoders>([](auto x){
                        using type = typename decltype(x)::type;
                        return compareFirstN<3>(type::prefix, sentenceType);
                    });
                }
                else {
                    if (index < std::size(sentenceType)) {
                        sentenceType[index++] = (char)b;
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
                        Meta::visitAt<decoders>(decoder, [&](auto d){
                            using type = typename decltype(d)::type;
                            type::process(b, index++, field);
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
        inline static void periodic() {
        }
        inline static State state = State::Undefined;
        inline static uint8_t index = 0;
        inline static uint8_t field = 0;
        inline static uint8_t decoder = std::numeric_limits<uint8_t>::max();
        inline static std::array<char, Sentence::SentenceTypeMaxWidth> sentenceType;
    };
}
