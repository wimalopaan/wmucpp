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
    
    template<typename UseInts = AVR::UseInterrupts<false>>
    struct GSV {
        inline static constexpr bool useISR = UseInts::value;
        inline static constexpr auto prefix = "GSV"_pgm;
        inline static void process(const std::byte b, const uint8_t index, const uint8_t field) {
            if (field == SatelliteNumberFieldNumber) {
                if (index < std::size(sNumber)) {
                    if (index == 0) {
                        mReceivedPackages = mReceivedPackages + 1;
                        copy(sNumber, raw);
                        fill(raw, std::byte{'\0'});
                    }
                    raw[index] = b;
                }
            }
        }  
        inline static uint16_t receivedPackages() {
            if constexpr(useISR) {
                Scoped<DisbaleInterrupt<RestoreState>> di;
            }
            return mReceivedPackages;
        }
        inline static void numberRaw(StringBuffer<Sentence::DecimalMaxWidth>& result) {
            if constexpr(useISR) {
                Scoped<DisbaleInterrupt<RestoreState>> di;
            }
            etl::copy(result, sNumber);
        }
    private:
        using count_t = std::conditional_t<useISR, volatile uint16_t, uint16_t>;
        inline static count_t mReceivedPackages{0};
        inline static constexpr uint8_t SatelliteNumberFieldNumber = 2;
        
        using arr_t = std::conditional_t<useISR, volatile std::array<std::byte, Sentence::IntegerMaxWidth>, std::array<std::byte, Sentence::IntegerMaxWidth>>;
        inline static arr_t raw;
        inline static arr_t sNumber;
    };
    
    template<typename UseInts = AVR::UseInterrupts<false>>
    struct VTG {
        inline static constexpr bool useISR = UseInts::value;
        inline static constexpr auto prefix = "VTG"_pgm;
        inline static void process(const std::byte b, const uint8_t index, const uint8_t field) {
            if (field == SpeedFieldNumber) {
                if (index < std::size(speed)) {
                    if (index == 0) {
                        mReceivedPackages = mReceivedPackages + 1;
                        copy(speed, raw);
                        fill(raw, std::byte{'\0'});
                    }
                    raw[index] = b;
                }
            }
        }  
        inline static uint16_t receivedPackages() {
            if constexpr(useISR) {
                Scoped<DisbaleInterrupt<RestoreState>> di;
            }
            return mReceivedPackages;
        }
        inline static void speedRaw(StringBuffer<Sentence::DecimalMaxWidth>& result) {
            if constexpr(useISR) {
                Scoped<DisbaleInterrupt<RestoreState>> di;
            }
            etl::copy(result, speed);
        }
    private:
        using count_t = std::conditional_t<useISR, volatile uint16_t, uint16_t>;
        inline static count_t mReceivedPackages{0};
        inline static constexpr uint8_t SpeedFieldNumber = 6;
        
        using arr_t = std::conditional_t<useISR, volatile std::array<std::byte, Sentence::DecimalMaxWidth>, std::array<std::byte, Sentence::DecimalMaxWidth>>;
        inline static arr_t raw;
        inline static arr_t speed;
    };

    template<typename UseInts = AVR::UseInterrupts<false>>
    struct RMC {
        inline static constexpr bool useISR = UseInts::value;
        inline static constexpr auto prefix = "RMC"_pgm;
        inline static void process(const std::byte b, const uint8_t index, const uint8_t field) {
            if (field == TimeFieldNumber) {
                if (index < std::size(time)) {
                    if (index == 0) {
                        mReceivedPackages = mReceivedPackages + 1; 
                        copy(time, rawtime);
                        fill(rawtime, std::byte{'\0'});
                    }
                    rawtime[index] = b;
                }
            }
            else if (field == DateFieldNumber) {
                if (index < std::size(date)) {
                    if (index == 0) {
                        mReceivedPackages = mReceivedPackages + 1;
                        copy(date, rawdate);
                        fill(rawdate, std::byte{'\0'});
                    }
                    rawdate[index] = b;
                }
            } 
        }  
        inline static uint16_t receivedPackages() {
            if constexpr(useISR) {
                Scoped<DisbaleInterrupt<RestoreState>> di;
            }
            return mReceivedPackages;
        }
        inline static void timeRaw(StringBuffer<Sentence::TimeMaxWidth>& result) {
            if constexpr(useISR) {
                Scoped<DisbaleInterrupt<RestoreState>> di;
            }
            copy(result, time);
        }
        inline static void dateRaw(StringBuffer<Sentence::DateMaxWidth>& result) {
            if constexpr(useISR) {
                Scoped<DisbaleInterrupt<RestoreState>> di;
            }
            copy(result, date);
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

    template<uint8_t N, typename DecoderList, typename UseIsr = AVR::UseInterrupts<false>>
    struct GpsProtocollAdapter;
    
    template<uint8_t N, typename... SentenceDecoder, bool useISR>
    struct GpsProtocollAdapter<N, Meta::List<SentenceDecoder...>, AVR::UseInterrupts<useISR>> {
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
                        sentenceType[index++] = etl::Char(b);
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
    private:
        using count_t = std::conditional_t<useISR, volatile uint16_t, uint16_t>;
        inline static count_t mReceivedBytes{0};
        inline static State state{State::Undefined};
        inline static uint8_t index{0};
        inline static uint8_t field{0};
        inline static uint8_t decoder{std::numeric_limits<uint8_t>::max()};
        inline static std::array<etl::Char, Sentence::SentenceTypeMaxWidth> sentenceType;
    };
}
