#pragma once

#include <cstdint>
#include <cstddef>
#include <limits>
#include <chrono>

#include <etl/concepts.h>
#include <etl/ranged.h>
#include <etl/algorithm.h>

#include <mcu/internals/usart.h>

// Ablauf des Starts
// 1) Reset-Nachrichten (mehrere)
// 2) Discovery beginnend mit 0x81 (1. Sensor)
//    Das erste Geräte lernet dadurch seine Id. Erst wenn die Id gelernt worden ist, d.h. auch die Antwortnachricht zurückgesendet ist, werden weiter Bus-Nachrichten weiter geleitet
// 3) Zum Weiterleiten wir einfach ein Bus-Treiber enabled (74-1g-126)
// 4) Dieser wird eingeschaltet, wenn die Adressen gelernt und die Antwort(en) gesendet wurden. 

namespace IBus {
    struct CheckSum final {
        inline void reset() {
            mSum = std::numeric_limits<uint16_t>::max();
        }
        inline std::byte operator+=(const std::byte b) {
            mSum -= static_cast<uint8_t>(b);
            return b;
        }
        inline std::byte highByte() const {
            return std::byte(mSum >> 8);
        }
        inline std::byte lowByte() const {
            return std::byte(mSum);
        }
        inline void highByte(const std::byte hb) {
            mH = hb;
        }
        inline void lowByte(const std::byte lb){
            mL = lb;
        }
        inline explicit operator bool() const {
            return (etl::nth_byte<0>(mSum) == mL) && (etl::nth_byte<1>(mSum) == mH);
        }
    private:
        std::byte mH{};
        std::byte mL{};
        uint16_t mSum = std::numeric_limits<uint16_t>::max();
    };
    using namespace std::literals::chrono;
    namespace Type {
        // https://github.com/betaflight/betaflight/tree/master/src/main/telemetry
        
        // FlySky i6A: TEMPERATURE, RPM_FLYSKY, EXTERNAL_VOLTAGE
        
        enum class type : uint8_t {
            NONE             = 0x00,
            TEMPERATURE      = 0x01, // 0.1°C per step, -40°C offset (400 = 0°C)
            RPM_FLYSKY       = 0x02,
            EXTERNAL_VOLTAGE = 0x03, // 0.01V per step
            CELL             = 0x04, // Avg Cell voltage
            BAT_CURR         = 0x05, // battery current A * 100
            FUEL             = 0x06, // remaining battery percentage / mah drawn otherwise or fuel level no unit!
            RPM              = 0x07, // throttle value / battery capacity
            CMP_HEAD         = 0x08, //Heading  0..360 deg, 0=north 2bytes
            CLIMB_RATE       = 0x09, //2 bytes m/s *100
            COG              = 0x0a, //2 bytes  Course over ground(NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. unknown max uint
            GPS_STATUS       = 0x0b, //2 bytes
            ACC_X            = 0x0c, //2 bytes m/s *100 signed
            ACC_Y            = 0x0d, //2 bytes m/s *100 signed
            ACC_Z            = 0x0e, //2 bytes m/s *100 signed
            ROLL             = 0x0f, //2 bytes deg *100 signed
            PITCH            = 0x10, //2 bytes deg *100 signed
            YAW              = 0x11, //2 bytes deg *100 signed
            VERTICAL_SPEED   = 0x12, //2 bytes m/s *100
            GROUND_SPEED     = 0x13, //2 bytes m/s *100 different unit than build-in sensor
            GPS_DIST         = 0x14, //2 bytes dist from home m unsigned
            ARMED            = 0x15, //2 bytes
            FLIGHT_MODE      = 0x16, //2 bytes
            PRES             = 0x41, // Pressure
            ODO1             = 0x7c, // Odometer1
            ODO2             = 0x7d, // Odometer2
            SPEED              = 0x7e, // Speed 2bytes km/h
            
            GPS_LAT          = 0x80, //4bytes signed WGS84 in degrees * 1E7
            GPS_LON          = 0x81, //4bytes signed WGS84 in degrees * 1E7
            GPS_ALT          = 0x82, //4bytes signed!!! GPS alt m*100
            ALT              = 0x83, //4bytes signed!!! Alt m*100
            ALT_MAX          = 0x84, //4bytes signed MaxAlt m*100
            
            ALT_FLYSKY       = 0xf9, // Altitude 2 bytes signed in m
#if defined(USE_TELEMETRY_IBUS_EXTENDED)
            GPS_FULL         = 0xfd,
            VOLT_FULL        = 0xf0,
            ACC_FULL         = 0xef,
#endif //defined(TELEMETRY_IBUS_EXTENDED)
            UNKNOWN          = 0xff
        };
    }

    namespace Switch {
        struct Protocol1 {
            
#ifdef USE_SBUS
            inline static constexpr auto sbusLut = []{
                std::array<uint8_t, 32> lut{};
                lut[0] = 0;
                lut[1] = 0;
                lut[2] = 1;
                lut[3] = 2;
                lut[4] = 2;
                lut[5] = 3;
                lut[6] = 3;
                lut[7] = 4;
                lut[8] = 4;
                lut[9] = 5;
                lut[10] = 5;
                lut[11] = 6;
                lut[12] = 6;
                lut[13] = 7;
                lut[14] = 7;
                lut[15] = 7;
                lut[16] = 8;
                lut[17] = 9;
                lut[18] = 9;
                lut[19] = 10;
                lut[20] = 10;
                lut[21] = 11;
                lut[22] = 11;
                lut[23] = 12;
                lut[24] = 12;
                lut[25] = 13;
                lut[26] = 13;
                lut[27] = 14;
                lut[28] = 14;
                lut[29] = 15;
                lut[30] = 15;
                lut[31] = 15;
                return lut;
            }();
#endif
            using addr_t  = etl::uint_ranged<uint8_t, 0, 7>; // 3 Bits
            using index_t = etl::uint_ranged<uint8_t, 0, 7>; // 3 Bits
            
            using param_t  = etl::uint_ranged<uint8_t, 0, 15>; // 4 Bits
#ifdef USE_SBUS
            using mode_t  = etl::uint_ranged<uint8_t, 0, 3>; // 2 Bits
            using pvalue_t  = etl::uint_ranged<uint8_t, 0, 15>; // 4 Bits
#else
            using mode_t  = etl::uint_ranged<uint8_t, 0, 7>; // 3 Bits
            using pvalue_t  = etl::uint_ranged<uint8_t, 0, 31>; // 5 Bits
#endif
            
//            inline static constexpr mode_t unused{0};
            inline static constexpr mode_t off{0};
            inline static constexpr mode_t on{1};
            inline static constexpr mode_t blink1{2};
            inline static constexpr mode_t blink2{3};

#ifdef USE_IBUS            
            inline static constexpr mode_t config{7};
#endif
            inline static constexpr param_t reset{0};
            inline static constexpr param_t pwm{1};
            inline static constexpr param_t blink1Intervall{2};
            inline static constexpr param_t blink1Duration{3};
            inline static constexpr param_t blink2Intervall{4};
            inline static constexpr param_t blink2Duration{5};
            inline static constexpr param_t passThruChannel{6};
            inline static constexpr param_t timeMpxMode{7};
            inline static constexpr param_t servoMin{12};
            inline static constexpr param_t servoMax{13};
            inline static constexpr param_t resetOrLearnAddress{14};
            inline static constexpr param_t broadCast{15};
            
            inline static constexpr pvalue_t bCastReset{1};
//            inline static constexpr pvalue_t timeMpxGraupner{2}; // 2 long sync
//            inline static constexpr pvalue_t timeMpxRobbe{3}; // 1 short sync
//            inline static constexpr pvalue_t timeMpxCP{4}; // 1 long sync
//            inline static constexpr pvalue_t timeMpxXXX{5}; // 2 short sync
#ifdef USE_SBUS
            inline static constexpr pvalue_t bCastOff{15};
#else
            inline static constexpr pvalue_t bCastOff{31};
#endif
            
            template<typename ValueType>
            inline static constexpr bool isLearnCode(const ValueType& v) {
                if (isControlMessage(v)) {
                    const auto p = toParameter(v);
                    if (p == Protocol1::resetOrLearnAddress) {
                        return true;
                    }
                }
                return false;
            }
            
            template<typename ValueType>
            inline static constexpr addr_t toAddress(const ValueType& v) {
                const uint16_t c = v.toInt() - ValueType::Lower;
//                return addr_t((c & (0x03 << 7)) >> 7);
                return addr_t((c & (0x07 << 6)) >> 6);
            }   
            template<typename ValueType>
            inline static constexpr index_t toIndex(const ValueType& v) {
                const uint16_t c = v.toInt() - ValueType::Lower;
//                return index_t((c & (0x07 << 4)) >> 4);
                return index_t((c & (0x07 << 3)) >> 3);
            }   
            template<typename ValueType>
            inline static constexpr mode_t toMode(const ValueType& v) {
                const uint16_t c = v.toInt() - ValueType::Lower;
//                return mode_t((c & 0x0f));
#ifdef USE_SBUS
                return mode_t(sbusLut[(c & 0x07)]);
#else
                return mode_t((c & 0x07));
#endif
            }   
            template<typename ValueType>
            inline static constexpr bool isControlMessage(const ValueType& v) {
                const uint16_t c = v.toInt() - ValueType::Lower;
                return (c & (0x01 << 9));
            }
            template<typename ValueType>
            inline static constexpr param_t toParameter(const ValueType& v) {
                const uint16_t c = v.toInt() - ValueType::Lower;
                return param_t((c & (0x0f << 5)) >> 5);
            }   
            template<typename ValueType>
            inline static constexpr pvalue_t toParameterValue(const ValueType& v) {
                uint16_t c = v.toInt() - ValueType::Lower;
#ifdef USE_SBUS
                return pvalue_t(sbusLut[(c & 0x1f)]);
#else
                return pvalue_t(c & 0x1f);
#endif
            }   
        };
        
        template<typename PA, typename ActorList, typename NVM = void>
        struct MultiAdapter;
        
        template<typename PA, typename... Actors, typename NVM>
        struct MultiAdapter<PA, Meta::List<Actors...>, NVM> {
            using channel_t = PA::channel_t;
            using value_t = PA::value_type;

            using protocol_t = Protocol1;
            
            using addr_t = Protocol1::addr_t;
            using index_t = Protocol1::index_t;
            using mode_t = Protocol1::mode_t;
            
            using param_t = Protocol1::param_t;
            using pvalue_t = Protocol1::pvalue_t;
            
            inline static constexpr uint8_t nActors = sizeof...(Actors);
            
            template<typename T>
            inline static constexpr bool isLearnCode(const T& v) {
                return Protocol1::isLearnCode(v);
            }
            inline static void init() {
            }
            static inline void channel(const channel_t c) {
                if (c) {
                    mChannel = c;
                }
            }

            static inline void address(const addr_t a) {
                mAddr = a;
            }
            inline static bool periodic() {
                const auto cv = PA::valueMapped(mChannel);
                const addr_t addr = Protocol1::toAddress(cv);
                const index_t index = Protocol1::toIndex(cv);
                const mode_t mode = Protocol1::toMode(cv);
                                
                if (Protocol1::isControlMessage(cv)) { // control (parameters)
                    const param_t param = Protocol1::toParameter(cv);
                    const pvalue_t value = Protocol1::toParameterValue(cv);

                    if (param == Protocol1::broadCast) {
                        if (value == Protocol1::bCastOff) {
                            lastOnIndex = lastindex_t{};
                            lastOnAddr = lastaddr_t{};
                            off(Meta::List<Actors...>{});                    
                        }
                    }
                    else if (lastOnIndex && lastOnAddr) {
                        index_t lastOn{lastOnIndex.toInt()};
                        if ((lastOnAddr.toInt() - mAddr) < nActors) {
                            etl::uint_ranged<uint8_t, 0, (nActors - 1)> lastAddrOffset{(uint8_t)(lastOnAddr.toInt() - mAddr)};
                            if (param == Protocol1::passThruChannel) {
                                if ((value >= 1) && (value <= 16)) {
                                    channel_t v{value};
                                    NVM::data().passThru({lastAddrOffset, lastOn}) = v.toInt() - 1;
                                    NVM::data().change();
                                }
                                else {
        //                            decltype(NVM::data()[lastOnIndex].passThru())::_;
                                    NVM::data().passThru({lastAddrOffset, lastOn}).setNaN();
                                    NVM::data().change();
                                }
                            }
                            else if (param == Protocol1::reset) {
                                if (value == Protocol1::bCastReset) {
                                    using sw_t = std::remove_cvref_t<decltype(NVM::data().sw({}))>;
                                    NVM::data().sw({lastAddrOffset, lastOn}) = sw_t{};
                                    NVM::data().change();
                                }
                            }
                            else if (param == Protocol1::timeMpxMode) {
                                NVM::data().mpxMode(lastAddrOffset, value);
                                NVM::data().change();
                            }
                            
                        }
                    }
                }
                else { // command (execute)
                    update(Meta::List<Actors...>{}, mode, index, addr);                    
                }
                return true;                
            }
        private:
            template<typename F, typename... AA>
            inline static void off(Meta::List<F, AA...>) {
                for(auto& sw : F::switches()) {
                    sw = F::SwState::Off;
                }
            }
            template<typename F, typename... AA>
            inline static void update(Meta::List<F, AA...>, const mode_t mode, const index_t index, const addr_t address) {
                update<F>(mode, index, address);
                if constexpr(sizeof...(AA) > 0) {
                    update(Meta::List<AA...>{}, mode, index, address);
                }
            }
            
            template<typename Actor>
            static inline void update(const mode_t mode, const index_t index, const addr_t address) {
//                Actor::_;
                if ((Actor::address + mAddr) == address) {
                    if (mode != Protocol1::off) {
                        lastOnIndex = index;
                        lastOnAddr = address;
                        if (mode == Protocol1::on) {
                            Actor::switches()[index] = Actor::SwState::Steady;
                        }
                        else if (mode == Protocol1::blink1) {
                            Actor::switches()[index] = Actor::SwState::Blink1;
                        }
                        else if (mode == Protocol1::blink2) {
                            Actor::switches()[index] = Actor::SwState::Blink2;
                        }
#ifdef USE_IBUS
                        else if (mode == Protocol1::config) {
                            Actor::switches()[index] = Actor::SwState::Off;
                        }
#endif
                    }
                    else {
                        lastOnAddr = lastaddr_t{};
                        lastOnIndex = lastindex_t{};
                        Actor::switches()[index] = Actor::SwState::Off;
                    }
                }
            }
        private:
            using lastindex_t = etl::uint_ranged_NaN<uint8_t, index_t::Lower, index_t::Upper>; 
            using lastaddr_t = etl::uint_ranged_NaN<uint8_t, 0, addr_t::Upper>; 
            static inline lastindex_t lastOnIndex;
            static inline channel_t mChannel{9};
            static inline addr_t mAddr{0};
            static inline lastaddr_t lastOnAddr; 
        };

        template<typename PA, typename Actor, typename Out = void>
        struct Digital {
            using channel_t = PA::channel_t;
            using value_t = PA::value_type;
            
            using protocol_t = Protocol1;
            
            using addr_t = Protocol1::addr_t;
            using index_t = Protocol1::index_t;
            using mode_t = Protocol1::mode_t;
            using param_t = Protocol1::param_t;
            using pvalue_t = Protocol1::pvalue_t;
            
            using blink_index_t = Out::blink_index_t;
            
            using tick_t = Out::tick_t;
            inline static constexpr auto blinkMax = tick_t::max();

            inline static constexpr auto pwmMax = Out::pwmMax;
            
            static inline void init(const channel_t c) {
                mChannel = c;
            }

            static inline void channel(const channel_t c) {
                if (c) {
                    mChannel = c;
                }
            }

            static inline void address(const addr_t a) {
                mAddr = a;
            }
            
            template<typename T>
            inline static constexpr bool isLearnCode(const T& v) {
                return Protocol1::isLearnCode(v);
            }
            static inline bool periodic() {
                const auto cv = PA::valueMapped(mChannel);

                if (!cv) return true;
                
                const addr_t addr = Protocol1::toAddress(cv);
                const index_t index = Protocol1::toIndex(cv);
                const mode_t mode = Protocol1::toMode(cv);
                                
                if (Protocol1::isControlMessage(cv)) { // control
                    const param_t param = Protocol1::toParameter(cv);
                    const pvalue_t value = Protocol1::toParameterValue(cv);
                    
                    lpv = value;
                    lpp = param;
                    
                    if (param == Protocol1::broadCast) {
                        if (value == Protocol1::bCastOff) {
                            lastOnIndex = lastindex_t{};
                            for(auto& s : Actor::switches()) {
                                s = Actor::SwState::Off;
                            }                        
                        }
                    }
                    else if (lastOnIndex) {
                        index_t lastOn{lastOnIndex.toInt()};
                        if (param == Protocol1::reset) {
                            if (value == Protocol1::bCastReset) {
                                Out::reset(lastOn);
                            }
                        }
                        else if (param == Protocol1::pwm) {
                            Actor::switches()[lastOn] = Actor::SwState::Steady;
                            const uint8_t pwm = ((uint16_t)value * pwmMax) / pvalue_t::Upper;
                            Out::pwm(lastOn, pwm);
                        }
                        else if (param == Protocol1::blink1Intervall) {
                            Actor::switches()[lastOn] = Actor::SwState::Blink1;
                            const uint16_t intervall = ((uint32_t)value * tick_t::max()) / pvalue_t::Upper;
                            Out::intervall2(lastOn, tick_t::fromRaw(intervall), blink_index_t{0});
                            Out::duration(lastOn, tick_t::fromRaw(intervall / 2), blink_index_t{0});
                        }
                        else if (param == Protocol1::blink1Duration) {
                            Actor::switches()[lastOn] = Actor::SwState::Blink1;
                            const uint16_t intervall = Out::intervall(lastOn, blink_index_t{0}).value;
                            const uint16_t duration = ((uint32_t)value * intervall) / pvalue_t::Upper; 
                            Out::duration(lastOn, tick_t::fromRaw(duration), blink_index_t{0});
                        }
                        else if (param == Protocol1::blink2Intervall) {
                            Actor::switches()[lastOn] = Actor::SwState::Blink2;
                            const uint16_t intervall = ((uint32_t)value * tick_t::max()) / pvalue_t::Upper;
                            Out::intervall2(lastOn, tick_t::fromRaw(intervall), blink_index_t{1});
                            Out::duration(lastOn, tick_t::fromRaw(intervall / 2), blink_index_t{1});
                        }
                        else if (param == Protocol1::blink2Duration) {
                            Actor::switches()[lastOn] = Actor::SwState::Blink2;
                            const uint16_t intervall = Out::intervall(lastOn, blink_index_t{1}).value;
                            const uint16_t duration = ((uint32_t)value * intervall) / pvalue_t::Upper; 
                            Out::duration(lastOn, tick_t::fromRaw(duration), blink_index_t{1});
                        }
                    }
                }
                else { // command
                    if (addr != mAddr) {
                        return false;
                    }
                    lmv = mode;
                    if (mode != Protocol1::off) {
                        lastOnIndex = index.toInt();
                        if (mode == Protocol1::on) {
                            Actor::switches()[index] = Actor::SwState::Steady;
                        }
                        else if (mode == Protocol1::blink1) {
                            Actor::switches()[index] = Actor::SwState::Blink1;
                        }
                        else if (mode == Protocol1::blink2) {
                            Actor::switches()[index] = Actor::SwState::Blink2;
                        }
                    }
                    else {
                        lastOnIndex = lastindex_t{};
                        Actor::switches()[index] = Actor::SwState::Off;
                    }
                }
                return true;                
            }
            static inline mode_t lmv;
            static inline pvalue_t lpv;
            static inline param_t lpp;
//        private: 
            using lastindex_t = etl::uint_ranged_NaN<uint8_t, index_t::Lower, index_t::Upper>; 
            static inline lastindex_t lastOnIndex;
            static inline channel_t mChannel{14}; // ch 16
            static inline addr_t    mAddr{0};
        };

    template<typename PA>
        struct Switch2 {
            using channel_t = PA::channel_t;
            using value_t = PA::value_type;
            
            static inline constexpr uint8_t nBinary{2};
            static inline constexpr uint8_t nTernary{5};
            
            static_assert([]{
                uint16_t v{1};
                for(uint8_t i = 0; i < nTernary; ++i) {
                    v *= 3;
                }
                for(uint8_t i = 0; i < nBinary; ++i) {
                    v *= 2;
                }
                return v <= (value_t::Upper - value_t::Lower);
            }(), "too much switches");
            
            static inline void init(channel_t c) {
                mChannel = c;
            }
            static inline bool periodic() {
                auto chValue = PA::value(mChannel).toInt();
                chValue -= value_t::Lower;
                
                constexpr uint16_t binaryMask = (1 << nBinary) - 1; 
                
                uint8_t bi = chValue & binaryMask;
                
                [&]<auto... N>(std::index_sequence<N...>) {
                    ((states2w[N] = (bi & (0x01 << N))), ...);                    
                }(std::make_index_sequence<nBinary>{});

                uint16_t tr = chValue >> nBinary;

                [&]<auto... N>(std::index_sequence<N...>) {
                    (((states3w[N] = (tr % 3)), tr /= 3), ...);
                }(std::make_index_sequence<nTernary>{});
                
                return true;                
            }
        private:                            
            static inline channel_t mChannel{15};
            static inline std::array<bool, nBinary>  states2w{};
            static inline std::array<uint8_t, nTernary> states3w{};
        };
    
        template<typename PA>
        struct Switch1 {
            static inline bool periodic() {
                using ch_t = PA::channel_t;
                uint16_t s = PA::value(ch_t{5}).toInt();

                s -= 992;
                
                uint8_t bi = s & 0x07;
                uint16_t tr = s >> 3;
                
                states2w[0] = bi & 0x01;
                states2w[1] = bi & 0x02;
                states2w[2] = bi & 0x04;

                states3w[0] = tr % 3;                
                tr /= 3;
                states3w[1] = tr % 3;                
                tr /= 3;
                states3w[2] = tr % 3;                
                tr /= 3;
                states3w[3] = tr % 3;                

                return true;                
            }            
            static inline std::array<bool, 8>  states2w{};
            static inline std::array<uint8_t, 16> states3w{};
        };
        
//        static inline std::array<uint8_t, 4> state{};
//        static inline std::array<bool, 16> sws{};
        
//        static inline uint16_t x{};
        
        template<typename PA>
        struct Provider {
            inline static constexpr auto ibus_type = IBus::Type::type::FLIGHT_MODE;
            
            static inline etl::uint_ranged<uint8_t, 0, 100> sc;
            
            inline static constexpr uint16_t value() {
                using ch_t = PA::channel_t;
                uint16_t x1 = (PA::value(ch_t{5}).toInt() - 1000) / 100;
//                x = std::min(x1, uint16_t(sws.size() - 1));
                
//                uint16_t y = PA::value(ch_t{6}).toInt();
//                if (y > 2000) {
//                    if (++sc == 3) {
//                        sws[x] = !sws[x];
//                    }
//                }
//                if (y < 1510) {
//                    sc.setToBottom();
//                }
                
//                return x;
//                return x * 10 + sws[x];
                
//                uint16_t x = (PA::value(ch_t{4}).toInt() - 1100 + 5) / 10;
//                uint8_t  sv1 = x % 3;
//                x /= 3;
//                uint8_t  sv2 = x % 3;
//                x /= 3;
//                uint8_t  sv3 = x % 3;
//                x /= 3;
//                uint8_t  sv4 = x % 3;
                
//                state[0] = sv1;
//                state[1] = sv2;
//                state[2] = sv3;
//                state[3] = sv4;
                
//                return sv1 + sv2 * 10 + sv3 * 100 + sv4 * 1000;
                return 0;    
            }
            static inline void init() {
            }
//            static inline std::array<uint8_t, 4> state{};
//            static inline std::array<bool, 16> sws{};
        };
        template<typename PA>
        struct ProviderState {
            inline static constexpr auto ibus_type = IBus::Type::type::FLIGHT_MODE;
            inline static constexpr uint16_t value() {
//                return sws[x];                
                return 0;
            }
            static inline void init() {
            }
        };
    }
    
    namespace Servo {
        template<auto N = 0, typename Dbg = void>
        struct ProtocollAdapter {
            // AFHDS2A: 14 channels
            // there should be an update to 16 / 18 channels
            enum class State : uint8_t {Undefined, GotStart20, Data, CheckL, CheckH};
            
            using value_type = etl::uint_ranged_NaN<uint16_t, 988, 2011>;
            
            using channel_t = etl::uint_ranged_NaN<uint8_t, 0, 17>;
            
            static inline value_type value(const channel_t ch) {
                if (const uint8_t chi = ch.toInt(); ch) {
                    if (chi < 14) {
                        const std::byte h = (*inactive)[2 * chi + 1] & 0x0f_B;
                        const std::byte l = (*inactive)[2 * chi];
                        return value_type{(uint16_t(h) << 8) + uint8_t(l)};
                    }
                    else if (chi < 18) {
                        const std::byte h1 = (*inactive)[6 * (chi - 14) + 1] & 0xf0_B;
                        const std::byte h2 = (*inactive)[6 * (chi - 14) + 3] & 0xf0_B;
                        const std::byte h3 = (*inactive)[6 * (chi - 14) + 5] & 0xf0_B;
                        return value_type{(uint8_t(h1) >> 4) + uint8_t(h2) + (uint16_t(h3) << 4)};
                    }
                }            
                return value_type{};
            }
            static inline value_type valueMapped(const channel_t ch) {
                return value(ch);
            }
            
            static inline bool process(const std::byte b) {
                switch(mState) {
                case State::Undefined:
                    csum.reset();
                    if (b == 0x20_B) {
                        csum += b;
                        mState = State::GotStart20;
                    }
                    break;
                case State::GotStart20:
                    if (b == 0x40_B) {
                        csum += b;
                        mState = State::Data;
                        mIndex.setToBottom();
                    }
                    else {
                        mState = State::Undefined;
                    }
                    break;
                case State::Data:
                    (*active)[mIndex] = b;
                    csum += b;
                    if (mIndex.isTop()) {
                        mState = State::CheckL;
                    }
                    else {
                        ++mIndex;
                    }
                    break;
                case State::CheckL:
                    csum.lowByte(b);
                    mState = State::CheckH;
                    break;
                case State::CheckH:
                    csum.highByte(b);
                    mState = State::Undefined;
                    if (csum) {
                        if constexpr (!std::is_same_v<Dbg, void>) {
                            Dbg::toggle();
                        }
                        using std::swap;
                        swap(active, inactive);
                    }
                    break;
                }
                return true;
            }
            
        private:
            using MesgType = std::array<std::byte, 28>;
            
            inline static CheckSum csum;
            inline static State mState{State::Undefined};
            inline static MesgType mData0; // 0x20, 0x40 , 28 Bytes, checkH, checkL
            inline static MesgType mData1; // 0x20, 0x40 , 28 Bytes, checkH, checkL
            inline static MesgType* active = &mData0;
            inline static MesgType* inactive = &mData1;
            inline static etl::index_type_t<MesgType> mIndex;
        };
    }
    
    using battery_voltage_t = External::Units::voltage<uint16_t, std::ratio<1,100>>;
    using current_t = External::Units::ampere<uint16_t, std::ratio<1,100>>;
    using temp_t = External::Units::celsius<uint16_t, std::ratio<1,10>>;
    using rpm_t = External::Units::RPM;
    
    template<typename CNumber, 
             template<typename CN, typename PA, typename ISR, typename RXL, typename TXL> typename Uart, 
             typename Baud, 
             typename ProviderList,
             typename Clock,
             typename DaisyChainEnable = void,
             etl::Concepts::NamedFlag useLoss = etl::NamedFlag<false>,
             etl::Concepts::NamedFlag useStats = etl::NamedFlag<false>,
             typename Debug = void, 
             typename MCU = DefaultMcuType>
    struct  Sensor;
    
    template<AVR::Concepts::ComponentPosition CNumber, 
             template<typename CN, typename PA, typename ISR, typename RXL, typename TXL> typename Uart, 
             etl::Concepts::NamedConstant Baud, 
             typename... Providers,
             typename Clock,
             typename DaisyChainEnable,
             etl::Concepts::NamedFlag useLoss,
             etl::Concepts::NamedFlag useStats,
             typename Debug, 
             AVR::Concepts::At01Series MCU>
    struct Sensor<CNumber, Uart, Baud, Meta::List<Providers...>, Clock, DaisyChainEnable, useLoss, useStats, Debug, MCU> final {
        struct NoDebug {
            static inline void init() {}
            template<auto T>
            static inline constexpr void set() {}
            static inline constexpr void set(std::byte) {}
        };
        
        struct LossProvider {
            inline static constexpr auto ibus_type = IBus::Type::type::FLIGHT_MODE;
            inline static constexpr void init() {}
            inline static constexpr uint16_t value() {
                return counter;
            }
            inline static void increment() {
                ++counter;
            }
            static inline uint16_t counter{};
        };
        struct StatisticProvider {
            inline static constexpr auto ibus_type = IBus::Type::type::FLIGHT_MODE;
            inline static constexpr void init() {}
            inline static constexpr uint16_t value() {
                return 10000 + (mPackets * 100) + mBytes;
            }
            inline static constexpr void incBytes() {
                if (++mBytes == 100) {
                    mBytes = 0;
                }
            }
            inline static constexpr void incPackets() {
                if (++mPackets == 100) {
                    mPackets = 0;
                }
            }
            static inline uint8_t mBytes{};
            static inline uint8_t mPackets{};
        };
        
        using debug = std::conditional_t<std::is_same_v<Debug, void>, NoDebug, Debug>;
        
        static inline constexpr auto UartNumber = CNumber::component_type::value;
        
        using provider_list_extern = Meta::List<Providers...>;

        using provider_list1 = std::conditional_t<useLoss::value, 
                                                 Meta::push_back<provider_list_extern, LossProvider>,
                                                 provider_list_extern>;
        using provider_list = std::conditional_t<useStats::value, 
                                                 Meta::push_back<provider_list1, StatisticProvider>,
                                                 provider_list1>;
        
        
        inline static constexpr auto numberOfProviders = Meta::size_v<provider_list>;
        
        inline static constexpr std::byte Cdiscover = 0x80_B;
        inline static constexpr std::byte CgetType  = 0x90_B;
        inline static constexpr std::byte CgetValue = 0xa0_B;
        inline static constexpr std::byte Creset    = 0xf0_B;
        
        using sensor_number_t = etl::uint_ranged_NaN<uint8_t, 1, 15>;
        
        struct ProtocollAdapter final {
            inline static constexpr uint8_t   Length    = 4;
            
            enum class ibus_state_t {Undefined = 0, Reset, 
                                     Length, Discover, Type, Value, Skip, 
                                     CheckSum, CheckSumSkip,
                                     Reply};
            
            ProtocollAdapter() = delete;
            
            inline static void start() {
                mState = ibus_state_t::Undefined;
                uart::template rxEnable<true>();
            }
            inline static void reset() {
                mFirstSensorNumber= sensor_number_t{};
                mLastSensorNumber = sensor_number_t{};
                mReceivedNumber   = sensor_number_t{};
                if constexpr(!std::is_same_v<DaisyChainEnable, void>) {
                    DaisyChainEnable::off();
                }
                start();
            }
            
            inline static bool process(const std::byte c) {
//                debug::template set<1>();
                StatisticProvider::incBytes();
                switch (mState) {
                case ibus_state_t::Undefined:
                    csum.reset();
                    responder::start();
                    if (length(c) == Length) {
                        mState = ibus_state_t::Length;
                        csum += c;
                    }
                    break;
                case ibus_state_t::Reset:
                    reset();
                    mState = ibus_state_t::CheckSumSkip;
                    break;
                case ibus_state_t::Length:
                    StatisticProvider::incPackets(); 
                    csum += c;  
                    mReceivedNumber = sensor_number_t{};
                    if (command(c) == Cdiscover) {
                        debug::template set<1>();
                        if (const sensor_number_t n{address(c)}; n) {
                            if (!mFirstSensorNumber) {
                                mFirstSensorNumber = n;
                                if (numberOfProviders == 1) {
                                    mLastSensorNumber = n;
                                }
                                mReceivedNumber = n;
                                mState = ibus_state_t::Discover;
                                responder::start(responder::reply_state_t::DiscoverWait);
                            }
                            else if (!mLastSensorNumber) {
                                const uint8_t index = n.toInt() - mFirstSensorNumber.toInt();
                                if (index == (numberOfProviders - 1)) {
                                    mLastSensorNumber = n;
                                }
                                mReceivedNumber = n;
                                mState = ibus_state_t::Discover;
                                responder::start(responder::reply_state_t::DiscoverWait);
                            }
                            else if (mFirstSensorNumber && mLastSensorNumber && inRange(n)) { // maybe a loss of connection
                                mReceivedNumber = n;
                                mState = ibus_state_t::Discover;
                                responder::start(responder::reply_state_t::DiscoverWait);
                                LossProvider::increment();
                            }
                            else {
                                mState = ibus_state_t::Skip;
                            }
                        }
                        else {
                            mState = ibus_state_t::Undefined;
                        }
                    }
                    else if (command(c) == CgetType) {
                        if (const sensor_number_t n{address(c)}; n) {
                            if (inRange(n)) {
                                mReceivedNumber = n;
                                mState = ibus_state_t::Type;                        
                                responder::start(responder::reply_state_t::TypeWait);
                            }
                            else {
                                mState = ibus_state_t::Skip;
                            }
                        }
                        else {
                            mState = ibus_state_t::Undefined;
                        }
                    }
                    else if (command(c) == CgetValue) {
                        if (const sensor_number_t n{address(c)}; n) {
                            if (inRange(n)) {
                                mReceivedNumber = n;
                                mState = ibus_state_t::Value;                        
                                responder::start(responder::reply_state_t::ValueWait);
                            }
                            else {
                                mState = ibus_state_t::Skip;
                            }
                        }
                        else {
                            mState = ibus_state_t::Undefined;
                        }
                    }
//                    else if (command(c) == Creset) {
                    else if (c == Creset) {
                        mState = ibus_state_t::Reset;                        
                    }
                    else {
                        mState = ibus_state_t::Undefined;
                    }
                    break;
                case ibus_state_t::Skip:
                    mState = ibus_state_t::CheckSumSkip;                        
                    break;
                case ibus_state_t::Discover:
                    csum.lowByte(c);
                    mState = ibus_state_t::CheckSum;                        
                    break;
                case ibus_state_t::Type:
                    csum.lowByte(c);
                    mState = ibus_state_t::CheckSum;                        
                    break;
                case ibus_state_t::Value:
                    csum.lowByte(c);
                    mState = ibus_state_t::CheckSum;                        
                    break;
                case ibus_state_t::CheckSum:
                    csum.highByte(c);
                    if (csum) { // mReceivedNumber valid
                        mState = ibus_state_t::Reply;                        
                        uart::template rxEnable<false>();
                    }
                    else {
                        mState = ibus_state_t::Undefined;
                    }
                    break;
                case ibus_state_t::CheckSumSkip:
                    mState = ibus_state_t::Undefined;                        
                    break;
                case ibus_state_t::Reply:
                    break;
                default:
                    break;
                }
//                debug::template set<0>();
                debug::set(std::byte(mState));
                return true;
            }
            inline static constexpr bool permitReply() {
                return mState == ibus_state_t::Reply;
            }
        private:
            static inline constexpr std::byte command(const std::byte b) {
                if (std::none(b & 0x0f_B)) {
                    return 0x00_B;
                }
                return (b & 0xf0_B) ;
            }
            static inline constexpr uint8_t length(const std::byte b) {
                if (std::any(b & 0xf0_B)) {
                    return {};
                }
                return uint8_t(b & 0x0f_B);
            }
            static inline constexpr sensor_number_t address(const std::byte b) {
                auto v = uint8_t(b & 0x0f_B);
                if (v != 0) {
                    return {v};
                }
                else {
                    return {};
                }
            }
            static inline CheckSum csum;
            static inline ibus_state_t  mState = ibus_state_t::Undefined;
        };
        
        using pa = ProtocollAdapter;
        using uart = Uart<CNumber, pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<8>>;

        inline static constexpr void init() {
            (Providers::init(), ...);
            uart::template init<Baud, AVR::HalfDuplex>();
            if constexpr(!std::is_same_v<DaisyChainEnable, void>) {
                DaisyChainEnable::init();
            }
            clear();
        }            
        
        inline static constexpr void clear() {
            if constexpr(!std::is_same_v<DaisyChainEnable, void>) {
                DaisyChainEnable::off();
            }
            pa::reset();
            responder::start();
            debug::init();
//            debug::template set<0x00>();
        }
        
        struct Responder final {
            inline static constexpr auto delayBeforeReply = 700_us;
            static inline constexpr auto intervall = Clock::intervall;
            //                    std::integral_constant<uint16_t, intervall.value>::_;
            static inline constexpr auto ticks_to_wait = delayBeforeReply / intervall;
            //                    std::integral_constant<uint8_t, ticks_to_wait>::_;
            static_assert(ticks_to_wait > 0);        
            using wait_t = etl::uint_ranged<uint8_t, 0, ticks_to_wait+1>;
            
            enum class reply_state_t {Undefined = 0, 
                                      DiscoverWait, TypeWait, ValueWait, 
                                      Discover, Type, Value,
                                      DaisyWait, DaisySet,
                                      Wait, WaitOver};
            
            Responder() = delete;
            inline static constexpr void start(const reply_state_t s = reply_state_t::Undefined) {
                mReply = s;
                mTicks.template set<etl::RangeCheck<false>>(0); 
            }
            inline static constexpr void ratePeriodic() {
                if (pa::permitReply()) {
                    ++mTicks;
                    if (mTicks.isTop()) {
                        switch(mReply) {
                        case reply_state_t::DiscoverWait:
                            mReply = reply_state_t::Discover;
                            break;
                        case reply_state_t::TypeWait:
                            mReply = reply_state_t::Type;
                            break;
                        case reply_state_t::ValueWait:
                            mReply = reply_state_t::Value;
                            break;
                        case reply_state_t::Undefined:
                        case reply_state_t::Discover:
                        case reply_state_t::Type:
                        case reply_state_t::Value:
                        case reply_state_t::Wait:
                        case reply_state_t::WaitOver:
                        case reply_state_t::DaisyWait:
                        case reply_state_t::DaisySet:
                            break;
                        default:
                            break;
                        }
                        debug::set(std::byte(uint8_t(mReply) + 16));
                    }
                }
            }
            inline static constexpr void periodic() {
                uart::periodic();
                switch(mReply) {
                case reply_state_t::Discover:
                    if (inRange(mReceivedNumber)) {
                        CheckSum cs;
                        uart::put(cs += 0x04_B);
                        uart::put(cs += Cdiscover | std::byte{mReceivedNumber.toInt()});
                        uart::put(cs.lowByte());
                        uart::put(cs.highByte());
                        mReply = reply_state_t::Wait;
                        debug::set(std::byte(uint8_t(mReply) + 16));
                    }
                    break;
                case reply_state_t::Type:
                    if (inRange(mReceivedNumber)) {
                        uint8_t index = mReceivedNumber.toInt() - mFirstSensorNumber.toInt();
                        auto type = IBus::Type::type::NONE;
                        Meta::visitAt<provider_list>(index, [&]<typename P>(Meta::Wrapper<P>){
                                                         type = P::ibus_type;
                                                     });
                        CheckSum cs;
                        uart::put(cs += 0x06_B);
                        uart::put(cs += CgetType | std::byte{mReceivedNumber.toInt()});
                        uart::put(cs += std::byte(type));
                        uart::put(cs += 0x02_B); // fix
                        uart::put(cs.lowByte());
                        uart::put(cs.highByte());
                        mReply = reply_state_t::DaisyWait;
                        debug::set(std::byte(uint8_t(mReply) + 16));
                    }
                    break;
                case reply_state_t::Value:
                    if (inRange(mReceivedNumber)) {
                        uint8_t index = mReceivedNumber.toInt() - mFirstSensorNumber.toInt();
                        uint16_t value{};
                        Meta::visitAt<provider_list>(index, [&]<typename P>(Meta::Wrapper<P>){
                                                         value = P::value();
                                                     });
                        CheckSum cs;
                        uart::put(cs += 0x06_B);
                        uart::put(cs += CgetValue | std::byte{mReceivedNumber.toInt()});
                        uart::put(cs += etl::nth_byte<0>(value));
                        uart::put(cs += etl::nth_byte<1>(value)); 
                        uart::put(cs.lowByte());
                        uart::put(cs.highByte());
                        mReply = reply_state_t::Wait;
                        debug::set(std::byte(uint8_t(mReply) + 16));
                    }
                    break;
                case reply_state_t::Wait:
                    if (uart::isIdle()) {
                        mReply = reply_state_t::WaitOver;
                        debug::set(std::byte(uint8_t(mReply) + 16));
                    }
                    break;
                case reply_state_t::DaisyWait:
                    if (uart::isIdle()) {
                        mReply = reply_state_t::DaisySet;
                        debug::set(std::byte(uint8_t(mReply) + 16));
                    }
                    break;
                case reply_state_t::WaitOver:
                    pa::start();
                    mReply = reply_state_t::Undefined;
                    debug::set(std::byte(uint8_t(mReply) + 16));
                    break;
                case reply_state_t::DaisySet:
                    pa::start();
                    mReply = reply_state_t::Undefined;
                    if constexpr(!std::is_same_v<DaisyChainEnable, void>) {
                        if (mLastSensorNumber) {
                            DaisyChainEnable::on();
                        }
                    }
                    debug::set(std::byte(uint8_t(mReply) + 16));
                    break;
                case reply_state_t::Undefined:
                case reply_state_t::DiscoverWait:
                case reply_state_t::TypeWait:
                case reply_state_t::ValueWait:
                    break;
                default:
                    break;
                }
            }
        private:
            static inline reply_state_t mReply = reply_state_t::Undefined;
            inline static wait_t mTicks;
        };
        
        using responder = Responder;
        
        inline static constexpr void ratePeriodic() {
            responder::ratePeriodic();
        }
        inline static constexpr void periodic() {
            responder::periodic();
        }
        
        private:
        static inline constexpr bool inRange(const sensor_number_t n) {
            if (!n) return false;
            const auto in = n.toInt();
            return (in >= mFirstSensorNumber.toInt()) && (in < (mFirstSensorNumber.toInt() + numberOfProviders));
        }
        
        static inline sensor_number_t mReceivedNumber;
        static inline sensor_number_t mFirstSensorNumber;
        static inline sensor_number_t mLastSensorNumber;
    };
}

