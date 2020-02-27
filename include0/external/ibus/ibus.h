#pragma once

#include <cstdint>
#include <cstddef>
#include <limits>
#include <chrono>

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
    using namespace std::literals::chrono;
    namespace Type {
        // https://github.com/betaflight/betaflight/tree/master/src/main/telemetry
        enum class type : uint8_t {
            NONE             = 0x00,
            TEMPERATURE      = 0x01,
            RPM_FLYSKY       = 0x02,
            EXTERNAL_VOLTAGE = 0x03,
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
            SPE              = 0x7e, // Speed 2bytes km/h
        
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
    
    template<typename CNumber, 
             template<typename CN, typename PA, typename ISR, typename RXL, typename TXL> typename Uart, 
             typename Baud, 
             typename ProviderList,
             typename Clock,
             typename DaisyChainEnable = void,
             typename Debug = void, 
             typename MCU = DefaultMcuType>
    struct  Sensor;
    
    template<AVR::Concepts::ComponentPosition CNumber, 
             template<typename CN, typename PA, typename ISR, typename RXL, typename TXL> typename Uart, 
             etl::Concepts::NamedConstant Baud, 
             typename... Providers,
             typename Clock,
             typename DaisyChainEnable,
             typename Debug, 
             AVR::Concepts::At01Series MCU>
    struct Sensor<CNumber, Uart, Baud, Meta::List<Providers...>, Clock, DaisyChainEnable, Debug, MCU> final {
        struct NoDebug {
            template<auto T>
            static inline constexpr void set() {}
        };
        
        using debug = std::conditional_t<std::is_same_v<Debug, void>, NoDebug, Debug>;
        
        static inline constexpr auto UartNumber = CNumber::component_type::value;

        using provider_list = Meta::List<Providers...>;
        inline static constexpr auto numberOfProviders = Meta::size_v<provider_list>;

        inline static constexpr std::byte Cdiscover = 0x80_B;
        inline static constexpr std::byte CgetType  = 0x90_B;
        inline static constexpr std::byte CgetValue = 0xa0_B;
        inline static constexpr std::byte Creset    = 0xf0_B;

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
        
        struct ProtocollAdapter final {
            inline static constexpr uint8_t   Length    = 4;

            enum class ibus_state_t {Undefined = 0, Reset, 
                                     Length, Discover, Type, Value, Skip, 
                                     CheckSum, CheckSumSkip,
                                     Reply};
            
            ProtocollAdapter() = delete;
            
            inline static void reset() {
                mState = ibus_state_t::Undefined;
                uart::template rxEnable<true>();
            }
            
            inline static bool process(const std::byte c) {
                ++nBytes;
                switch (mState) {
                case ibus_state_t::Undefined:
                    debug::template set<0x01>();
                    csum.reset();
                    responder::start();
                    if (length(c) == Length) {
                        mState = ibus_state_t::Length;
                        csum += c;
                    }
                    break;
                case ibus_state_t::Reset:
                    debug::template set<0x0a>();
                    mState = ibus_state_t::CheckSumSkip;
                    break;
                case ibus_state_t::Length:
                    debug::template set<0x02>();
                    ++nPackets;
                    csum += c;  
                    mReceivedNumber = sensor_number_t{};
                    if (command(c) == Cdiscover) {
                        debug::template set<0x0b>();
                        if (const sensor_number_t n{uint8_t(c & 0x0f_B)}) {
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
                                const uint8_t index = n - mFirstSensorNumber;
                                if (index == (numberOfProviders - 1)) {
                                    mLastSensorNumber = n;
                                }
                                mReceivedNumber = n;
                                mState = ibus_state_t::Discover;
                                responder::start(responder::reply_state_t::DiscoverWait);
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
                        debug::template set<0x0c>();
                        if (const sensor_number_t n{uint8_t(c & 0x0f_B)}) {
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
                        debug::template set<0x0d>();
                        if (const sensor_number_t n{uint8_t(c & 0x0f_B)}) {
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
                    else if (command(c) == Creset) {
                        debug::template set<0x0e>();
                        mState = ibus_state_t::Reset;                        
                    }
                    else {
                        mState = ibus_state_t::Undefined;
                    }
                    break;
                case ibus_state_t::Skip:
                    debug::template set<0x08>();
                    mState = ibus_state_t::CheckSumSkip;                        
                    break;
                case ibus_state_t::Discover:
                    debug::template set<0x03>();
                    csum.lowByte(c);
                    mState = ibus_state_t::CheckSum;                        
                    break;
                case ibus_state_t::Type:
                    debug::template set<0x06>();
                    mState = ibus_state_t::CheckSum;                        
                    break;
                case ibus_state_t::Value:
                    debug::template set<0x07>();
                    mState = ibus_state_t::CheckSum;                        
                    break;
                case ibus_state_t::CheckSum:
                    debug::template set<0x04>();
                    csum.highByte(c);
                    if (csum) {
                        mState = ibus_state_t::Reply;                        
                    }
                    else {
                        mState = ibus_state_t::Undefined;
                    }
                    break;
                case ibus_state_t::CheckSumSkip:
                    debug::template set<0x09>();
                    mState = ibus_state_t::Undefined;                        
                    break;
                case ibus_state_t::Reply:
                    debug::template set<0x05>();
                    uart::template rxEnable<false>();
                    break;
                default:
                    break;
                }
                return true;
            }
            inline static constexpr bool permitReply() {
                return mState == ibus_state_t::Reply;
            }
//        private:
            static inline constexpr std::byte command(const std::byte b) {
                return b & 0xf0_B;
            }
            static inline constexpr uint8_t length(const std::byte b) {
                return uint8_t(b & 0x0f_B);
            }
            static inline CheckSum csum;
            static inline ibus_state_t  mState = ibus_state_t::Undefined;
        };
        
        using pa = ProtocollAdapter;
        using uart = Uart<CNumber, pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<8>>;
        
        inline static constexpr void init() {
            uart::template init<Baud, AVR::HalfDuplex>();
            pa::reset();
            if constexpr(!std::is_same_v<DaisyChainEnable, void>) {
                DaisyChainEnable::off();
            }
            if constexpr(!std::is_same_v<Debug, void>) {
                Debug::init();
                Debug::template set<0x00>();
            }
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
                                      Wait};
            
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
                            debug::template set<0x10>();                            
                            mReply = reply_state_t::Discover;
                            break;
                        case reply_state_t::TypeWait:
                            debug::template set<0x11>();
                            mReply = reply_state_t::Type;
                            break;
                        case reply_state_t::ValueWait:
                            debug::template set<0x12>();
                            mReply = reply_state_t::Value;
                            break;
                        case reply_state_t::Undefined:
                        case reply_state_t::Discover:
                        case reply_state_t::Type:
                        case reply_state_t::Value:
                        case reply_state_t::Wait:
                            break;
                        default:
                            break;
                        }
                    }
                }
            }
            inline static constexpr void periodic() {
                uart::periodic();
                switch(mReply) {
                case reply_state_t::Discover:
                    debug::template set<0x13>();
                    if (inRange(mReceivedNumber)) {
                        CheckSum cs;
                        uart::put(cs += 0x04_B);
                        uart::put(cs += Cdiscover | std::byte{mReceivedNumber.toInt()});
                        uart::put(cs.lowByte());
                        uart::put(cs.highByte());
                        mReply = reply_state_t::Wait;
                    }
                    break;
                case reply_state_t::Type:
                    debug::template set<0x14>();
                    if (inRange(mReceivedNumber)) {
                        CheckSum cs;
                        uart::put(cs += 0x06_B);
                        uart::put(cs += CgetType | std::byte{mReceivedNumber.toInt()});
                        uart::put(cs += 0x01_B);
                        uart::put(cs += 0x02_B); // fix
                        uart::put(cs.lowByte());
                        uart::put(cs.highByte());
                        mReply = reply_state_t::Wait;
                    }
                    break;
                case reply_state_t::Value:
                    debug::template set<0x15>();
                    if (inRange(mReceivedNumber)) {
                        CheckSum cs;
                        uart::put(cs += 0x06_B);
                        uart::put(cs += CgetValue | std::byte{mReceivedNumber.toInt()});
                        uart::put(cs += 0x10_B);
                        uart::put(cs += 0x10_B); 
                        uart::put(cs.lowByte());
                        uart::put(cs.highByte());
                        mReply = reply_state_t::Wait;
                    }
                    break;
                case reply_state_t::Wait:
                    debug::template set<0x16>();
                    if (uart::isEmpty()) {
                        pa::reset();
                        mReply = reply_state_t::Undefined;
                    }
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
//        private:
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
        
    //private:
        static inline uint16_t nPackets{};
        static inline uint16_t nBytes{};
        
        using sensor_number_t = etl::uint_ranged_NaN<uint8_t, 1, 15>;
        
//        static inline constexpr bool isOwn(const sensor_number_t n) {
//            if (!n) return false;
//            return (n >= mFirstSensorNumber) && (n <= mLastSensorNumber);
//        }
//        static inline constexpr bool hasLearnedAddresses() {
//            return mFirstSensorNumber && mLastSensorNumber;
//        }
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

