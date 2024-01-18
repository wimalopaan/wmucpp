#pragma once


#include "mcu/mcu.h"
#include "units.h"
#include "concepts.h"
#include "mcu/mcu_traits.h"

#include <type_traits>
#include <concepts>
#include <cstddef>
#include <algorithm>
#include <span>
#include <array>

#include "fonts.h"

namespace External {

template<typename TWI, typename Timer, typename term = void>
struct SSD1306 {
    static inline constexpr std::array initData{	// Initialization Sequence
                0x00_B,     // Control
                0x20_B, 0x00_B,		// Set Memory Addressing Mode
                0x21_B, 0x00_B, 0x7f_B,
                0x22_B, 0x00_B, 0x07_B,
                0xB0_B,			// Set Page Start Address for Page Addressing Mode, 0-7
                0xC0_B,			// Set COM Output Scan Direction
                0x00_B,			// --set low column address
                0x10_B,			// --set high column address
                0x40_B,			// --set start line address
                0x81_B, 0x7F_B,		// Set contrast control register
                0xA8_B, 0x3F_B,		// Set multiplex ratio(1 to 64)
                0xD3_B, 0x38_B,		// Set display offset. 00 = no offset
                0xD5_B,			// --set display clock divide ratio/oscillator frequency
                0x80_B,			// --set divide ratio
                0xDA_B, 0x12_B,		// Set com pins hardware configuration
                0x8D_B, 0x14_B,		// Set DC-DC enable
                0xA4_B,			// Output RAM to Display
                0xAF_B // On
    };

    enum class State : uint8_t {Undefined, Init, Clear, Run};

    inline static constexpr bool scroll{false};

    inline static constexpr uint8_t pxColumns{128};
    inline static constexpr uint8_t pxRows{64};
    inline static constexpr uint8_t pages{pxRows / 8};
    static inline constexpr uint8_t address = 0x3c;
    static inline constexpr External::Tick<Timer> refreshTicks{20ms};
    static inline constexpr External::Tick<Timer> initTicks{100ms};

    static inline constexpr uint8_t offset{1};

    using frameBuffer = std::array<std::byte, pages * pxColumns + offset>;

    using i2c = TWI;

    using font = Fonts::Font<6,8>;

    static inline constexpr uint8_t charsCols = pxColumns / font::Width;
    static inline constexpr uint8_t lines = pxRows / font::Height;

    inline static void clear() {
        for(auto& d : mData) {
            d = 0x00_B;
        }
        mData[0] = 0x40_B;
    }

    inline static void home() {
        mCharPosition.reset();
    }

    inline static void put(const char c) {
        auto shiftUp = []{
            std::copy(std::begin(mData) + offset + pxColumns, std::end(mData), std::begin(mData) + offset);
        };
        if (c == '\r') {
            mCharPosition.resetX();
            return;
        }
        if (c == '\n') {
            if constexpr(scroll) {
                mCharPosition.incY(shiftUp);
            }
            else {
                mCharPosition.incY();
            }
            return;
        }
        std::copy(std::begin(font()[c]), std::end(font()[c]), &mData[offset] + (mCharPosition.x() * font::Width) + (mCharPosition.y() * pxColumns));
        if constexpr(scroll) {
            mCharPosition.next(shiftUp);
        }
        else {
            ++mCharPosition;
        }
    }
    inline static void put(const std::byte b) {
        put(uint8_t(b));
    }

    inline static std::byte get() {
        return 0x00_B;
    }

    static inline void init() {
        i2c::init();
        i2c::address(address);
    }
    static inline void periodic() {
        i2c::periodic();
    }
    static inline void ratePeriodic() {
        const auto oldState = mState;
        ++mStateTicks;

        switch(mState) {
        case State::Undefined:
            mStateTicks.on(initTicks, [] {
                mState = State::Init;
            });
            break;
        case State::Init:
            if (i2c::isReady()) {
                mState = State::Clear;
            }
            break;
        case State::Clear:
            mState = State::Run;
            break;
        case State::Run:
            mStateTicks.on(refreshTicks, []{
                i2c::ratePeriodic();
            });
            break;
        }
        if (oldState != mState) {
            mStateTicks.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                i2c::write(initData);
                break;
            case State::Clear:
                clear();
                i2c::restart();
                break;
            case State::Run:
                break;
            }
        }
    }
    static inline bool isIdle() {
        return i2c::isIdle();
    }
private:
    static inline State mState{State::Undefined};
    static inline External::Tick<Timer> mStateTicks;
    static inline auto& mData = i2c::data();
    static inline etl::uint2D_ranged<uint8_t, charsCols, lines> mCharPosition;
};

}

#if 0

template<typename TWI, typename Timer, typename term>
struct SSD1306 {
    inline static constexpr auto reset_puls_width = 5_us;

    inline static constexpr std::byte DispOff = 0xAE_B;
    inline static constexpr std::byte DispOn = 0xAF_B;

    inline static constexpr std::byte White = 0x01_B;
    inline static constexpr std::byte Black = 0x00_B;

    inline static constexpr uint16_t Width  = 128;
    inline static constexpr uint16_t Height = 64;
    inline static constexpr uint16_t Size = Width * Height / 8;

    using font = Fonts::Font<6,8>;

    struct InitGenerator {
        inline static constexpr auto
            data = std::make_array<std::byte>(	// Initialization Sequence
                0x00_B,     // Control
                0xAE_B,		// Display OFF (sleep mode)
                0x20_B, 0x00_B,		// Set Memory Addressing Mode
                // 00=Horizontal Addressing Mode; 01=Vertical Addressing Mode;
                // 10=Page Addressing Mode (RESET); 11=Invalid
                0xB0_B,			// Set Page Start Address for Page Addressing Mode, 0-7
                0xC8_B,			// Set COM Output Scan Direction
                0x00_B,			// --set low column address
                0x10_B,			// --set high column address
                0x40_B,			// --set start line address
                0x81_B, 0xCF_B,		// Set contrast control register
                0xA1_B,			// Set Segment Re-map. A0=address mapped; A1=address 127 mapped.
                0xA6_B,			// Set display mode. A6=Normal; A7=Inverse
                0xA8_B, 0x3F_B,		// Set multiplex ratio(1 to 64)
                0xA4_B,			// Output RAM to Display
                // 0xA4=Output follows RAM content; 0xA5,Output ignores RAM content
                0xD3_B, 0x00_B,		// Set display offset. 00 = no offset
                0xD5_B,			// --set display clock divide ratio/oscillator frequency
                0xF0_B,			// --set divide ratio
                0xD9_B, 0x22_B,		// Set pre-charge period
                0xDA_B, 0x12_B,		// Set com pins hardware configuration
                0xDB_B,			// --set vcomh
                0x20_B,			// 0x20,0.77xVcc
                0x8D_B, 0x14_B,		// Set DC-DC enable
                0xAF_B // On
                //                                                                            0xA4_B
                );
        constexpr auto operator()() {
            return data;
        }
    };
    inline static constexpr typename AVR::Pgm::Util::Converter<InitGenerator>::pgm_type init_sequence{};

public:
    using twi = TWI;
    using timer = Timer;

    static inline constexpr auto address = AVR::Twi::Address{0x3c};
    static inline constexpr External::Tick<timer> initTicks{100_ms};
    static inline constexpr External::Tick<timer> debugTicks{500_ms};

    static inline void init() {
        twi::init();
    }
    static inline void periodic() {
        twi::periodic();
    }

    inline static void clear() {
        std::array<std::byte, 2> a{};
        a[0] = 0x00_B;
        std::array<std::byte, 129> cmdAndCol{};
        cmdAndCol[0] = 0x40_B;

        for(uint8_t page = 0; page < 8; ++page) {
            a[1] = std::byte(0xb0 + page);
            twi::write(address, a);

            twi::write(address, cmdAndCol);
        }
    }
    inline static void put(char c) {
        twi::write(address, 0x40_B, font()[c]);
    }
    inline static bool home() {
        auto cs = std::make_array(0x00_B, 0xb0_B, 0x21_B, 0x00_B, 0x7f_B);
        return twi::write(address, cs);
    }

    enum class State : uint8_t {Undefined, Init, Run};

    static inline bool isIdle() {
        return twi::isIdle();
    }

    static inline void ratePeriodic() {
        const auto oldState{mState};
        ++mStateTicks;
        ++mDebugTicks;

        (++mDebugTicks).on(debugTicks, []{
            auto r = twi::state();
            etl::outl<term>("S: "_pgm, (uint8_t)r);
        });

        switch(mState) {
        case State::Undefined:
            mStateTicks.on(initTicks, [] {
                mState = State::Init;
            });
            break;
        case State::Init:
            if (twi::isIdle()) {
                mState = State::Run;
            }
            break;
        case State::Run:
            break;
        }
        if (oldState != mState) {
            mStateTicks.reset();
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
            {
                auto r = twi::write(address, init_sequence);
                etl::outl<term>("SSD1306: init: "_pgm, (uint8_t)r);
            }
            break;
            case State::Run:
                etl::outl<term>("SSD1306: Run"_pgm);
                break;
            }
        }

    }
private:
    static inline State mState{State::Undefined};
    static inline External::Tick<timer> mStateTicks;
    static inline External::Tick<timer> mDebugTicks;
};
}


namespace detail {
    namespace SSD1306 {
        template<typename = void>
        struct Base {
            inline static constexpr auto reset_puls_width = 5_us;

            inline static constexpr std::byte DispOff = 0xAE_B;
            inline static constexpr std::byte DispOn = 0xAF_B;

            inline static constexpr std::byte White = 0x01_B;
            inline static constexpr std::byte Black = 0x00_B;

            inline static constexpr uint16_t Width  = 128;
            inline static constexpr uint16_t Height = 64;
            inline static constexpr uint16_t Size = Width * Height / 8;

            using font = Font<6,8>;
            struct InitGenerator {
                inline static constexpr auto data = std::make_array<std::byte>(	// Initialization Sequence
                                                                                0xAE_B,		// Display OFF (sleep mode)
                                                                                0x20_B, 0x00_B,		// Set Memory Addressing Mode
                                                                                // 00=Horizontal Addressing Mode; 01=Vertical Addressing Mode;
                                                                                // 10=Page Addressing Mode (RESET); 11=Invalid
                                                                                0xB0_B,			// Set Page Start Address for Page Addressing Mode, 0-7
                                                                                0xC8_B,			// Set COM Output Scan Direction
                                                                                0x00_B,			// --set low column address
                                                                                0x10_B,			// --set high column address
                                                                                0x40_B,			// --set start line address
                                                                                0x81_B, 0xCF_B,		// Set contrast control register
                                                                                0xA1_B,			// Set Segment Re-map. A0=address mapped; A1=address 127 mapped.
                                                                                0xA6_B,			// Set display mode. A6=Normal; A7=Inverse
                                                                                0xA8_B, 0x3F_B,		// Set multiplex ratio(1 to 64)
                                                                                0xA4_B,			// Output RAM to Display
                                                                                // 0xA4=Output follows RAM content; 0xA5,Output ignores RAM content
                                                                                0xD3_B, 0x00_B,		// Set display offset. 00 = no offset
                                                                                0xD5_B,			// --set display clock divide ratio/oscillator frequency
                                                                                0xF0_B,			// --set divide ratio
                                                                                0xD9_B, 0x22_B,		// Set pre-charge period
                                                                                0xDA_B, 0x12_B,		// Set com pins hardware configuration
                                                                                0xDB_B,			// --set vcomh
                                                                                0x20_B,			// 0x20,0.77xVcc
                                                                                0x8D_B, 0x14_B,		// Set DC-DC enable
                                                                                0xAF_B
                                                                                );
                constexpr auto operator()() {
                    return data;
                }
            };
            inline static constexpr typename ::Util::Pgm::Converter<InitGenerator>::pgm_type init_sequence{};
        };
        struct SPI {};
        struct TWI {};
        template<typename Dev, typename DCPin, typename ResetPin = void>
        struct SpiEndpoint {
            typedef SPI type;
            typedef Dev dev_type;
            typedef ResetPin reset_type;
            typedef DCPin dc_type;
        };
        template<typename Dev, const ::TWI::Address& Address>
        struct TwiEndpoint {
            typedef TWI type;
            typedef Dev TWIMaster;
            inline static constexpr auto address = Address;
        };
        template<typename E>
        concept bool Spi() {
            return std::is_same<typename E::type, detail::SSD1306::SPI>::value;
        }
        template<typename E>
        concept bool Twi() {
            return std::is_same<typename E::type, detail::SSD1306::TWI>::value;
        }
    }
}

template<typename E>
class SSD1306;

template<detail::SSD1306::Spi EndPoint>
class SSD1306<EndPoint> : public detail::SSD1306::Base<> {
    using spi = typename EndPoint::dev_type;
    using reset = typename EndPoint::reset_type;
    using dc = typename EndPoint::dc_type;
    inline static void send_command(std::byte b) {
        dc::low();
        spi::template put<true>(b);
        spi::off();
    }
    inline static void send_data(std::byte b) {
        dc::high();
        spi::template put<true>(b);
        spi::off();
    }
public:
    inline static void init() {
        dc::template dir<AVR::Output>();
        dc::high();
        spi::init();
        if constexpr(!std::is_same<reset, void>::value) {
            reset::template dir<AVR::Output>();
            reset::high();
            reset::low();
            Util::delay(reset_puls_width);
            reset::high();
        }
        for(auto v : init_sequence) {
            send_command(v);
        }
    }
    inline static void put(char c) {
        for(auto v : font()[c]) {
            send_data(v);
        }
    }
    inline static void put(const PgmStringView& string) {
        for(uint8_t i = 0, c = 0; (c = string[i]) != '\0'; ++i) {
            put(c);
        };
    }
    inline static void clear() {
        for(uint8_t page = 0; page < 8; ++page) {
            send_command(std::byte(0xb0 + page));
            for(uint8_t column = 0; column < 128; ++column) {
                send_data(0x00_B);
            }
        }
    }
#ifdef USE_SSD1306_IMAGE
    inline static void image() {
        uint16_t index = 0;
        for(uint8_t page = 0; page < 8; ++page) {
            send_command(std::byte{0xb0 + page});
            for(uint8_t column = 0; column < 128; ++column) {
                send_data(std::byte{image1[index++]});
            }
        }
    }
    struct ImageGenerator {
        inline static constexpr auto data = std::make_array<uint8_t>(
                                                0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                                0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                                0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                                0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                                0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                                0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                                0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x55,0x55,0x55,0x55,0x55,0x55,0x55,
                                                0x55,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,
                                                0xaa,0xaa,0xaa,0x01,0x00,0x00,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x00,0x00,
                                                0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x01,0x00,0x00,0x40,0x00,0x00,0x00,0x00,
                                                0x00,0x00,0x00,0x00,0x00,0x01,0x00,0xc0,0x06,0x00,0x00,0x80,0x00,0x00,0x00,
                                                0x00,0x00,0x00,0x00,0x00,0x80,0x00,0x00,0x20,0x08,0x00,0x00,0x40,0x55,0x55,
                                                0x55,0x55,0x55,0x55,0x55,0x55,0xd5,0x00,0x00,0x10,0x10,0x00,0x00,0x80,0xaa,
                                                0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0x00,0x00,0x10,0x10,0x00,0x00,0x00,
                                                0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x10,0x00,0x00,
                                                0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x20,0x00,
                                                0x00,0x00,0x00,0x00,0x00,0x15,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x10,
                                                0x00,0x00,0x00,0x00,0x00,0xf8,0xf5,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x10,
                                                0x10,0x00,0x00,0x00,0x00,0x00,0x06,0x00,0x0e,0x00,0x00,0x00,0x00,0x00,0x00,
                                                0x20,0x08,0x00,0x00,0x00,0x00,0x80,0x01,0x00,0x30,0x00,0x00,0x00,0x00,0x00,
                                                0x00,0xc0,0x06,0x00,0x00,0x00,0x00,0x60,0x00,0x00,0xc0,0x00,0x00,0x00,0x00,
                                                0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x10,0x00,0x00,0x00,0x01,0x00,0x00,
                                                0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0c,0x00,0x00,0x00,0x06,0x00,
                                                0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x08,
                                                0x00,0x00,0x00,0x15,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,
                                                0x10,0x00,0x00,0x80,0x75,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x00,0x00,0x00,
                                                0x00,0x10,0x00,0x00,0x00,0x41,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x00,0x00,
                                                0x00,0x00,0x20,0x00,0x00,0x80,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x00,
                                                0x00,0x00,0x00,0x40,0x00,0x00,0x00,0x41,0x00,0x00,0x00,0x00,0x00,0x00,0x20,
                                                0x00,0x00,0x00,0x00,0x40,0x00,0x00,0x80,0x20,0x00,0x00,0x00,0x00,0x00,0x00,
                                                0x20,0x00,0x00,0x00,0x00,0x80,0x00,0x00,0x80,0x40,0x00,0x00,0x00,0x00,0x00,
                                                0x00,0x10,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x21,0x00,0x00,0x00,0x00,
                                                0x00,0x00,0x10,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x80,0x40,0x00,0x00,0x00,
                                                0x00,0x00,0x00,0x10,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x80,0x20,0x00,0x00,
                                                0x00,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x80,0x20,0x00,
                                                0x00,0x00,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x41,
                                                0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x80,
                                                0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x02,0x00,
                                                0x80,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x02,
                                                0x00,0x80,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x00,0x00,
                                                0x02,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x00,
                                                0x00,0x02,0x00,0x80,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x00,0x00,0x00,
                                                0x00,0x00,0x02,0x00,0x80,0x41,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x00,0x00,
                                                0x00,0x00,0x00,0x02,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x00,
                                                0x00,0x00,0x00,0x00,0x02,0x00,0x80,0x21,0x00,0x00,0x00,0x00,0x00,0x00,0x08,
                                                0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x41,0x00,0x00,0x00,0x00,0x00,0x00,
                                                0x08,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x80,0x20,0x00,0x00,0x04,0x00,0x00,
                                                0x00,0x10,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x80,0x40,0x00,0x00,0x3f,0x00,
                                                0x00,0x00,0x10,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x80,0x20,0x00,0x80,0x40,
                                                0x00,0x00,0x00,0x10,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x80,0x20,0x00,0x40,
                                                0x40,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x41,0x00,
                                                0x40,0x80,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x40,0x00,0x00,0x80,0x20,
                                                0x00,0x40,0x80,0x00,0x00,0x00,0x40,0x00,0x00,0x00,0x00,0x40,0x00,0x00,0x80,
                                                0x21,0x00,0x20,0x00,0x01,0x00,0x00,0xc0,0x00,0x00,0x00,0x00,0x20,0x00,0x00,
                                                0x00,0x41,0x00,0x40,0x80,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x10,0x00,
                                                0x00,0x80,0x20,0x00,0x60,0x80,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x08,
                                                0x00,0x00,0x80,0x20,0x00,0x40,0x80,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x00,
                                                0x04,0x00,0x00,0x80,0x40,0x00,0x80,0x40,0x00,0x00,0x00,0x00,0x04,0x00,0x00,
                                                0x00,0x02,0x00,0x00,0x00,0x21,0x00,0x00,0x41,0x00,0x00,0x00,0x00,0x10,0x00,
                                                0x00,0x00,0x01,0x00,0x00,0x80,0x20,0x00,0x00,0x36,0x00,0x00,0x00,0x00,0x60,
                                                0x00,0x00,0xc0,0x00,0x00,0x00,0x80,0x41,0x00,0x00,0x08,0x00,0x00,0x00,0x00,
                                                0x80,0x01,0x00,0x30,0x00,0x00,0x00,0x00,0x35,0x00,0x00,0x00,0x00,0x00,0x00,
                                                0x00,0x00,0x0e,0x00,0x0f,0x00,0x00,0x00,0x00,0x15,0x00,0x00,0x00,0x00,0x00,
                                                0x00,0x00,0x00,0xf8,0xfb,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                                0x00,0x00,0x00,0x00,0x00,0x0a,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                                0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                                0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                                0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                                0x00,0x00,0x00,0x00
                                                );

        inline static constexpr bool getPixelXBM(uint8_t x, uint8_t y, const std::array<uint8_t, 1024>& xbm) {
            uint16_t b = (x / 8) + (y * 16);
            uint8_t x1 = (x % 8) ;
            uint8_t  mask = (1 << x1);
            if (xbm[b] & mask) {
                return true;
            }
            return false;
        }

        inline static constexpr void setPixelSSD(uint8_t x, uint8_t y, std::array<uint8_t, 1024>& ssd) {
            uint16_t b = (x % 128) + (y / 8) * 128;
            uint8_t y1 = y % 8;
            uint8_t mask = (1 << y1);
            ssd[b] |= mask;
        }

        inline static constexpr auto data1 = []{
            std::array<uint8_t, 1024> d;
            for(uint8_t x = 0; x < 128; ++x) {
                for(uint8_t y = 0; y < 64; ++y) {
                    if (getPixelXBM(x, y, data)) {
                        setPixelSSD(x, y, d);
                    }
                }
            }
            return d;
        }();

        constexpr auto operator()() {
            return data1;
        }
    };
    inline static constexpr typename Util::Pgm::Converter<ImageGenerator>::pgm_type image1{};
#endif
};

template<detail::SSD1306::Twi EndPoint>
class SSD1306<EndPoint> : public detail::SSD1306::Base<> {
    using TWIMaster = typename EndPoint::TWIMaster;
    inline static constexpr auto Address = EndPoint::address;
    using master_sync = typename TWIMaster::master_type;

public:
    inline static bool init() {
        return TWIMaster::template startWrite<Address>(init_sequence, 0x00_B);
    }
    inline static void clear() {
        std::array<std::byte, 1> a;
        std::array<std::byte, 128> col;
        for(uint8_t page = 0; page < 8; ++page) {
            a[0] = std::byte(0xb0 + page);
            master_sync::template write<Address>(a, 0x00_B);
            master_sync::template write<Address>(col, 0x40_B);
        }
    }
    inline static void put(char c) {
        TWIMaster::template startWrite<Address>(font()[c], 0x40_B);
    }
    inline static void put(PgmStringView string) {
        char c{'\0'};
        for(uint8_t i = 0; (c = string[i]) != '\0'; ++i) {
            put(c);
        };
    }
    inline static bool home() {
        auto cs = std::make_array(0xb0_B, 0x21_B, 0x00_B, 0x7f_B);
        return TWIMaster::template startWrite<Address>(cs, 0x00_B);
    }

    inline static bool gotoxy(uint8_t x, uint8_t y) {
        x = x * 6;					// one char: 6 pixel width
        auto cs = std::make_array<std::byte>(std::byte(0xb0+y), 0x21_B, std::byte{x}, 0x7f_B);
        return TWIMaster::template startWrite<Address>(cs, 0x00_B);
    }
private:
};


#endif
