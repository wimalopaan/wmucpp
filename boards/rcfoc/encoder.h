#pragma once

#include "parity.h"

namespace External::AS5147 {
    enum class Register : uint16_t {NOP = 0x000, ERRFL = 0x0001, PROG = 0x0003, DIAAGC = 0x3ffc, MAG = 0x3ffd, ANGLUNC = 0x3ffe, ANGLECOM = 0x3fff,
                                    SETTING1 = 0x0018, SETTING2 = 0x0019};
    
    struct Command {
        using value_type = std::byte;
        
        inline Command(const Register r = Register::NOP, const bool read = false) {
            mData[1] = etl::nth_byte<1>(uint16_t(r));
            mData[0] = etl::nth_byte<0>(uint16_t(r));
            if (read) {
                mData[1] |= 0x40_B;
            }
            bool p = ::Util::ParityGenerator::isEvenParity(mData);
            if (p) {
                mData[1] |= 0x80_B;
            }
        }
        inline Command(const std::byte d) {
            mData[1] = 0x00_B;
            mData[0] = d;
            bool p = ::Util::ParityGenerator::isEvenParity(d);
            if (p) {
                mData[1] |= 0x80_B;
            }
        }        
        inline constexpr const std::byte& operator[](const uint8_t i) const {
            if (i == 0) {
                return mData[1];
            }         
            return mData[0];
        }
        static constexpr uint8_t size() {
            return 2;
        }
    private:
        std::array<std::byte, 2> mData;
    };
    struct Result {
        using value_type = std::byte;
        constexpr std::byte& operator[](const uint8_t i) {
            if (i == 0) {
                return mData[1];
            }         
            return mData[0];
        }
        constexpr const std::byte& operator[](const uint8_t i) const{
            if (i == 0) {
                return mData[1];
            }         
            return mData[0];
        }
        static constexpr uint8_t size() {
            return 2;
        }
    private:
        std::array<std::byte, 2> mData;
    };
    
    template<template<typename, typename> typename Spi, typename ResoBits = etl::integral_constant<uint8_t, 11>>
    struct Encoder {
        static inline constexpr uint8_t maxBits = 14;
        static inline constexpr uint8_t bits = ResoBits::value;
        static inline constexpr uint8_t shift = maxBits - bits;
        
        static_assert(bits <= 14);
        
        using spi = Spi<Command, Result>;

        using raw_type = etl::typeForBits_t<bits>;
//        raw_type::_;
        static inline constexpr raw_type maxValue = (1 << bits) - 1;
//        std::integral_constant<raw_type, maxValue>::_;
        
        using angle_type = etl::uint_ranged<raw_type, 0, maxValue>;
        
        static inline void pwmOn(const bool b) {
            if (b) {
                spi::put(Command{Register::SETTING1}, Command{0x01_B << 7});
            }
        }
        
        template<bool increaseTime = false>
        static inline void read() { // approx 6,5us @ 32MHz (8MHz SPI)
            spi::template put<increaseTime>(Command{Register::ANGLECOM, true}, Command{Register::NOP, false});
        }
        
        static inline void init() {
            spi::init();
            spi::mode1();
        }
        
        static inline void periodic() {
        }
        
        static inline angle_type angle() {
            const uint8_t u = uint8_t(spi::result()[0] & 0x3f_B);
            const uint8_t l = uint8_t(spi::result()[1]);
            return angle_type(((raw_type(u) << 8) | l) >> shift);
        }
    private: 
    };
}
