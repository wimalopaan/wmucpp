#pragma once

#include <chrono>

#include "concepts.h"
#include "format.h"

namespace etl {
    using namespace std::literals::chrono;
 
    template<Device Device, bool ensure = false>
    constexpr void put(std::byte b) {
        if constexpr(ensure) {
            while(!Device::put(b)) {
//                AVR::Util::delay(1_us);
            }
        }
        else {
            Device::put(b);
        }
    }
    
    template<typename Type>
    struct basic_iomanip {
        typedef Type iomanip_type;
    };
    
    struct CRLF{};
    struct LF{};
    struct CR{};
    
    template <typename Mode>
    struct lineTerminator final : public basic_iomanip<lineTerminator<Mode>> {};
    
    template<typename DeviceType, typename LT = lineTerminator<CRLF>>
    struct basic_ostream final {
        typedef DeviceType device_type;
        typedef LT line_terminator_type;
    };
    
    template<Unsigned T, uint8_t Bits>
    struct Fraction;
    template<typename T, uint8_t Bits>
    struct FixedPoint;
    
    namespace detail {

        template<Stream Stream, Container C>
        requires (sizeof(typename C::value_type) == 1)
        constexpr inline void out_impl(const C& a) {
            for(const typename C::value_type c : a) {
                if (c == typename C::value_type{'\0'}) {
                    break;
                }
                put<typename Stream::device_type>(std::byte{c});
            };   
        }
        
        template<Stream Stream, typename V> 
        requires Signed<V> || Unsigned<V>
        constexpr inline void out_impl(V v) {
            std::array<Char, numberOfDigits<V>()> buffer;
            itoa(v, buffer);
            out_impl<Stream>(buffer);
        }

        template<Stream Stream>
        constexpr inline void out_impl(std::byte b) {
            constexpr uint8_t Base = 16;
            std::array<Char, numberOfDigits<uint8_t, Base>()> buffer;
            etl::fill(buffer, Char{'0'});
            itoa_r<Base>(std::to_integer<uint8_t>(b), buffer);
            out_impl<Stream>("0x"_pgm);
            out_impl<Stream>(buffer);
        }
        
        template<Stream Stream>
        constexpr inline void out_impl(Char v) {
            put<typename Stream::device_type>(std::byte{v});
        }
        
        template<Stream Stream, typename T>
        requires std::is_same_v<std::remove_cv_t<T>, bool>
        constexpr inline void out_impl(T b) {
            if (b) {
                out_impl<Stream>("true"_pgm);
            }
            else {
                out_impl<Stream>("false"_pgm);
            }
        }

        template<Stream Stream>
        constexpr inline void out_impl(lineTerminator<CRLF>) {
            put<typename Stream::device_type>(std::byte{'\r'});
            put<typename Stream::device_type>(std::byte{'\n'});
        }
        template<Stream Stream>
        constexpr inline void out_impl(lineTerminator<LF>) {
            put<typename Stream::device_type>(std::byte{'\n'});
        }
        template<Stream Stream>
        constexpr inline void out_impl(lineTerminator<CR>) {
            put<typename Stream::device_type>(std::byte{'\r'});
        }

        //        template<MCU::Stream Stream>
//        void out(const DateTime::TimeTm& t) {
//            std::out<Stream>("Time["_pgm, (uint8_t)t.mTime.tm_mday, Char{'/'}, (uint8_t)(t.mTime.tm_mon + 1), Char{'/'}, 
//                             (uint16_t)(t.mTime.tm_year + 1900), Char{' '}, 
//                             (uint8_t)t.mTime.tm_hour, Char{':'}, (uint8_t)t.mTime.tm_min, Char{':'}, 
//                             (uint8_t)t.mTime.tm_sec, Char{']'});
//        }
        
//        template<MCU::Stream Stream>
//        void out(std::percent p) {
//            out<Stream>(p.value());
//            out<Stream>(Char{'%'});
//        }
        
//        template<MCU::Stream Stream>
//        void out(const std::RPM& r) {
//            out<Stream>(r.value());
//            out<Stream>("Upm"_pgm);
//        }
//        template<MCU::Stream Stream>
//        void out(const std::hertz& f) {
//            out<Stream>(f.value);
//            out<Stream>("Hz"_pgm);
//        }
//        template<MCU::Stream Stream>
//        void out(const std::milliseconds& d) {
//            out<Stream>(d.value);
//            out<Stream>("ms"_pgm);
//        }
//        template<MCU::Stream Stream>
//        void out(const std::microseconds& d) {
//            out<Stream>(d.value);
//            out<Stream>("us"_pgm);
//        }

        template<etl::Concepts::Stream Stream, typename T, uint8_t Bits>
        inline void out_impl(const Fraction<T, Bits>& f);
        
        template<etl::Concepts::Stream Stream, etl::Concepts::Signed T, uint8_t Bits>
        inline void out_impl(const FixedPoint<T, Bits>& f);

        template<etl::Concepts::Stream Stream, etl::Concepts::Unsigned T, uint8_t Bits>
        inline void out_impl(const FixedPoint<T, Bits>& f);
        
    } // detail
    
    template<Stream Stream, typename... TT>
    inline constexpr void out(const TT&... v __attribute__((unused))) {
        using ::etl::detail::out_impl;
        if constexpr(!std::is_same_v<typename Stream::device_type, void>) {
            ((out_impl<Stream>(v)),...);
        }
    }

    template<Stream Stream, typename... TT>
    inline constexpr void outl(const TT&... v __attribute__((unused))) {
        if constexpr(!std::is_same_v<typename Stream::device_type, void>) {
            using ::etl::detail::out_impl;
            ((out_impl<Stream>(v)),..., out_impl<Stream>(typename Stream::line_terminator_type()));
        }
    }
        
}
