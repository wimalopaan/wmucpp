#pragma once

#include "concepts.h"

namespace etl {
    using namespace std::literals::chrono;
//    template<Device Device, bool ensure = false>
//    void put(const char* str) {
//        while(*str) {
//            Device::put(std::byte{*str});
//            ++str;
//        }
//    }
    
    template<Device Device, bool ensure = false>
    void put(std::byte b) {
        if constexpr(ensure) {
            while(!Device::put(b)) {
                AVR::Util::delay(1_us);
            }
        }
        else {
            Device::put(b);
        }
    }
    
//    template<Device Device, Container C, bool ensure = false>
//    void put(const C& c) {
//        for(uint8_t i = 0; i < c.size; ++i) {
//            if constexpr(ensure) {
//                while(!Device::put(std::byte{c[i]})) {
//                    AVR::Util::delay(1_us);
//                }
//            }
//            else {
//                Device::put(c[i]);
//            }
//        }
//    }
    
//    template<Device Device>
//    void putl(const char* str) {
//        Device::put(str);
//        Device::put(std::byte{'\n'});
//    }
    
    
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
    
    template<Stream Stream, typename... TT> void out(const TT&... v);
    template<Stream Stream, typename... TT> void outl(const TT&... v);
    
    namespace detail {
        template<Stream Stream, typename V> 
        requires Signed<V> || Unsigned<V>
        inline void out(V v) {
            std::array<Char, numberOfDigits<V>() + 1> buffer;
            itoa(v, buffer);
            put<typename Stream::device_type>(buffer);
        }

        template<Stream Stream>
        inline void out(std::byte b) {
            constexpr uint8_t Base = 16;
            std::array<Char, numberOfDigits<uint8_t, Base>() + 1> buffer;
            itoa_r<Base>(std::to_integer<uint8_t>(b), buffer);
            out<Stream>("0x"_pgm);
            put<typename Stream::device_type>(buffer);
        }
        
        template<Stream Stream>
        inline void out(Char v) {
            put<typename Stream::device_type>(std::byte{v});
        }
        
        template<Stream Stream>
        inline void out(bool b) {
            if (b) {
                out<Stream>("true"_pgm);
            }
            else {
                out<Stream>("false"_pgm);
            }
        }
        template<Stream Stream>
        inline void out(lineTerminator<CRLF>) {
            put<typename Stream::device_type>(std::byte{'\r'});
            put<typename Stream::device_type>(std::byte{'\n'});
        }
        template<Stream Stream>
        inline void out(lineTerminator<LF>) {
            put<typename Stream::device_type>(std::byte{'\n'});
        }
        template<Stream Stream>
        inline void out(lineTerminator<CR>) {
            put<typename Stream::device_type>(std::byte{'\r'});
        }
        template<Stream Stream, Container C>
        requires (sizeof(typename C::value_type) == 1)
        inline void out(const C& a) {
            for(const typename C::value_type c : a) {
                if (c == typename C::value_type{'\0'}) {
                    break;
                }
                put<typename Stream::device_type>(std::byte{c});
            };   
        }
//        template<MCU::Stream Stream>
//        void out(PgmStringView s) {
//            char c = '\0';
//            for(uint8_t i = 0; (c = s[i]) != '\0'; ++i) {
//                Util::put<typename Stream::device_type, Config::ensureTerminalOutput>(std::byte{c});
//            };   
//        }
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
    } // detail
    
    template<Stream Stream, typename... TT>
    void out(const TT&... v __attribute__((unused))) {
        if constexpr(!std::is_same<typename Stream::device_type, void>::value) {
            ((detail::out<Stream>(v)),...);
        }
        
    }
    template<Stream Stream, typename... TT>
    void outl(const TT&... v __attribute__((unused))) {
        if constexpr(!std::is_same<typename Stream::device_type, void>::value) {
            ((detail::out<Stream>(v)),..., detail::out<Stream>(typename Stream::line_terminator_type()));
        }
    }
    
    
    
}
