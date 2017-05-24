/*
 * WMuCpp - Bare Metal C++ 
 * Copyright (C) 2016, 2017 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
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

#include "config.h"
#include "mcu/concepts.h"
#include "util/util.h"
//#include "util/fixedpoint.h"
#include "std/time.h"
#include "std/concepts.h"
#include "units/percent.h"
#include "container/pgmstring.h"

namespace std {
    
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
    
    template<MCU::Stream Stream, typename... TT> void out(const TT&... v);
    template<MCU::Stream Stream, typename... TT> void outl(const TT&... v);
    
    namespace detail {
        template<MCU::Stream Stream, Unsigned V> // concept
        void out(V v) {
            std::array<char, Util::numberOfDigits<V>() + 1> buffer;
            Util::itoa(v, buffer);
            Util::put<typename Stream::device_type, Config::ensureTerminalOutput>(&buffer[0]);
        }
        template<MCU::Stream Stream, Signed V> // concept
        void out(V v) {
            std::array<char, Util::numberOfDigits<V>() + 1> buffer;
            Util::itoa(v, buffer);
            Util::put<typename Stream::device_type, Config::ensureTerminalOutput>(&buffer[0]);
        }
        template<MCU::Stream Stream>
        void out(std::byte b) {
            constexpr uint8_t Base = 16;
            std::array<char, Util::numberOfDigits<uint8_t, Base>() + 1> buffer;
            Util::itoa_r<Base>(std::to_integer<uint8_t>(b), buffer);
            out<Stream>("0x"_pgm);
            Util::put<typename Stream::device_type, Config::ensureTerminalOutput>(&buffer[0]);
        }
        template<MCU::Stream Stream>
        void out(char v) {
            Util::put<typename Stream::device_type, Config::ensureTerminalOutput>(v);
        }
        template<MCU::Stream Stream>
        void out(std::lineTerminator<CRLF>) {
            Util::put<typename Stream::device_type, Config::ensureTerminalOutput>('\r');
            Util::put<typename Stream::device_type, Config::ensureTerminalOutput>('\n');
        }
        template<MCU::Stream Stream>
        void out(std::lineTerminator<LF>) {
            Util::put<typename Stream::device_type, Config::ensureTerminalOutput>('\n');
        }
        template<MCU::Stream Stream, uint16_t L>
        void out(const std::array<char, L>& a) {
            for(char c : a) {
                if (c == '\0') {
                    break;
                }
                Util::put<typename Stream::device_type, Config::ensureTerminalOutput>(c);
            };   
        }
        template<MCU::Stream Stream, uint8_t L>
        void out(const StringBuffer<L>& a) {
            for(char c : a) {
                if (c == '\0') {
                    break;
                }
                Util::put<typename Stream::device_type, Config::ensureTerminalOutput>(c);
            };   
        }
        template<MCU::Stream Stream>
        void out(const PgmStringView& s) {
            const char * ptr = s.ptrToPgmData;
            while (char c = pgm_read_byte(ptr++)) {
                Util::put<typename Stream::device_type, Config::ensureTerminalOutput>(c);
            };   
        }
        template<MCU::Stream Stream>
        void out(const DateTime::TimeTm& t) {
            std::out<Stream>("Time["_pgm, (uint8_t)t.mTime.tm_mday, '/', (uint8_t)(t.mTime.tm_mon + 1), '/', (uint16_t)(t.mTime.tm_year + 1900), ' '
                             , (uint8_t)t.mTime.tm_hour, ':', (uint8_t)t.mTime.tm_min, ':', (uint8_t)t.mTime.tm_sec, ']');
        }
        
        template<MCU::Stream Stream>
        void out(const std::percent& p) {
            out<Stream>(p.value());
            out<Stream>('%');
        }
        
        template<MCU::Stream Stream>
        void out(const std::hertz& f) {
            out<Stream>(f.value);
            out<Stream>("Hz"_pgm);
        }
        template<MCU::Stream Stream>
        void out(const std::milliseconds& d) {
            out<Stream>(d.value);
            out<Stream>("ms"_pgm);
        }
        template<MCU::Stream Stream>
        void out(const std::microseconds& d) {
            out<Stream>(d.value);
            out<Stream>("us"_pgm);
        }
        template<MCU::Stream Stream>
        void out(const Config&) {
            std::outl<Stream>("fMcu: "_pgm, Config::fMcu);
            std::outl<Stream>("fMcuMhZ: "_pgm, Config::fMcuMhz);
            std::outl<Stream>("Timer::NumberOfTimers: "_pgm, Config::Timer::NumberOfTimers);
            std::outl<Stream>("Timer::frequency: "_pgm, Config::Timer::frequency);
            std::outl<Stream>("Timer::resolution: "_pgm, Config::Timer::resolution);
            std::outl<Stream>("EventManager::EventQueueLength: "_pgm, Config::EventManager::EventQueueLength);
            std::outl<Stream>("Usart::SendQueueLength: "_pgm, Config::Usart::SendQueueLength);
            std::outl<Stream>("Usart::ReceiveQueueLength: "_pgm, Config::Usart::RecvQueueLength);
            std::outl<Stream>("ensureTerminalOutput: "_pgm, Config::ensureTerminalOutput);
            std::outl<Stream>("disableCout: "_pgm, Config::disableCout);
            std::outl<Stream>("SoftSpiMaster::pulseDelay: "_pgm, Config::SoftSpiMaster::pulseDelay);
            std::outl<Stream>("Button::buttonTicksForPressed: "_pgm, Config::Button::buttonTicksForPressed);
        }
        
    } // detail
    
    template<MCU::Stream Stream, typename... TT>
    void out(const TT&... v __attribute__((unused))) {
        if constexpr(!std::is_same<typename Stream::device_type, void>::value) {
            ((detail::out<Stream>(v)),...);
        }
        
    }
    template<MCU::Stream Stream, typename... TT>
    void outl(const TT&... v __attribute__((unused))) {
        if constexpr(!std::is_same<typename Stream::device_type, void>::value) {
            ((detail::out<Stream>(v)),..., detail::out<Stream>(typename Stream::line_terminator_type()));
        }
    }
    
} //!std

template<MCU::Stream Stream>
Stream& operator<<(Stream& o, const char* str) {
    if constexpr(!Config::disableCout) {
        Util::put<typename Stream::device_type, Config::ensureTerminalOutput>(str);
    }
    return o;
}
template<typename Buffer>
BufferDevice<Buffer>& operator<<(BufferDevice<Buffer>& o, const char* str) {
    while(*str) {
        o.put(*str++);
    }
    return o;
}
template<MCU::Stream Stream>
Stream& operator<<(Stream& o, char c) {
    if constexpr(!Config::disableCout) {
        Util::put<typename Stream::device_type, Config::ensureTerminalOutput>(c);
    }
    return o;
}
template<typename Buffer>
BufferDevice<Buffer>& operator<<(BufferDevice<Buffer>& o, char c) {
    o.put(c);
    return o;
}

template<MCU::Stream Stream>
Stream& operator<<(Stream& o, uint8_t v) {
    if constexpr(!Config::disableCout) {
        std::detail::out<Stream>(v);
    }
    return o;
}
template<typename Buffer>
BufferDevice<Buffer>& operator<<(BufferDevice<Buffer>& o, uint8_t v) {
    Util::itoa(v, o.buffer());
    return o;
}

template<MCU::Stream Stream>
Stream& operator<<(Stream& o, int8_t v) {
    if constexpr(!Config::disableCout) {
        std::detail::out<Stream>(v);
    }
    return o;
}

template<MCU::Stream Stream>
Stream& operator<<(Stream& o, uint16_t v) {
    if constexpr(!Config::disableCout) {
        std::detail::out<Stream>(v);
    }
    return o;
}

template<MCU::Stream Stream>
Stream& operator<<(Stream& o, int16_t v) {
    if constexpr(!Config::disableCout) {
        std::detail::out<Stream>(v);
    }
    return o;
}

template<MCU::Stream Stream>
Stream& operator<<(Stream& o, uint32_t v) {
    if constexpr(!Config::disableCout) {
        std::detail::out<Stream>(v);
    }
    return o;
}

template<MCU::Stream Stream>
Stream& operator<<(Stream& o, int32_t v) {
    if constexpr(!Config::disableCout) {
        std::detail::out<Stream>(v);
    }
    return o;
}

template<MCU::Stream Stream>
Stream& operator<<(Stream& o, const std::lineTerminator<std::CRLF>& t) {
    if constexpr(!Config::disableCout) {
        std::detail::out<Stream>(t);
        return o;
    }
    return o;
}
template<MCU::Stream Stream>
Stream& operator<<(Stream& o, const std::lineTerminator<std::CR>&) {
    if constexpr(!Config::disableCout) {
        return o << '\r';
    }
    return o;
}
template<MCU::Stream Stream>
Stream& operator<<(Stream& o, const std::lineTerminator<std::LF>&) {
    if constexpr(!Config::disableCout) {
        return o << '\n';
    }
    return o;
}

template<MCU::Stream Stream>
Stream& operator<<(Stream& o, const std::hertz& f) {
    if constexpr(!Config::disableCout) {
        return o << f.value << "Hz"_pgm;
    }
    return o;
}

template<MCU::Stream Stream>
Stream& operator<<(Stream& o, const std::megahertz& f) {
    if constexpr(!Config::disableCout) {
        return o << f.value << "MHz"_pgm;
    }
    return o;
}

template<MCU::Stream Stream>
Stream& operator<<(Stream& o, const std::milliseconds& d) {
    if constexpr(!Config::disableCout) {
        return o << d.value << "ms"_pgm;
    }
    return o;
}

template<MCU::Stream Stream>
Stream& operator<<(Stream& o, const std::microseconds& d) {
    if constexpr(!Config::disableCout) {
        return o << d.value << "us"_pgm;
    }
    return o;
}

template<MCU::Stream Stream>
Stream& operator<<(Stream& o, bool b) {
    if constexpr(!Config::disableCout) {
        if (b) {
            return o << "true"_pgm;
        }
        else {
            return o << "false"_pgm;
        }
    }
    return o;
}

template<MCU::Stream Stream, typename T>
Stream& operator<<(Stream& o, const Fraction<T>& f) {
    std::array<char, Util::numberOfDigits<Fraction<T>>()> buffer;
    Util::ftoa(f, buffer);
    Util::put<typename Stream::device_type, Config::ensureTerminalOutput>(&buffer[0]);
    return o;    
}

template<MCU::Stream Stream, typename T, uint16_t L>
Stream& operator<<(Stream& o, const std::array<T, L>& a) {
    if constexpr(!Config::disableCout) {
        o << "{ "_pgm;
        for(const auto& i : a) {
            o << i << ' ';
        }
        o << '}';
    }
    return o;
}

template<MCU::Stream Stream>
Stream& operator<<(Stream& o, const Config&) {
    if constexpr(!Config::disableCout) {
        o << "fMcu: "_pgm << Config::fMcu << "\r\n"_pgm;
        o << "fMcuMhZ: "_pgm << Config::fMcuMhz << "\r\n"_pgm;
        o << "Timer::NumberOfTimers: "_pgm << Config::Timer::NumberOfTimers << "\r\n"_pgm;
        o << "Timer::frequency: "_pgm << Config::Timer::frequency << "\r\n"_pgm;
        o << "Timer::resolution: "_pgm << Config::Timer::resolution << "\r\n"_pgm;
        o << "EventManager::EventQueueLength: "_pgm << Config::EventManager::EventQueueLength << "\r\n"_pgm;
        o << "Usart::SendQueueLength: "_pgm << Config::Usart::SendQueueLength << "\r\n"_pgm;
        o << "Usart::ReceiveQueueLength: "_pgm << Config::Usart::RecvQueueLength << "\r\n"_pgm;
        o << "ensureTerminalOutput: "_pgm << Config::ensureTerminalOutput << "\r\n"_pgm;
        o << "disableCout: "_pgm << Config::disableCout << "\r\n"_pgm;
        o << "SoftSpiMaster::pulseDelay: "_pgm << Config::SoftSpiMaster::pulseDelay << "\r\n"_pgm;
        o << "Button::buttonTicksForPressed: "_pgm << Config::Button::buttonTicksForPressed << "\r\n"_pgm;
    }
    return o;
}

//template<typename Stream, typename C, C... CC>
//Stream& operator<<(Stream& out, const PgmString<C, CC...>& s) {
//    const char * ptr = s.data;
//    while (char c = pgm_read_byte(ptr++)) {
//        out << c;
//    };
//    return out;
//}

template<MCU::Stream Stream>
Stream& operator<<(Stream& out, const PgmStringView& s) {
    const char * ptr = s.ptrToPgmData;
    while (char c = pgm_read_byte(ptr++)) {
        out << c;
    };
    return out;
}

template<MCU::Stream Stream>
Stream& operator<<(Stream& o, const std::RPM& rpm) {
    if constexpr(!Config::disableCout) {
        return o << rpm.value() << "RPM"_pgm;
    }
    return o;
}

template<MCU::Stream Stream>
Stream& operator<<(Stream& o, const std::percent& p) {
    if constexpr(!Config::disableCout) {
        return o << p.value() << '%';
    }
    return o;
}

template<typename Stream>
Stream& operator<<(Stream& out, const DateTime::TimeTm& t) {
    return out << "Time["_pgm << (uint8_t)t.mTime.tm_mday << '/' << (uint8_t)(t.mTime.tm_mon + 1) << '/' << (uint16_t)(t.mTime.tm_year + 1900) << ' '
               << (uint8_t)t.mTime.tm_hour << ':' << (uint8_t)t.mTime.tm_min << ':' << (uint8_t)t.mTime.tm_sec << ']';
}

