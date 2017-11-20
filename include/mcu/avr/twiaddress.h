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

#include "config.h"
#include "container/pgmstring.h"
#include "util/types.h"

namespace TWI {
    class Address;
}

namespace std::detail {
    template<MCU::Stream Stream> void out(const TWI::Address& rom);
}

namespace TWI {
    
    struct Write {
        static constexpr bool write = true;
    };
    struct Read {
        static constexpr bool write = false;
    };
    
    struct Range {
        const uint8_t pointer = 0;
        const uint8_t number  = 0;
    };
    
    template<typename Mode> class BusAddress;
    
    class Address {
        template<typename Stream> friend Stream& operator<<(Stream& o, const TWI::Address& a);
        template<MCU::Stream Stream> friend void std::detail::out(const TWI::Address& a);
        template<typename Mode> friend class BusAddress;
        friend bool operator<(Address lhs, Address rhs);
        friend bool operator==(Address lhs, Address rhs);
    public:
        static constexpr uint8_t lowest = 0x08;
        static constexpr uint8_t highest = 0x77;
        
        constexpr Address() {}
        explicit constexpr Address(std::byte a) : mDevice(std::to_integer<uint8_t>(a)) {}
        
        static Address fromBusValue(std::byte busValue) {
            return Address{(busValue >> 1)};
        }
        
        Address& operator++() {
            ++mDevice;
            return *this;
        }
        inline Address operator++(int) {
            Address copy(*this);
            ++*this;
            return copy;
        }
        inline explicit operator bool() const {
            return (mDevice >= lowest) && (mDevice <= highest);
        }
    private:
        uint8_t mDevice{0};
    };
    inline bool operator<(Address lhs, Address rhs) {
        return lhs.mDevice < rhs.mDevice;
    }
    inline bool operator==(Address lhs, Address rhs) {
        return lhs.mDevice == rhs.mDevice;
    }
    inline bool operator<=(Address lhs, Address rhs) {
        return (lhs == rhs) || (lhs < rhs);
    }
    
    template<typename Stream>
    Stream& operator<<(Stream& o, const TWI::Address& a) {
        if (!Config::disableCout) {
            o << "I2C["_pgm << a.mDevice << ']';
        }
        return o;
    }
    
    static constexpr Address minimumAddress{std::byte{Address::lowest}};
    static constexpr Address maximumAddress{std::byte{Address::highest}};
    
    template<typename Mode>
    class BusAddress {
    public:
        explicit constexpr BusAddress(Address device) : 
            mAddress(std::byte(Mode::write ? (device.mDevice << 1) : (device.mDevice << 1) | 0x01)) {}
        inline constexpr std::byte value() const {
            return mAddress;
        }
        static inline constexpr bool isWrite(std::byte address) {
            return std::none(address & 0x01_B);
        }
        static inline constexpr std::byte deviceAddressValue(std::byte busAdressValue) {
            return busAdressValue >> 1;
        }
    private:
        const std::byte mAddress{0};    
    };
}

namespace std {
    template<MCU::Stream Stream, typename... TT> void out(const TT&... v);
    namespace detail {
        template<MCU::Stream Stream>
        void out(const TWI::Address& a) {
            std::out<Stream>("I2C["_pgm, a.mDevice, Char{']'});
        }
    }
}
