#pragma once

#include <cstddef>
#include <cstdint>

namespace AVR {
    namespace Pgm {
        template<typename P = std::byte>
        struct Ptr {
            inline constexpr explicit Ptr(const P* ptr) : value{ptr} {}
            inline constexpr const P* operator->() const {
                return value;
            }
            inline constexpr const std::byte* raw() const {
                return (const std::byte*)value;
            }
//        private:
            const P* const value{nullptr};
        };
        
        template<typename P>
        struct ByteRange {
            struct Iterator {
                inline explicit Iterator(const std::byte* p) : pgmPtr{p} {}
                inline void operator++() {
                    ++pgmPtr;
                }
                inline bool operator!=(const Iterator& rhs) {
                    return pgmPtr != rhs.pgmPtr;
                }
                inline std::byte operator*() const {
                    return std::byte{pgm_read_byte(pgmPtr)};
                }
            private:
                const std::byte* pgmPtr{nullptr};
            };

            inline constexpr ByteRange(const Ptr<P>& ptr) : pgmPtr{reinterpret_cast<const std::byte*>(ptr.value)} {}  
            
            inline const Iterator begin() const {
                return Iterator(pgmPtr);
            }
            inline const Iterator end() const {
                return Iterator(pgmPtr + sizeof(P));
            }
        private:
            const std::byte* const pgmPtr{nullptr};
        };
    }
}

