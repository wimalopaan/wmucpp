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
                return reinterpret_cast<const std::byte*>(value);
            }
//        private: // structural
            const P* const value{nullptr};
        };
        
        template<typename P, size_t ByteOffset = 0>
        struct ByteRange {
            struct Iterator {
                inline explicit Iterator(const std::byte* p) : pgmPtr{p} {}
                inline void operator++() {
                    ++pgmPtr;
                }
                inline Iterator operator++(int) {
                    Iterator copy{*this};
                    ++pgmPtr;
                    return copy;
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

            inline constexpr ByteRange(const Ptr<P>& ptr) : pgmPtr{ptr.raw() + ByteOffset} {}  
            
            using value_type = std::byte;
            using size_type = std::conditional_t<sizeof(P) <= 256, uint8_t, uint16_t>;
            
            static inline constexpr size_type size() {
                return sizeof(P);
            }
            
            inline const Iterator begin() const {
                return Iterator(pgmPtr);
            }
            inline const Iterator end() const {
                return Iterator(pgmPtr + sizeof(P));
            }
            inline std::byte operator[](size_type index) const {
                return std::byte{pgm_read_byte(pgmPtr + index)};
            }
        private:
            const std::byte* const pgmPtr{nullptr};
        };
    }
}

