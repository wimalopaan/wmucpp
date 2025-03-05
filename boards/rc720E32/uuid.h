#pragma once

#include <cstdio>
#include <cstring>

namespace Mcu::Stm {
    struct Uuid {
        static inline uint32_t get() {
            uint32_t uid[3];
            uid[0] = *(uint32_t *)UID_BASE;
            uid[1] = *(uint32_t *)(UID_BASE + 4);
            uid[2] = *(uint32_t *)(UID_BASE + 8);
            return Hash32Len5to12 ((const char *)&uid[0], 12);
        }
        private:
        static inline uint32_t fetch32(const char * const p) {
            uint32_t result;
            memcpy(&result, p, sizeof(result));
            return result;
        }
        static inline uint32_t rotate32(const uint32_t val, const int shift) {
          // Avoid shifting by 32: doing so yields an undefined result.
          return shift == 0 ? val : ((val >> shift) | (val << (32 - shift)));
        }
        // A 32-bit to 32-bit integer hash copied from Murmur3.
        static inline uint32_t fmix(uint32_t h){
          h ^= h >> 16;
          h *= 0x85ebca6b;
          h ^= h >> 13;
          h *= 0xc2b2ae35;
          h ^= h >> 16;
          return h;
        }
        static inline uint32_t mur(uint32_t a, uint32_t h) {
          // Helper from Murmur3 for combining two 32-bit values.
          a *= c1;
          a = rotate32(a, 17);
          a *= c2;
          h ^= a;
          h = rotate32(h, 19);
          return h * 5 + 0xe6546b64;
        }
        static inline uint32_t Hash32Len5to12(const char * const s, const size_t len) {
          uint32_t a = (uint32_t)len, b = a * 5, c = 9, d = b;
          a += fetch32(s);
          b += fetch32(s + len - 4);
          c += fetch32(s + ((len >> 1) & 4));
          return fmix(mur(c, mur(b, mur(a, d))));
        }
        // Magic numbers for 32-bit hashing.  Copied from Murmur3.
        static inline constexpr uint32_t c1 = 0xcc9e2d51;
        static inline constexpr uint32_t c2 = 0x1b873593;
    };
}



