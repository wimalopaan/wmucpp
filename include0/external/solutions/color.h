#pragma once

#include <cstdint>
#include <limits>

namespace External {
    
    struct ColorSequenceRGB final {};
    struct ColorSequenceGRB final {};
    
    struct Red final {
        uint8_t value = 0;  
    };
    struct Green final {
        uint8_t value = 0;  
    };
    struct Blue final {
        uint8_t value = 0;  
    };
    
    template<typename C = ColorSequenceRGB> struct cRGB;
    
    template<>
    struct cRGB<ColorSequenceRGB> {
        static constexpr cRGB createFrom(const std::array<uint8_t, 3> bytes) {
            return {Red{bytes[0]}, Green{bytes[1]}, Blue{bytes[2]}};
        }
        
        constexpr cRGB() {}
        cRGB(const volatile cRGB& o) : r(o.r), g(o.g), b(o.b) {}
        constexpr cRGB(const cRGB& o) : r(o.r), g(o.g), b(o.b) {}
        constexpr explicit cRGB(uint8_t v) : r(v), g(v), b(v) {}
        constexpr cRGB(Red red, Green green, Blue blue) : r(red.value), g(green.value), b(blue.value) {}
        constexpr cRGB(Red v) : r(v.value) {}
        constexpr cRGB(Green v) : g(v.value) {}
        constexpr cRGB(Blue v) : b(v.value) {}
        constexpr cRGB& operator=(const cRGB& o) = default;
        constexpr cRGB& operator+=(const cRGB& c) {
            if (r > 0) {
                r = (r + c.r) / 2;
            }
            else {
                r = c.r;
            }
            if (g > 0) {
                g = (g + c.g) / 2;
            }
            else {
                g = c.g;
            }
            if (b > 0) {
                b = (b + c.b) / 2;
            }
            else {
                b = c.b;
            }
            return *this;
        }
        //    constexpr cRGB& operator*=(const std::percent& p) {
        //        g = std::expand(p, uint8_t(0), g);
        //        r = std::expand(p, uint8_t(0), r);
        //        b = std::expand(p, uint8_t(0), b);
        //        return *this;        
        //    }
        
        uint8_t r = 0;
        uint8_t g = 0;
        uint8_t b = 0;
    };
    
    using Crgb = cRGB<ColorSequenceRGB>;
    
    //constexpr cRGB<ColorSequenceRGB> operator*(cRGB<ColorSequenceRGB> c, const std::percent& p) {
    //    return c *= p;
    //}
    
    template<>
    struct cRGB<ColorSequenceGRB> {
        static constexpr cRGB createFrom(const std::array<std::byte, 3> bytes) {
            return {Red{std::to_integer<uint8_t>(bytes[0])}, 
                Green{std::to_integer<uint8_t>(bytes[1])}, 
                Blue{std::to_integer<uint8_t>(bytes[2])}};
        }
        constexpr cRGB() {}
        cRGB(const volatile cRGB& o) : g(o.g), r(o.r), b(o.b) {}
        constexpr cRGB(const cRGB& o) : g(o.g), r(o.r), b(o.b) {}
        constexpr explicit cRGB(uint8_t v) : g(v), r(v), b(v) {}
        constexpr cRGB(Red red, Green green, Blue blue) : g(green.value), r(red.value), b(blue.value) {}
        constexpr cRGB(Red v) : r(v.value) {}
        constexpr cRGB(Green v) : g(v.value) {}
        constexpr cRGB(Blue v) : b(v.value) {}
        constexpr cRGB& operator=(const cRGB& o) = default;
        constexpr cRGB& operator+=(const cRGB& c) {
            if (r > 0) {
                r = (r + c.r) / 2u;
            }
            else {
                r = c.r;
            }
            if (g > 0) {
                g = (g + c.g) / 2u;
            }
            else {
                g = c.g;
            }
            if (b > 0) {
                b = (b + c.b) / 2u;
            }
            else {
                b = c.b;
            }
            return *this;
        }
        //    constexpr cRGB& operator*=(const std::percent& p) {
        //        g = std::expand(p, uint8_t(0), g);
        //        r = std::expand(p, uint8_t(0), r);
        //        b = std::expand(p, uint8_t(0), b);
        //        return *this;        
        //    }
        uint8_t g = 0;
        uint8_t r = 0;
        uint8_t b = 0;
    };
    
    //constexpr cRGB<ColorSequenceGRB> operator*(cRGB<ColorSequenceGRB> c, const std::percent& p) {
    //    return c *= p;
    //}
}
