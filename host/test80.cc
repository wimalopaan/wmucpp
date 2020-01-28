#include <cstddef>
#include <cstdint>
#include <cassert>
#include <iostream>
#include <string>

namespace Bounded {
    template<typename T, T Lower, T Upper>
    struct int_ranged {
        explicit constexpr int_ranged(T v) : value{v} {
            assert(v >= Lower); assert(v <= Upper);
        }
        constexpr void operator+=(T rhs) {
            value = std::clamp(T(value + rhs), Lower, Upper);
        }
        constexpr void operator+=(int_ranged rhs) {
            value = std::clamp(T(value + rhs.value), Lower, Upper);
        }
        constexpr operator T() const {
            return value;
        }
    private:
        T value{}; 
    };
}

namespace Geom2D {
    template<typename T = intmax_t>
    struct X {
        template<typename U>
        explicit constexpr X(U v) : value(v) {}
        T value{};
    };
    template<typename T = intmax_t>
    struct Y {
        template<typename U>
        explicit constexpr Y(U v) : value{v} {}
        constexpr void operator+=(Y rhs) {
            value += rhs.value;
        }
        T value{};
    };
    template<typename XT = X<intmax_t>, typename YT = Y<intmax_t>>
    struct Point {
        explicit constexpr Point(XT x, YT y) : mX{x}, mY{y} {}
        
        constexpr void operator+=(YT dy) {
            mY += dy;
        }
    private:
        XT mX{};
        YT mY{};
    };
}

namespace UI {
    template<size_t Width, size_t Height>
    struct Display {
        using x_t = Geom2D::X<Bounded::int_ranged<uint8_t, 0, Width>>;
        using y_t = Geom2D::Y<Bounded::int_ranged<uint8_t, 0, Height>>;
        using point_t = Geom2D::Point<x_t, y_t>;
        
//        inline static void drawText(const std::string &text, const x_t x, const y_t y) {}
//        inline static void drawText(const x_t x, const y_t y, const std::string &text) {}
        inline static void drawText(const point_t &p, const std::string &text) {}
        inline static void drawText(const std::string& text, const point_t &p) {}
    };
}

using display = UI::Display<100, 100>;
using x_t = display::x_t;
using y_t = display::y_t;
using point_t = display::point_t;

int main(){
    using namespace std::literals;
    
//    display::drawText(x_t{1}, y_t{2}, "Bla"s);
    
    auto x1 = x_t{1};
    auto y1 = y_t{2};
    
//    display::drawText(x1, y1, "Bla"s);
    
    auto p1 = point_t{x1, y1};
    display::drawText(p1, "Bla"s);
    
    p1 += y_t{10};
    display::drawText(p1, "Bla"s);
    
//    auto what = x1 + y1;
}
