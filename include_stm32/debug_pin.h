#pragma once

namespace Debug {
    template<typename Pin> struct Scoped;

    template<> struct Scoped<void> {};

    template<typename Pin>
    struct Scoped {
        Scoped() {
            Pin::set();
        }
        ~Scoped() {
            Pin::reset();
        }
    };

    template<typename P> struct Pin;
    template<> struct Pin<void> {
        static inline void set() {
        }
        static inline void reset() {
        }
    };

    template<typename P>
    struct Pin {
        static inline void set() {
            P::set();
        }
        static inline void reset() {
            P::reset();
        }
    };
}
