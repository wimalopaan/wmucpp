#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stddef.h>

// üblicherweise aus der C++-Standardbibliothek <utility>
namespace std {
    template<auto... II> struct index_sequence {};
    
    template<typename T, T V> struct integral_constant {
        using type = T;
        static inline constexpr T value{V};
    };
    
    namespace detail {
        template<typename, auto> struct add;
        template<auto... II, auto I>
        struct add<index_sequence<II...>, I> {
            using type = index_sequence<II..., I>;  
        };
        
        template<auto N>
        struct make_index_sequence {
            using type = add<typename make_index_sequence<N-1>::type, N>::type;
        };
        template<>
        struct make_index_sequence<0> {
            using type = index_sequence<0>;  
        };
    }
    template<auto N>
    using make_index_sequence = typename detail::make_index_sequence<N - 1>::type;
    
    template<typename T1, typename T2>
    static constexpr bool is_same_v = std::integral_constant<bool, false>::value;

    template<typename T>
    static constexpr bool is_same_v<T, T> = std::integral_constant<bool, true>::value;
    
    template<typename T>
    concept Unsigned = std::is_same_v<T, uint8_t> || std::is_same_v<T, uint16_t> || std::is_same_v<T, uint32_t>;
}

namespace Bcd {
    template<auto N>
    struct Digit {
        static inline constexpr uint8_t position{N};
        const uint8_t value{};
    };
    template<auto N>
    struct Unpacked {
        const uint8_t digits[N]{};
        template<auto I>
        constexpr Digit<I> get() const {
            return {digits[I]};
        }
    };
    
    template<typename> struct numberOfDigits;
    template<> struct numberOfDigits<uint8_t> : std::integral_constant<uint8_t, 3> {};
    template<> struct numberOfDigits<uint16_t> : std::integral_constant<uint8_t, 5> {};
    template<> struct numberOfDigits<uint32_t> : std::integral_constant<uint8_t, 10> {};
    
    template<auto N, std::Unsigned T>
    requires (N > 0) && (N <= numberOfDigits<T>::value)
    constexpr Unpacked<N> toUnpacked(const T display_value) {
        T remaining{display_value};
        auto getLastDigit = [&](auto){
            const uint8_t v = remaining % 10;
            remaining /= 10;
            return v;
        };
        auto makeUnpacked = [&]<auto... II>(std::index_sequence<II...>){
            return Unpacked<N>{getLastDigit(II)...}; 
        };
        return makeUnpacked(std::make_index_sequence<N>{});
    }
}
namespace std {
    template<typename> struct tuple_size; // üblicherweise aus <tuple>

    template<auto N>
    struct tuple_size<const Bcd::Unpacked<N>> { // Spezalisierung für structured-binding
        static inline constexpr uint8_t value{N}; 
    };
    
    template<auto, typename> struct tuple_element; // üblicherweise aus <tuple>
    
    template<auto I, auto N>
    struct tuple_element<I, const Bcd::Unpacked<N>> { // Spezalisierung für structured-binding
        using type = const typename Bcd::Digit<I>;
    };
}

namespace Bcd::tests {
    static_assert([]{
    const auto [d0, d1, d2, d3, d4, d5, d6, d7, d8, d9] = Bcd::toUnpacked<10>(123456789u);
        if (d4.value != 5) return false;
        if (d3.value != 6) return false;
        if (d2.value != 7) return false;
        if (d1.value != 8) return false;
        if (d0.value != 9) return false;
        return true;
    }(), "Test0 failed");
    static_assert([]{
        const auto [d0, d1, d2, d3, d4] = Bcd::toUnpacked<5>(345u);
        if (d4.value != 0) return false;
        if (d3.value != 0) return false;
        if (d2.value != 3) return false;
        if (d1.value != 4) return false;
        if (d0.value != 5) return false;
        return true;
    }(), "Test1 failed");
    static_assert([]{
        const auto [d0, d1, d2] = toUnpacked<3>(987u);
        if (d2.value != 9) return false;
        if (d1.value != 8) return false;
        if (d0.value != 7) return false;
        return true;
    }(), "Test2 failed");
}

namespace {
    constexpr uint8_t T1{0xFE};  // Basis Transistor 1 (PNP) Einerstelle
    constexpr uint8_t T2{0xFD};  // Basis Transistor 2  Zehnerstelle
    constexpr uint8_t T3{0xFB};  // Basis Transistor 3 Hunderterstelle

    template<auto N>
    constexpr uint8_t segments(const Bcd::Digit<N> d) {
        static constexpr uint8_t segmenttable[] { 0x03, 0xF3, 0x25, 0x0D, 0x99, 0x49, 0x41, 0x1F, 0x01, 0x19, 0xFE } ;
        return segmenttable[d.value];
    }
    
    void init() {
        DDRD  = 0xFF; // PORTD auf Ausgang setzen
        DDRB  = 0xFF;
        PORTD = 0xFF; //Ausgabeport für 7-Segment-Ziffern
        PORTB = 0xFF; // Ansteuerung von T1 bis T3
    }
    
    template<typename T>
    struct FSM {
        enum class State : uint8_t {Digit0, Digit1, Digit2};
        
        static inline void ratePeriodic() {
            const auto oldState{mState};
            switch(mState) {
            case State::Digit0:
                mState = State::Digit1;
                break;
            case State::Digit1:
                mState = State::Digit2;
                break;
            case State::Digit2:
                mState = State::Digit0;
                break;
            }
            if (oldState != mState) {
                const auto [d0, d1, d2] = Bcd::toUnpacked<3>(mValue);
                switch(mState) {
                case State::Digit0:
                    PORTB = T1;
                    PORTD = segments(d0);
                    break;
                case State::Digit1:
                    PORTB = T2;
                    PORTD = segments(d1);
                    break;
                case State::Digit2:
                    PORTB = T3;
                    PORTD = segments(d2);
                    break;
                }
            }
        }
        static inline void value(const T v) {
            mValue = v;
        }
    private:
        static inline T mValue;
        static inline State mState{State::Digit0};
    };
}
int main() {
    using fsm = FSM<uint16_t>;
    init();
    fsm::value(345);
    while(true) {
        fsm::ratePeriodic();
        _delay_ms(7);
    }
}

