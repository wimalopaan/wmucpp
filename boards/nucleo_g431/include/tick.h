#pragma once

#include <chrono>
#include <compare>

namespace External {
#if 0
    template<typename T = uint8_t, T MAX = std::numeric_limits<T>::max()>
    struct CountDown {
        using value_type = etl::uint_ranged<T, 0, MAX>;
        
        inline constexpr CountDown() = default;
        
        inline constexpr CountDown& operator++() {
            ++mValue;
            return *this;
        }

        template<typename F>
        inline constexpr void on(const T& t, const F f) {
            if (mValue >= t) {
                f();
                reset();
            }
        }
        inline constexpr void reset() {
            mValue.setToBottom();
        }
        
    private:
        value_type mValue{};
    };
#endif
    template<typename Timer, typename T = uint16_t, T MAX = std::numeric_limits<T>::max()>
    struct Tick {
        inline static constexpr auto intervall = Timer::intervall;
        
        inline constexpr Tick() = default;
        
        template<typename R, typename P>
        inline constexpr explicit Tick(const std::chrono::duration<R, P>& v) : value(v / intervall) {
//            static_assert(v / intervall <= std::numeric_limits<T>::max());
        }
        
        template<typename R, typename P>
        inline constexpr void operator=(const std::chrono::duration<R, P>& v) {
            value.set(v / intervall);
        }  
        
        inline constexpr auto time() const {
            return intervall * value.toInt();
        }
        
        inline explicit constexpr operator bool() const {
            return value != 0;
        }
        inline constexpr Tick& operator++() {
            ++value;
            return *this;
        }
        inline constexpr Tick& operator--() {
            --value;
            return *this;
        }

        inline constexpr void operator++() volatile {
            ++value;
        }
        
        template<typename F>
        inline constexpr void on(const Tick& t, F f) {
            if (value >= t.value) {
                f();
                reset();
            }
        }
        template<typename F>
        inline constexpr void on(const Tick& t, F f) volatile {
            if (value >= t.value) {
                f();
                reset();
            }
        }
        template<typename F>
        inline constexpr void match(const Tick& t, F f) const{
            if (value == t.value) {
                f();
            }
        }

        template<typename F>
        inline constexpr void testForNext(const Tick& t, F f) {
            if (value == t.value) {
                if (f()) {
                    reset();
                }
            }
            else {
                ++value;
            }
        }
        
        inline constexpr auto operator<=>(const Tick& rhs) const = default;
        
        inline constexpr void reset() {
            value.toBottom();
        }

        inline constexpr void reset() volatile {
            value.toBottom();
        }

        inline static constexpr Tick fromRaw(const T v) {
            return Tick(v);
        }

        inline static constexpr T max() {
            return decltype(value)::Upper;
        }
        
        inline constexpr auto operator/(const uint16_t& d) const {
            return Tick(value / d);    
        }

        inline constexpr auto operator*(const uint16_t& m) const {
            return Tick(value * m);    
        }
        
    private:
        etl::ranged<0u, MAX> value;
        
    private:
        inline constexpr Tick(const T v) : value{v} {}
    };
}
