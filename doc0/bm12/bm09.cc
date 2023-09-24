#include <avr/io.h>
#include <avr/interrupt.h>

#include <std/array>
#include <std/compare>

//#define USE_INTERRUPTS

// > Header: isr.h
namespace Interrupt {
    struct ScopedInterruptDisable {
        ScopedInterruptDisable() {
            cli();
        }
        ~ScopedInterruptDisable() {
            SREG = save; 
        }
    private:
        const uint8_t save{SREG};
    };
    
    template<typename T> struct volatile_atomic;
    
    template<typename T>
    requires (sizeof(T) > 1)
    struct volatile_atomic<T> {
        constexpr volatile_atomic() = default;
        volatile_atomic(const volatile_atomic&) = delete;
        void operator=(const volatile_atomic&) = delete;
        
        void operator=(const T v) {
            mValue = v;
        }
        void operator=(const T v) volatile {
            ScopedInterruptDisable di;
            mValue = v;
        }
        void operator++() volatile {
            ScopedInterruptDisable di;
            mValue = mValue + 1; // <> read-modify-write: possible lost-update -> DisableInterrupts
        }
        void operator++() {
            ++mValue;
        }
        void operator+=(const T v) {
            mValue += v;
        }
        void operator+=(const T v) volatile {
            ScopedInterruptDisable di;
            mValue = mValue + v; // <> read-modify-write: possible lost-update -> DisableInterrupt
        }
        operator T() const {
            return mValue;
        }
        operator T() const volatile {
            ScopedInterruptDisable di;
            return mValue;
        }
        template<bool useInterrupts = true>
        T modify(auto f) volatile {
            if constexpr(useInterrupts) {
                ScopedInterruptDisable di;
                const T prev = mValue;
                mValue = f(prev);
                return prev;
            }
            else {
                const T prev = mValue;
                mValue = f(prev);
                return prev;
            }
        }
    private:
        T mValue{}; 
    };
    
    template<typename T>
    requires (sizeof(T) == 1)
    struct volatile_atomic<T> {
        constexpr volatile_atomic() = default;
        volatile_atomic(const volatile_atomic&) = delete;
        void operator=(const volatile_atomic&) = delete;
        
        void operator=(const T v) {
            mValue = v;
        }
        void operator=(const T v) volatile {
            mValue = v;
        }
        void operator++() {
            ++mValue; 
        }
        void operator+=(const T v)  {
            mValue += v;                        
        }
        void operator+=(const T v) volatile {
            ScopedInterruptDisable di;
            mValue += v;                        
        }
        void operator++() volatile {
            ScopedInterruptDisable di;
            ++mValue; // <> read-modify-write: possible lost-update -> DisableInterrupts
        }
        operator T() const {
            return mValue;
        }
        operator T() const volatile {
            return mValue;
        }
        template<bool useInterrupts = true>
        T modify(auto f) volatile {
            if constexpr(useInterrupts) {
                ScopedInterruptDisable di;
                const T prev = mValue;
                mValue = f(prev);
                return prev;
            }
            else {
                const T prev = mValue;
                mValue = f(prev);
                return prev;
            }
        }
        void on(const T v, auto f) volatile {
            ScopedInterruptDisable di;
            if (mValue == v) {
                f(mValue);
            }
        }
        void on(const T v, auto f) {
            if (mValue == v) {
                f(mValue);
            }
        }
    private:
        T mValue{}; 
    };
}
// <

// > Header: meta.h
namespace Meta {
    template<typename... T> struct List;
    
    template<typename F, typename...>
    struct first {
        using type = F;  
    };
    template<typename... T>
    using first_t = first<T...>::type;
    
    template<typename... T>
    struct all_same {
        using f = first<T...>::type;
        static inline constexpr bool value = (std::is_same_v<f, T> && ...);
    };
    template<typename... T>
    static inline constexpr bool all_same_v = all_same<T...>::value;
    
    template<typename S>
    struct make_list;
    
    template<typename T, T... I>
    struct make_list<std::integer_sequence<T, I...>> {
        using type = List<std::integral_constant<T, I>...>;
    };
    
    template<typename S>
    using make_list_t = make_list<S>::type;
}
// <

// > Header: type_traits.h

namespace etl {
    template<auto V>
    struct signedTypeFromValue {
        using type = typename std::conditional<(V > std::numeric_limits<int32_t>::max()), int64_t, 
                              typename std::conditional<(V > std::numeric_limits<int16_t>::max()), int32_t,
                                typename std::conditional<(V > std::numeric_limits<int8_t>::max()), int16_t, int8_t>::type>::type>::type;
    };
    template<auto V>
    using signedTypeFromValue_t = signedTypeFromValue<V>::type;

    template<auto V>
    struct unsignedTypeFromValue {
        using type = typename std::conditional<(V > std::numeric_limits<uint32_t>::max()), uint64_t, 
                              typename std::conditional<(V > std::numeric_limits<uint16_t>::max()), uint32_t,
                                typename std::conditional<(V > std::numeric_limits<uint8_t>::max()), uint16_t, uint8_t>::type>::type>::type;
    };
    template<auto V>
    using unsignedTypeFromValue_t = unsignedTypeFromValue<V>::type;
    
    template<auto V>
    struct typeFromValue {
        using type = std::conditional_t<(V < 0), signedTypeFromValue_t<-V>, unsignedTypeFromValue_t<size_t(V)>>;
    };
    template<auto V>
    using typeFromValue_t = typeFromValue<V>::type;
    
    static_assert(std::is_same_v<typeFromValue_t<1>, uint8_t>);
    static_assert(std::is_same_v<typeFromValue_t<-1>, int8_t>);
    static_assert(std::is_same_v<typeFromValue_t<128>, uint8_t>);
    static_assert(std::is_same_v<typeFromValue_t<-127>, int8_t>);
    static_assert(std::is_same_v<typeFromValue_t<65535>, uint16_t>);
    static_assert(std::is_same_v<typeFromValue_t<-128>, int16_t>);
    
    template<typename T> 
    struct TypeParameter {
        using type = T;
    };
}

// <

// > Header: math.h
namespace Math {
    template<typename T>
    inline constexpr bool isPowerof2(const T v) {
        return v && ((v & (v - 1)) == 0);
    }
    
    template<typename T>
    inline consteval size_t log2(const T v) {
        for(size_t i{}; i < size_t(-1); ++i) {
            if (v == (1 << i)) return i;
        }
        std::unreachable();
    }
    template<typename T>
    concept Unsigned = std::is_unsigned_v<T>;

    template<typename T>
    concept Signed = std::is_signed_v<T>;
}
// <

// > Header : ranged.h
__attribute__((__error__("assert error"))) inline void constant_assert();

template<bool B> using RangeCheck = std::integral_constant<bool, B>;

template<Math::Signed T, T LowerBound, T UpperBound, T InitialValue = LowerBound>
requires((LowerBound <= UpperBound) && (LowerBound <= InitialValue) && (InitialValue <= UpperBound))
struct int_ranged final {
    inline static constexpr T Lower = LowerBound;
    inline static constexpr T Upper = UpperBound;
    inline static constexpr T Mid = (UpperBound + LowerBound) / 2;
    using value_type = T;
 
    constexpr int_ranged() = default;
    
    constexpr explicit int_ranged(const T v, const RangeCheck<true> = RangeCheck<true>{}) {
        if (__builtin_constant_p(v)) {
            if ((v >= Lower) && (v <= Upper)) {
                mValue = v;    
            }
            else {
                constant_assert();
            }
        }
        else {
            assert(v >= LowerBound);
            assert(v <= UpperBound);
            mValue = std::clamp({v}, LowerBound, UpperBound);
        }
    }
    consteval explicit int_ranged(const T v, const RangeCheck<false>) {
        assert(v >= LowerBound);
        assert(v <= UpperBound);
        mValue = v;
    }
    constexpr int_ranged(const int_ranged&) = default;
    constexpr int_ranged(const volatile int_ranged& v) : mValue{v.mValue}{};
    
    constexpr void operator+=(const T rhs) {
        mValue = std::clamp(value_type(mValue + rhs), LowerBound, UpperBound);
    }
    constexpr void operator+=(const T rhs) volatile {
        mValue = std::clamp(value_type(mValue + rhs), LowerBound, UpperBound);
    }
    constexpr void operator/=(const T rhs) {
        mValue = std::clamp(value_type(mValue / rhs), LowerBound, UpperBound);
    }
    constexpr int_ranged& operator=(const int_ranged& rhs) = default;

    template<typename Check = RangeCheck<true>>
    constexpr void set(const T rhs) {
        assert(rhs >= LowerBound);
        assert(rhs <= UpperBound);
        if constexpr(Check::value) {
            if (__builtin_constant_p(rhs)) {
                if ((rhs >= Lower) && (rhs <= Upper)) {
                    mValue = rhs;    
                }
                else {
                    constant_assert();
                }
            }
            else {
                mValue = std::clamp(rhs, LowerBound, UpperBound);
            }
        }
        else {
            mValue = rhs;
        }
    }

    [[deprecated("use set...()")]] constexpr int_ranged& operator=(const T rhs){
        assert(rhs >= LowerBound);
        assert(rhs <= UpperBound);
        if (__builtin_constant_p(rhs)) {
             if ((rhs >= Lower) && (rhs <= Upper)) {
                 mValue = rhs;    
             }
             else {
                 constant_assert();
             }
         }
        else {
             mValue = std::clamp(rhs, LowerBound, UpperBound);
        }
        return *this;
    }
    constexpr operator T() const {
        return mValue;
    }
private: 
    T mValue{InitialValue};
};

template<Math::Unsigned T, T LowerBound, T UpperBound>
requires(LowerBound <= UpperBound)
struct uint_ranged_circular final {
    inline static constexpr size_t module{UpperBound + 1};
    inline static constexpr T module_mask = UpperBound;
    inline static constexpr bool use_mask_modulo = (LowerBound == 0) && (Math::isPowerof2(module));
    inline static constexpr T Lower = LowerBound;
    inline static constexpr T Upper = UpperBound;
    using value_type = T;
    
    inline constexpr uint_ranged_circular() = default;
    constexpr uint_ranged_circular(const uint_ranged_circular& rhs) = default;
    
    constexpr explicit uint_ranged_circular(const T v) requires(use_mask_modulo) : mValue(v & module_mask) {
        assert(v >= LowerBound);
        assert(v <= UpperBound);
    }
    constexpr explicit uint_ranged_circular(const T v) requires(!use_mask_modulo) : mValue(v) {
        assert(v >= LowerBound);
        assert(v <= UpperBound);
        if (v < LowerBound) {
            mValue = LowerBound;
        }
        else if (v > UpperBound) {
            mValue = UpperBound;
        }
    }
    constexpr void operator+=(const T rhs) requires(use_mask_modulo) {
        mValue += rhs;
        mValue &= module_mask;
    }
    inline constexpr operator T() const {
        return mValue;
    }
private: 
    T mValue{LowerBound};
};

template<typename T, auto L, auto U, auto I, typename V>
int_ranged<T, L, U, I> operator/(int_ranged<T, L, U, I> lhs, const V rhs) {
    lhs /= rhs;
    return lhs;
} 

template<typename T, auto L, auto U>
struct std::is_integral<uint_ranged_circular<T, L, U>> : std::true_type {};

template<typename T, auto L, auto U, auto I>
struct std::is_integral<int_ranged<T, L, U, I>> : std::true_type {};

// <

// > Header: concepts.h
namespace Concepts {
    template<typename P>
    concept Pin = requires(P p) {
        P::init();              
        P::pullup();
        P::read();
    };
}
// <

// > Header: mcu.h
struct A;
struct B;
struct C;

template<bool b> struct UseInterrupts : std::integral_constant<bool, b> {};

template<bool b> struct Output : std::integral_constant<bool, b> {};

template<typename C> struct Address;

template<typename P, typename ValueType = uint8_t>
struct Gpio {
    using value_t = ValueType;
    static inline constexpr uint8_t size = 8 * sizeof(value_t);
    static inline auto& port{Address<Gpio<P>>::value};
};

template<>
struct Address<Gpio<A>> {
    static inline auto* const& value{&PORTA};    
};
template<>
struct Address<Gpio<C>> {
    static inline auto* const& value{&PORTC};    
};
// <

// > Header: code.h
template<typename Bits, typename ValueType = uint8_t> struct Gray;

template<uint8_t... N, template<typename...> typename L, typename ValueType>
requires((N < (8 * sizeof(ValueType))) && ...)
struct Gray<L<std::integral_constant<uint8_t, N>...>, ValueType> {
    using value_t = ValueType;
    static inline constexpr size_t size{sizeof...(N)};
    static constexpr value_t binary(const value_t gray) {
        return []<auto... II>(std::index_sequence<II...>, const value_t g){
            return (grayBitToXor<N, II>(g) ^ ...);            
        }(std::make_index_sequence<size>{}, gray);
    }
private:
    using s_value_t = std::make_signed_t<value_t>;
    template<auto B>
    static inline constexpr value_t bitMask{1 << B};
    template<auto K>
    static inline constexpr value_t xorMask{(1 << K) - 1};
    template<auto B, auto I>
    static constexpr s_value_t grayBitToXor(const value_t b) {
        return (b & bitMask<B>) ? xorMask<size - I> : value_t{0};
    }

    static constexpr value_t spreadBits(const uint8_t c){
        std::array<uint8_t, size> a{(1 << N)...};
        value_t r{};
        for(uint8_t i{}; i < (sizeof(value_t) * 8); ++i) {
            if (c & (1U << i)) {
                r |= a[size - i - 1];
            }
        }
        return r;
    }
    static_assert([]{
    std::array<uint8_t, 1 << size> gray;
    for(uint8_t i{}; auto& g : gray) {
        uint8_t sg = i ^ (i >> 1);
        g = spreadBits(sg);
        ++i;
    }
    for(uint8_t i{}; auto g : gray) {
        if (binary(g) != i++) return false;            
    }
    return true;
    }(), "Gray [test 1]: wrong encoding to binary");
};
// <
    
// > Header: gpio.h
template<uint8_t N, typename GPIO, typename Output = Output<false>>
requires(N < GPIO::size)
struct Pin {
    using gpio_t = GPIO;
    using output_t = Output;
    using value_t = GPIO::value_t;    
    static inline constexpr uint8_t number{N};
    static inline constexpr value_t mask{1U << N};
    static inline auto& port{GPIO::port};
    
    static void init() {
        if constexpr(Output::value) {
            port->DIR |= mask;
        }
    }
    static void pullup() {
        pinctl() |= PORT_PULLUPEN_bm;
    }
    static bool read() requires(!Output::value) {
        return port->IN & mask;
    }
private:
    static auto& pinctl() {
        return *(&port->PIN0CTRL + N);
    }
};
    
template<typename... Pins> struct PinGroup;

template<Concepts::Pin... Pins>
requires (Meta::all_same_v<typename Pins::gpio_t...>) && (Meta::all_same_v<typename Pins::output_t...>)
struct PinGroup<Pins...> {
    using pin0_t = Meta::first_t<Pins...>;
    using gpio_t = pin0_t::gpio_t;
    using value_t = gpio_t::value_t;
    using pinNumbers_t = Meta::List<std::integral_constant<uint8_t, Pins::number>...>;

    static inline constexpr value_t mask{((1U << Pins::number) | ...)};
    static inline auto& port{gpio_t::port};
    
    using output_t = pin0_t::output_t;                            
                            
    static void init() {
        if constexpr(output_t::value) {
            port.DIR |= mask;
        }
    }
    static void pullup() {
        port->PINCONFIG = PORT_PULLUPEN_bm;
        port->PINCTRLUPD = mask;
    }
    static value_t read() requires(!output_t::value) {
        return port->IN & mask;
    }
    static value_t raw() requires(!output_t::value) {
        return port->IN;
    }
};

template<Concepts::Pin... Pins>
requires (!Meta::all_same_v<typename Pins::gpio_t...>) && (Meta::all_same_v<typename Pins::output_t...>)
struct PinGroup<Pins...> {
    static_assert(Meta::all_same_v<typename Pins::value_t...>);
    using value_t = Meta::first_t<Pins...>::value_t;
    using s_value_t = std::make_signed_t<value_t>;
    using pinNumbers_t = Meta::make_list_t<std::make_integer_sequence<uint8_t, sizeof...(Pins)>>;
    using output_t = Meta::first_t<Pins...>::output_t;                            
                            
    static void init() {
        (Pins::init(), ...);
    }
    static void pullup() {
        (Pins::pullup(), ...);
    }
                            
    template<auto I>
    static inline constexpr s_value_t xorgen(const bool b) {
        return b ? ((1U << (I + 1)) - 1) : 0;                          
    }

    static inline constexpr s_value_t binaryFromGray() requires(!output_t::value) {
        return []<auto... II>(std::index_sequence<II...>){
            return (xorgen<II>(Pins::read()) ^ ...);
        }(std::make_index_sequence<sizeof...(Pins)>{});
    }
};
// <

// > Header: rotary.h
template<uint8_t N>
requires(Math::isPowerof2(N))
struct TransitionsPerDent : std::integral_constant<uint8_t, N> {
    static inline constexpr uint8_t shift = Math::log2(N);
};

template<typename T> struct AccumulatorType : etl::TypeParameter<T> {};
template<typename T> struct ValueType : etl::TypeParameter<T> {};

template<typename PGroup, typename AccumulatorType, typename ValueType, typename Trans = TransitionsPerDent<4>, typename UuseInts = UseInterrupts<false>>
struct RotaryEncoder;

template<typename PGroup, typename accu_t, typename value_t, typename Trans, bool useInterrupts>
requires (std::is_signed_v<accu_t> && (std::is_integral_v<value_t> || std::is_same_v<value_t, void>))
struct RotaryEncoder<PGroup, AccumulatorType<accu_t>, ValueType<value_t>, Trans, UseInterrupts<useInterrupts>> {
    
    using s_value_t = std::conditional_t<std::is_same_v<value_t, void>, accu_t, value_t>;

    static void init() {
        PGroup::init();
        PGroup::pullup();
    }    
    class ASync {
        static void update() {
            const auto newState = []{
                if constexpr(requires{PGroup::binaryFromGray();}) {
                    return PGroup::binaryFromGray();
                }
                else {
                    using gray = Gray<typename PGroup::pinNumbers_t, typename PGroup::value_t>;
                    return gray::binary(PGroup::raw());
                }
            }();
            const int8_t diff = mLastState - newState; 
            if (diff & 0b01) {  // one-step change (maybe three or more)
                mLastState = newState;                    
                mAccu += accu_t((diff & 0b10) - 1);   // map: 0,2 -> -1,1
            }        
        }
        static inline Interrupt::volatile_atomic<accu_t> mAccu{};
    public:
        static void isr() {
            update(); 
        }
        static inline volatile auto& vAccu{mAccu};
    };

    static void periodic() requires(!useInterrupts) {
        ASync::isr();
    }
    static s_value_t value() {
        if constexpr(std::is_same_v<value_t, void>) {
            if constexpr(Trans::value != 1) {
                const accu_t v = ASync::vAccu;
                return v / Trans::value;
            }
            else {
                return ASync::vAccu;
            }
        }
        else {
            if constexpr(Trans::value != 1) {
                mValue += (ASync::vAccu.template modify<useInterrupts>([](const accu_t v){
                    return v & (Trans::value - 1); 
                }) >> Trans::shift);
            }
            else {
                mValue += ASync::vAccu.template modify<useInterrupts>([](const accu_t v){
                    return 0;
                });
            }
            return mValue;
        }                                                              
    }
private:
    static inline uint8_t mLastState{};
    static inline s_value_t mValue{};
};
// <

// > main.cc
using pa5 = Pin<5, Gpio<A>>;
using pa6 = Pin<6, Gpio<A>>;
using pc0 = Pin<0, Gpio<C>>;

using rotaryPins = PinGroup<pa5, pa6>;

#ifdef USE_INTERRUPTS
using rot = RotaryEncoder<rotaryPins, AccumulatorType<int8_t>, ValueType<int32_t>, TransitionsPerDent<2>, UseInterrupts<true>>;
#else
using r_t = int_ranged<int32_t, -5, 5, 0>;
using u_t = uint_ranged_circular<uint8_t, 0, 7>;
//using rot = RotaryEncoder<rotaryPins, AccumulatorType<int8_t>, ValueType<int32_t>, TransitionsPerDent<2>, UseInterrupts<false>>;
using rot = RotaryEncoder<rotaryPins, AccumulatorType<int8_t>, ValueType<r_t>, TransitionsPerDent<2>, UseInterrupts<false>>;
//using rot = RotaryEncoder<rotaryPins, AccumulatorType<int32_t>, ValueType<void>, TransitionsPerDent<2>, UseInterrupts<false>>;
//using rot = RotaryEncoder<rotaryPins, AccumulatorType<r_t>, ValueType<void>, TransitionsPerDent<2>, UseInterrupts<false>>;
//using rot = RotaryEncoder<rotaryPins, AccumulatorType<int8_t>, ValueType<u_t>, TransitionsPerDent<2>, UseInterrupts<false>>;
#endif

#define LEDS_DDR    PORTD_DIR
#define LEDS        PORTD_OUT

int main() {
#ifdef USE_INTERRUPTS
    TCB0.INTCTRL = TCB_CAPT_bm;
    TCB0.CCMP = 4000; // 1ms
    TCB0.CTRLA = TCB_ENABLE_bm;
    sei();
#endif
    LEDS_DDR = 0xff;
    
    rot::init();

    while(true) {
#ifndef USE_INTERRUPTS
        rot::periodic();
#endif
        LEDS = rot::value();
    }
}

#ifdef USE_INTERRUPTS
ISR(TCB0_INT_vect) {       
    TCB0.INTFLAGS = TCB_CAPT_bm;
    rot::ASync::isr();
}
#endif

// <
