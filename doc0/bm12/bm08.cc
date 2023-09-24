#include <avr/io.h>
#include <avr/interrupt.h>

#include <std/array>

#define USE_INTERRUPTS

namespace Interrupt {
    struct ScopedInterruptDisable {
        ScopedInterruptDisable() {
            asm volatile("cli" : : :); // prevents reorder among other volatile accesses, but no memory barrier
        }
        ~ScopedInterruptDisable() {
            SREG = save; // prevents reorder among other volatile accesses, but no memory barrier
        }
    private:
        const uint8_t save{SREG};
    };
    
    template<typename T>
    class volatile_atomic;
    
    template<typename T>
    requires (sizeof(T) > 1)
    class volatile_atomic<T> {
    public:
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
        void modify(auto f) volatile {
            if constexpr(useInterrupts) {
                ScopedInterruptDisable di;
                mValue = f(mValue);
            }
            else {
                mValue = f(mValue);
            }
        }
    private:
        T mValue{}; 
    };
    
    template<typename T>
    requires (sizeof(T) == 1)
    class volatile_atomic<T> {
    public:
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
        void modify(auto f) volatile {
            if constexpr(useInterrupts) {
                ScopedInterruptDisable di;
                mValue = f(mValue);
            }
            else {
                mValue = f(mValue);
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

namespace Meta {
    template<auto... N> struct List;
}

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
}

namespace AVR {
    
}

struct A;
struct B;
struct C;

template<bool b>
struct Output : std::integral_constant<bool, b> {};

template<typename C> struct Address;

template<typename P, typename ValueType = uint8_t>
struct Gpio {
    using value_t = ValueType;
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

template<typename Bits, typename ValueType = uint8_t>
struct Gray;

template<auto... N, template<auto...> typename L, typename ValueType>
requires((N < (8 * sizeof(ValueType))) && ...)
struct Gray<L<N...>, ValueType> {
    using value_t = ValueType;
    static inline constexpr size_t size{sizeof...(N)};
    static constexpr value_t binary(const value_t gray) {
        return []<auto... II>(std::index_sequence<II...>, const value_t g){
            return (grayBitToXor<N, II>(g) ^ ...);            
        }(std::make_index_sequence<size>{}, gray);
    }
private:
    template<auto B>
    static inline constexpr value_t bitMask_v{1 << B};
    template<auto K>
    static inline constexpr value_t xorMask_v{(1 << K) - 1};

    template<auto B, auto I>
    static constexpr value_t grayBitToXor(const value_t b) {
        return (b & bitMask_v<B>) ? xorMask_v<size - I> : value_t{0};
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
    }());
};

template<typename Pins, typename GPIO, typename Output>
struct PinGroup;

template<auto... N, template<auto...> typename L, typename GPIO, typename Output>
requires((N < (8 * sizeof(typename GPIO::value_t))) && ...)
struct PinGroup<L<N...>, GPIO, Output> {
    using value_t = typename GPIO::value_t;
    using pinNumbers_t = Meta::List<N...>;
    
    static inline constexpr value_t mask{((1U << N) | ...)};
    static inline auto& port{GPIO::port};
    
    static void init() {
        if constexpr(Output::value) {
            port.DIR |= mask;
        }
    }
    static void pullup() {
        port->PINCONFIG = PORT_PULLUPEN_bm;
        port->PINCTRLUPD = mask;
    }
    static value_t read() requires(!Output::value) {
        return port->IN & mask;
    }
    static value_t raw() requires(!Output::value) {
        return port->IN;
    }
};

template<uint8_t N>
requires(Math::isPowerof2(N))
struct TransitionsPerDent : std::integral_constant<uint8_t, N> {
    static inline constexpr uint8_t shift = Math::log2(N);
};

template<typename PGroup, typename AccumulatorType, typename ValueType, typename Trans = TransitionsPerDent<4>, bool useInterrupts = false>
struct RotaryEncoder {
    using value_t = ValueType;
    using accu_t  = AccumulatorType;
    using gray = Gray<typename PGroup::pinNumbers_t, typename PGroup::value_t>;
    
    static void init() {
        PGroup::init();
        PGroup::pullup();
    }    

    struct ASync {
        static void isr() {
            update(); 
        }
    private:
        static inline void update() {
            const auto newState = gray::binary(PGroup::raw());
            const int8_t diff = mLastState - newState; 
            if (diff & 0b01) {  // one-step change (maybe three or more)
                mLastState = newState;                    
                mAccu += (diff & 0b10) - 1;   // map: 0,2 -> -1,1
            }        
        }
        static inline Interrupt::volatile_atomic<accu_t> mAccu{};
    public:
        static inline volatile auto& vAccu{mAccu};
    };

    static void periodic() requires(!useInterrupts) {
        ASync::isr();
    }
    static value_t value() {
        if constexpr(Trans::value != 1) {
            ASync::vAccu.template modify<useInterrupts>([](const accu_t v){
                mValue += (v >> Trans::shift);
                return v & (Trans::value - 1); 
            });
        }
        else {
            ASync::vAccu.template modify<useInterrupts>([](const accu_t v){
                mValue += v;
                return 0;
            });
        }
        return mValue;
    }
private:
    static inline uint8_t mLastState{};
    static inline value_t mValue{};
};

using rotaryPins = PinGroup<Meta::List<5, 6>, Gpio<A>, Output<false>>;

#ifdef USE_INTERRUPTS
using rot = RotaryEncoder<rotaryPins, int8_t, int32_t, TransitionsPerDent<2>, true>;
#else
using rot = RotaryEncoder<rotaryPins, int8_t, int32_t, TransitionsPerDent<2>, false>;
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
