#include <avr/io.h>
#include <avr/interrupt.h>

#include <std/utility>
#include <std/array>

#define USE_INTERRUPTS

template<auto... N> struct List;

template<typename T>
volatile T& volatile_access(T& v) {
    return static_cast<volatile T&>(v);
}

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

struct A;
struct B;
struct C;

template<bool b>
struct Output {
    static inline constexpr bool value{b};
};

template<typename C> struct Address;

template<typename P, typename ValueType = uint8_t>
struct Gpio {
    using value_t = ValueType;
    static inline value_t* const in = reinterpret_cast<value_t*>(Address<Gpio<P>>::in);
    static inline value_t* const multiCtl = reinterpret_cast<value_t*>(Address<Gpio<P>>::multiCtl);
    static inline value_t* const multiUpd = reinterpret_cast<value_t*>(Address<Gpio<P>>::multiUpd);
};

#ifdef __AVR_ARCH__
# undef _MMIO_BYTE
# define _MMIO_BYTE(adr) ((uintptr_t)(adr))
# define _MMIO_BYTE_CHANGED
#endif

template<>
struct Address<Gpio<A>> {
    static inline constexpr uintptr_t in  = PORTA_IN;    
    static inline constexpr uintptr_t out = PORTA_OUT;      
    static inline constexpr uintptr_t dir = PORTA_DIR;      
    static inline constexpr uintptr_t multiCtl = PORTA_PINCONFIG;      
    static inline constexpr uintptr_t multiUpd = PORTA_PINCTRLUPD;      
};
template<>
struct Address<Gpio<C>> {
    static inline constexpr uintptr_t in  = PORTC_IN;    
    static inline constexpr uintptr_t out = PORTC_OUT;      
    static inline constexpr uintptr_t dir = PORTC_DIR;      
};

#ifdef _MMIO_BYTE_CHANGED
# undef _MMIO_BYTE
# define _MMIO_BYTE(addr) (*(volatile uint8_t*)(addr))
#endif

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
    }());
};

template<typename Pins, typename GPIO, typename Output>
struct PinGroup;

template<auto... N, template<auto...> typename L, typename GPIO, typename Output>
requires((N < (8 * sizeof(typename GPIO::value_t))) && ...)
struct PinGroup<L<N...>, GPIO, Output> {
    using value_t = typename GPIO::value_t;
    using pins_t = List<N...>;
    
    static inline constexpr value_t mask{((1U << N) | ...)};
    static inline value_t* const& gpioIn  = GPIO::in;
    static inline value_t* const& gpioOut = GPIO::out;
    static inline value_t* const& gpioDir = GPIO::dir;
    static inline value_t* const& gpioMultiCtl = GPIO::multiCtl;
    static inline value_t* const& gpioMultiUpd = GPIO::multiUpd;
    
    static void init() {
        if constexpr(Output::value) {
            volatile_access(gpioDir) = volatile_access(gpioDir) | mask;
        }
    }
    static void pullup() {
        volatile_access(*gpioMultiCtl) = PORT_PULLUPEN_bm;    
        volatile_access(*gpioMultiUpd) = mask;    
    }
    static value_t read() requires(!Output::value) {
        return volatile_access(*gpioIn) & mask;
    }
    static value_t raw() requires(!Output::value) {
        return volatile_access(*gpioIn);
    }
};

struct ScopedInterruptDisable {
    ScopedInterruptDisable() {
        cli();
    }
    ~ScopedInterruptDisable() {
        SREG = save; // prevents reorder among other volatile accesses, but no memory barrier
    }
private:
    const uint8_t save{SREG};
};

template<uint8_t N>
requires(isPowerof2(N))
struct TransitionsPerDent {
    static inline constexpr uint8_t value{N};
};

template<typename PGroup, typename AccumulatorType, typename ValueType, typename Trans = TransitionsPerDent<4>, bool useInterrupts = false>
struct RotaryEncoder {
    using value_t = ValueType;
    using accu_t  = AccumulatorType;
    using gray = Gray<typename PGroup::pins_t, typename PGroup::value_t>;
//    using gray = Gray<typename PGroup::pins_t, int8_t>;
    
    static void init() {
        PGroup::init();
        PGroup::pullup();
    }    
    
    static void isr() requires(useInterrupts){
        update(); // used
    }

    static void periodic() requires(!useInterrupts) {
        update();
    }
    static value_t value() {
        static constexpr uint8_t shift = log2(Trans::value);
        if constexpr(useInterrupts) {
            if constexpr(Trans::value != 1) {
                int8_t x ;
                {
                    ScopedInterruptDisable di;
                    x = volatile_access(mAccu);
                    volatile_access(mAccu) = x & (Trans::value - 1); 
                }
                mValue += x >> shift;
            }
            else {
                mValue += volatile_access(mAccu);
                volatile_access(mAccu) = 0;                
            }
        }
        else {
            if constexpr(Trans::value != 1) {
                mValue += (mAccu >> shift);
                mAccu &= (Trans::value - 1); 
            }
            else {
                mValue += mAccu;
                mAccu = 0;                
            }
        }
        return mValue;
    }
private:
    static inline void update() {
//        uint8_t tmp = PGroup::raw();
//        int8_t newState = 0;
//        if ( tmp & (1U << 5) ) newState = 3u;
//        if ( tmp & (1U << 6) ) newState ^= 1u;   // convert gray to binary
        const auto newState = gray::binary(PGroup::raw());
        const int8_t diff = mLastState - newState; 
        if (diff & 0b01) {  // one-step change (maybe three or more)
            mLastState = newState;                    
            mAccu += (diff & 0b10) - 1;   // map: 0,2 -> -1,1
        }        
    }
    static inline uint8_t mLastState{};
    static inline accu_t mAccu{};
    static inline value_t mValue{};
};

using rotaryPins = PinGroup<List<5, 6>, Gpio<A>, Output<false>>;

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
    rot::isr();
}
#endif
