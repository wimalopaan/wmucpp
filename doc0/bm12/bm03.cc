#include <avr/io.h>
#include <avr/interrupt.h>

#define USE_INTERRUPTS

#define F_TIMER1        100          // Timer 1 frequency /Hz

#ifdef __AVR_ARCH__
# undef _MMIO_BYTE
# define _MMIO_BYTE(adr) ((uintptr_t)(adr))
# define _MMIO_BYTE_CHANGED
#endif

template<typename T>
volatile T& volatile_access(T& v) {
    return static_cast<volatile T&>(v);
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
};

template<>
struct Address<Gpio<A>> {
    static inline constexpr uintptr_t in  = PORTA_IN;    
    static inline constexpr uintptr_t out = PORTA_OUT;      
    static inline constexpr uintptr_t dir = PORTA_DIR;      
    static inline constexpr uintptr_t multiCtl = PORTA_PINCONFIG;      
    static inline constexpr uintptr_t multiUpd = PORTA_PINCTRLUPD;      
};

template<uint8_t N, typename GPIO, typename Output>
struct Pin {
    static inline constexpr GPIO::value_t mask{1U << N};
    static inline GPIO::value_t* const& gpioIn  = GPIO::in;
    static inline GPIO::value_t* const& gpioOut = GPIO::out;
    static inline GPIO::value_t* const& gpioDir = GPIO::dir;
    static void init() {
        if constexpr(Output::value) {
            volatile_access(gpioDir) = volatile_access(gpioDir) | mask;
        }
    }
    static bool read() requires(!Output::value) {
        return volatile_access(*gpioIn) & mask;
    }
};

#ifdef _MMIO_BYTE_CHANGED
# undef _MMIO_BYTE
# define _MMIO_BYTE(addr) (*(volatile uint8_t*)(addr))
#endif

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



template<uint8_t N>
struct TransitionsPerDent {
    static inline constexpr uint8_t value{N};
};

template<typename PinA, typename PinB, typename ValueType, typename Trans = TransitionsPerDent<4>, bool useInterrupts = false>
struct RotaryEncoder {
    using value_t = ValueType;
    static void init() {
        PinA::init();
        PinB::init();
    }    
    static void periodic() requires(useInterrupts){
        const uint8_t newState = (PinA::read() ? 2 : 0) + (PinB::read() ? 1 : 0);    
        switch (mLastState) {
        case 0:
            if (newState == 1) {
                volatile_access(mValue) += 1;
            }
            else if (newState == 2) {
                volatile_access(mValue) -= 1;
            }
            break;
        case 1:
            if (newState == 0) {
                volatile_access(mValue) -= 1;
            }
            break;
        case 2:
            if (newState == 0) {
                volatile_access(mValue) += 1;
            }
            break;
        case 3:
            break;
        default:
            __builtin_unreachable();
        break;
        }
        mLastState = newState;
    }
    static void periodic() requires(!useInterrupts){
        const uint8_t newState = (PinA::read() ? 2 : 0) + (PinB::read() ? 1 : 0);    
        switch (mLastState) {
        case 0:
            if (newState == 1) {
                mValue += 1;
            }
            else if (newState == 2) {
                mValue -= 1;
            }
            break;
        case 1:
            if (newState == 0) {
                mValue -= 1;
            }
            break;
        case 2:
            if (newState == 0) {
                mValue += 1;
            }
            break;
        case 3:
            break;
        default:
            __builtin_unreachable();
        break;
        }
        mLastState = newState;
    }
    static value_t value() {
        if constexpr(useInterrupts) {
            ScopedInterruptDisable di;
            return volatile_access(mValue) / Trans::value;
        }
        else {
            return mValue / Trans::value;
        }
    }
private:
    static inline int8_t mLastState{};
    static inline value_t mValue{};
};

using pa5 = Pin<5, Gpio<A>, Output<false>>;
using pa6 = Pin<6, Gpio<A>, Output<false>>;

#ifdef USE_INTERRUPTS
using rot = RotaryEncoder<pa5, pa6, uint32_t, TransitionsPerDent<4>, true>;
#else
using rot = RotaryEncoder<pc2, pc4, uint32_t, TransitionsPerDent<4>, false>;
#endif

int main() {

    rot::init();
    while(true) {
        int32_t count{};
#ifndef USE_INTERRUPTS
        rot::periodic();
#endif
        count = rot::value();
    }
}

#ifdef USE_INTERRUPTS
ISR( TIMER1_COMPA_vect ) {           // 1ms for manual movement
    rot::periodic();
}
#endif
