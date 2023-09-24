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
    static inline value_t* const multiCtl = reinterpret_cast<value_t*>(Address<Gpio<P>>::multiCtl);
    static inline value_t* const multiUpd = reinterpret_cast<value_t*>(Address<Gpio<P>>::multiUpd);
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

template<typename PinA, typename PinB, typename AccumulatorType, typename ValueType, typename Trans = TransitionsPerDent<4>, bool useInterrupts = false>
struct RotaryEncoder {
    using value_t = ValueType;
    using accu_t  = AccumulatorType;
    static void init() {
        PinA::init();
        PinB::init();
    }    
    static void periodic() {
        const uint8_t newState = []{
            uint8_t v{};
            if (PinA::read()) v  = 0b11;
            if (PinB::read()) v ^= 0b01;   // convert gray to binary
            return v;
        }(); 
        const int8_t diff = mLastState - newState;               // difference last - new
        if (diff & 0b01) {                // bit 0 = value (1)
            mLastState = newState;                    // store new as next last
            mAccu += (diff & 0b10) - 1;   // bit 1 = direction (+/-)
        }
    }
    static value_t value() {
        if constexpr(useInterrupts) {
            ScopedInterruptDisable di;
            mValue += volatile_access(mAccu) / Trans::value;
            volatile_access(mAccu) = 0;
        }
        else {
            mValue += mAccu / Trans::value;
            mAccu = 0;
        }
        return mValue;
    }
private:
    static inline uint8_t mLastState{};
    static inline accu_t mAccu{};
    static inline value_t mValue{};
};

using pa5 = Pin<5, Gpio<A>, Output<false>>;
using pa6 = Pin<6, Gpio<A>, Output<false>>;

#ifdef USE_INTERRUPTS
using rot = RotaryEncoder<pa5, pa6, uint8_t, int32_t, TransitionsPerDent<2>, true>;
#else
using rot = RotaryEncoder<pa5, pa6, uint8_t, int32_t, TransitionsPerDent<2>, false>;
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
        int32_t count{};
#ifndef USE_INTERRUPTS
        rot::periodic();
#endif
        count = rot::value();
    }
}

#ifdef USE_INTERRUPTS
ISR(TCB0_INT_vect) {       
    TCB0.INTFLAGS = TCB_CAPT_bm;
    rot::periodic();
}
#endif
