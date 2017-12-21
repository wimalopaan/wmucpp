unsigned volatile PORTB;

template<int p>
struct Port {
    enum class Value : unsigned { High, Low };
    static inline void set (Value);
    static constexpr Value High = Value::High;
    static constexpr Value Low = Value::Low;
};

//template<> 
//constexpr Port<7>::Value Port<7>::High;
//template<> 
//constexpr Port<7>::Value Port<7>::Low;

template<>
inline void Port<7>::set(Port<7>::Value value) {
    if (Value::High == value)
        PORTB |= 2;
    else
        PORTB &= ~2;
}

int main() {
    Port<7>::set(Port<7>::Value::Low);
    Port<7>::set(PORTB ? Port<7>::Value::Low : Port<7>::Value::High);
}


