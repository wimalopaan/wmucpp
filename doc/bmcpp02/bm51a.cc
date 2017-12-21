unsigned volatile PORTB;

template<int p>
struct Port
{
  enum class Value : unsigned { High, Low };
  static inline void set (Value value) {
      if (Value::High == value)
        PORTB |= (1 << p);
      else
        PORTB &= ~(1 << p);
  }
  static constexpr Value High = Value::High;
  static constexpr Value Low = Value::Low;
};


int main() {
  Port<7>::set (Port<7>::Value::Low);
  Port<7>::set (PORTB ? Port<7>::Value::Low : Port<7>::Value::High);
}


