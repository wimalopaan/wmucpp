namespace std {
    enum class byte : unsigned char {};
}

struct Uart {
    inline static volatile std::byte txd;    
    inline static volatile std::byte rxd;    
};

int main() {
  constexpr auto EXIT{'~'};

  char c{};
  while (c = char(Uart::rxd)) {
    switch (c) {
      case '.':
      case ',':
      case ';':
      case '-':
      case '_':
      case '!':
      case '?':
        c = ' ';
        break;
      case EXIT:
        Uart::txd = std::byte('@');
        return 0;
    }
    Uart::txd = std::byte{c};
  }
}
