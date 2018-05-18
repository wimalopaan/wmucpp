#define NDEBUG

#include <stdio.h>
#include <optional>
#include <cassert>
#include <cstddef>
#include <algorithm>

struct AsciiChar {
    explicit inline AsciiChar(unsigned char v = 0) : value(v) {
        assert(value < 128);
    }
    unsigned char value {};
};

template<typename T> inline std::optional<T> get();

template<>
inline std::optional<AsciiChar> get<AsciiChar>() {
    if (int c = getchar(); c != EOF) {
        return AsciiChar{static_cast<unsigned char>(c)};
    }    
    return {};
}
template<>
inline std::optional<std::byte> get<std::byte>() {
    if (int c = getchar(); c != EOF) {
        return std::byte(c);
    }    
    return {};
}

inline bool put(AsciiChar c) {
    return putchar(c.value) != EOF;
}

inline bool put(std::byte c) {
    return putchar(static_cast<int>(c)) != EOF;
}

template<typename T>
struct Console {
    struct Input {
        struct Iterator {
            explicit inline Iterator(bool) {}   
            inline Iterator() = default;
            inline bool operator!=(const Iterator& rhs) {
                c = getchar();
                return c != rhs.c;
            }
            inline Iterator& operator++() {
                return *this;
            }
            inline Iterator operator++(int) {
                return *this;
            }
            inline T operator*() const {
                return T(c);    
            }
        private:
            int c = EOF;
        };  
        inline Iterator begin() const {
            return Iterator(true);    
        }
        inline Iterator end() const {
            return Iterator();    
        }
    };
    struct Output {
        struct Iterator {
            inline explicit Iterator(typename Console::Output& c) : console(c) {}
            inline Iterator& operator++() {
                return *this;
            }
            inline Iterator operator++(int) {
                return *this;
            }
            inline Iterator& operator*() {
                return *this;
            }
            inline Iterator& operator=(T c) {
                if (putchar((int)c) == EOF) {
                    console.failed = true;
                }
                return *this;
            }
        private:
            typename Console::Output& console;
        };
        inline typename Output::Iterator begin() {
            return typename Output::Iterator(*this);        
        }
        inline bool fail() const {
            return failed;
        }
    private:
        bool failed = false;
    };
};

int main(){
//    Console<std::byte>::Input input;
//    Console<std::byte>::Output output;
//    std::copy(std::begin(input), std::end(input), std::begin(output));
//    return output.fail();
    
    while(const auto c = get<AsciiChar>()) {
        if (!put(*c)) {
            return 1;
        }
    }
//    while(auto c = get<std::byte>()) {
//        if (!put(*c)) {
//            return 1;
//        }
//    }
    return 0;
}
