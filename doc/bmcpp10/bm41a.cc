#define NDEBUG

#include <stdio.h>
#include <optional>
#include <cassert>
#include <cstddef>
#include <algorithm>

struct Ressource {
    uint8_t data;
    bool    ok;
};
namespace {  
    volatile Ressource channel;
}

struct AsciiChar {
    explicit inline AsciiChar(unsigned char v = 0) : value(v) {
        assert(value < 128);
    }
    unsigned char value {};
};

template<typename T> 
inline std::optional<T> get() {
    if (channel.ok) {
        return T{channel.data};
    }
    return {};
}

inline bool put(AsciiChar c) {
    channel.data = c.value;
    return channel.ok;
}

inline bool put(std::byte c) {
    channel.data = static_cast<uint8_t>(c);
    return channel.ok;
}

template<typename T>
struct Console {
    struct Input {
        struct Iterator {
            explicit inline Iterator(bool) {}   
            inline Iterator() = default;
            inline bool operator!=(const Iterator& rhs) {
                c = *get<T>();
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
            T c;
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
                if (!put(c)) {
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

int main() {
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
