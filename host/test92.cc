#include<cstdint>
#include <array>

namespace {
    constexpr auto global_init = []{
        std::array<int, 100> data{};
        for(uint8_t i{0}; auto& e : data) {
            e = ++i;
        }
        return data;
    }();
}

constinit auto global{global_init};

//uint8_t global2[100] {};

int main() {
    return global_init[10];
}
