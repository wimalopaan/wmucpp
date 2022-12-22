#pragma once

#include <array>

struct Config {
    Config() = delete;
    struct element_t {
        int a;
        int b;
    };
    
    static inline const auto& data() {
        return elements;
    }
    
private:
    inline static const std::array elements = {element_t{1, 2}, element_t{3, 4}};    
};


