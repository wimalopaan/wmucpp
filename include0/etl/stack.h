#pragma once

namespace std {
    template<typename T, typename C>
    struct stack {
        inline constexpr void push(T&& v) {
            container.push_back(std::forward<T>(v));
        }
        inline constexpr void pop() {
            container.pop_back();
        }
        inline constexpr T& top() {
            return container.back();
        }
        inline constexpr const T& top() const {
            container.back();
        }
    private:
        C container;
    };
}
