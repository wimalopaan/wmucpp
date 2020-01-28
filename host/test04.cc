#include <cassert>
#include <algorithm>
#include <string>
#include <array>
#include <optional>
#include <iostream>

using namespace std::literals::string_literals;

template<typename C>
concept Container = requires(C c) {
                    typename C::value_type;
                    typename C::difference_type;
                    c.cbegin();
                    c.end();
                    };

template<Container C>
auto index(const C& container, const typename C::value_type pattern) -> std::optional<typename C::difference_type> {
    if (const auto it{std::find(std::cbegin(container), std::cend(container), pattern)}; 
            it != std::cend(container)) {
        return {std::distance(std::cbegin(container), it)};
    }
    return {};
}

auto commandindex(const std::string& pattern) {
	const std::array commands{"on"s, "off"s, "left"s, "right"s, "up"s, "down"s, "step"s, "back"s, "erase"s};
    return index(commands, pattern);
}

int main(const int argc, const char* const* argv) {
    assert(argc > 1);
    assert(argv[1]);
    if (const auto r{commandindex(argv[1])}; r) {
        return *r;
    }
    else {
        return -1;
    }
}

