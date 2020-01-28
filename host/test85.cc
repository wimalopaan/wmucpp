// make all things const

#include <array>
#include <vector>
#include <string>
#include <cmath>
#include <numbers>
#include <iostream>

namespace {
    int getId() {
        static int id{0};
        return id;
    }    
    volatile int r;
    int getR() {
        return r;
    }
    struct A {
        using value_type = int;
        using container_type = std::vector<value_type>;

        value_type getId() const {
            return m1;
        }
        int64_t accumulate(size_t start, size_t end) const;
    private:
        const value_type m1{getId()};
        const container_type m2{[]{
                container_type values;
                values.push_back(getR());
                values.push_back(getR());
                values.push_back(getR());
                return values;}()};
    };
    int64_t A::accumulate(const size_t start, const size_t end) const {
        int64_t sum{};
        for(auto index{start}; index < std::min(end, m2.size()); ++index) {
            sum += m2[index];
        }
        return sum;
    }
}

template<typename S>
concept Stream = requires(S s) {
                 s.operator<<(int{});
};
template<typename C>
concept Container = requires(C c) {
                    std::begin(c);
                    std::end(c);
};

template<Stream S, Container C>
auto& operator<<(S& stream, const C& container) {
    for(const auto& c : container) {
        stream << c;
    }
    return stream;
}

int main(const int argc, const char* const* const argv) {
    const std::vector<std::string> arguments{argv, argv + argc};

    std::cout << arguments;    
    
    
    const auto x1{1};
    const auto x2{x1 + 10};
    
    const auto a{[]{
            std::array<int, 10> values{};
            for(double a{}; auto& v : values) {
                v = std::sin(a);
                a += 2.0 * std::numbers::pi / values.size();
            }
            return values;}()};

    
    return x1 + x2 + A{}.accumulate(0, 1);
}
