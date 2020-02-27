#include <iostream>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cassert>

static inline constexpr int compareInts(const void* const a, const void* const b) {
    const int* const s1 = (const int*) a;
    const int* const s2 = (const int*) b;
    return (*s1 > *s2) - (*s1 < *s2);
}

int main() {
    srand(time(nullptr));
    const size_t asize = 1'000'000;
    
    std::vector<int> a1;
    std::generate_n(std::back_inserter(a1), asize, rand);
    
    const auto a2 = (int*) malloc(asize * sizeof(int));
    std::copy(std::begin(a1), std::end(a1), a2);
    
    const auto start1 = std::chrono::system_clock::now();
    std::sort(std::begin(a1), std::end(a1));
    const auto end1 = std::chrono::system_clock::now();
    
    assert(a2);
    const auto start2 = std::chrono::system_clock::now();
    std::qsort(a2, asize, sizeof(int), compareInts);
    const auto end2 = std::chrono::system_clock::now();
    free(a2);
    
    std::chrono::duration<double> diff1 = end1 - start1;
    std::chrono::duration<double> diff2 = end2 - start2;
    std::cout << "C++: " << diff1.count() << '\n';
    std::cout << "C  : " << diff2.count() << '\n';
}
