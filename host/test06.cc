#include <cstdlib>
#include <utility>

namespace  {
    using namespace std;
    
    int g(int v);
    int h(int v);
    bool check();
    
    int foo() {
        int v1; // non-const
        int v2;
        
        if (check()) {
            v1 = g(1);
            v2 = g(2);
        }
        else {
            v1 = h(1);
            v2 = h(2);
        }
        
        v1 = 42; // ups
        
        return v1 + v2;
    }
    int bar() {
        const auto [v1, v2] {[]{
            return check() ? pair{g(1), g(2)} : pair{h(1), h(2)};
        }()};
        
//        v1 = 42;
        
        return v1 + v2;            
    }
    int bar2() {
        const auto v1v2_initializer = []{return check() ? pair{g(1), g(2)} : pair{h(1), h(2)};};

        const auto [v1, v2] {v1v2_initializer()};
        
//        v1 = 42;
        
        return v1 + v2;            
    }
}

int main() {
    foo();
    bar();
}

namespace  {
    using namespace std;
    
    int g(const int v) {
        return {rand() + 2 * v};
    }
    int h(const int v) {
        return {rand() + v};
    }
    bool check() {
        return {(rand() % 2) == 0};
    }
}
