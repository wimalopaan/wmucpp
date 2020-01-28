#include <iostream>
#include <chrono>
#include <cmath>

void mySleep(std::chrono::nanoseconds t) {
    std::cout << "t: " << t.count() << '\n';
}
template<typename R, typename P> requires (std::is_floating_point_v<R>)
void mySleep(const std::chrono::duration<R, P>& t) {
    std::cout << "ms: " << std::chrono::duration_cast<std::chrono::milliseconds>(t).count() << '\n';
}

int main() {
    using namespace std::chrono;

    auto t1 = 1s * acos(-1.0);
    mySleep(t1);
    
    auto t2= log(10.0) * 1s;
    mySleep(t2);
//    decltype(t2)::_;

    //    mySleep(1s/10e4);
//    mySleep(1s);
//    mySleep(10ms);
//    mySleep(10ns);
}
