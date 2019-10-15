#include <limits>
#include <array>

namespace  {
}

int main() {
    std::array<char, 3> a {1, 2, 3};

    //    char a[] = {1, 2, 3};

    char max;
    
//    for(char i = 0; i < 3; ++i) {
//        if (a.mData[i] > max) max = a.mData[i];   
        
//    }
//    return max;
    
    for(const auto& i : a) {
        if (i > max) max = i;   
    }
    
    return max;
    
    int v1 = 1;
    int v2 = 2;
    int v3 = 3;
    
    
    if (v1 > max)  max = v1;
    if (v2 > max)  max = v2;
    if (v3 > max)  max = v3;
    
    return max;
}
