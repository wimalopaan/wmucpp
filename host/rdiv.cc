#include <iostream>
#include <algorithm>
#include <array>
#include <cmath>

class RDiv {
    double parallel(const double a, const double b){
        double z = a * b;
        double n = a + b;
        if (n != 0) {
            return z / n; 
        }
        return 0.0;
    };

    struct R {
        double r11{};
        double r12{};
        double r21{};
        double r22{};
        
        double k{};
        double diff{};
    };
    
    const double open = 1.0e12;
public:
    explicit RDiv(const double k) : mK{k} {}
    
    template<typename T, auto N>
    R compute4(const std::array<T, N>& eseries, bool open1 = false, bool open2 = false, double rmin = 0.0) {
        R m;
        double min = std::numeric_limits<double>::max();
        for(size_t i1 = 0; i1 < N; ++i1) {
            for(size_t i2 = 0; i2 < N; ++i2) {
                for(size_t i3 = 0; i3 < N; ++i3) {
                    for(size_t i4 = 0; i4 < N; ++i4) {
                        const double r11 = eseries[i1];
                        double r12 = eseries[i2];
                        if (open1) {
                            r12 = open;
                        }
            
                        const double r21 = eseries[i3];
                        double r22 = eseries[i4];
                        if (open2) {
                            r22 = open;
                        }
                        
                        const double r1 = parallel(r11, r12);                        
                        const double r2 = parallel(r21, r22);        
                        
                        const double k = r1/r2 + 1;
                        
                        const double diff = std::fabs(k - mK);
                        
                        R v{r11, r12, r21, r22, k, diff};
                        
                        if ((r1 + r2) >= rmin) {
                            if (diff < min) {
                                min = diff;
                                m = v;
                            }
                        }
                    }
                }
            }
        }
        return m;
    }
private:
    const double mK{};
};


//const auto E24 = []{
//    const double base = 10;
//    std::array<double, 24> a;
//    const size_t N = a.size();
//    const double root = std::pow(10.0, 1.0/N);
//    for(size_t i{0}; i < N; ++i) {
//        a[i] = 100 * std::round(base * std::pow(root, i));
//    }

//    std::array<double, 3 * a.size()> b;
//    for(size_t i{0}; i < N; ++i) {
//        b[i] = a[i] / 10;
//    }   
//    for(size_t i{0}; i < N; ++i) {
//        b[i + a.size()] = a[i];
//    }   
//    for(size_t i{0}; i < N; ++i) {
//        b[i + 2 * a.size()] = a[i] * 10;
//    }   
    
//    return b;
//}();

//const std::array E24{510, 560, 620, 680, 750, 820, 910, 
//                                                1000, 1100, 1200, 1300, 1500, 1600, 1800, 2000, 2200, 2400, 2700, 3000, 3300, 3600, 3900, 4300, 4700, 5100, 5600, 6200, 6800, 7500, 8200, 9100, 
//                                                10000, 11000, 12000, 13000, 15000, 16000, 18000, 20000, 22000, 24000, 27000, 30000, 33000, 36000, 39000, 43000, 47000};

const std::array E24{1'000'000'000, 
                     6'800'000, 3'160, 430'000, 12'000, 10'000, 68'000, 560'000, 100'000, 220'000, 16'000, 
                     330'000, 3'000, 100, 2'100, 1'000'000, 1'000, 30'000, 470, 
                     220, 150, 120'000, 620'000, 1'500, 8'200, 1'800,
                     51'000, 3'900, 390, 24'000};

int main() {
    const double vref = 4.3;
    const double steps = 1023;
    const double delta = 0.03;
    
    const double k = delta * steps / vref;
    
    std::cout << "K: " << k << '\n';
    
    RDiv rdiv{k};
    
    auto rs = rdiv.compute4(E24, false, false, 10000);
    std::cout << "R11: " << rs.r11 << "\tR12: " << rs.r12 <<"\tR21: " << rs.r21 << "\tR22: " << rs.r22 << "\tk: " << rs.k << "\td: " << rs.diff << '\n';

    rs = rdiv.compute4(E24, true, false, 10000);
    std::cout << "R11: " << rs.r11 << "\tR12: " << rs.r12 <<"\tR21: " << rs.r21 << "\tR22: " << rs.r22 << "\tk: " << rs.k << "\td: " << rs.diff << '\n';
    
    rs = rdiv.compute4(E24, false, true, 10000);
    std::cout << "R11: " << rs.r11 << "\tR12: " << rs.r12 <<"\tR21: " << rs.r21 << "\tR22: " << rs.r22 << "\tk: " << rs.k << "\td: " << rs.diff << '\n';

    rs = rdiv.compute4(E24, true, true, 10000);
    std::cout << "R11: " << rs.r11 << "\tR12: " << rs.r12 <<"\tR21: " << rs.r21 << "\tR22: " << rs.r22 << "\tk: " << rs.k << "\td: " << rs.diff << '\n';
}
