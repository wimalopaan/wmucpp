#include <cstdint>
#include <array>
#include <algorithm>
#include <optional>
#include <iostream>

//inline static const std::array<double, 13> E12 {1000, 1200, 1500, 1800, 2200, 2700, 3300, 3900, 4700, 5600, 6800, 8200, 10000 };
//inline static const std::array<double, 24> E12 {390, 470, 560, 680, 820, 1000, 1200, 1500, 1800, 2200, 2700, 3300, 3900, 4700, 5600, 6800, 8200, 10000, 12000, 15000, 18000, 22000, 27000, 33000};
//inline static const std::array<double, 48> E12 {510, 560, 620, 680, 750, 820, 910, 
//                                                1000, 1100, 1200, 1300, 1500, 1600, 1800, 2000, 2200, 2400, 2700, 3000, 3300, 3600, 3900, 4300, 4700, 5100, 5600, 6200, 6800, 7500, 8200, 9100, 
//                                                10000, 11000, 12000, 13000, 15000, 16000, 18000, 20000, 22000, 24000, 27000, 30000, 33000, 36000, 39000, 43000, 47000};
// vorhandene Widerstände
inline static const std::array E12{1'000'000'000, 
                     6'800'000, 3'160, 
//                                   430'000, 
                                   12'000, 10'000, 68'000, 560'000, 100'000, 220'000, 16'000, 
                     330'000, 3'000, 100, 2'100, 1'000'000, 1'000, 30'000, 470, 
                     220, 150, 120'000, 620'000, 1'500, 8'200, 1'800,
                     51'000, 3'900, 390, 24'000, 10, 4'700};

template<auto N>
struct Sim {
    std::array<double, (1 << N)> R;
    double Ro;
    
    explicit Sim(const double Ro, const std::array<double, N>& Rs) :
        Ro{Ro}
    {
        auto parallel = [](const double a, const double b){
            double z = a * b;
            double n = a + b;
            if (n != 0) {
                return z / n; 
            }
            return 0.0;
        };
        for(uint8_t i = 0; i < R.size(); ++i) {
            std::array<double, N> rs;
            std::fill(std::begin(rs), std::end(rs), 1000'000'000.0);
         
            [&]<auto... II>(std::index_sequence<II...>){
                auto check = [&]<auto I>(std::integral_constant<size_t, I>) {
                             if (i & (0x01 << I)) {
                                rs[I] = Rs[I];
                                }
                                          };
                (check(std::integral_constant<size_t, II>{}), ...);
            }(std::make_index_sequence<N>{});
            
//            for(uint8_t b = 0; b < N; ++b) {
//                if (i & (0x01 << b)) {
//                    rs[b] = Rs[b];
//                }
//            }
            double ra = parallel(rs[0], rs[1]);
            for(uint8_t b = 2; b < N; ++b) {
                ra = parallel(ra, rs[b]);
            }
            R[i] = ra;
        }
    }
    
    auto result() const {
        const double Vref = 5.0;
//        const double amax = 4095;
        const double amax = 2047;
        const double vmax = 5.0;
        const double p = 0.01;
        std::array<std::pair<uint16_t, uint16_t>, (1 << N)> ii;
        for(uint8_t i = 0; const auto& r: R) {
            const double v = vmax * r / (r + Ro); 
            const double vl = v * (1.0 - p);
            const double al = vl * amax / Vref;
            const double vh = v * (1.0 + p);
            const double ah = vh * amax / Vref;
            
            ii[i].first = std::min(amax, al);
            ii[i].second= std::min(amax, ah);
            ++i;
        }
        sort(std::begin(ii), std::end(ii));
        return ii;
    };
    
    uint16_t mindiff(const std::array<std::pair<uint16_t, uint16_t>, (1 << N)>& r) {
        int16_t diff{4096};
        for(size_t i = 0; i < r.size() - 1; ++i) {
            int16_t d = r[i + 1].first - r[i].second;
            if (d < 0) {
                d = 0;
            }
            if (d < diff) {
                diff = d;
            }
        }
        return diff;
    }
    
};

int main() {
    uint16_t diff = 0;
    uint16_t mA;
    uint16_t m0;
    uint16_t m1;
    uint16_t m2;
    uint16_t m3;
    uint16_t m4;
    for(size_t iA = 0; iA < E12.size(); ++iA) {
        for(size_t i0 = 0; i0 < E12.size(); ++i0) {
            for(size_t i1 = i0; i1 < E12.size(); ++i1) {
                for(size_t i2 = i1; i2 < E12.size(); ++i2) {
                    for(size_t i3 = i2; i3 < E12.size(); ++i3) {
                        for(size_t i4 = i3; i4 < E12.size(); ++i4) {
                            Sim s1 = Sim<5>(E12[iA], {{E12[i0], E12[i1], E12[i2], E12[i3], E12[i4]}});
                            auto r = s1.result();
                            auto d = s1.mindiff(r);
                            if (d > diff) {
                                diff = d;
                                mA = iA;
                                m0 = i0;
                                m1 = i1;
                                m2 = i2;
                                m3 = i3;
                                m4 = i4;
                            }
                        }
                    }   
                }
            }
        }
    }
//    std:: cout << diff << '\n';
//    std:: cout << mA << '\n';
//    std:: cout << m0 << '\n';
//    std:: cout << m1 << '\n';
//    std:: cout << m2 << '\n';
//    std:: cout << m3 << '\n';
//    std:: cout << m4 << '\n';
    std:: cout << E12[mA] << '\n';
    std:: cout << E12[m0] << '\n';
    std:: cout << E12[m1] << '\n';
    std:: cout << E12[m2] << '\n';
    std:: cout << E12[m3] << '\n';
    std:: cout << E12[m4] << '\n';
    
    Sim s1 = Sim<5>(E12[mA], {E12[m0], E12[m1], E12[m2], E12[m3], E12[m4]});
    auto r = s1.result();
    
    for(auto& i: r) {
        std::cout << i.first << "," << i.second << '\n';
    }
    
    std::cout << s1.mindiff(r) << '\n';
}
