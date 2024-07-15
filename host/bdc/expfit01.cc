#include <iostream>
#include <array>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cstdint>
#include <limits>
#include <algorithm>
#include <cmath>
#include <iterator>
#include <optional>

#include "CMSIS/DSP/Source/TransformFunctions/TransformFunctions.c"
#include "CMSIS/DSP/Source/CommonTables/CommonTables.c"
#include "CMSIS/DSP/Source/ComplexMathFunctions/ComplexMathFunctions.c"
#include "CMSIS/DSP/Source/FastMathFunctions/FastMathFunctions.c"
#include "CMSIS/DSP/Source/BasicMathFunctions/BasicMathFunctions.c"
#include "CMSIS/DSP/Source/StatisticsFunctions/StatisticsFunctions.c"
#include "CMSIS/DSP/Source/MatrixFunctions/MatrixFunctions.c"

class RLEstimator {
    // https://math.stackexchange.com/questions/3953633/how-change-equation-y-a1-ebx-as-linear-regression-to-approximate-a-and
    public:
    explicit RLEstimator(const float dx, const float x0, const float yfirst) :
        x{x0}, dx{dx}, yPrev{-yfirst} {}
    void process(const float y) {
        const float sActual = sPrev + 0.5f * (y + yPrev) * dx;
        sums[0][0] += sActual * sActual;
        sums[0][1] += sActual * x;
        sums[0][2] += sActual;
        sums[1][1] += x * x;
        sums[1][2] += x;
        sums[2][2] += 1;
        rsum[0] += sActual * y;
        rsum[1] += x * y;
        rsum[2] += y;
        sPrev = sActual;
        yPrev = y;
        x += dx;
    }
    std::optional<std::pair<float, float>> compute() {
        float inv[3][3] {};
        float abc[3] {};
        arm_matrix_instance_f32 min;
        arm_matrix_instance_f32 minv;
        arm_matrix_instance_f32 mr;
        arm_matrix_instance_f32 mabc;
        sums[1][0] = sums[0][1];
        sums[2][0] = sums[0][2];
        sums[2][1] = sums[1][2];

        arm_mat_init_f32(&min, 3, 3, &sums[0][0]);
        arm_mat_init_f32(&minv, 3, 3, &inv[0][0]);
        arm_mat_init_f32(&mr, 3, 1, &rsum[0]);
        arm_mat_init_f32(&mabc, 3, 1, &abc[0]);
        if (auto e = arm_mat_inverse_f32(&min, &minv); e != ARM_MATH_SUCCESS) {
            return {};
        }
        if (auto e = arm_mat_mult_f32(&minv, &mr, &mabc); e != ARM_MATH_SUCCESS) {
            return {};
        }
        return {std::pair{abc[0], abc[1]}};
    }
    private:
    float x{0};
    float dx{1};
    float yPrev{0.0f};
    float sPrev{0.0f};

    // s^2 sx  s
    // sx  x^2 x
    // s   x   1
    std::array<std::array<float, 3>, 3> sums{};

    // sy
    // xy
    // y
    std::array<float, 3> rsum{};
};

// class RLEstimator {
//     public:
//     explicit RLEstimator(const float dx, const float x0, const float yfirst) : x{x0}, dx{dx}, yPrev{-yfirst} {
//     }
//     void process(const float y) {
//         const float sActual = sPrev + 0.5f * (y + yPrev) * dx;
//         sum_s2 += sActual * sActual;
//         sum_s_x += sActual * x;
//         sum_s += sActual;
//         sum_x2 += x * x;
//         sum_x += x;
//         sum_s_y += sActual * y;
//         sum_x_y += x * y;
//         sum_y += y;
//         sPrev = sActual;
//         yPrev = y;
//         x += dx;
//         sum_one += 1;
//     }
//     std::optional<std::pair<float, float>> compute() {
//         float in[3][3] {};
//         float inv[3][3] {};
//         float r[3] {};
//         float abc[3] {};
//         arm_matrix_instance_f32 min;
//         arm_matrix_instance_f32 minv;
//         arm_matrix_instance_f32 mr;
//         arm_matrix_instance_f32 mabc;

//         in[0][0] = sum_s2;
//         in[0][1] = sum_s_x;
//         in[0][2] = sum_s;
//         in[1][0] = sum_s_x;
//         in[1][1] = sum_x2;
//         in[1][2] = sum_x;
//         in[2][0] = sum_s;
//         in[2][1] = sum_x;
//         in[2][2] = sum_one;

//         r[0] = sum_s_y;
//         r[1] = sum_x_y;
//         r[2] = sum_y;

//         arm_mat_init_f32(&min, 3, 3, &in[0][0]);
//         arm_mat_init_f32(&minv, 3, 3, &inv[0][0]);
//         arm_mat_init_f32(&mr, 3, 1, &r[0]);
//         arm_mat_init_f32(&mabc, 3, 1, &abc[0]);
//         if (auto e = arm_mat_inverse_f32(&min, &minv); e != ARM_MATH_SUCCESS) {
//             return {};
//         }
//         if (auto e = arm_mat_mult_f32(&minv, &mr, &mabc); e != ARM_MATH_SUCCESS) {
//             return {};
//         }
//         return {std::pair{abc[0], abc[1]}};
//     }
//     private:
//     float sum_one{0};
//     float x{0};
//     float dx{1};
//     float yPrev{0.0f};
//     float sPrev{0.0f};
//     float sum_s2{0.0f};
//     float sum_s_x{0.0f};
//     float sum_s{0.0f};
//     float sum_x2{0.0f};
//     float sum_x{0.0f};
//     float sum_s_y{0.0f};
//     float sum_x_y{0.0f};
//     float sum_y{0.0f};
// };

int main(const int argc, const char* const* const argv) {
    std::vector<std::string> args{argv, argv + argc};
    std::cout << "expfit01\n";
    if (args.size() <= 1) {
        std::cerr << "filename?\n";
        return -1;
    }
    std::cout << "file: " << args[1] << "\n";

    std::vector<std::vector<std::pair<float, float>>> adcValues;

    std::ifstream dataFile{args[1]};

    if (!dataFile.is_open()) {
        std::cerr << "open?\n";
        return -1;
    }
    std::string line;
#if 1
    size_t n = 0;
    while(std::getline(dataFile, line)) {
        std::stringstream lineStream{line};
        std::string token;
        float i;
        float v;
        if (std::getline(lineStream, token, ',')) {
            if (!std::isdigit(token[0])) {
                adcValues.push_back(std::vector<std::pair<float, float>>{});
                n = 0;
                continue;
            }
            else {
                i = std::stof(token);
            }
        }
        if (std::getline(lineStream, token, ',')) {
            v = std::stof(token);
        }
        // std::cout << i << ' ' << v << '\n';
        if (n == 0) {
            adcValues.back().push_back(std::pair{0.0f, v});
        }
        if (i > 0) {
            adcValues.back().push_back(std::pair{i, v});
        }
        ++n;
    }

    for(const auto& s : adcValues) {
        if (s.size() == 0) continue;
        std::cout << "set: " << s.size() << '\n';
        RLEstimator rle{1, 0, s[0].first};
        float Am = 0;
        for (const auto& v : s) {
            // std::cout << "i: " << v.first << " v: " << v.second << '\n';
            rle.process(v.first);
            Am += v.second;
        }
        Am /= s.size();
        float Um = (33.33 / 4096) * Am;
        if (const auto ab = rle.compute()) {
            float a = -ab->second / ab->first;
            float b = ab->first;
            float ki = 201.77;
            float Rm = (Um * ki) / a;
            std::cout << "Um: " << Um << " Rm: " << Rm << '\n';
            std::cout << "a: " << a << " b: " << b << '\n';

            // int x = 1;
            int x = 0;
            for (const auto& v : s) {
                float est = a * (1.0f - exp(b * x));
                std::cout << "i: " << v.first << " v: " << v.second << " iest: " << est << '\n';
                ++x;
            }
        }
    }
#endif

#if 0
    std::array<float, 10> testy{-4.2, -0.3, 1.6, 3.4, 3.2, 3.8, 3.8, 4.3, 3.7, 4.2};
    RLEstimator rle{0.5, -0.5, testy[0]};
    for (const auto& y : testy) {
        rle.process(y);
    }
    const auto ab = rle.compute();
    std::cout << "a: " << ab.first << ' ' << ab.second << '\n';
#endif
}
