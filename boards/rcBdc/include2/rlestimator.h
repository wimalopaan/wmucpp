#pragma once

#include <optional>
#include <utility>

#include "cmsis.h"

namespace Statistics  {

    template<typename T = float>
    class RLEstimator {
        // https://math.stackexchange.com/questions/3953633/how-change-equation-y-a1-ebx-as-linear-regression-to-approximate-a-and
        public:

        using value_type = std::remove_cv_t<T>;

        explicit RLEstimator(const value_type dx, const value_type x0, const value_type yfirst) :
            x{x0}, dx{dx}, yPrev{-yfirst} {}
        void process(const value_type y) {
            const value_type sActual = sPrev + 0.5f * (y + yPrev) * dx;
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
        std::optional<std::pair<value_type, value_type>> compute() {
            value_type inv[3][3] {};
            value_type abc[3] {};
            arm_matrix_instance_f32 min;
            arm_matrix_instance_f32 minv;
            arm_matrix_instance_f32 mr;
            arm_matrix_instance_f32 mabc;
            sums[1][0] = sums[0][1];
            sums[2][0] = sums[0][2];
            sums[2][1] = sums[1][2];

            arm_mat_init_f32(&min, 3, 3, (float*)&sums[0][0]);
            arm_mat_init_f32(&minv, 3, 3, &inv[0][0]);
            arm_mat_init_f32(&mr, 3, 1, (float*)&rsum[0]);
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
        T x{0};
        value_type dx{1};
        T yPrev{0.0f};
        T sPrev{0.0f};

        // s^2 sx  s
        // sx  x^2 x
        // s   x   1
        std::array<std::array<T, 3>, 3> sums{};

        // sy
        // xy
        // y
        std::array<T, 3> rsum{};
    };
}
