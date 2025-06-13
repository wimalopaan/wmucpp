#include <iostream>
#include <utility>
#include <numbers>
#include <cmath>

template<typename T, typename U = int32_t>
std::pair<T, T> distanceCourse(const U factor, const U lat1_deg, const U lon1_deg, const U lat2_deg, const U lon2_deg) {
    const T R = 6371000;
    const T scale = T{1.0} / factor;
    const T degToRad = std::numbers::pi / 180.0;
    const T lat1 = lat1_deg * degToRad * scale;
    const T lat2 = lat2_deg * degToRad * scale;
    const T lon1 = lon1_deg * degToRad * scale;
    const T lon2 = lon2_deg * degToRad * scale;
    const T dlat = lat2 - lat1;
    const T dlon = lon2 - lon1;
    const T sin_dlat2 = std::sin(dlat / 2);
    const T sin_dlon2 = std::sin(dlon / 2);
    const T cos_lat1 = std::cos(lat1);
    const T cos_lat2 = std::cos(lat2);
    const T a = sin_dlat2 * sin_dlat2 + cos_lat1 * cos_lat2 * sin_dlon2 * sin_dlon2;
    const T c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    const T dist = R * c;
    const T x = cos_lat1 * std::sin(lat2) - std::sin(lat1) * cos_lat2 * std::cos(dlon);
    const T y = std::sin(dlon) * cos_lat2;
    const T course = std::atan2(y, x) / degToRad;
    return {dist, course};
}
template<typename T = float, typename U = int32_t>
std::pair<T, T> distanceCourseNear(const U factor, const U lat1_deg, const U lon1_deg, const U lat2_deg, const U lon2_deg) {
    const T R = 6371000;
    const T scale = T{1.0} / factor;
    const T degToRad = std::numbers::pi / 180.0;

    const T lat_mean_rad = (lat1_deg + lat1_deg) * degToRad * scale / 2;
    const T cos_lat_mean = std::cos(lat_mean_rad);

    const int64_t dlat = (lat2_deg - lat1_deg);
    const int64_t dlon = (lon2_deg - lon1_deg) * cos_lat_mean;
    const U d_deg = std::sqrt(dlat * dlat + dlon * dlon); // contains: factor

    const T dist = (d_deg * degToRad * R) / factor;
    const T course = std::atan2(dlon, dlat) / degToRad;
    return {dist, course};
}
int main() {
    // int32_t rlat1 = 493157311;
    // int32_t rlon1 = 72631201;
    // int32_t rlat1 = 493158311;
    // int32_t rlon1 =  72632201;
    int32_t rlat1 = 493157174;
    int32_t rlon1 = 72631114;

    int32_t rlat2 = 493157173;
    int32_t rlon2 = 72631114;

    auto [dd, cd] = distanceCourse<double>(10'000'000, rlat1, rlon1, rlat2, rlon2);
    printf("dd=%3.7f cd=%3.7f\n", dd, cd);

    auto [df, cf] = distanceCourse<float>(10'000'000, rlat1, rlon1, rlat2, rlon2);
    printf("df=%3.7f cf=%3.7f\n", df, cf);

    auto [df3, cf3] = distanceCourseNear(10'000'000, rlat1, rlon1, rlat2, rlon2);
    printf("dF=%3.7f cF=%3.7f\n", df3, cf3);

    auto [df3l, cf3l] = distanceCourse<float>(1'000'000, rlat1 / 10, rlon1 / 10, rlat2 / 10, rlon2 / 10);
    printf("dF=%3.7f cF=%3.7f\n", df3l, cf3l);

}
