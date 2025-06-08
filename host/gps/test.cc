#include <iostream>
#include <utility>
#include <numbers>
#include <cmath>

std::pair<double, double> distanceCourse_D(int32_t lat1_deg, int32_t lon1_deg, int32_t lat2_deg, int32_t lon2_deg) {
    double R = 6371000;
    double scale = 1e-7;
    double degToRad = std::numbers::pi / 180.0;
    double lat1 = lat1_deg * degToRad * scale;
    double lat2 = lat2_deg * degToRad * scale;
    double lon1 = lon1_deg * degToRad * scale;
    double lon2 = lon2_deg * degToRad * scale;
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    // printf("DD lat1=%3.17f lon1=%3.17f lat2=%3.17f lon2=%3.17f\n", lat1, lon1, lat2, lon2);
    // printf("DD dlat=%3.17f dlon=%3.17f\n", dlat, dlon);
    double sin_dlat2 = std::sin(dlat / 2);
    double sin_dlon2 = std::sin(dlon / 2);
    double cos_lat1 = std::cos(lat1);
    double cos_lat2 = std::cos(lat2);
    double a = sin_dlat2 * sin_dlat2 + cos_lat1 * cos_lat2 * sin_dlon2 * sin_dlon2;
    // printf("DD a=%3.17f\n", a);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    double dist = R * c;
    double x = cos_lat1 * std::sin(lat2) - std::sin(lat1) * cos_lat2 * std::cos(dlon);
    double y = std::sin(dlon) * cos_lat2;
    double course = std::atan2(y, x) / degToRad;
    return std::pair<double, double>{dist, course};
}

std::pair<float, float> distanceCourse_F(int32_t factor, int32_t lat1_deg, int32_t lon1_deg, int32_t lat2_deg, int32_t lon2_deg) {
    float R = 6371000;
    float scale = 1.0f / factor;
    float degToRad = std::numbers::pi / 180.0;
    float lat1 = lat1_deg * degToRad * scale;
    float lat2 = lat2_deg * degToRad * scale;
    float lon1 = lon1_deg * degToRad * scale;
    float lon2 = lon2_deg * degToRad * scale;
    float dlat = lat2 - lat1;
    float dlon = lon2 - lon1;
    // printf("DF lat1=%3.17f lon1=%3.17f lat2=%3.17f lon2=%3.17f\n", lat1, lon1, lat2, lon2);
    // printf("DF dlat=%3.17f dlon=%3.17f\n", dlat, dlon);
    float sin_dlat2 = std::sin(dlat / 2);
    float sin_dlon2 = std::sin(dlon / 2);
    float cos_lat1 = std::cos(lat1);
    float cos_lat2 = std::cos(lat2);
    float a = sin_dlat2 * sin_dlat2 + cos_lat1 * cos_lat2 * sin_dlon2 * sin_dlon2;
    // printf("DF a=%3.17f\n", a);
    float c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    float dist = R * c;
    float x = cos_lat1 * std::sin(lat2) - std::sin(lat1) * cos_lat2 * std::cos(dlon);
    float y = std::sin(dlon) * cos_lat2;
    float course = std::atan2(y, x) / degToRad;
    return std::pair<float, float>{dist, course};
}
std::pair<float, float> distanceCourse_F3(int32_t factor, int32_t lat1_deg, int32_t lon1_deg, int32_t lat2_deg, int32_t lon2_deg) {
    float R = 6371000;
    float scale = 1.0f / factor;
    float degToRad = std::numbers::pi / 180.0;

    float lat_mean_rad = (lat1_deg + lat1_deg) * degToRad * scale / 2;
    float cos_lat_mean = std::cos(lat_mean_rad);

    int64_t dlat = (lat2_deg - lat1_deg);
    int64_t dlon = (lon2_deg - lon1_deg) * cos_lat_mean;
    int32_t d_deg = std::sqrt(dlat * dlat + dlon * dlon); // contains: factor

    float rd = (d_deg * degToRad * R) / factor;
    // printf("DF3 d=%d rdd=%f\n", d_deg, rd);

    float course = std::atan2(dlon, dlat) / degToRad;
    return {rd, course};
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

    auto [dd, cd] = distanceCourse_D(rlat1, rlon1, rlat2, rlon2);
    printf("dd=%3.7f cd=%3.7f\n", dd, cd);

    auto [df, cf] = distanceCourse_F(10'000'000, rlat1, rlon1, rlat2, rlon2);
    printf("df=%3.7f cf=%3.7f\n", df, cf);

    auto [df3, cf3] = distanceCourse_F3(10'000'000, rlat1, rlon1, rlat2, rlon2);
    printf("dF=%3.7f cF=%3.7f\n", df3, cf3);

    auto [df3l, cf3l] = distanceCourse_F(1'000'000, rlat1 / 10, rlon1 / 10, rlat2 / 10, rlon2 / 10);
    printf("dF=%3.7f cF=%3.7f\n", df3l, cf3l);

}
