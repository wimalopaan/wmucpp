#pragma once

template<typename D>
struct Adapter0 {
    static inline const auto& data = D::magnitudeWeighted();
    static inline const char* const title = "weighted mag(FFT)";
};
template<typename D>
struct Adapter1 {
    static inline const auto& data = D::maxWeighted();
    static inline const char* const title = "max";
};
template<typename D>
struct Adapter2 {
    static inline const auto& data = D::rpmPos();
    static inline const char* const title = "est(Um,i,Rm)";
};
template<typename D>
struct Adapter3 {
    static inline const auto& data = D::windowPlot();
    static inline const char* const title = "window";
};
template<typename D>
struct Adapter4 {
    static inline const auto& data = D::magnitude();
    static inline const char* const title = "mag(FFT)";
};
template<typename D>
struct Adapter5 {
    static inline const auto& data = D::indexCutoff();
    static inline const char* const title = "iC";
};

