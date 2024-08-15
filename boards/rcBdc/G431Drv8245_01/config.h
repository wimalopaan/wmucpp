#pragma once

namespace detail {
    template<typename T> concept ConceptSubSFactor= requires(T) { T::subSampleFactor(0u); };
    template<typename T> struct hasSubSFactor: std::integral_constant<bool, false> {};
    template<ConceptSubSFactor T> struct hasSubSFactor<T> : std::integral_constant<bool, true> {};

    template<typename T> concept ConceptPwm = requires(T) { T::pwm(0u); };
    template<typename T> struct hasPwm : std::integral_constant<bool, false> {};
    template<ConceptPwm T> struct hasPwm<T> : std::integral_constant<bool, true> {};

    template<typename T> concept ConceptCutoff= requires(T) { T::cutoff(0u); };
    template<typename T> struct hasCutoff: std::integral_constant<bool, false> {};
    template<ConceptCutoff T> struct hasCutoff<T> : std::integral_constant<bool, true> {};
}

template<typename... Listener>
struct Config {
    using listeners = Meta::List<Listener...>;
    using pwmListeners = Meta::filter<detail::hasPwm, listeners>;
    using cutoffListeners = Meta::filter<detail::hasCutoff, listeners>;
    using ssubfListeners = Meta::filter<detail::hasSubSFactor, listeners>;
    // using pwmListeners::_;
    // using cutoffListeners::_;

    static inline uint16_t fCutoff = 1'000;
    static inline uint16_t fSample = 4 * fCutoff;
    static inline uint16_t subSamplingFactor = 6;
    static inline uint16_t fPwm = fSample * subSamplingFactor;

    static inline uint16_t fPwmIdentify = 400;
    // static inline uint16_t fPwmIdentify = 200;
    static inline uint16_t dutyIdentifyInc = 50;

    static inline void init() {
        // (Listener::init(), ...);
        pwm(fPwm);
        cutoff(fCutoff);
        subSampleFactor(subSamplingFactor);
    }
    static inline void subSampleFactor(const uint16_t f) {
        subSamplingFactor = f;
        [&]<typename... L>(Meta::List<L...>){
            (L::subSampleFactor(f), ...);
        }(ssubfListeners{});
    }
    static inline void pwm(const uint16_t f) {
        fPwm = f;
        [&]<typename... L>(Meta::List<L...>){
            (L::pwm(f), ...);
        }(pwmListeners{});
    }
    static inline void cutoff(const uint16_t f) {
        fCutoff = f;
        [&]<typename... L>(Meta::List<L...>){
            (L::cutoff(f), ...);
        }(cutoffListeners{});
    }
};
