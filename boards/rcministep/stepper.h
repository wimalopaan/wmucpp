#pragma once

#include <cmath>
#include <mcu/avr.h>

#include <mcu/internals/port.h>

#include <mcu/internals/pwm.h>

#include <etl/ranged.h>

namespace External {
    namespace StepMotor {

        template<typename PWM, typename Lut0, typename Lut1>
        struct MP6513Driver {
            enum class State : uint8_t {
                Init, A, B, C, D
            };
            enum class Pol : uint8_t {Positiv, Negativ};
            enum class Rot : uint8_t {Forward, Backward};
        
            inline static constexpr uint16_t steps = 512;
            static_assert(etl::isPowerof2(steps));
            
            inline static constexpr uint16_t period = 1024; // 16KHz
            inline static constexpr uint8_t scaleBits = 5;
            inline static constexpr uint8_t scale = (1 << scaleBits);
        //    std::integral_constant<uint8_t, scale>::_;
            inline static constexpr uint16_t maxPeriodValue = period - 1;
            
            inline static constexpr uint8_t periodBits = etl::minimumBitsForValue(maxPeriodValue);
            inline static constexpr uint8_t maxScaleBits = 16 - periodBits;
            static_assert(maxScaleBits >= scaleBits);
            
            using scale_type = etl::uint_ranged<uint8_t, 1, scale>;
            using index_type = etl::uint_ranged_circular<uint16_t, 0, steps - 1> ;
            
            template<size_t Steps, double E = 1.0>    
            struct Generator {
                constexpr auto operator()() {
                    std::array<uint16_t, Steps> data;
                    for(uint16_t i = 0; i < Steps; ++i) {
                        data[i] = maxPeriodValue * pow(sin((i * M_PI) / steps), E);
                    }
                    return data;
                }
            };
            using Sine10 = AVR::Pgm::Util::Converter<Generator<steps, 1.0>>::pgm_type;
            using Sine12 = AVR::Pgm::Util::Converter<Generator<steps, 1.2>>::pgm_type;
            using Sine14 = AVR::Pgm::Util::Converter<Generator<steps, 1.4>>::pgm_type;
            using Sine15 = AVR::Pgm::Util::Converter<Generator<steps, 1.5>>::pgm_type;
            using Sine16 = AVR::Pgm::Util::Converter<Generator<steps, 1.6>>::pgm_type;
            using Sine18 = AVR::Pgm::Util::Converter<Generator<steps, 1.8>>::pgm_type;
            using Sine20 = AVR::Pgm::Util::Converter<Generator<steps, 2.0>>::pgm_type;
            
            inline static void init() {
                PWM::init();
                PWM::period(period);
                PWM::template on<Meta::List<AVR::PWM::WO<0>, AVR::PWM::WO<1>>>();
                PWM::template invert<AVR::PWM::WO<0>>();
                Lut0::init(0xff_B);
                PWM::template invert<AVR::PWM::WO<1>>();
                Lut1::init(0xff_B);
            }  
            inline static void periodic() {
            }
        
            inline static void direction(const Rot d) {
                mDirection = d;
            }
            
#ifndef NO_SHIFT
            template<typename T, auto L, auto U>
            static inline void shift(const etl::uint_ranged<T, L, U>& b) {
                constexpr T mid = b.mid();
                constexpr T delta = (U - L) / 2;
                
                if (const T v = b.toInt(); v > mid) {
                    const T d = v - mid;            
                    const T x = etl::scale(d, etl::Intervall{0u, delta + 1}, etl::Intervall{0u, steps/8});
                    deltaShift = x;
                }
                else if (v < mid) {
                    const T d = mid - v;            
                    const T x = etl::scale(d, etl::Intervall{0u, delta + 1}, etl::Intervall{0u, steps/8});
                    deltaShift = -x;
                }
                else {
                    deltaShift = 0;
                }   
            }
#endif            
            template<typename T, auto L, auto U>
            static inline void balance(const etl::uint_ranged<T, L, U>& b) {
                constexpr T mid = b.mid();
                constexpr T delta = (U - L) / 2;
                
                if (b > mid) {
                    const T d = b - mid;            
                    const T x = etl::scale(d, etl::Intervall{0u, delta}, etl::Intervall{0u, 16u});
                    mScale0.set(scale - x);
                    mScale1.setToTop();
                }
                else if (b < mid) {
                    const T d = mid - b;            
                    const T x = etl::scale(d, etl::Intervall{0u, delta}, etl::Intervall{0u, 16u});
                    mScale0.setToTop();
                    mScale1.set(scale - x);
                }
                else {
                    mScale0.setToTop();
                    mScale1.setToTop();
                }
            }

            template<typename T, auto L, auto U>
            static inline void current(const etl::uint_ranged<T, L, U>& b) {
                const T x = etl::scaleTo<scale_type>(b);
                mCurrentMax.set<etl::RangeCheck<false>>(x);
            }

            template<typename T, auto L, auto U>
            static inline void sine(const etl::uint_ranged<T, L, U>& s) {
                const T x = etl::scaleTo<etl::uint_ranged<uint8_t, 0, 7>>(s);
                mSineType = x;
            }
            
            inline static void setDuty() {
#ifndef NO_SHIFT
                index0 = index1.leftShift((steps / 2) + deltaShift);
#endif
                const auto s0 = std::min(mScale0, mCurrentMax);
                const auto s1 = std::min(mScale1, mCurrentMax);

                switch(mSineType) {
                case 1:
                    PWM::template duty<Meta::List<AVR::PWM::WO<0>>>((s0 * Sine12::value(index0)) >> scaleBits);
                    PWM::template duty<Meta::List<AVR::PWM::WO<1>>>((s1 * Sine12::value(index1)) >> scaleBits);
                    break;
                case 2:
                    PWM::template duty<Meta::List<AVR::PWM::WO<0>>>((s0 * Sine14::value(index0)) >> scaleBits);
                    PWM::template duty<Meta::List<AVR::PWM::WO<1>>>((s1 * Sine14::value(index1)) >> scaleBits);
                    break;
                case 3:
                    PWM::template duty<Meta::List<AVR::PWM::WO<0>>>((s0 * Sine15::value(index0)) >> scaleBits);
                    PWM::template duty<Meta::List<AVR::PWM::WO<1>>>((s1 * Sine15::value(index1)) >> scaleBits);
                    break;
                case 4:
                    PWM::template duty<Meta::List<AVR::PWM::WO<0>>>((s0 * Sine16::value(index0)) >> scaleBits);
                    PWM::template duty<Meta::List<AVR::PWM::WO<1>>>((s1 * Sine16::value(index1)) >> scaleBits);
                    break;
                case 5:
                    PWM::template duty<Meta::List<AVR::PWM::WO<0>>>((s0 * Sine18::value(index0)) >> scaleBits);
                    PWM::template duty<Meta::List<AVR::PWM::WO<1>>>((s1 * Sine18::value(index1)) >> scaleBits);
                    break;
                case 6:
                    PWM::template duty<Meta::List<AVR::PWM::WO<0>>>((s0 * Sine20::value(index0)) >> scaleBits);
                    PWM::template duty<Meta::List<AVR::PWM::WO<1>>>((s1 * Sine20::value(index1)) >> scaleBits);
                    break;
                case 0:
                default:
                    PWM::template duty<Meta::List<AVR::PWM::WO<0>>>((s0 * Sine10::value(index0)) >> scaleBits);
                    PWM::template duty<Meta::List<AVR::PWM::WO<1>>>((s1 * Sine10::value(index1)) >> scaleBits);
                    break;
                }
            }
            inline static void ratePeriodic() {
                const auto oldState = mState;
                switch(mState) {
                case State::Init:
                    mState = State::A;
                    break;
                case State::A:
                    setDuty();
                    if (index1.isBottom()){
                        mState = State::B;
                    }
                    break;
                case State::B:
                    setDuty();
                    if (index0.isBottom()){
                        mState = State::C;
                    }
                    break;
                case State::C:
                    setDuty();
                    if (index1.isBottom()){
                        mState = State::D;
                    }
                    break;
                case State::D:
                    setDuty();
                    if (index0.isBottom()){
                        mState = State::A;
                    }            
                    break;
                }
                if (oldState != mState) {
                    if (mDirection == Rot::Forward) {
                        switch(mState) {
                        case State::Init:
                            break;
                        case State::A:
                            set<Pol::Positiv, 0>();
                            break;
                        case State::B:
                            set<Pol::Negativ, 1>();
                            break;
                        case State::C:
                            set<Pol::Negativ, 0>();
                            break;
                        case State::D:
                            set<Pol::Positiv, 1>();
                            break;
                        }
                    }
                    else {
                        switch(mState) {
                        case State::Init:
                            break;
                        case State::A:
                            set<Pol::Negativ, 0>();
                            break;
                        case State::B:
                            set<Pol::Negativ, 1>();
                            break;
                        case State::C:
                            set<Pol::Positiv, 0>();
                            break;
                        case State::D:
                            set<Pol::Positiv, 1>();
                            break;
                        }
                    }
                }
#ifdef NO_SHIFT
                ++index0;
#endif
                ++index1;
            }  
        //private:    
            template<auto D, auto N>
            static inline void set() {
                PWM::onOvlWait([]{
                    if constexpr(D == Pol::Positiv) {
                        PWM::template on<Meta::List<AVR::PWM::WO<N>>>();
                        if constexpr(N == 0) {
                            Lut0::init(0xff_B);
                        }
                        else {
                            Lut1::init(0xff_B);
                        }
                    }
                    else {
                        PWM::template off<Meta::List<AVR::PWM::WO<N>>>();
                        PWM::template reset<AVR::PWM::WO<N>>();
                        if constexpr(N == 0) {
                            Lut0::init(~0xaa_B);
                        }
                        else {
                            Lut1::init(~0xcc_B);
                        }
                    }
                });
            }
            static inline uint8_t mSineType{0};
            
            static inline scale_type mCurrentMax{scale_type::Lower};
            
            static inline scale_type mScale0{scale};
            static inline scale_type mScale1{scale};
        
            static inline State mState{State::Init};
#ifndef NO_SHIFT
            static inline int16_t deltaShift{0};
#endif
            
            inline static index_type index0{steps / 2};
            inline static index_type index1{0};
            
            inline static Rot mDirection{Rot::Backward};
        };
    
        
        template<typename PWM, typename AN, typename BN, etl::Concepts::NamedFlag Invert, typename Lut0, typename Lut1>
        struct DirPhaseDriver {
            enum class State : uint8_t {
                Init, A, B, C, D
            };
            enum class Pol : uint8_t {Positiv, Negativ};
            enum class Rot : uint8_t {Forward, Backward};
        
            inline static constexpr bool invert = Invert::value;
            
            inline static constexpr uint16_t steps = 512;
            static_assert(etl::isPowerof2(steps));
            
            inline static constexpr uint16_t period = 1024; // 16KHz
            inline static constexpr uint8_t scaleBits = 5;
            inline static constexpr uint8_t scale = (1 << scaleBits);
        //    std::integral_constant<uint8_t, scale>::_;
            inline static constexpr uint16_t maxPeriodValue = period - 1;
            
            inline static constexpr uint8_t periodBits = etl::minimumBitsForValue(maxPeriodValue);
            inline static constexpr uint8_t maxScaleBits = 16 - periodBits;
            static_assert(maxScaleBits >= scaleBits);
            
            using scale_type = etl::uint_ranged<uint8_t, 1, scale>;
            using index_type = etl::uint_ranged_circular<uint16_t, 0, steps - 1> ;
            
            template<size_t Steps>    
            struct Generator {
                constexpr auto operator()() {
                    std::array<uint16_t, Steps> data;
                    for(uint16_t i = 0; i < Steps; ++i) {
                        data[i] = maxPeriodValue * sin((i * M_PI) / steps);
                    }
                    return data;
                }
            };
            using Sine = AVR::Pgm::Util::Converter<Generator<steps>>::pgm_type;
            
            inline static void init() {
                AN::template dir<AVR::Output>();
                BN::template dir<AVR::Output>();
                AN::off();
                BN::off();
        
                PWM::template init<Meta::List<AVR::PWM::WO<0>, AVR::PWM::WO<1>>>();
                PWM::period(period);
                PWM::template on<Meta::List<AVR::PWM::WO<0>, AVR::PWM::WO<1>>>();
            }  
            inline static void periodic() {
            }
        
            inline static void direction(const Rot d) {
                mDirection = d;
            }
            
            template<typename T, auto L, auto U>
            static inline void shift(const etl::uint_ranged<T, L, U>& b) {
                constexpr T mid = b.mid();
                constexpr T delta = (U - L) / 2;
                
                if (b > mid) {
                    const T d = b - mid;            
                    const T x = etl::scale(d, etl::Intervall{0u, delta + 1}, etl::Intervall{0u, steps/8});
                    deltaShift = x;
                }
                else if (b < mid) {
                    const T d = mid - b;            
                    const T x = etl::scale(d, etl::Intervall{0u, delta + 1}, etl::Intervall{0u, steps/8});
                    deltaShift = -x;
                }
                else {
                    deltaShift = 0;
                }   
            }
            
            template<typename T, auto L, auto U>
            static inline void balance(const etl::uint_ranged<T, L, U>& b) {
                constexpr T mid = b.mid();
                constexpr T delta = (U - L) / 2;
                
                if (b > mid) {
                    const T d = b - mid;            
                    const T x = etl::scale(d, etl::Intervall{0u, delta}, etl::Intervall{0u, 16u});
                    mScale0.set(scale - x);
                    mScale1.setToTop();
                }
                else if (b < mid) {
                    const T d = mid - b;            
                    const T x = etl::scale(d, etl::Intervall{0u, delta}, etl::Intervall{0u, 16u});
                    mScale0.setToTop();
                    mScale1.set(scale - x);
                }
                else {
                    mScale0.setToTop();
                    mScale1.setToTop();
                }
            }
            
            inline static void setDuty() {
                index0 = index_type((index1.toInt() + steps / 2 + deltaShift));
                
                PWM::template duty<Meta::List<AVR::PWM::WO<0>>>((mScale0 * Sine::value(index0)) >> scaleBits);
                PWM::template duty<Meta::List<AVR::PWM::WO<1>>>((mScale1 * Sine::value(index1)) >> scaleBits);
            }
            inline static void ratePeriodic() {
                const auto oldState = mState;
                switch(mState) {
                case State::Init:
                    mState = State::A;
                    break;
                case State::A:
                    setDuty();
                    if (index1.isBottom()){
                        mState = State::B;
                    }
                    break;
                case State::B:
                    setDuty();
                    if (index0.isBottom()){
                        mState = State::C;
                    }
                    break;
                case State::C:
                    setDuty();
                    if (index1.isBottom()){
                        mState = State::D;
                    }
                    break;
                case State::D:
                    setDuty();
                    if (index0.isBottom()){
                        mState = State::A;
                    }            
                    break;
                }
                if (oldState != mState) {
                    if (mDirection == Rot::Forward) {
                        switch(mState) {
                        case State::Init:
                            break;
                        case State::A:
                            set<Pol::Positiv, AN, 0>();
                            break;
                        case State::B:
                            set<Pol::Negativ, BN, 1>();
                            break;
                        case State::C:
                            set<Pol::Negativ, AN, 0>();
                            break;
                        case State::D:
                            set<Pol::Positiv, BN, 1>();
                            break;
                        }
                    }
                    else {
                        switch(mState) {
                        case State::Init:
                            break;
                        case State::A:
                            set<Pol::Negativ, AN, 0>();
                            break;
                        case State::B:
                            set<Pol::Negativ, BN, 1>();
                            break;
                        case State::C:
                            set<Pol::Positiv, AN, 0>();
                            break;
                        case State::D:
                            set<Pol::Positiv, BN, 1>();
                            break;
                        }
                    }
                }
                ++index1;
            }  
        private:    
            template<auto D, typename P, auto N>
            static inline void set() {
                PWM::onOvlWait([]{
                    P::template dir<AVR::Input>();
                    if constexpr(D == Pol::Positiv) {
                        if constexpr(N == 0) {
                            if constexpr(!std::is_same_v<Lut0, void>) {
                                Lut0::init(0xaa_B);
                            }
                        }
                        else {
                            if constexpr(!std::is_same_v<Lut1, void>) {
                                Lut1::init(0xcc_B);
                            }
                        }
                        PWM::template noinvert<AVR::PWM::WO<N>>();
                        P::off();
                    }
                    else {
                        if constexpr(invert) {
                            if constexpr(N == 0) {
                                if constexpr(!std::is_same_v<Lut0, void>) {
                                    Lut0::init(~0xaa_B);
                                }
                            }
                            else {
                                if constexpr(!std::is_same_v<Lut1, void>) {
                                    Lut1::init(~0xcc_B);
                                }
                            }
                            PWM::template invert<AVR::PWM::WO<N>>();
                        }
                        P::on();
                    }
                    P::template dir<AVR::Output>();
                });
            }
            static inline scale_type mScale0{scale};
            static inline scale_type mScale1{scale};
        
            static inline State mState{State::Init};
            static inline int16_t deltaShift{0};
            
            inline static index_type index0{steps / 2};
            inline static index_type index1{0};
            
            inline static Rot mDirection{Rot::Backward};
        };
    }
}
