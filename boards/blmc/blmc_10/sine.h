#pragma once

#include <mcu/avr.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/port.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/adcomparator.h>
#include <mcu/internals/capture.h>
#include <mcu/internals/pwm.h>

#include <external/units/physical.h>
#include <external/units/percent.h>

#include <etl/fixedpoint.h>
#include <etl/output.h>

#include <std/chrono>

#include <math.h>

namespace BLDC {
    using namespace AVR;
    using namespace External::Units;
    using namespace External::Units::literals;
    using namespace std::literals::chrono;
    
    namespace Sine4 {
        
        template<typename RotTimer, typename ComTimer, typename PWM, typename Commuter, typename AC>
        struct Controller {
            using all_channels = typename PWM::all_channels;
            static inline constexpr uint16_t PwmMax = 100;
            using value_type = etl::typeForValue_t<PwmMax>;
            using scale_type = etl::ScaledInteger<etl::uint_ranged<value_type, 1, 100>, std::ratio<1,100>>;
            
            static inline constexpr auto gen = []<uint16_t Size, uint16_t Max = 1000>(std::integral_constant<uint16_t, Size>, 
                                                                                      std::integral_constant<uint16_t, Max> = std::integral_constant<uint16_t, Max>{}){
                                                                                using value_type = etl::typeForValue_t<Max>;        
                                                                                std::array<value_type, Size> t;
                                                                                for(uint16_t i = 0; i < t.size(); ++i) {
                                                                                    t[i] = (Max / 2) * (cos((i * 2 * M_PI) / t.size()) + 1.0);
            }
                                                                                return t;
                                                                        };
        
        enum class State : uint8_t {Off, Sine, Align, Align2, RampUp, ClosedLoop, ClosedLoopError};

        struct AcHandler : AVR::IsrBaseHandler<AVR::ISR::AdComparator<0>::Edge>{
            static inline void isr() {
                AC::onInterrupt([&](){
//                    static uint16_t last = 0;

//                    uint16_t actual = RotTimer::counter();
//                    if (actual >= last) {
//                        mCommutationDiff = actual - last;
//                    }
//                    else {
//                        mCommutationDiff = actual + (std::numeric_limits<uint16_t>::max() - last); 
//                    }
//                    last = actual;
                    
                    uint8_t c = 0;
                    for(uint8_t i = 0; i < mMaxDelay; ++i) {
                        if (!(AC::get() ^ AC::get())) {
                            if(++c > mDelay) {
                                break;
                            }
                        }
                        else {
                            c = 0;
                        }
                    }
//                    ++mCommutes;
                    if (mState == State::ClosedLoop) {
                        Commuter::next();
                    }
                });
            };
        };
        
        inline static volatile State mState = State::Off;

        inline static  uint8_t mStep = 0;
        inline static uint8_t mDelayFactor = 4;
//        inline static uint16_t mErrorCount = 0;
//        inline static uint8_t mGoodCommutes = 10;
//        inline static volatile uint16_t mCommutationDiff = 0;
//        inline static volatile uint16_t mCommutes = 0;
        inline static volatile uint16_t mDelay = 4;
        inline static volatile uint16_t mMaxDelay = 10;

        inline static void closedLoop(bool on) {
            if (on) {
                mState = State::ClosedLoop;
            }
            else {
                mState = State::Sine;
            }
        }
        
                
        struct ComTimerHandler : AVR::IsrBaseHandler<typename ComTimer::interrupt_type>{
            static inline void isr() {
                ComTimer::onInterrupt([&]{
                    isrPeriodic();
                });
            }
        };
        
        inline static void init() {
            AC::init();
            AC::template enableInterrupts<false>();
            Commuter::init();
            RotTimer::init();
            ComTimer::init();
            ComTimer::period(mActualPeriod);
            ComTimer::template enableInterrupts<false>();
        }
        
        template<uint16_t Size>
        inline static auto setSine() {
            static constexpr auto sine_table = gen(std::integral_constant<uint16_t, Size>{}, std::integral_constant<uint16_t, PwmMax>{});
            using size_type = decltype(sine_table)::size_type;
            using value_type = decltype(sine_table)::value_type;
            constexpr size_type shift = sine_table.size() / 3;
            
            static etl::uint_ranged_circular<size_type, 0, Size - 1> index{}; 
            
            std::array<value_type, 3> v;
            v[0] = sine_table[index];
            v[1] = sine_table[index.template leftShift<shift>()];
            v[2] = sine_table[index.template leftShift<2 * shift>()];
            
            ++index;
            if (index == 0) {
                if (mState == State::ClosedLoop) {
//                    Commuter::template floating<0>();
//                    Commuter::template floating<1>();
//                    Commuter::template floating<2>();
                    
                    ComTimer::template enableInterrupts<false>();
                    AC::template enableInterrupts<true>();
                    
                    Commuter::set(typename Commuter::state_type{1});
                    
//                    PWM::template duty<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>(70);
                    PWM::template duty<all_channels>(PwmMax * mScale);

                    leave = true;
                }
                else {
                    mOldActualSineTable = mActualSineTable;
                }
            }
            return v;
        }
        
        inline static volatile bool leave = false;
        
        inline static void isrPeriodic() {
            using value_type = etl::typeForValue_t<PwmMax>;
            std::array<value_type, 3> v;
//            decltype(v)::_;
            
            switch (mOldActualSineTable) {
            case 0:
                v = setSine<6 * 100>();
                break;
            case 1:
                v = setSine<6 * 50>();
                break;
            case 2:
                v = setSine<6 * 25>();
                break;
            case 3:
                v = setSine<6 * 12>();
                break;
            case 4:
                v = setSine<6 * 6>();
                break;
            case 5:
                v = setSine<6 * 3>();
                break;
            default:
                break;
            }
                       
            if (leave) {
                leave = false;
                return;
            }
            
            PWM::template duty<Meta::List<AVR::PWM::WO<0>>>(v[0] * mScale);
            PWM::template duty<Meta::List<AVR::PWM::WO<1>>>(v[1] * mScale);
            PWM::template duty<Meta::List<AVR::PWM::WO<2>>>(v[2] * mScale);
        }
        
        inline static void periodic() {
        }
        
        
        inline static void speed(etl::uint_ranged<uint16_t, 0, 160> s) {
            constexpr uint16_t sMin = 10;
            constexpr uint32_t periodBase = 20000;
            uint8_t t = 0;
//            leave = false;
            
            if (s < sMin) {
                off();
                mState = State::Off;
                return;
            }
            else if ((s >= sMin) && (s < (2 * sMin))) {
                mActualPeriod = periodBase - (periodBase * (s - sMin)) / 20 ;
                t = 0;
            }
            else if ((s >= 2* sMin) && (s < (4 * sMin))) {
                mActualPeriod = periodBase - (periodBase * (s - 2*sMin)) / 40 ;
                t = 1;
            }
            else if ((s >= 4* sMin) && (s < (8 * sMin))) {
                mActualPeriod = periodBase - (periodBase * (s - 4*sMin)) / 80 ;
                t = 2;
            }
            else if ((s >= 8* sMin) && (s <= (16 * sMin))) {
                mActualPeriod = periodBase - (periodBase * (s - 8*sMin)) / 160 ;
                t = 3;
            }
            
            mState = State::Sine;
            AC::template enableInterrupts<false>();
            ComTimer::template enableInterrupts<true>();
            
            ComTimer::period(mActualPeriod);
            mActualSineTable = t;
            Commuter::template pwm<0>();
            Commuter::template pwm<1>();
            Commuter::template pwm<2>();
        }
        
        
        inline static void off() {
            mDoFloat = false;
            Commuter::off();
            AC::template enableInterrupts<false>();
        }
        
        inline static void start() {
            AC::template enableInterrupts<false>();
            ComTimer::template enableInterrupts<true>();
            mScale = scale_type{10};
            mActualSineTable = 0;
            mOldActualSineTable = 0;
            mActualPeriod = 20000;
            ComTimer::period(mActualPeriod);
        }
        
        inline static void pwmInc() {
            ++mScale;
            PWM::template duty<all_channels>(PwmMax * mScale);
        }
        inline static void pwmDec() {
            --mScale;
            PWM::template duty<all_channels>(PwmMax * mScale);
        }
        inline static void incPeriod() {
            mActualPeriod += 1000U;
            ComTimer::period(mActualPeriod);
        }
        inline static void decPeriod() {
            mActualPeriod -= 1000U;
            ComTimer::period(mActualPeriod);
        }
        inline static void nextTable() {
            ++mActualSineTable;
        }
        inline static void prevTable() {
            --mActualSineTable;
        }
        
        static inline volatile scale_type mScale{100};
        
        //            inline static uint16_t mActualPwm = 0;
        inline static volatile etl::uint_ranged<uint16_t, 1000, 40000> mActualPeriod{20000};
        inline static volatile etl::uint_ranged<uint8_t, 0, 5> mActualSineTable{};
        inline static etl::uint_ranged<uint8_t, 0, 5> mOldActualSineTable{};
        
        inline static bool mDoFloat = false;
        
//        inline static volatile Commuter::index_type mIndex;
    };
}

namespace Sine3 {
    
    template<typename RotTimer, typename ComTimer, typename PWM, typename Commuter>
    struct Controller {
        
        static inline constexpr auto gen = []<uint16_t Size, uint16_t Max = 1000>(std::integral_constant<uint16_t, Size>, std::integral_constant<uint16_t, Max> = std::integral_constant<uint16_t, Max>{}){
                                                                            std::array<uint16_t, Size> t;
                                                                            for(uint16_t i = 0; i < t.size(); ++i){
            t[i] = (Max / 2) * (cos((i * 2 * M_PI) / t.size()) + 1.0);
        }
        return t;
    };
    
    struct ComTimerHandler : AVR::IsrBaseHandler<typename ComTimer::interrupt_type>{
        static inline void isr() {
            ComTimer::onInterrupt([&]{
                isrPeriodic();
            });
        }
    };
    
    inline static void init() {
        Commuter::init();
        RotTimer::init();
        ComTimer::init();
        ComTimer::period(mActualPeriod);
        ComTimer::template enableInterrupts<true>();
    }
    
    template<uint16_t Size>
    inline static auto setSine() {
        static constexpr auto sine_table = gen(std::integral_constant<uint16_t, Size>{});
        using size_type = decltype(sine_table)::size_type;
        constexpr size_type shift = sine_table.size() / 3;
        
        static etl::uint_ranged_circular<size_type, 0, Size - 1> index{}; 
        static etl::uint_ranged_circular<size_type, 0, shift / 2> offset{}; 
        ++offset;
        if (offset == 0) {
            ++mIndex;
        }
        
        std::array<uint16_t, 3> v;
        v[0] = sine_table[index];
        v[1] = sine_table[index.template leftShift<shift>()];
        v[2] = sine_table[index.template leftShift<2 * shift>()];
        ++index;
        if (index == 0) {
            mOldActualSineTable = mActualSineTable;
        }
        return v;
        //                std::integral_constant<uint16_t, sine_table[256]>::_;
    }
    
    inline static void isrPeriodic() {
        std::array<uint16_t, 3> v;
        
        switch (mOldActualSineTable) {
        case 0:
            v = setSine<512>();
            break;
        case 1:
            v = setSine<256>();
            break;
        case 2:
            v = setSine<128>();
            break;
        case 3:
            v = setSine<64>();
            break;
        case 4:
            v = setSine<32>();
            break;
        case 5:
            v = setSine<16>();
            break;
        default:
            break;
        }
        
        PWM::template duty<Meta::List<AVR::PWM::WO<0>>>((10 * v[0]) / mScale);
        PWM::template duty<Meta::List<AVR::PWM::WO<1>>>((10 * v[1]) / mScale);
        PWM::template duty<Meta::List<AVR::PWM::WO<2>>>((10 * v[2]) / mScale);
        
        if (mDoFloat) {
            constexpr uint16_t max = 1000;
            constexpr uint16_t delta = max / 5;
            constexpr uint16_t vmin = (max / 2) - delta;
            constexpr uint16_t vmax = (max / 2) + delta;
            
            if ((v[0] > vmin) && (v[0] < vmax)) {
                Commuter::template floating<0>();
            }
            else {
                Commuter::template pwm<0>();
            }
            if ((v[1] > vmin) && (v[1] < vmax)) {
                Commuter::template floating<1>();
            }
            else {
                Commuter::template pwm<1>();
            }
            if ((v[2] > vmin) && (v[2] < vmax)) {
                Commuter::template floating<2>();
            }
            else {
                Commuter::template pwm<2>();
            }
        } 
    }
    
    static inline volatile uint16_t mScale = 200;
    
    inline static void periodic() {
    }
    
    inline static void doFloat( bool f) {
        if (f) {
            mDoFloat = true;
        }
        else {
            mDoFloat = false;
            Commuter::template pwm<0>();
            Commuter::template pwm<1>();
            Commuter::template pwm<2>();
        }
    }
    
    inline static void speed(etl::uint_ranged<uint16_t, 0, 160> s) {
        constexpr uint16_t sMin = 10;
        constexpr uint32_t periodBase = 20000;
        uint8_t t = 0;
        
        if (s < sMin) {
            off();
            return;
        }
        else if ((s >= sMin) && (s < (2 * sMin))) {
            mActualPeriod = periodBase - (periodBase * (s - sMin)) / 20 ;
            t = 0;
        }
        else if ((s >= 2* sMin) && (s < (4 * sMin))) {
            mActualPeriod = periodBase - (periodBase * (s - 2*sMin)) / 40 ;
            t = 1;
        }
        else if ((s >= 4* sMin) && (s < (8 * sMin))) {
            mActualPeriod = periodBase - (periodBase * (s - 4*sMin)) / 80 ;
            t = 2;
        }
        else if ((s >= 8* sMin) && (s <= (16 * sMin))) {
            mActualPeriod = periodBase - (periodBase * (s - 8*sMin)) / 160 ;
            t = 3;
        }
        
        //                constexpr uint16_t scaleBase = 30;
        //                uint8_t sc = scaleBase - (scaleBase * (s - sMin)) / (3 * 150);
        //                mScale = sc;
        
        ComTimer::period(mActualPeriod);
        mActualSineTable = t;
        Commuter::template pwm<0>();
        Commuter::template pwm<1>();
        Commuter::template pwm<2>();
    }
    
    
    inline static void off() {
        mDoFloat = false;
        Commuter::off();
    }
    inline static void start() {
        mScale = 200;
        mActualSineTable = 0;
        mOldActualSineTable = 0;
        mActualPeriod = 20000;
        ComTimer::period(mActualPeriod);
        doFloat(false);
    }
    inline static void pwmInc() {
        if (mScale > 50) {
            mScale = mScale - 1;
        }
    }
    inline static void pwmDec() {
        if (mScale < 300) {
            mScale = mScale + 1;
        }
    }
    inline static void incPeriod() {
        if (mActualPeriod < 64000) {
            mActualPeriod = mActualPeriod + 1000;
        } 
        ComTimer::period(mActualPeriod);
    }
    inline static void decPeriod() {
        if ( mActualPeriod > 1000) {
            mActualPeriod = mActualPeriod - 1000;
        }
        ComTimer::period(mActualPeriod);
    }
    inline static void nextTable() {
        ++mActualSineTable;
    }
    inline static void prevTable() {
        --mActualSineTable;
    }
    
    //            inline static uint16_t mActualPwm = 0;
    inline static volatile uint16_t mActualPeriod = 20000;
    inline static volatile etl::uint_ranged<uint8_t, 0, 5> mActualSineTable{};
    inline static etl::uint_ranged<uint8_t, 0, 5> mOldActualSineTable{};
    
    inline static bool mDoFloat = false;
    
    inline static Commuter::state_type mIndex;
};
}


namespace Sine2 {
    
    template<typename RotTimer, typename ComTimer, typename PWM>
    struct Controller {
        
        using all_channels = typename PWM::all_channels;
        
        static inline constexpr auto gen = []<uint16_t Size, uint16_t Max = 1000>(std::integral_constant<uint16_t, Size>, std::integral_constant<uint16_t, Max> = std::integral_constant<uint16_t, Max>{}){
                                                                            std::array<uint16_t, Size> t;
                                                                            for(uint16_t i = 0; i < t.size(); ++i){
            t[i] = (Max / 2) * (cos((i * 2 * M_PI) / t.size()) + 1.0);
        }
        return t;
    };
    
    struct ComTimerHandler : AVR::IsrBaseHandler<typename ComTimer::interrupt_type>{
        static inline void isr() {
            ComTimer::onInterrupt([&]{
                isrPeriodic();
            });
        }
    };
    
    inline static void init() {
        RotTimer::init();
        ComTimer::init();
        ComTimer::period(mActualPeriod);
        ComTimer::template enableInterrupts<true>();
    }
    
    template<uint16_t Size>
    inline static auto setSine() {
        using index_t = etl::uint_ranged_circular<uint16_t,0,Size - 1>; 
        static index_t index; 
        static constexpr auto sine_table = gen(std::integral_constant<uint16_t, Size>{});
        constexpr uint16_t shift = sine_table.size() / 3;
        std::array<uint16_t, 3> v;
        v[0] = sine_table[index];
        v[1] = sine_table[index.template leftShift<shift>()];
        v[2] = sine_table[index.template leftShift<2 * shift>()];
        ++index;
        if (index == sine_table.size()) {
            index = index_t{0};
            mOldActualSineTable = mActualSineTable;
        }
        return v;
        //            std::integral_constant<uint16_t, sine_table[0]>::_;
        //                std::integral_constant<uint16_t, sine_table[256]>::_;
    }
    
    inline static void isrPeriodic() {
        std::array<uint16_t, 3> v;
        
        switch (mOldActualSineTable) {
        case 0:
            v = setSine<512>();
            break;
        case 1:
            v = setSine<256>();
            break;
        case 2:
            v = setSine<128>();
            break;
        case 3:
            v = setSine<64>();
            break;
        case 4:
            v = setSine<32>();
            break;
        case 5:
            v = setSine<16>();
            break;
        default:
            break;
        }
        
        PWM::template duty<Meta::List<AVR::PWM::WO<0>>>(v[0] / mScale);
        PWM::template duty<Meta::List<AVR::PWM::WO<1>>>(v[1] / mScale);
        PWM::template duty<Meta::List<AVR::PWM::WO<2>>>(v[2] / mScale);
        
    }
    
    static inline volatile uint16_t mScale = 20;
    
    inline static void periodic() {
    }
    
    inline static void off() {
        PWM::template off<all_channels>();
    }
    inline static void start() {
        mScale = 20;
        mActualSineTable = 0;
        mOldActualSineTable = 0;
        mActualPeriod = 20000;
        ComTimer::period(mActualPeriod);
        PWM::template on<all_channels>();
    }
    inline static void pwmInc() {
        if (mScale > 5) {
            mScale = mScale - 1;
        }
    }
    inline static void pwmDec() {
        if (mScale < 30) {
            mScale = mScale + 1;
        }
    }
    inline static void incPeriod() {
        if (mActualPeriod < 64000) {
            mActualPeriod += 1000;
        } 
        ComTimer::period(mActualPeriod);
    }
    inline static void decPeriod() {
        if ( mActualPeriod > 1000) {
            mActualPeriod -= 1000;
        }
        ComTimer::period(mActualPeriod);
    }
    inline static void nextTable() {
        ++mActualSineTable;
    }
    inline static void prevTable() {
        --mActualSineTable;
    }
    
    //            inline static uint16_t mActualPwm = 0;
    inline static uint16_t mActualPeriod{20000};
    inline static etl::uint_ranged<uint8_t, 0, 5> mActualSineTable{};
    inline static etl::uint_ranged<uint8_t, 0, 5> mOldActualSineTable{};
};
}

namespace Sine {
    
    template<typename RotTimer, typename ComTimer, typename PWM>
    struct Controller {
        
        inline static constexpr auto sine_table = []{
            std::array<uint16_t, 256> t;
            for(uint16_t i = 0; i < t.size(); ++i){
                t[i] = 500 * (cos((i * 2 * M_PI) / t.size()) + 1.0);
            }
            return t;
        }();
        
        //            std::integral_constant<uint16_t, sine_table[2100]>::_;
        
        inline static void init() {
            RotTimer::init();
            ComTimer::init();
            mActualPwm = 0;
            
        }
        
        inline static void ratePeriodic() {
            constexpr uint16_t shift = sine_table.size() / 3;
            //                static uint16_t index = 0;
            static etl::uint_ranged_circular<uint16_t, 0, sine_table.size() - 1> index;
            
            //                decltype(index + shift)::_;
            
            uint16_t v0 = sine_table[index];
            uint16_t v1 = sine_table[index.template leftShift<shift>()];
            uint16_t v2 = sine_table[index.template leftShift<2 * shift>()];
//            uint16_t v1 = sine_table[(index + shift).toInt()];
//            uint16_t v2 = sine_table[(index + 2 * shift).toInt()];
            //                uint16_t v0 = sine_table[index];
            //                uint16_t v1 = sine_table[(index + shift) % sine_table.size()];
            //                uint16_t v2 = sine_table[(index + 2 * shift) % sine_table.size()];
            
            constexpr uint16_t scale = 20;
            
            v0 /= scale;
            v1 /= scale;
            v2 /= scale;
            
            PWM::template duty<Meta::List<AVR::PWM::WO<0>>>(v0);
            PWM::template duty<Meta::List<AVR::PWM::WO<1>>>(v1);
            PWM::template duty<Meta::List<AVR::PWM::WO<2>>>(v2);
            
            //                index = (index + 1) % sine_table.size();
            ++index;
        }
        
        inline static void periodic() {
            
        }
        
        inline static void off() {
            
        }
        inline static void start() {
            
        }
        inline static void pwmInc() {
            
        }
        inline static void pwmDec() {
            
        }
        
        inline static uint16_t mActualPwm;
        
    };
}
}
