#pragma once

#include <cstdint>
#include "fixedpoint.h"

namespace Control {
    template<typename ValueType = uint16_t, typename T = etl::FixedPoint<int16_t, 10>>
    struct PD {
//        template<typename Stream, typename V, typename F> friend constexpr inline void out_impl(const PD<V, F>& pid);

        using value_type = ValueType;
        using error_type = std::make_signed_t<value_type>;
        using factor_type = T;
        
        constexpr PD(const factor_type& p, const factor_type& d, const value_type& maxError, const value_type& maxCv) : 
            pFactor(p), dFactor(d), maxError(maxError), maxCorrection{maxCv} {}
        
        constexpr inline error_type correctionValue(const value_type& actual, const value_type& setPoint) {
            av = actual;
            sp = setPoint;
            const error_type error = std::clamp(actual - setPoint, -maxError, maxError);
            ev = error;
            const error_type pterm = error * pFactor;
            pt = pterm;
        
            const error_type dterm = (actual - prevValue) * dFactor;    
            prevValue = actual;
            
            if constexpr(!std::is_same_v<T, float>) {
                return cv = std::clamp(pterm + dterm, -maxCorrection, maxCorrection);
            }
            else {
                return cv = std::clamp((value_type)(pterm + dterm), -maxCorrection, maxCorrection);
            }
        }
        constexpr inline void reset() {
            prevValue = value_type{0};
        }
    private:
        error_type cv{};
        error_type pt{};
        value_type prevValue{0};
        value_type av{};
        value_type sp{};
        value_type ev{};
        factor_type di{};
        const factor_type pFactor;
        const factor_type dFactor;
        const value_type maxError{0};
        const value_type maxCorrection{0};
    };
    
    template<typename ValueType = uint16_t, typename T = etl::FixedPoint<int16_t, 10>>
    struct PID {
        template<typename Stream, typename V, typename F> friend constexpr inline void out_impl(const PID<V, F>& pid);

        using value_type = ValueType;
        using error_type = std::make_signed_t<value_type>;
        using factor_type = T;
        
        constexpr PID(const factor_type& p, const factor_type& i, const factor_type& d, const value_type& maxError, const value_type& maxCv) : 
            pFactor(p), iFactor(i), dFactor(d), maxError(maxError), maxCorrection{maxCv} {}
        
        constexpr inline error_type correctionValue(const value_type& actual, const value_type& setPoint) {
            av = actual;
            sp = setPoint;
            const error_type error = std::clamp(actual - setPoint, -maxError, maxError);
            ev = error;
            const error_type pterm = error * pFactor;
            pt = pterm;
            di = iFactor * error;
        
            if (di > factor_type{0.0}) {
                if constexpr(!std::is_same_v<T, float>) {
                    if (integral < (integral.max() - di)) {
                        integral += di;
                    }
                }
                else {
                    integral += di;
                }
            }
            else if (di < factor_type{0.0}) {
                if constexpr(!std::is_same_v<T, float>) {
                    if (integral > (integral.min() - di)) {
                        integral += di;
                    }
                }
                else {
                    integral += di;
                    
                }
            }

            const error_type dterm = (actual - prevValue) * dFactor;    
            prevValue = actual;
            
            if constexpr(!std::is_same_v<T, float>) {
                return cv = std::clamp(pterm + integral.integer() + dterm, -maxCorrection, maxCorrection);
            }
            else {
                return cv = std::clamp((value_type)(pterm + integral + dterm), -maxCorrection, maxCorrection);
            }
        }
        constexpr inline void reset() {
            prevValue = value_type{0};
            integral = factor_type{0.0};
        }
    private:
        error_type cv{};
        error_type pt{};
        value_type prevValue{0};
        value_type av{};
        value_type sp{};
        value_type ev{};
        factor_type di{};
        factor_type integral{0.0};
        const factor_type pFactor;
        const factor_type iFactor;
        const factor_type dFactor;
        const value_type maxError{0};
        const value_type maxCorrection{0};
    };
    
    template<typename Stream, typename V, typename F> 
    constexpr inline void out_impl(const PID<V, F>& pid) {
        if constexpr(!std::is_same_v<F, float>) {
            etl::out<Stream>("sp: "_pgm, pid.sp, " in: "_pgm, pid.av, " er: "_pgm, pid.ev, " pt: "_pgm, pid.pt, " it:"_pgm, pid.integral, " cv:"_pgm, pid.cv);
        }
        else {
            etl::out<Stream>("sp: "_pgm, pid.sp, " in: "_pgm, pid.av, 
                             " er: "_pgm, (V)pid.ev, " pt: "_pgm, (V)pid.pt, " it:"_pgm, (V)pid.integral, " cv:"_pgm, pid.cv);
        }
    }
}
