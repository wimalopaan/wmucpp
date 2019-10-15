#pragma once

#include <cstdint>
#include "fixedpoint.h"

namespace Control {
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
            error_type error = std::clamp(actual - setPoint, -maxError, maxError);
            ev = error;
            error_type pterm = error * pFactor;
            pt = pterm;
            di = iFactor * error;
        
            if (di > factor_type{0.0}) {
                if (integral < (integral.max() - di)) {
                    integral += di;
                }
            }
            else if (di < factor_type{0.0}) {
                if (integral > (integral.min() - di)) {
                    integral += di;
                }
            }
            
            error_type dterm = (actual - prevValue) * dFactor;    
            prevValue = actual;
            
            return cv = std::clamp(pterm + integral.integer() + dterm, -maxCorrection, maxCorrection);
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
        etl::out<Stream>("in: "_pgm, pid.av, " er: "_pgm, pid.ev, " pt: "_pgm, pid.pt, " it:"_pgm, pid.integral, " cv:"_pgm, pid.cv);
    }
}
