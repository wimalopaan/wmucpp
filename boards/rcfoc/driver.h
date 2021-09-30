#pragma once

template<typename PWM, typename StepsPerRotation = std::integral_constant<size_t, 2048>, typename Poles = std::integral_constant<uint8_t, 7>, typename ST = etl::uint_ranged<uint16_t, 0, 1023>>
struct Driver {
    using pwm = PWM;
    static inline constexpr uint16_t steps = StepsPerRotation::value; // generate 4k-table (use O(log(N)) in pgm generator
    
    using index_type = etl::uint_ranged<uint16_t, 0, steps - 1>;
    using angle_type = etl::uint_ranged_circular<uint16_t, 0, steps - 1>;
    
    static inline constexpr uint16_t period = 2048; // 16 KHz
    static inline constexpr uint8_t periodBits = etl::minimumBitsForValue(period - 1);
    static inline constexpr uint8_t scaleShift = 16 - periodBits;
//    static inline constexpr uint8_t scaleBits = 16 - periodBits;
//    static inline constexpr uint8_t scaleMax = (1 << scaleBits) - 1;
    
//    using scale_type = etl::uint_ranged<uint8_t, 0, scaleMax>;
    using scale_type = ST;
    using torque_type = etl::int_ranged<int16_t, -int16_t(scale_type::Upper), scale_type::Upper>;
    
    static inline constexpr uint8_t poles = Poles::value;

    static inline constexpr uint16_t piHalf = steps / (4 * poles);

    static inline constexpr uint16_t electric_convolution = period / poles;
    
    template<size_t Steps, size_t Shift>    
    struct Generator {
        constexpr auto operator()() {
            std::array<uint8_t, Steps> data;
            for(uint16_t i = 0; i < Steps; ++i) {
                const uint16_t ie = i * poles;
                data[i] = 255.0 * (1.0 + sin(((ie - Shift) * 2.0 * M_PI) / Steps)) / 2.0;
            }
            return data;
        }
    };
    using Sine0 = AVR::Pgm::Util::Converter<Generator<steps, 0>>::pgm_type;
    using Sine1 = AVR::Pgm::Util::Converter<Generator<steps, (steps / 3)>>::pgm_type;
    using Sine2 = AVR::Pgm::Util::Converter<Generator<steps, ((2 * steps) / 3)>>::pgm_type;
    
    static inline angle_type normalizedOffset() {
        uint16_t v = mIndex;
        while(v > electric_convolution) {
            v -= electric_convolution;
        }
        return angle_type{v};
    }
    
    
    static inline void init() {
        pwm::init();
        pwm::period(period);
        pwm::template on<Meta::List<AVR::PWM::WO<0>, AVR::PWM::WO<1>, AVR::PWM::WO<2>>>();
    }
    static inline void periodic() {
    }
    static inline void ratePeriodic() {
    }
    static inline void angleMaxTorqueLeft(const index_type& a, const index_type& offset) {
        uint16_t i = a.toInt() + offset.toInt() + piHalf;
        while(i >= steps) {
            i -= steps;    
        }
        mIndex = index_type{i};
        setDuty();
    }
    
    static inline void torque(const torque_type& t, const index_type& a, const index_type& offset) {
        const int16_t t_pi2 = std::clamp(t.toInt(), -int16_t(piHalf), int16_t(piHalf));
        int16_t i = a.toInt() + offset.toInt() + t_pi2;
        while(i >= int16_t(steps)) {
            i -= steps;    
        }
        while(i < 0) {
            i += steps;    
        }
        mIndex.set(i);
        setDuty();
    }
    static inline void torque(const int32_t& t, const index_type& a, const index_type& offset) {
        const int16_t t_pi2 = std::clamp(t, -int32_t(piHalf), int32_t(piHalf));
        int16_t i = a.toInt() + offset.toInt() + t_pi2;
        while(i >= int16_t(steps)) {
            i -= steps;    
        }
        while(i < 0) {
            i += steps;    
        }
        mIndex.set(i);
        setDuty();
    }

    static inline void angleMaxTorqueRight(const index_type& a, const index_type& offset) {
        int16_t i = a.toInt() + offset.toInt() - piHalf;
        while(i >= steps) {
            i -= steps;    
        }
        while(i < 0) {
            i += steps;    
        }
        mIndex.set(i);
        setDuty();
    }
    static inline void angle(const index_type& a) {
        mIndex = a;
        setDuty();
    }
    static inline void angle(const angle_type& a) {
        mIndex.set(a.toInt());
        setDuty();
    }
    
    static inline void scale(const scale_type& s) {
        constexpr uint8_t scaleBits = etl::minimumBitsForValue(scale_type::Upper);
        static_assert(scaleBits >= 8);
        constexpr uint8_t shift = scaleBits - 8;
        mScale = s.toInt() >> shift;
    }
    
private:    
    inline static void setDuty() {
        PWM::template duty<Meta::List<AVR::PWM::WO<0>>>((uint16_t(mScale) * Sine0::value(mIndex)) >> scaleShift);
        PWM::template duty<Meta::List<AVR::PWM::WO<1>>>((uint16_t(mScale) * Sine1::value(mIndex)) >> scaleShift);
        PWM::template duty<Meta::List<AVR::PWM::WO<2>>>((uint16_t(mScale) * Sine2::value(mIndex)) >> scaleShift);
    }
    static inline uint8_t mScale{0};
    
    inline static index_type mIndex{0};
};
