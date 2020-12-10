#pragma once

#include "mcu/internals/eeprom.h"

template<typename ADC, uint8_t Channel>
struct ACS723U40Provider {
    struct OffsetProvider {
        inline static constexpr auto ibus_type = IBus::Type::type::BAT_CURR;
        inline static constexpr void init() {}
        inline static constexpr uint16_t value() {
            return offset();
        }
    };
    
    using index_t = ADC::index_type;
    static_assert(Channel <= index_t::Upper);
    inline static constexpr auto channel = index_t{Channel};
#ifdef FS_I6S
    inline static constexpr auto ibus_type = IBus::Type::type::TEMPERATURE; // FS-I6S zeigt keinen Strom an
#else
    inline static constexpr auto ibus_type = IBus::Type::type::BAT_CURR;
#endif
    inline static constexpr void init() {}

    inline static constexpr void accumulateCalibration() {
        if (!mCCount.isTop()) {
            ++mCCount;
            mCalib += ADC::value(channel).toInt();
        }
    }
    inline static constexpr void setCalibration() {
        mOffset = mCalib / mCCount;
        mOffset *= 34;
        mOffset >>= 3;
    }
    inline static constexpr uint16_t offset() {
        return mOffset;
    }
    
    inline static constexpr uint16_t value() {
#ifdef FS_I6S
        return currentConverter::convert(ADC::value(channel)).value / 10 + 400;
#else
        const auto raw = ADC::value(channel);
        if (raw.isTop()) {
            return 0;
        }
        else {
            auto v = raw.toInt();
            v *= 34;
            v >>= 3;
            if (v > mOffset) {
                return v - mOffset;
            }
            else {
                return 0;
            }
        }
#endif
    }
private:
    static inline etl::uint_ranged<uint8_t, 0, 63> mCCount;
    static inline uint16_t mCalib{};
    static inline uint16_t mOffset{500};
};

