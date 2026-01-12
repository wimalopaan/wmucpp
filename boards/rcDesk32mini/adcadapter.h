/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2025 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <cstdint>
#include <algorithm>

#include "rc/rc_2.h"

template<typename Config>
struct AdcAdapter {
	using adc = Config::adc;
	using storage = Config::storage;
	static inline constexpr uint8_t size = adc::nChannels;
	static inline const auto& values() {
		return mValues;
	}
	static inline void ratePeriodic() {
		adcToSbus();
	}
	static inline void injectChannel(const uint8_t ch, const uint16_t value) {
		if (ch < mValues.size()) {
			injectionChannel[ch] = true;
			mValues[ch] = value;			
		}
	}
	private:
	static inline void adcToSbus() {
		for(uint8_t i = 0; i < std::min((uint8_t)storage::eeprom.calibration.size(), size); ++i) {
			if (!injectionChannel[i]) {
				const int32_t v1 = (adc::values()[i] - storage::eeprom.calibration[i].mid);
                int32_t sb = 0;
                if (v1 >= 0) {
                    sb = (v1 * RC::Protokoll::SBus::V2::span) / (storage::eeprom.calibration[i].max - storage::eeprom.calibration[i].mid) + RC::Protokoll::SBus::V2::mid;
                }
                else {
                    sb = (v1 * RC::Protokoll::SBus::V2::span) / (storage::eeprom.calibration[i].mid - storage::eeprom.calibration[i].min) + RC::Protokoll::SBus::V2::mid;
                }
                // const int32_t sb = (v1 * RC::Protokoll::SBus::V2::amp) / storage::eeprom.calibration[i].span + RC::Protokoll::SBus::V2::mid;
				mValues[i] = std::clamp(sb, (int32_t)RC::Protokoll::SBus::V2::min, (int32_t)RC::Protokoll::SBus::V2::max);
			}
		}
	}
	static inline auto mValues = []{
		std::array<uint16_t, 16> a;
		for(auto& v: a) {
			v = RC::Protokoll::Crsf::V4::mid;
		}
		return a;
	}();
	static inline std::array<bool, 16> injectionChannel{};
};
