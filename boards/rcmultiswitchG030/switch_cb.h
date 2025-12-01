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

#include "eeprom.h"

template<typename Config>
struct SwitchCallback {
    using storage = Config::storage;
    using debug = Config::debug;
    using bsws = Config::bsws;
	using patgen0 = Config::patgen0;
	using patgen1 = Config::patgen1;
	using patgen2 = Config::patgen2;
	using patgen3 = Config::patgen3;
	using plist = Meta::List<patgen0, patgen1, patgen2, patgen3>;

#ifdef USE_PATTERNS
	static inline void patternStart(const uint8_t adrIndex, const uint8_t pattern) {
		if (adrIndex == EEProm::AdrIndex::Virtual) {
			if (pattern == 4) {
				patgen0::event(patgen0::Event::Chain);			
			}
			else if (pattern == 5) {
				if constexpr(!std::is_same_v<patgen1, void>) {
					patgen1::event(patgen1::Event::Chain);			
				}
			}
			else if (pattern == 6) {
				if constexpr(!std::is_same_v<patgen2, void>) {
					patgen2::event(patgen2::Event::Chain);			
				}
			}
			else if (pattern == 7) {
				if constexpr(!std::is_same_v<patgen3, void>) {
					patgen3::event(patgen3::Event::Chain);			
				}
			}
		}
	}
	static inline void patternStopAll(const uint8_t group) {
		if (group > 0) {
			if (group == storage::eeprom.pattern[0].group) {	
				patgen0::event(patgen0::Event::Stop, false);				
			}
			else if (group == storage::eeprom.pattern[1].group) {	
				if constexpr(!std::is_same_v<patgen1, void>) {
					patgen1::event(patgen1::Event::Stop, false);				
				}
			}
			else if (group == storage::eeprom.pattern[2].group) {	
				if constexpr(!std::is_same_v<patgen2, void>) {
					patgen2::event(patgen2::Event::Stop, false);				
				}
			}
			else if (group == storage::eeprom.pattern[3].group) {	
				if constexpr(!std::is_same_v<patgen3, void>) {
					patgen3::event(patgen3::Event::Stop, false);				
				}
			}
		}
	}
#endif
    static inline void prop(const uint8_t adrIndex, const uint8_t channel, const uint8_t duty) {
        if ((adrIndex == EEProm::AdrIndex::Switch) && (channel < 8)) {
                IO::outl<debug>("# prop: ", channel, " duty: ", duty);
                Meta::visitAt<bsws>(channel, [&]<typename SW>(Meta::Wrapper<SW>){
                                        SW::duty(duty);
                                    });
        }
#ifdef USE_VIRTUALS
        else if ((adrIndex == EEProm::AdrIndex::Virtual) && (channel < storage::eeprom.virtuals.size())) {

        }
#endif
    }
    static inline void set(const uint8_t adrIndex, const uint8_t sw) {
        IO::outl<debug>("# set: ", sw);
        for(uint8_t i = 0; i < 8; ++i) {
            const uint8_t mask = (0x01 << i);
            setIndex(adrIndex, i, (sw & mask));
        }
    }
    static inline void setIndex(const uint8_t adrIndex, const uint8_t swIndex, const bool on) {
        if (adrIndex == EEProm::AdrIndex::Switch) {
#ifdef USE_VIRTUALS
            const bool use = (storage::eeprom.use_virtuals == 0) || !isMemberOfVirtual(swIndex);
#else
            const bool use = true;
#endif
            if (use) {
                setIndex(swIndex, on);
            }
        }
#ifdef USE_VIRTUALS
        else if (adrIndex == EEProm::AdrIndex::Virtual) {
            setVirtualIndex(swIndex, on);
        }
#endif
    }
    private:
	static inline bool isMemberOfPattern([[maybe_unused]] const uint8_t swIndex) {
#ifdef USE_PATTERNS
		for(uint8_t i = 0; i < 4; ++i) {
			if (storage::eeprom.pattern[i].type > 0) {
				for(const uint8_t m : storage::eeprom.pattern[i].member) {
					if (m > 0) {
						if ((m - 1) == swIndex) {
							return true;
						}
					}
				}
			}			
		}
#endif
		return false;
	}
    static inline void setIndex(const uint8_t swIndex, const bool on) {
		if (isMemberOfPattern(swIndex)) {
			return;
		}
        Meta::visitAt<bsws>(swIndex, [&]<typename SW>(Meta::Wrapper<SW>){
                                SW::event(on ? SW::Event::On : SW::Event::Off);
                            });
    }
    static inline void setVirtualIndex(const uint8_t i, const bool on) {
        if (i < storage::eeprom.virtuals.size()) {
            IO::outl<debug>("# sVI: ", i, " ", uint8_t(on));
            for(uint8_t k = 0; k < storage::eeprom.virtuals[i].member.size(); ++k) {
                if (storage::eeprom.virtuals[i].member[k] > 0) {
                    setIndex(storage::eeprom.virtuals[i].member[k] - 1, on);
                }
            }
        }
		else { // higher virtuals are pattern
			const uint8_t pattern = (i - storage::eeprom.virtuals.size());
			if (pattern == 0) {
				if (on) {
					patgen0::event(patgen0::Event::Start);
				}
				else {
					patgen0::event(patgen0::Event::Stop);
				}
			}
			else if (pattern == 1) {
				if constexpr(!std::is_same_v<patgen1, void>) {
					if (on) {
						patgen1::event(patgen1::Event::Start);
					}
					else {
						patgen1::event(patgen1::Event::Stop);
					}
				}
			}
			else if (pattern == 2) {
				if constexpr(!std::is_same_v<patgen2, void>) {
					if (on) {
						patgen2::event(patgen2::Event::Start);
					}
					else {
						patgen2::event(patgen2::Event::Stop);
					}
				}
			}
			else if (pattern == 3) {
				if constexpr(!std::is_same_v<patgen3, void>) {
					if (on) {
						patgen3::event(patgen3::Event::Start);
					}
					else {
						patgen3::event(patgen3::Event::Stop);
					}
				}
			}
		}
    }
    static inline bool isMemberOfVirtual(const uint8_t i) {
        for(uint8_t v = 0; v < storage::eeprom.virtuals.size(); ++v) {
            for(uint8_t m = 0; m < storage::eeprom.virtuals[v].member.size(); ++m) {
                if (storage::eeprom.virtuals[v].member[m] > 0) {
                    if (i == (storage::eeprom.virtuals[v].member[m] - 1)) {
                        return true;
                    }
                }
            }
        }
        return false;
    }
};
