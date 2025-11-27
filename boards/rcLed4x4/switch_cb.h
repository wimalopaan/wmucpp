#pragma once

#include <array>

template<typename Config>
struct SwitchCallback {
	using pca = Config::pca;
	using storage = Config::storage;
	
	static inline std::array<bool, 16> prevSwitchState{};
	static inline std::array<uint8_t, 4> groupCount{};
	static inline void setGroupIndex(const uint8_t index, const bool on) {
		if (on) {
			pca::groupStart(index);
		}
		else {
			pca::groupStop(index);
		}
	}
	static inline void setGroup(const uint8_t state8) {
		for(uint8_t i = 0; i < 4; ++i) {
			if (state8 & (1 << i)) {
				pca::groupStart(i);
			}
			else {
				pca::groupStop(i);
			}
		}
	}
	static inline void setVirtualIndex(const uint8_t i, const bool on) {
		if (on) {
			for(uint8_t k = 0; k < storage::eeprom.virtuals[i].member.size(); ++k) {
				if (storage::eeprom.virtuals[i].member[k] >= 0) {
					setIndex(storage::eeprom.virtuals[i].member[k], true);
				}
			}
		}
		else {
			for(uint8_t k = 0; k < storage::eeprom.virtuals[i].member.size(); ++k) {
				if (storage::eeprom.virtuals[i].member[k] >= 0) {
					setIndex(storage::eeprom.virtuals[i].member[k], false);
				}
			}
		}
	}
	static inline void setVirtual(const uint8_t state8) {
		if (storage::eeprom.use_virtuals > 0) {
			for(uint8_t i = 0; i < 8; ++i) {
				if (state8 & (1 << i)) {
					setVirtualIndex(i, true);
				}
				else {
					setVirtualIndex(i, false);
				}
			}
		}
	}
	static inline bool isMemberOfVirtual(const uint8_t i) {
		for(uint8_t v = 0; v < storage::eeprom.virtuals.size(); ++v) {
			for(uint8_t k = 0; k < storage::eeprom.virtuals[v].member.size(); ++k) {
				if (i == storage::eeprom.virtuals[v].member[k]) {
					return true;
				}
			}
		}
		return false;
	}
	static inline void set(const uint8_t state8) {
		for(uint8_t i = 0; i < 8; ++i) {
			const bool use = (storage::eeprom.use_virtuals == 0) || !isMemberOfVirtual(i);
			if (use) {
				if (state8 & (1 << i)) {
					setIndex(i, true);
				}
				else {
					setIndex(i, false);
				}
			}
		}
	}
	static inline void set2(const uint8_t state8) {
		for(uint8_t i = 0; i < 8; ++i) {
			const bool use = (storage::eeprom.use_virtuals == 0) || !isMemberOfVirtual(i + 8);
			if (use) {
				if (state8 & (1 << i)) {
					setIndex(i + 8, true);
				}
				else {
					setIndex(i + 8, false);
				}
			}
		}
	}
	static inline void setIndex(const uint8_t index, const bool on) {
		if (on) {
			if (storage::eeprom.outputs[index].groupStart > 0) {
				const uint8_t group = storage::eeprom.outputs[index].group;
				if ((group > 0) && !prevSwitchState[index]) {
					if (storage::eeprom.groups[group - 1].mode == 0) {
						pca::groupStart(group - 1);
						groupCount[group - 1] += 1;
					}
					else {
						pca::groupHoldRampRestore(group - 1);
						pca::groupStart(group - 1);
					}
				}
			}
			pca::ledControl(index, storage::eeprom.outputs[index].control);
			prevSwitchState[index] = true;
		}
		else {
			pca::ledControl(index, 0);
			if (storage::eeprom.outputs[index].groupStart > 0) {
				const uint8_t group = storage::eeprom.outputs[index].group;
				if ((group > 0) && prevSwitchState[index]) {
					if (storage::eeprom.groups[group - 1].mode == 0) {
						groupCount[group - 1] -= 1;
						if (groupCount[group - 1] == 0) {
							pca::groupStop(group - 1);
						}
					}
					else {
						pca::groupStop(group - 1);
						pca::groupClear(group - 1);
					}
				}
			}
			prevSwitchState[index] = false;
		}
	}
};
