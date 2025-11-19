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

#include "rc/crsf_2.h"
#include "eeprom.h"

template<typename Config>
struct CrsfCallback {
    using storage = Config::storage;
    using debug = Config::debug;

    using Param_t = RC::Protokoll::Crsf::V4::Parameter<uint8_t>;
    using PType = Param_t::Type;

    using switchCallback = Config::switchCallback;
    struct DecoderConfig {
        using debug = CrsfCallback::debug;
        using storage = CrsfCallback::storage;
        using callback = switchCallback;
    };
    using swDecoder = RC::Protokoll::Crsf::V4::Util::SwitchDecoder<DecoderConfig>;

    using bsws = Config::bswList;
    using pwms = Config::pwmList;
    using timer = Config::timer;
    using crsf = Config::crsf;
    using messageBuffer = crsf::messageBuffer;
    using telemetry = Config::telemetry;
	using patgen = Config::patgen;

    static inline constexpr const char* const title = "MultiSwitch-E@";
    using name_t = std::array<char, 32>;

    static inline constexpr void disableTelemetry()
    requires(!std::is_same_v<telemetry, void>) {
        IO::outl<debug>("# disable telemetry");
        telemetry::disableWithAutoOn();
    }
    static inline constexpr void updateName(name_t& n) {
        strncpy(&n[0], title, n.size());
        auto r = std::to_chars(std::begin(n) + strlen(title), std::end(n), storage::eeprom.addresses[EEProm::AdrIndex::Switch]);
        *r.ptr++ = ':';
        r = std::to_chars(r.ptr, std::end(n), storage::eeprom.crsf_address);
        *r.ptr++ = '\0';
    }
    static inline void update() {
        updateName(mName);
    }
    static inline void setParameterValue(const uint8_t index, const auto data, const uint8_t paylength) {
        if (RC::Protokoll::Crsf::V4::Util::setParameter(params, index, data, paylength)) {
            storage::save();
        }
        update();
    }
    static inline Param_t parameter(const uint8_t index) {
        IO::outl<debug>("# cb param i: ", index);
        if (index < params.size()) {
            return params[index];
        }
        return {};
    }
    static inline bool isCommand(const uint8_t index) {
        IO::outl<debug>("# cb isCom i: ", index);
        if (index < params.size()) {
            return params[index].type == PType::Command;
        }
        return false;
    }
    static inline const char* name() {
        return &mName[0];
    }
    static inline uint32_t serialNumber() {
        return mSerialNumber;
    }
    static inline uint32_t hwVersion() {
        return mHWVersion;
    }
    static inline uint32_t swVersion() {
        return mSWVersion;
    }
    static inline uint8_t numberOfParameters() {
        return params.size() - 1;
    }
    static inline uint8_t protocolVersion() {
        return 0;
    }
    static inline void command(const auto payload, [[maybe_unused]] const uint8_t paylength) {
        swDecoder::process(payload, paylength);
    }
    static inline void serialize(const uint8_t index, auto& buffer, const RC::Protokoll::Crsf::V4::Lua::CmdStep step = RC::Protokoll::Crsf::V4::Lua::CmdStep::Idle) {
        params[index].serialize(buffer, params, step, index);
    }
    static inline void callbacks(const bool eepromMode = false) {
        const bool prevMode = mEepromMode;
        mEepromMode = eepromMode;
        for(const auto p: params) {
            if (p.cb) {
                p.cb(p.value());
            }
        }
        mEepromMode = prevMode;
    }
private:
    static inline bool mEepromMode = false;
    static inline constexpr uint32_t mSerialNumber{1234};
    static inline constexpr uint32_t mHWVersion{HW_VERSION};
    static inline constexpr uint32_t mSWVersion{SW_VERSION};
    static inline constexpr auto mVersionString = [](){
        std::array<char, 16> s{};
        auto [ptr, e] = std::to_chars(std::begin(s), std::end(s), mHWVersion);
        *ptr++ = ':';
        auto r = std::to_chars(ptr, std::end(s), mSWVersion);
        *r.ptr = '\0';
        return s;
    }();
    static inline name_t mName = []{
        name_t name{};
        updateName(name);
        return name;
    }();
	static inline void setPattern() {
		uint8_t l = 0;
		for(const auto& m : storage::eeprom.pattern[0].member) {
			if (m > 0) {
				patgen::outputPosition(m - 1, l++);
			}
		}
		patgen::length(l);
	}
    static inline constexpr uint8_t addParent(auto& c, const Param_t& p) {
        c.push_back(p);
        return c.size() - 1;
    }
    static inline constexpr void addNode(auto& c, const Param_t& p) {
        c.push_back(p);
    }
#ifdef USE_VIRTUALS
    template<uint8_t index>
    static inline constexpr void addVirtual(auto& p, const uint8_t parent, const char* const name) {
        const uint8_t parent2 = addParent(p, Param_t{parent, PType::Folder, name});
        addNode(p, Param_t{parent2, PType::Sel, "Member 0", "None;Out0;Out1;Out2;Out3;Out4;Out5;Out6;Out7", &storage::eeprom.virtuals[index].member[0], 0, 8, [](const uint8_t){return true;}});
        addNode(p, Param_t{parent2, PType::Sel, "Member 1", "None;Out0;Out1;Out2;Out3;Out4;Out5;Out6;Out7", &storage::eeprom.virtuals[index].member[1], 0, 8, [](const uint8_t){return true;}});
        addNode(p, Param_t{parent2, PType::Sel, "Member 2", "None;Out0;Out1;Out2;Out3;Out4;Out5;Out6;Out7", &storage::eeprom.virtuals[index].member[2], 0, 8, [](const uint8_t){return true;}});
        addNode(p, Param_t{parent2, PType::Sel, "Member 3", "None;Out0;Out1;Out2;Out3;Out4;Out5;Out6;Out7", &storage::eeprom.virtuals[index].member[3], 0, 8, [](const uint8_t){return true;}});
    }
#endif
    template<uint8_t index>
    static inline constexpr void addOutput(auto& p, const uint8_t parent, const char* const name) {
        const uint8_t parent2 = addParent(p, Param_t{parent, PType::Folder, name});
        addNode(p, Param_t{parent2, PType::Info, name, &mName[0]});
        addNode(p, Param_t{parent2, PType::Sel, "PWM Mode", "Off;On;Remote", &storage::eeprom.outputs[index].pwm, 0, 2, [](const uint8_t v){Meta::nth_element<index, bsws>::pwm(v); return true;}});
        addNode(p, Param_t{.parent = parent2, .type = PType::U8, .name = "PWM Duty", .value_ptr = &storage::eeprom.outputs[index].pwmDuty, .min = 1, .max = 99, .cb = [](const uint8_t v){Meta::nth_element<index, bsws>::duty((storage::eeprom.outputs[index].pwm == 2)?0:v); return true;}, .unitString = "%"});
        addNode(p, Param_t{parent2, PType::U8, "PWM Expo", nullptr, &storage::eeprom.outputs[index].pwmScale, 0, 100, [](const uint8_t v){Meta::nth_element<index, bsws>::expo(v); return true;}});
        addNode(p, Param_t{parent2, PType::Sel, "Intervall Mode", "Off;On;Morse;Pattern", &storage::eeprom.outputs[index].blink, 0, 2, [](const uint8_t v){Meta::nth_element<index, bsws>::blink(v); return true;}});
        addNode(p, Param_t{.parent = parent2, .type = PType::U8, .name = "Intervall(on)", .value_ptr = &storage::eeprom.outputs[index].blinkOnTime, .min = 1, .max = 255, .cb = [](const uint8_t v){Meta::nth_element<index, bsws>::on_dezi(v); return true;}, .unitString = "*50ms"});
        addNode(p, Param_t{.parent = parent2, .type = PType::U8, .name = "Intervall(off)", .value_ptr = &storage::eeprom.outputs[index].blinkOffTime, .min = 1, .max = 255, .cb = [](const uint8_t v){Meta::nth_element<index, bsws>::off_dezi(v); return true;}, .unitString = "*50ms"});
        addNode(p, Param_t{.parent = parent2, .type = PType::U8, .name = "Intervall(count)", .value_ptr = &storage::eeprom.outputs[index].flashCount, .min = 1, .max = 4, .cb = [](const uint8_t v){Meta::nth_element<index, bsws>::flash_count(v); return true;}});
        addNode(p, Param_t{parent2, PType::Sel, "Test", "Off;On", nullptr, 0, 1, [](const uint8_t v){Meta::nth_element<index, bsws>::on(v); return false;}});
    }
    static inline auto params = []{
        etl::FixedVector<Param_t, 137> p;
		// etl::FixedVector<Param_t, 132> p;
        addNode(p, Param_t{0, PType::Folder, ""}); // unvisible top folder
        addNode(p, Param_t{0, PType::Info, "Version(HW/SW)", &mVersionString[0]});
        auto parent = addParent(p, Param_t{0, PType::Folder, "Global"});
        addNode(p, Param_t{parent, PType::U8, "Switch Addr", nullptr, &storage::eeprom.addresses[EEProm::AdrIndex::Switch], 0, 255, [](const uint8_t){update(); return true;}});
#ifdef USE_VIRTUALS
        addNode(p, Param_t{parent, PType::U8, "Virtual Addr", nullptr, &storage::eeprom.addresses[EEProm::AdrIndex::Virtual], 0, 255, [](const uint8_t){update(); return true;}});
#endif
#ifdef HW_MSW10
        addNode(p, Param_t{.parent = parent, .type = PType::U8, .name = "PWM f G1 (O3,2)", .value_ptr= &storage::eeprom.pwm1, .min = 1, .max = 200, .cb = [](const uint8_t v){Meta::nth_element<0, pwms>::freqCenties(v); return true;}, .def = 10, .unitString = " *100Hz"});
        addNode(p, Param_t{.parent = parent, .type = PType::U8, .name = "PWM f G2 (O1,4,5,6)", .value_ptr = &storage::eeprom.pwm2, .min = 1, .max = 200, .cb = [](const uint8_t v){Meta::nth_element<1, pwms>::freqCenties(v); return true;}, .def = 10, .unitString = " *100Hz"});
        addNode(p, Param_t{.parent = parent, .type = PType::U8, .name = "PWM f G3 (O7)", .value_ptr = &storage::eeprom.pwm3, .min = 1, .max = 200, .cb = [](const uint8_t v){Meta::nth_element<2, pwms>::freqCenties(v); return true;}, .def = 10, .unitString = " *100Hz"});
        addNode(p, Param_t{.parent = parent, .type = PType::U8, .name = "PWM f G4 (O0)", .value_ptr = &storage::eeprom.pwm4, .min = 1, .max = 200, .cb = [](const uint8_t v){Meta::nth_element<3, pwms>::freqCenties(v); return true;}, .def = 10, .unitString = " *100Hz"});
#endif
#ifdef HW_NUCLEO
        addNode(p, Param_t{.parent = parent, .type = PType::U8, .name = "PWM f G1 (O2,3,7)", .value_ptr= &storage::eeprom.pwm1, .min = 1, .max = 200, .cb = [](const uint8_t v){Meta::nth_element<0, pwms>::freqCenties(v); return true;}, .def = 10, .unitString = " *100Hz"});
        addNode(p, Param_t{.parent = parent, .type = PType::U8, .name = "PWM f G2 (O0,1)", .value_ptr = &storage::eeprom.pwm2, .min = 1, .max = 200, .cb = [](const uint8_t v){Meta::nth_element<1, pwms>::freqCenties(v); return true;}, .def = 10, .unitString = " *100Hz"});
        addNode(p, Param_t{.parent = parent, .type = PType::U8, .name = "PWM f G3 (O5)", .value_ptr = &storage::eeprom.pwm3, .min = 1, .max = 200, .cb = [](const uint8_t v){Meta::nth_element<2, pwms>::freqCenties(v); return true;}, .def = 10, .unitString = " *100Hz"});
        addNode(p, Param_t{.parent = parent, .type = PType::U8, .name = "PWM f G4 (O4,6)", .value_ptr = &storage::eeprom.pwm4, .min = 1, .max = 200, .cb = [](const uint8_t v){Meta::nth_element<3, pwms>::freqCenties(v); return true;}, .def = 10, .unitString = " *100Hz"});
#endif
#ifdef HW_WEACT
        addNode(p, Param_t{.parent = parent, .type = PType::U8, .name = "PWM f G1 (O4,7)", .value_ptr= &storage::eeprom.pwm1, .min = 1, .max = 200, .cb = [](const uint8_t v){Meta::nth_element<0, pwms>::freqCenties(v); return true;}, .def = 10, .unitString = " *100Hz"});
        addNode(p, Param_t{.parent = parent, .type = PType::U8, .name = "PWM f G2 (O5,6)", .value_ptr = &storage::eeprom.pwm2, .min = 1, .max = 200, .cb = [](const uint8_t v){Meta::nth_element<1, pwms>::freqCenties(v); return true;}, .def = 10, .unitString = " *100Hz"});
        addNode(p, Param_t{.parent = parent, .type = PType::U8, .name = "PWM f G3 (O0,1,2,3)", .value_ptr = &storage::eeprom.pwm3, .min = 1, .max = 200, .cb = [](const uint8_t v){Meta::nth_element<2, pwms>::freqCenties(v); return true;}, .def = 10, .unitString = " *100Hz"});
#endif
#ifdef HW_MSW11
        addNode(p, Param_t{.parent = parent, .type = PType::U8, .name = "PWM f G1 (O0,1,2,3)", .value_ptr = &storage::eeprom.pwm3, .min = 1, .max = 200, .cb = [](const uint8_t v){Meta::nth_element<0, pwms>::freqCenties(v); return true;}, .def = 10, .unitString = " *100Hz"});
        addNode(p, Param_t{.parent = parent, .type = PType::U8, .name = "PWM f G2 (O4,5,6,7)", .value_ptr= &storage::eeprom.pwm1, .min = 1, .max = 200, .cb = [](const uint8_t v){Meta::nth_element<1, pwms>::freqCenties(v); return true;}, .def = 10, .unitString = " *100Hz"});
#endif

        addNode(p, Param_t{parent, PType::U8, "CRSF Addr", nullptr, &storage::eeprom.crsf_address, 0xc0, 0xcf, [](const uint8_t v){
                       #ifdef USE_RESPONSE_SLOT
                               if (!mEepromMode) {
                                   const uint8_t slot = 2 * (v - 0xc0);
                                   storage::eeprom.response_slot = slot;
                                   crsf::output::telemetrySlot(slot);
                               }
                       #endif
                               crsf::address(std::byte{v});
                               return true;
                           }});
#ifdef HW_MSW11
        addNode(p, Param_t{parent, PType::U8, "Cells Id", nullptr, &storage::eeprom.cells_id, 0, 255, [](const uint8_t){return true;}});
        addNode(p, Param_t{parent, PType::U8, "Temp Id", nullptr, &storage::eeprom.temp_id, 0, 255, [](const uint8_t){return true;}});
#endif
#ifdef USE_RESPONSE_SLOT
        addNode(p, Param_t{parent, PType::U8, "Response Slot", nullptr, &storage::eeprom.response_slot, 0, 15, [](const uint8_t v){crsf::output::telemetrySlot(v); return true;}});
        addNode(p, Param_t{parent, PType::Sel, "Config resp.", "Button;Allways on", &storage::eeprom.telemetry, 0, 1});
#endif

        addOutput<0>(p, 0, "Output 0");
        addOutput<1>(p, 0, "Output 1");
        addOutput<2>(p, 0, "Output 2");
        addOutput<3>(p, 0, "Output 3");
        addOutput<4>(p, 0, "Output 4");
        addOutput<5>(p, 0, "Output 5");
        addOutput<6>(p, 0, "Output 6");
        addOutput<7>(p, 0, "Output 7");

#ifdef USE_MORSE
        parent = addParent(p, Param_t{0, PType::Folder, "Morse"});
        addNode(p, Param_t{parent, PType::Str, "Text", nullptr, nullptr, 0, 0, nullptr, 0, 0, 0, &storage::eeprom.morse_text[0]}); // not supported by elrsv3.lua?
        addNode(p, Param_t{parent, PType::U8,  "Dit duration", nullptr, &storage::eeprom.morse_dit, 1, 10, [](const uint8_t v){
                               Meta::visit<bsws>([&]<typename T>(Meta::Wrapper<T>){
                                   T::morse_dit_dezi(v);
                               }); return true;}, 1, 0, 1, nullptr, " [100ms]"});
        addNode(p, Param_t{parent, PType::U8,  "Dah duration", nullptr, &storage::eeprom.morse_dah, 1, 10, [](const uint8_t v){
                               Meta::visit<bsws>([&]<typename T>(Meta::Wrapper<T>){
                                   T::morse_dah_dezi(v);
                               });
                               return true;}, 1, 0, 1, nullptr, " [100ms]"});
        addNode(p, Param_t{parent, PType::U8,  "Intra S. Gap dur.", nullptr, &storage::eeprom.morse_gap, 1, 10, [](const uint8_t v){
                               Meta::visit<bsws>([&]<typename T>(Meta::Wrapper<T>){
                                   T::morse_gap_dezi(v);
                               });
                               return true;}, 1, 0, 1, nullptr, " [100ms]"});
        addNode(p, Param_t{parent, PType::U8,  "Inter S. Gap dur.", nullptr, &storage::eeprom.morse_igap, 1, 10, [](const uint8_t v){
                               Meta::visit<bsws>([&]<typename T>(Meta::Wrapper<T>){
                                   T::morse_igap_dezi(v);
                               });
                               return true;}, 1, 0, 1, nullptr, " [100ms]"});
#endif
#ifdef USE_VIRTUALS
        parent = addParent(p, Param_t{0, PType::Folder, "Virtuals"});
        addNode(p, Param_t{parent, PType::Sel, "Enable Virtuals", "Off;On", &storage::eeprom.use_virtuals, 0, 1, [](const uint8_t){return true;}});
        addVirtual<0>(p, parent, "Virtual 0");
        addVirtual<1>(p, parent, "Virtual 1");
        addVirtual<2>(p, parent, "Virtual 2");
        addVirtual<3>(p, parent, "Virtual 3");
#endif
#ifdef USE_PATTERNS
        parent = addParent(p, Param_t{0, PType::Folder, "Pattern"});
        addNode(p, Param_t{parent, PType::Sel, "Pattern Virt. 4", "Off;Cont-LR;Cont-RL;Cont-LRL;Single-LR;Single-RL;Single-LRL", &storage::eeprom.pattern[0].type, 0, 7, [](const uint8_t v){patgen::setMode(v); return true;}});
		addNode(p, Param_t{parent, PType::U8,  "On", nullptr, &storage::eeprom.pattern[0].onTime, 1, 255, [](const uint8_t v){patgen::ontime(10*v); return true;}, 100, 0, 1, nullptr, "[*10ms]"});
		addNode(p, Param_t{parent, PType::U8,  "Off", nullptr, &storage::eeprom.pattern[0].offTime, 1, 255, [](const uint8_t v){patgen::offtime(10*v); return true;}, 10, 0, 1, nullptr, "[*10ms]"});

		addNode(p, Param_t{parent, PType::Sel, "Member 0", "None;Out0;Out1;Out2;Out3;Out4;Out5;Out6;Out7", &storage::eeprom.pattern[0].member[0], 0, 8, [](const uint8_t){setPattern(); return true;}});
		addNode(p, Param_t{parent, PType::Sel, "Member 1", "None;Out0;Out1;Out2;Out3;Out4;Out5;Out6;Out7", &storage::eeprom.pattern[0].member[1], 0, 8, [](const uint8_t){setPattern(); return true;}});
		addNode(p, Param_t{parent, PType::Sel, "Member 2", "None;Out0;Out1;Out2;Out3;Out4;Out5;Out6;Out7", &storage::eeprom.pattern[0].member[2], 0, 8, [](const uint8_t){setPattern(); return true;}});
		addNode(p, Param_t{parent, PType::Sel, "Member 3", "None;Out0;Out1;Out2;Out3;Out4;Out5;Out6;Out7", &storage::eeprom.pattern[0].member[3], 0, 8, [](const uint8_t){setPattern(); return true;}});
		addNode(p, Param_t{parent, PType::Sel, "Member 4", "None;Out0;Out1;Out2;Out3;Out4;Out5;Out6;Out7", &storage::eeprom.pattern[0].member[4], 0, 8, [](const uint8_t){setPattern(); return true;}});
		addNode(p, Param_t{parent, PType::Sel, "Member 5", "None;Out0;Out1;Out2;Out3;Out4;Out5;Out6;Out7", &storage::eeprom.pattern[0].member[5], 0, 8, [](const uint8_t){setPattern(); return true;}});
		addNode(p, Param_t{parent, PType::Sel, "Member 6", "None;Out0;Out1;Out2;Out3;Out4;Out5;Out6;Out7", &storage::eeprom.pattern[0].member[6], 0, 8, [](const uint8_t){setPattern(); return true;}});
		addNode(p, Param_t{parent, PType::Sel, "Member 7", "None;Out0;Out1;Out2;Out3;Out4;Out5;Out6;Out7", &storage::eeprom.pattern[0].member[7], 0, 8, [](const uint8_t){setPattern(); return true;}});

		addNode(p, Param_t{parent, PType::U8,  "Next Address", nullptr, &storage::eeprom.pattern[0].next_address, 0, 255, [](const uint8_t){return true;}});
		addNode(p, Param_t{parent, PType::U8,  "Group", nullptr, &storage::eeprom.pattern[0].group, 0, 255, [](const uint8_t){return true;}});

		addNode(p, Param_t{parent, PType::Sel, "Test", "Off;On", nullptr, 0, 1, [](const uint8_t v){patgen::event((v == 0) ? patgen::Event::Stop : patgen::Event::Start); return false;}});
#endif
#ifdef USE_OPERATE_MENU
        parent = addParent(p, Param_t{0, PType::Folder, "Operate"});
        addNode(p, Param_t{parent, PType::Sel, "Output 0", "Off;On", 0, 0, 1, [](const uint8_t v){Meta::nth_element<0, bsws>::on(v); return false;}});
        addNode(p, Param_t{parent, PType::Sel, "Output 1", "Off;On", 0, 0, 1, [](const uint8_t v){Meta::nth_element<1, bsws>::on(v); return false;}});
        addNode(p, Param_t{parent, PType::Sel, "Output 2", "Off;On", 0, 0, 1, [](const uint8_t v){Meta::nth_element<2, bsws>::on(v); return false;}});
        addNode(p, Param_t{parent, PType::Sel, "Output 3", "Off;On", 0, 0, 1, [](const uint8_t v){Meta::nth_element<3, bsws>::on(v); return false;}});
        addNode(p, Param_t{parent, PType::Sel, "Output 4", "Off;On", 0, 0, 1, [](const uint8_t v){Meta::nth_element<4, bsws>::on(v); return false;}});
        addNode(p, Param_t{parent, PType::Sel, "Output 5", "Off;On", 0, 0, 1, [](const uint8_t v){Meta::nth_element<5, bsws>::on(v); return false;}});
        addNode(p, Param_t{parent, PType::Sel, "Output 6", "Off;On", 0, 0, 1, [](const uint8_t v){Meta::nth_element<6, bsws>::on(v); return false;}});
        addNode(p, Param_t{parent, PType::Sel, "Output 7", "Off;On", 0, 0, 1, [](const uint8_t v){Meta::nth_element<7, bsws>::on(v); return false;}});
#endif
        if (p.size() >= p.capacity()) {
            void fp();
            fp(); // compile-time check (call to undefined function)
        }
        return p;
    }();
};
