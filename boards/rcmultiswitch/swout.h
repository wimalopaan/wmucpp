#pragma once

#include "board.h"

namespace External {
    template<typename OutList, typename PWM, typename StateProvider, typename NVM>
    struct Output {
        inline static constexpr uint8_t size() {
            return Meta::size_v<OutList>;
        }
        static_assert(size() == StateProvider::size());
        
        using pwm_index_t = typename PWM::index_type;
        using index_t = etl::uint_ranged<uint8_t, 0, size() - 1>;
             
//        pwm_index_t::_;
        
        inline static constexpr auto pwmMax = PWM::pwmMax;
        
        using nvm_t = std::remove_cvref_t<decltype(NVM::data())>;
        using nvm_data_t = nvm_t::value_type;
        
        using blinks_type = std::remove_cvref_t<decltype(nvm_data_t{}.blinks())>;
        using blink_index_t = etl::uint_ranged<uint8_t, 0, blinks_type::size()>;
        
        using tick_t = nvm_data_t::tick_t;
        inline static constexpr auto blinkMax = tick_t::max();
        
        static inline void init() {
            PWM::init();
            StateProvider::init();
            Meta::visit<OutList>([]<typename L>(Meta::Wrapper<L>){
                                     L::off();
                                     L::template dir<AVR::Output>();
                                 });
            for (uint8_t out = 0; out < StateProvider::size(); ++out) {
                if (out <= pwm_index_t::Upper) {
                    const auto p = pwm_index_t(out);
                    if (auto v = NVM::data()[out].pwmValue()) {
                        PWM::pwm(p, 0);
                        PWM::on(p);
                    }
                }
            }
            
        }        
        
        inline static void setSwitchOff(const index_t index) {
            if (index <= pwm_index_t::Upper) {
                const auto li = pwm_index_t(index);
                if (const auto v = NVM::data()[index].pwmValue()) {
                    PWM::pwm(li, 0);
                    PWM::on(li);
                }
                else {
                    PWM::off(li);
                    off(index);
                }
            }
            else {
                off(index);
            }
        }
        inline static void setSwitchOn(const index_t index) {
            if (index <= pwm_index_t::Upper) {
                const auto li = pwm_index_t(index);
                if (const auto v = NVM::data()[index].pwmValue()) {
                    PWM::on(li);
                    PWM::pwm(li, v.toInt());
                }
                else {
                    PWM::off(li);
                    on(index);
                }
            }
            else {
                on(index);
            }    
        }
        
        inline static void setSwitch(const index_t index) {
            if (StateProvider::switches()[index] == StateProvider::SwState::Off) {
                setSwitchOff(index);
//                blinkTicks[index] = NVM::data()[index].blinks()[blinkIndex].intervall;
            }
            else if ((StateProvider::switches()[index] == StateProvider::SwState::Blink1) || (StateProvider::switches()[index] == StateProvider::SwState::Blink2)) {
                blink_index_t blinkIndex((StateProvider::switches()[index] == StateProvider::SwState::Blink1) ? 0 : 1);
                                
                if (NVM::data()[index].blinks()[blinkIndex].duration) {
                    blinkTicks[index].match(NVM::data()[index].blinks()[blinkIndex].duration, [&]{
                        setSwitchOff(index);
                    });
                    blinkTicks[index].on(NVM::data()[index].blinks()[blinkIndex].intervall, [&]{
                        setSwitchOn(index);
                    });
                    ++blinkTicks[index];
                }
                else {
                    setSwitchOn(index);
                }
            }
            else if (StateProvider::switches()[index] == StateProvider::SwState::Steady) {
                setSwitchOn(index);                
            }
        }
        inline static void setSwitches() {
            for(uint8_t s = 0; s <= index_t::Upper; ++s) {
                setSwitch(index_t{s});
            }        
        }
        
        inline static void pwm(const index_t index, const uint8_t p) {
            NVM::data()[index].pwmValue(p);
            NVM::data().change();
            if (index <= pwm_index_t::Upper) {
                const auto li = pwm_index_t(index);
                if (const auto v = NVM::data()[index].pwmValue()) {
                    PWM::pwm(li, v.toInt());
                }
            }
        }
        
        inline static void reset(const index_t index) {
            NVM::data()[index] = nvm_data_t{};
            NVM::data().change();
        }
        inline static void duration(const index_t index, const tick_t d, const blink_index_t blinkIndex) {
            NVM::data()[index].blinks()[blinkIndex].duration = d;
            NVM::data().change();
        }
        inline static void intervall(const index_t index, const tick_t i, const blink_index_t blinkIndex) {
            NVM::data()[index].blinks()[blinkIndex].intervall = i;
            NVM::data().change();
            blinkTicks[index] = std::min(blinkTicks[index], i);
        }
        inline static void intervall2(const index_t index, const tick_t i, const blink_index_t blinkIndex) {
            NVM::data()[index].blinks()[blinkIndex].intervall = i;
            NVM::data().change();
//            blinkTicks[index] = std::min(blinkTicks[index], i);
        }
        inline static auto intervall(const index_t index, const blink_index_t blinkIndex) {
            return NVM::data()[index].blinks()[blinkIndex].intervall;
        }
    private:
        inline static void on(const index_t index) {
            Meta::visitAt<OutList>(index, []<typename L>(Meta::Wrapper<L>){
                                       L::on();
                                   });
        }
        inline static void off(const index_t index) {
            Meta::visitAt<OutList>(index, []<typename L>(Meta::Wrapper<L>){
                                       L::off();
                                   });
        }
        inline static std::array<Storage::tick_type, size()> blinkTicks;
//        inline static blink_index_t blinkIndex;
    };
}

