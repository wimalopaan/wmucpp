#pragma once

#include <cstdint>
#include <cassert>
#include <utility>

namespace External {
    namespace PCA9745 {
        
        struct Registers {
            struct Ramp {
                std::byte rate{};
                std::byte step{};
                std::byte hold{};
                std::byte iref{};
            };
    //        std::byte grppwm{};
    //        std::byte grpfreq{};
            std::array<std::byte, 16> pwms{};
            std::array<std::byte, 16> irefs{};
            std::array<Ramp, 4> ramps{};
            std::array<std::byte, 2> grad_modes{};
            std::array<std::byte, 4> grad_grps{};
            std::byte grad_cntl;
        };
        
        enum class Current : uint8_t {C0, C1, C2, C3};
        
        struct Read{};
        struct Write{};

        struct Address {
            std::byte raw{0xff}; 
        };
        
        struct Command {
            using value_type = std::byte;
            
            Command() = default;
            
            explicit Command(Write, const uint8_t a, const std::byte v) : address{((std::byte{a} & 0x7f_B) << 1) | 0x00_B}, data{v} {}        
            
            template<uint8_t N>
            std::byte get() const {
                if constexpr(N == 0) {
                    return address.raw;
                }
                else if constexpr(N == 1) {
                    return data;
                }
                else {
                    static_assert(std::false_v<Spi>);
                }
            }
            constexpr const std::byte& operator[](const uint8_t i) const {
                if (i == 0) {
                    return address.raw;
                }         
                return data;
            }
            static constexpr uint8_t size() {
                return 2;
            }
        private:
            Address address{};
            std::byte data{0xff};
        };
        
        
        template<template<typename Command> typename Spi, AVR::Concepts::Pin OePin, AVR::Concepts::Pin IRef1Pin, AVR::Concepts::Pin IRef2Pin, typename NVM>
        struct Controller {
            using ggroup_t = etl::uint_ranged<uint8_t, 0, 3>;
            using giref_t  = etl::uint_ranged<uint8_t, 0, 255>;
            using grrate_t = etl::uint_ranged<uint8_t, 0, 63>;
            using grupdn_t = etl::uint_ranged<uint8_t, 0, 3>;
            using grstep_t = etl::uint_ranged<uint8_t, 0, 127>;
            using ghonoff_t = etl::uint_ranged<uint8_t, 0, 3>;
            using ghtime_t = etl::uint_ranged<uint8_t, 0, 7>;
        
            using index_type = etl::uint_ranged<uint8_t, 0, 15>;
            
            using spi = Spi<Command>;
            using oePin = OePin;
            
            using iref1Pin = IRef1Pin;
            using iref2Pin = IRef2Pin;
            
            static inline void current(const Current c) {
                curr = c;
                NVM::data().change();
                
                switch(c) {
                case Current::C0:
                    iref1Pin::template dir<AVR::Input>();
                    iref2Pin::template dir<AVR::Input>();
                    break;
                case Current::C1:
                    iref1Pin::template dir<AVR::Input>();
                    iref2Pin::template dir<AVR::Output>();
                    iref2Pin::low();
                    break;
                case Current::C2:
                    iref1Pin::template dir<AVR::Output>();
                    iref1Pin::low();
                    iref2Pin::template dir<AVR::Input>();
                    break;
                case Current::C3:
                    iref1Pin::template dir<AVR::Output>();
                    iref1Pin::low();
                    iref2Pin::template dir<AVR::Output>();
                    iref2Pin::low();
                    break;
                default:
                    iref1Pin::template dir<AVR::Output>();
                    iref1Pin::low();
                    iref2Pin::template dir<AVR::Output>();
                    iref2Pin::low();
                }
            }    
            static inline void init() {
                current(curr);
                
                oePin::template dir<AVR::Output>();
                oePin::low();
                
                spi::init();
               
                spi::put(Command{Write{}, 0, 0x00_B}); // normal mode
                spi::put(Command{Write{}, 1, 0x14_B}); // clear / exponnetial
                spi::put(Command{Write{}, 0x40, 0x00_B}); // pwmall
                spi::put(Command{Write{}, 0x41, 0x00_B}); // irefall
                
                spi::flush();
                
                for(const auto& i : etl::make_range<index_type>{}) {
                    pwm_out(i);
                }
                spi::flush();
                
                for(const auto& i : etl::make_range<index_type>{}) {
                    current_out(i);
                }
                spi::flush();

                for(const auto& i : etl::make_range<ggroup_t>{}) {
                    gradation_out(i);
                }
                spi::flush();
                
                for(const auto& i : etl::make_range<ggroup_t>{}) {
                    group_out(i);
                }
                spi::flush();

                enableGradation_out();
                spi::flush();
                
                registers.grad_cntl = 0xff_B; // continous
                for(const auto& i : etl::make_range<ggroup_t, ggroup_t>{}) {
                    gradationStart(i);
                }
                spi::flush();
                
                off();
            }
            
            
            enum class LedMode : uint8_t {Off = 0, On, Individual1, Individual2}; 
            
            static inline void out(const index_type i, const LedMode m) {
                const uint8_t offset = i.toInt() / 4;
                const uint8_t shift  = (i.toInt() % 4) * 2;
                
                const std::byte b    = std::byte{m} << shift;
                const std::byte mask = 0x03_B << shift;
                        
                ledouts[offset] = (ledouts[offset] & ~mask) | b;
                spi::put(Command{Write{}, uint8_t(2 + offset), ledouts[offset]});            
            }
            
            static inline void off() {
                for(uint8_t i{0}; auto& l : ledouts) {
                    l = 0x00_B;
                    spi::put(Command{Write{}, i, l});
                    ++i;            
                }
            }
           
            static inline void pwm(const index_type i, const uint8_t pwm) {
                registers.pwms[i] = std::byte{pwm};
                NVM::data().change();
                pwm_out(i);
            }
        
            static inline void current(const index_type i, const uint8_t iref) {
                registers.irefs[i] = std::byte{iref};
                current_out(i);
                NVM::data().change();
            }

            static inline void enableGradation(const index_type i, const bool on = true) {
                const uint8_t offset = i.toInt() / 8;
                const uint8_t shift  = i.toInt() % 8;
                const std::byte b    = on ? (0x01_B << shift) : 0x00_B;
                const std::byte mask = 0x01_B << shift;
                
                registers.grad_modes[offset] = (registers.grad_modes[offset] & ~mask) | b;
                enableGradation_out();
                NVM::data().change();
            }
            
            static inline void group(const index_type i, const ggroup_t g) {
                const uint8_t offset = i.toInt() / 4;
                const uint8_t shift  = (i.toInt() % 4) * 2;
                
                const std::byte b    = std::byte{g.toInt()} << shift;
                const std::byte mask = 0x03_B << shift;
                        
                registers.grad_grps[offset] = (registers.grad_grps[offset] & ~mask) | b;
                group_out(offset);
                NVM::data().change();
            }
        
            static inline void gradationStop(const ggroup_t g) {
                const std::byte b = 0x03_B << (g.toInt() * 2);
                registers.grad_cntl = (registers.grad_cntl & ~b);
                gradCtrl_out();
            }
            
            static inline void gradationStop() {
                registers.grad_cntl = 0x00_B;
                gradCtrl_out();
            }
            
            static inline void gradationStartSingle(const std::byte b) {
                registers.grad_cntl = b;
                gradCtrl_out();
            }

            static inline void gradationStart(const ggroup_t g, const bool cycle = true) {
                const std::byte b = (cycle ? 0x03_B : 0x02_B) << (g.toInt() * 2);
                registers.grad_cntl = (registers.grad_cntl & ~b) | b;
                gradCtrl_out();
            }

            static inline void gradationI(const ggroup_t g, const giref_t i) {
                registers.ramps[g].iref = std::byte{i.toInt()};
                gradation_out(g);
                NVM::data().change();
            }    
            static inline void gradationR(const ggroup_t g, const grrate_t r) {
                registers.ramps[g].rate = (registers.ramps[g].rate & 0xc0_B) | std::byte(r.toInt());
                gradation_out(g);
                NVM::data().change();
            }    
            static inline void gradationR(const ggroup_t g, const grupdn_t ud) {
                registers.ramps[g].rate = (registers.ramps[g].rate & 0x3f_B) | (std::byte(ud.toInt()) << 6);
                gradation_out(g);
                NVM::data().change();
            }    
            static inline void gradationR(const ggroup_t g, const grstep_t s) {
                registers.ramps[g].step = std::byte(s.toInt());
                gradation_out(g);
                NVM::data().change();
            }    
            static inline void gradationH(const ggroup_t g, const ghonoff_t h) {
                registers.ramps[g].hold = (registers.ramps[g].rate & 0x3f_B) | (std::byte(h.toInt()) << 6);
                gradation_out(g);
                NVM::data().change();
            }    
            static inline void gradationHon(const ggroup_t g, const ghtime_t o) {
                registers.ramps[g].hold = (registers.ramps[g].rate & 0xC7_B) | (std::byte(o.toInt()) << 3);
                gradation_out(g);
                NVM::data().change();
            }    
            static inline void gradationHoff(const ggroup_t g, const ghtime_t o) {
                registers.ramps[g].hold = (registers.ramps[g].rate & 0xF8_B) | std::byte(o.toInt());
                gradation_out(g);
                NVM::data().change();
            }    
            
            static inline void periodic() {
                spi::periodic();
            }
            
            static inline void pushGradation(const ggroup_t g, const Registers::Ramp& ramp) { // temporär
                spi::put(Command{Write{}, uint8_t(0x28 + g * 4), ramp.rate});            
                spi::put(Command{Write{}, uint8_t(0x29 + g * 4), ramp.step});                    
                spi::put(Command{Write{}, uint8_t(0x2a + g * 4), ramp.hold});                    
                spi::put(Command{Write{}, uint8_t(0x2b + g * 4), ramp.iref});            
            }
            
        private:
            static inline std::array<std::byte, 4> ledouts{};
            static inline Current& curr = NVM::data().current;
            static inline Registers& registers = NVM::data().registers;

            static inline void gradation_out(const uint8_t g) {
                pushGradation(ggroup_t{g, etl::RangeCheck<false>{}}, registers.ramps[g]);
            }
            
            static inline void group_out(const uint8_t offset) {
                spi::put(Command{Write{}, uint8_t(0x3a + offset), registers.grad_grps[offset]});            
            }

            static inline void current_out(const uint8_t i) {
                spi::put(Command{Write{}, uint8_t(0x18 + i), registers.irefs[i]});            
            }

            static inline void pwm_out(const uint8_t i) {
                spi::put(Command{Write{}, uint8_t(0x08 + i), registers.pwms[i]});            
            }

            static inline void gradCtrl_out() {
                spi::put(Command{Write{}, 0x3e, registers.grad_cntl});            
            }

            static inline void enableGradation_out() {
                spi::put(Command{Write{}, uint8_t(0x38), registers.grad_modes[0]}); //normal or gradiation
                spi::put(Command{Write{}, uint8_t(0x39), registers.grad_modes[1]}); //normal or gradiation
            }
        };
    }
}

