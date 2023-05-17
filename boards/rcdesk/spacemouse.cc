#define NDEBUG

#include <mcu/avr.h>
#include <mcu/common/delay.h>

#include <mcu/internals/ccp.h>
#include <mcu/internals/clock.h>
#include <mcu/internals/systemclock.h>
#include <mcu/internals/usart.h>
#include <mcu/internals/portmux.h>
#include <mcu/internals/adc.h>
#include <mcu/internals/eeprom.h>

#include <external/hal/adccontroller.h>
#include <external/hal/alarmtimer.h>
#include <external/bluetooth/roboremo.h>
#include <external/bluetooth/qtrobo.h>
#include <external/sbus/sbus.h>
#include <external/solutions/spacemouse.h>

#include <external/solutions/button.h>
#include <external/solutions/rotaryencoder.h>

#include <std/chrono>
#include <std/algorithm>

#include <etl/output.h>
#include <etl/meta.h>

template<typename... BB>
struct Buttons {
    enum class State : uint8_t {Off, ShortPress, LongPress};
    
    using buttonList = Meta::List<BB...>;
    
    inline static void init() {
        (BB::init(), ...);
    }
    
    inline static void ratePeriodic() {
        Meta::visit<buttonList>([]<typename B>(Meta::Wrapper<B>){
                                    using event_t = B::Press;
                                    B::periodic();
                                    constexpr auto i = Meta::index_v<buttonList, B>;
                                    switch(const event_t e = B::event()) {
                                        case event_t::None:
                                        break;
                                        case event_t::Short:
                                        mStates[i] = State::ShortPress;
                                        break;
                                        case event_t::Long:
                                        mStates[i] = State::LongPress;
                                        break;
                                        case event_t::Release:
                                        mStates[i] = State::Off;
                                        break;
                                    }
                                });
    }
    inline static const auto& states() {
        return mStates;
    }
private:
    inline static std::array<State, sizeof...(BB)> mStates;
};

// 3wege Schalter
//       mit pullup        ohne pullup
// 1)      H                  H
// 2)      L                  L
// 3)      H                  L

template<typename Pin>
struct Switch3State {
    enum class State : uint8_t {Low, Mid, High};
    inline static void init() {
        Pin::template dir<AVR::Input>();
    }
    inline static void ratePeriodic() {
        Pin::template pullup<true>();
        bool v1 = Pin::read();
        Pin::template pullup<false>();
        bool v2 = Pin::read();
        
        if (v1 && v2) {
            mState = State::High;
        }
        else if (!(v1 || v2)) {
            mState = State::Low;
        }
        else {
            mState = State::Mid;
        }
    }
private:
    inline static State mState{State::Mid};
};

namespace {
    uint16_t encode(const auto v, const uint8_t func, const uint8_t module) {
        const uint32_t e = (v + 8 * func + 64 * module); 
        return (e * 1638 + 819) / 1024;
    }
}

namespace Application {
    using namespace std::literals::chrono;

    template<typename Adc>
    struct Data final : public EEProm::DataBase<Data<Adc>> {
        struct Calibration {
            Adc::value_type min{200};
            Adc::value_type max{800};
        };
        auto magic() const {
            return mMagic;
        }
        void clear() {
            mMagic = 42;    
            etl::fill(stickCalibrations, Calibration{});
        }
        auto& calibrations() {
            return stickCalibrations;
        }
    private:
        uint8_t mMagic{};
        std::array<Calibration, Adc::NumberOfChannels> stickCalibrations; 
    };
    
    template<typename Timer, typename SBus, typename Robo, typename SM, typename StickButtons, typename Adc, typename SwitchList, typename RotaryList, typename NVM, typename Term>
    struct GFSM;
    template<typename Timer, typename SBus, typename Robo, typename SM, typename StickButtons, typename Adc, typename... Switches, typename... Rotarys, typename NVM, typename Term>
    struct GFSM<Timer, SBus, Robo, SM, StickButtons, Adc, Meta::List<Switches...>, Meta::List<Rotarys...>, NVM, Term> {
        using TermDev = Term::device_type;

        using sm_pa = SM::protocoll_adapter_type;
        using axis_index_t = sm_pa::index_t;
        
//        using robo_pa = Robo::protocoll_adapter_type;

        using rotary_list = Meta::List<Rotarys...>;
        
        using rot0 = Meta::nth_element<0, rotary_list>;
        using rot1 = Meta::nth_element<1, rotary_list>;
        
        using adc_i_t = Adc::index_type;
        using adc_v_t = Adc::value_type;
        static inline constexpr uint8_t numberOfSticks = adc_i_t::Upper + 1;
        static_assert(numberOfSticks <= 16);
        
        enum class State : uint8_t {Undefined, Init, Calibrate, Run};
        
        static inline constexpr External::Tick<Timer> eepromTicks{1000_ms};
        static inline constexpr External::Tick<Timer> initTicks{500_ms};
        static inline constexpr External::Tick<Timer> roboTicks{500_ms};
        static inline constexpr External::Tick<Timer> sbusTicks{14_ms};
        static_assert(sbusTicks.value > 1);

        static inline constexpr External::Tick<Timer> debugTicks{500_ms};
        
        static inline auto& appData = NVM::data();
        
        static inline void init() {
            NVM::init();
            if (!((appData.magic() == 42))) {
                appData.clear();
                appData.change();
            }            
            
            Timer::init();

            if constexpr(!std::is_same_v<SM, void>) {
                SM::template init<AVR::BaudRate<38400>>();
            }
            Robo::template init<AVR::BaudRate<9600>>();    
            SBus::init();
            StickButtons::init();
            (Switches::init(), ...);
            Adc::init();
            (Rotarys::init(typename Rotarys::value_type{SBus::sbus_mid}), ...);
            
            mState = State::Init;
        }    
        static inline void periodic() {
            NVM::saveIfNeeded([&]{
                etl::outl<Term>("save eep"_pgm);
            });
            Adc::periodic();
            if constexpr(!std::is_same_v<TermDev, void>) {
                TermDev::periodic();
            }
            else {
                if constexpr(!std::is_same_v<SM, void>) {
                    SM::periodic();
                }
            }
            SBus::periodic();
            Robo::periodic();
            SM::periodic();
        }    
        static inline void ratePeriodic() {
            StickButtons::ratePeriodic();
            (Switches::ratePeriodic(), ...);
            (Rotarys::rateProcess(), ...);

            (++mDebugTicks).on(debugTicks, debug);
            (++mEepromTicks).on(eepromTicks, []{
                appData.expire();
            });
            (++mSBusTicks).on(sbusTicks, []{
                SBus::ratePeriodic();                    
            });
            
            ++mStateTicks;
            const auto oldState = mState;
            switch(mState) {
            case State::Undefined:
                break;
            case State::Init:
                if (etl::all_of(StickButtons::states(), StickButtons::State::LongPress)) {
                    mState = State::Calibrate;
                }
                mStateTicks.on(initTicks, []{
                    mState = State::Run;
                });
                break;
            case State::Calibrate:
                []<auto... II>(std::index_sequence<II...>){
                    (Stick<II>::calibrate(), ...);
                }(std::make_index_sequence<numberOfSticks>{});
                break;
            case State::Run:
                update();            
                break;
            }
            if (oldState != mState) {
                mStateTicks.reset();
                switch(mState) {
                case State::Undefined:
                    etl::outl<Term>("S U"_pgm);
                    break;
                case State::Init: 
                    etl::outl<Term>("S I"_pgm);
                    break;
                case State::Calibrate:
                    etl::outl<Term>("S C"_pgm);
                    break;
                case State::Run:
                    etl::outl<Term>("S R"_pgm);
                    SM::put(0xad_B);
                    SM::put(sm_pa::endSymbol);
                    SM::put(0xae_B);
                    SM::put(sm_pa::endSymbol);
                    break;
                }
            }
        }   
    private:
        inline static void debug() {
            etl::outl<Term>(
                        "b: "_pgm, sm_pa::bytes(),
                        " p: "_pgm, sm_pa::packages(),
                        " a0: "_pgm, sm_pa::axis(axis_index_t{0}),
                        " a1: "_pgm, sm_pa::axis(axis_index_t{1}),
                        " a2: "_pgm, sm_pa::axis(axis_index_t{2}),
                        " a3: "_pgm, sm_pa::axis(axis_index_t{3}),
                        " a4: "_pgm, sm_pa::axis(axis_index_t{4}),
                        " a5: "_pgm, sm_pa::axis(axis_index_t{5})
                    );
   
        }
        
        template<uint8_t I>
        struct Stick {
            inline static constexpr adc_i_t index{I};
            static inline void calibrate() {
                auto& max = appData.calibrations()[I].max;
                auto& min = appData.calibrations()[I].min;
                const auto v = Adc::value(index);
                if (max.isBottom()) {
                    min = max = v;
                }
                else {
                    max = std::max(max, v);
                    min = std::min(min, v);
                    appData.change();
                }
            }
            static inline auto sbus() {
                using v_t = SBus::value_type;
                const auto& max = appData.calibrations()[I].max;
                const auto& min = appData.calibrations()[I].min;
                return v_t{etl::scale(Adc::value(index).toInt(), etl::Intervall{min.toInt(), max.toInt()}, etl::Intervall{SBus::sbus_min, SBus::sbus_max})};
            }
        };
        static inline void update() {
            using i_t = SBus::index_type;
            using v_t = SBus::value_type;
            []<auto... II>(std::index_sequence<II...>){
                ((SBus::set(i_t(II), Stick<II>::sbus())), ...);
            }(std::make_index_sequence<numberOfSticks>{});

            SBus::set(i_t{numberOfSticks}, rot0::value());
            SBus::set(i_t{numberOfSticks + 1}, rot1::value());

            constexpr uint16_t d = (SBus::sbus_max - SBus::sbus_min) / 2;
            
            int16_t v0 = 2 * (sm_pa::axis(axis_index_t{0}) - 8192) + SBus::sbus_mid;
            int16_t v1 = 2 * (sm_pa::axis(axis_index_t{1}) - 8192) + SBus::sbus_mid;
            int16_t v2 = 2 * (sm_pa::axis(axis_index_t{2}) - 8192) + SBus::sbus_mid;
            int16_t v3 = 2 * (sm_pa::axis(axis_index_t{3}) - 8192) + SBus::sbus_mid;
            int16_t v4 = 2 * (sm_pa::axis(axis_index_t{4}) - 8192) + SBus::sbus_mid;
            int16_t v5 = 2 * (sm_pa::axis(axis_index_t{5}) - 8192) + SBus::sbus_mid;
            SBus::set(i_t{10}, v_t{v0});
            SBus::set(i_t{11}, v_t{v1});
            SBus::set(i_t{12}, v_t{v2});
            SBus::set(i_t{13}, v_t{v3});
            SBus::set(i_t{14}, v_t{v4});
            SBus::set(i_t{15}, v_t{v5});
        }
        
        static inline State mState{State::Undefined};
        static inline External::Tick<Timer> mSBusTicks;
        static inline External::Tick<Timer> mStateTicks;
        static inline External::Tick<Timer> mDebugTicks;
        static inline External::Tick<Timer> mEepromTicks;
        static inline External::Tick<Timer> mRoboTicks;
    };
}

using namespace AVR;
using namespace std::literals::chrono;
using namespace External::Units::literals;

namespace  {
    constexpr auto fRtc = 500_Hz; // for rotary
}

using ccp = Cpu::Ccp<>;
using clock = Clock<>;

using usart0Position = Portmux::Position<Component::Usart<0>, Portmux::Default>; // LUA / SpaceMouse
using usart1Position = Portmux::Position<Component::Usart<1>, Portmux::Default>; // SBus-out
using usart2Position = Portmux::Position<Component::Usart<2>, Portmux::Default>; // Bluetooth / Robo

using portmux = Portmux::StaticMapper<Meta::List<usart0Position, usart1Position, usart2Position>>;


using sm_pa = External::SpaceMouse::ProtocollAdapter<0>;
using sm = Usart<usart0Position, sm_pa, UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<8>>;

using systemTimer = SystemTimer<Component::Rtc<0>, fRtc>;

using sbus = External::SBus::Output::Generator<usart1Position, systemTimer>;

using robo_pa = External::QtRobo::ProtocollAdapter<0>;
using robo = Usart<usart2Position, robo_pa, UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<128>>;
using terminal = etl::basic_ostream<robo>;

using adc = Adc<Component::Adc<0>, AVR::Resolution<10>, Vref::V4_3>;
using adcController = External::Hal::AdcController<adc, Meta::NList<0, 1, 2, 4, 6, 7>>; // 1e = temp

using button0pin = ActiveLow<Pin<Port<D>, 3>, Input>;
using button1pin = ActiveLow<Pin<Port<D>, 5>, Input>;
using button0 = External::Button<button0pin, systemTimer, External::Tick<systemTimer>(50_ms), External::Tick<systemTimer>(300_ms)>;
using button1 = External::Button<button1pin, systemTimer, External::Tick<systemTimer>(50_ms), External::Tick<systemTimer>(300_ms)>;
using buttons = Buttons<button0, button1>;

using sw0pin = Pin<Port<A>, 2>;
using sw0 = Switch3State<sw0pin>;

using rotary0A = Pin<Port<E>, 0>;
using rotary0B = Pin<Port<E>, 1>;
using rotary1A = Pin<Port<F>, 2>;
using rotary1B = Pin<Port<F>, 3>;
using rot_t = etl::uint_ranged_circular<uint16_t, sbus::sbus_min, sbus::sbus_max>;
using rotary0 = External::RotaryEncoder<rotary0A, rotary0B, rot_t>;
using rotary1 = External::RotaryEncoder<rotary1A, rotary1B, rot_t>;

using eeprom = EEProm::Controller<Application::Data<adcController>>;

using gfsm = Application::GFSM<systemTimer, sbus, robo, sm, buttons, adcController, Meta::List<sw0>, Meta::List<rotary0, rotary1>, eeprom, terminal>;

int main() {
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
    
    systemTimer::init(); 
    
    gfsm::init();
    
    etl::outl<terminal>("rcboard spacemouse"_pgm);
    
    while(true) {
        gfsm::periodic();
        systemTimer::periodic([&]{
            gfsm::ratePeriodic();
        });
    }
}

#ifndef NDEBUG
[[noreturn]] inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], unsigned int line [[maybe_unused]]) noexcept {
    etl::outl<terminal>("Assertion failed: "_pgm, expr, etl::Char{','}, file, etl::Char{','}, line);
    while(true) {
//        terminalDevice::periodic();
//        dbg1::toggle();
    }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
