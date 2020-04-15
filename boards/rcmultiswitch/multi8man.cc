#define NDEBUG

#include "board.h"

using terminalDevice = AVR::Usart<usart0Position, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<2>, AVR::SendQueueLength<512>>;
using terminal = etl::basic_ostream<terminalDevice>;

namespace External {
    namespace Ppm {
        template<typename Ppm, uint8_t N>
        struct MultiSwitch {
            enum class State : uint8_t {UnDefined, CountUp, CountUpPause, ToggleUp, CountDown, CountDownPause, ToggleDown};
            enum class SwState : uint8_t {Off, On};
            
            using index_t = etl::uint_ranged<uint8_t, 0, ((N / 2) - 1)>;
            
            inline static constexpr uint8_t size() {
                return N;
            }
            inline static void init() {
                Ppm::init();
            }
            inline static void periodic() {
                Ppm::onCapture([]{
                    const auto v = Ppm::value(); 
                    if (!v) return;
                    switch(mState) {
                    case State::UnDefined:
                        if (isUp(v)) {
                            mState = State::CountUp;
                            count.setToBottom();
                            index.setToBottom();
                        }
                        if (isDown(v)) {
                            mState = State::CountDown;
                            count.setToBottom();
                            index.setToBottom();
                        }
                        break;
                    case State::CountUp:
                        if (isUp(v)) {
                            ++count;
                            if (count > setCycles) {
                                toggle1(index);
                                mState = State::ToggleUp;
                            }
                        }
                        else {
                            if (isNeutral(v)) {
                                if (count > stepCycles) {
                                    mState = State::CountUpPause;
                                    count.setToBottom();
                                }
                                else {
                                    mState = State::UnDefined;
                                }
                            }
                        }
                        break;
                    case State::ToggleUp:
                        if (isNeutral(v)) {
                            mState = State::UnDefined;
                        }
                        break;
                    case State::CountUpPause:
                        if (isNeutral(v)) {
                            ++count;
                            if (count > resetCycles) {
                                mState = State::UnDefined;
                            }
                        }
                        else {
                            if (isUp(v)) {
                                if (count > stepCycles) {
                                    mState = State::CountUp;
                                    count.setToBottom();
                                    ++index;
                                }
                                else {
                                    mState = State::UnDefined;
                                }
                            }
                        }
                        break;
                    case State::CountDown:
                        if (isDown(v)) {
                            ++count;
                            if (count > setCycles) {
                                toggle2(index);
                                mState = State::ToggleDown;
                            }
                        }
                        else {
                            if (isNeutral(v)) {
                                if (count > stepCycles) {
                                    mState = State::CountDownPause;
                                    count.setToBottom();
                                }
                                else {
                                    mState = State::UnDefined;
                                }
                            }
                        }
                        break;
                    case State::ToggleDown:
                        if (isNeutral(v)) {
                            mState = State::UnDefined;
                        }
                        break;
                    case State::CountDownPause:
                        if (isNeutral(v)) {
                            ++count;
                            if (count > resetCycles) {
                                mState = State::UnDefined;
                            }
                        }
                        else {
                            if (isDown(v)) {
                                if (count > stepCycles) {
                                    mState = State::CountDown;
                                    count.setToBottom();
                                    ++index;
                                }
                                else {
                                    mState = State::UnDefined;
                                }
                            }
                        }
                        break;
                    }
                });
            }
            //        private:
            static inline etl::uint_ranged<uint8_t, 0, 200> count;
            
            static inline constexpr uint8_t stepCycles{10};
            static inline constexpr uint8_t setCycles{50};
            static inline constexpr uint8_t resetCycles{50};
            
            static inline std::array<SwState, N> swStates{};
            static inline constexpr uint16_t thresh_up = Ppm::medium + (Ppm::span / 4);
            static inline constexpr uint16_t thresh_down = Ppm::medium - (Ppm::span / 4);
            static inline constexpr uint16_t thresh_neutral_low  = Ppm::medium - (Ppm::span / 8);
            static inline constexpr uint16_t thresh_neutral_high = Ppm::medium + (Ppm::span / 8);
            static inline index_t index;
            static inline State mState{State::UnDefined};
            
            static inline void toggle1(const index_t i) {
                if (swStates[i] == SwState::Off) {
                    swStates[i] = SwState::On;
                }
                else {
                    swStates[i] = SwState::Off;
                }
            }
            static inline void toggle2(const index_t k) {
                const auto i = k + (N/2);
                if (swStates[i] == SwState::Off) {
                    swStates[i] = SwState::On;
                }
                else {
                    swStates[i] = SwState::Off;
                }
            }
            
            static inline bool isNeutral(const auto v) {
                return (v >= thresh_neutral_low) && (v <= thresh_neutral_high);
            }
            static inline bool isUp(const auto v) {
                return (v >= thresh_up);
            }
            static inline bool isDown(const auto v) {
                return (v <= thresh_down);
            }
        };
    }
}

using multi = External::Ppm::MultiSwitch<ppm, 8>;

int main() {
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
    
    evrouter::init();
    
    terminalDevice::init<BaudRate<9600>>();
    terminalDevice::rxEnable<false>();
    
    adcController::init();
    
    systemTimer::init();
    
    Meta::visit<ledList>([]<typename L>(Meta::Wrapper<L>){
                             L::template dir<Output>();
                         });
    
    multi::init();
    
    const auto periodicTimer = alarmTimer::create(100_ms, External::Hal::AlarmFlags::Periodic);
    
    etl::outl<terminal>("multi8mk4"_pgm);
    
    using index_t = etl::uint_ranged_circular<uint8_t, 0, multi::size() - 1>;
    using led_index_t = etl::uint_ranged<uint8_t, 0, 7>;
    
    index_t index;
    
//    constexpr auto ch0 = adcController::index_type{0};
    
//    for(auto jv : JumperIntervalls) {
//        etl::outl<terminal>(jv.first, ", "_pgm, jv.second);
//    }
    
    while(true) {
        adcController::periodic();
        terminalDevice::periodic();
        multi::periodic();        
        
        auto li = led_index_t(index);
        
        if (multi::swStates[index] == multi::SwState::Off) {
            Meta::visitAt<ledList>(li, []<typename L>(Meta::Wrapper<L>){
                                       L::off();
                                   });
        }            
        else if (multi::swStates[index] == multi::SwState::On) {
            Meta::visitAt<ledList>(li, []<typename L>(Meta::Wrapper<L>){
                                       L::on();
                                   });
        }            
        ++index;
        systemTimer::periodic([&]{
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    etl::out<terminal>("sw: [ "_pgm);
                    for(auto l : multi::swStates) {
                        etl::out<terminal>(uint8_t(l), " "_pgm);
                    }
                    etl::outl<terminal>(" ] a: "_pgm, getConfigValue());
                }
            });
        });
    }
}

