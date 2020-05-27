// PWM
// 1) einen einschalten
// 2) Jumper 1
// 3) mit SW7/SW8 hoch und runter

// Blinken
// 1) einen einschalten
// 2) Jumper 2
// 3) mit SW7/SW8 Frequenz -> 50% Tastverh.
// 4) Jumper 3
// 5) mit SW7/SW8 Tastverhältnis ändern

#define NDEBUG

#include "board.h"
#include "swout.h"

using terminalDevice = AVR::Usart<usart0Position, External::Hal::NullProtocollAdapter, AVR::UseInterrupts<false>>;
using terminal = etl::basic_ostream<terminalDevice>;

namespace External {
    namespace Graupner {
        template<typename Ppm, uint8_t N>
        struct MultiSwitch {
            enum class State : uint8_t {UnDefined, GotSync1, GotSync2};
            enum class SwState : uint8_t {Off, Up, Down};
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
                        if (v >= Ppm::ppmMax) {
                            mState = State::GotSync1;
                        }
                        break;
                    case State::GotSync1:
                        if (v >= Ppm::ppmMax) {
                            mState = State::GotSync2;
                            index.setToBottom();
                        }
                        else {
                            mState = State::UnDefined;
                        }
                        break;
                    case State::GotSync2:
                        if (v < Ppm::ppmMax) {
                            if (v >= thresh_up) {
                                swStates[index] = SwState::Up;
                            }
                            else if (v <= thresh_down) {
                                swStates[index] = SwState::Down;
                            }
                            else {
                                swStates[index] = SwState::Off;
                            }
                        }
                        else {
                            mState = State::GotSync1;
                        }
                        if (index.isTop()) {
                            mState = State::UnDefined;
                        }
                        else {
                            ++index;
                        }
                        break;
                    }
                });
            }
//        private:
            static inline std::array<SwState, N> swStates{};
            static inline constexpr uint16_t thresh_up = Ppm::medium + (Ppm::span / 4);
            static inline constexpr uint16_t thresh_down = Ppm::medium - (Ppm::span / 4);
            static inline etl::uint_ranged<uint8_t, 0, (N - 1)> index;
            static inline State mState{State::UnDefined};
        };
    }
}

using mk4 = External::Graupner::MultiSwitch<ppm, 4>;

int main() {
    portmux::init();
    
    ccp::unlock([]{
        clock::prescale<1>();
    });
    
    evrouter::init();
   
    terminalDevice::init<BaudRate<9600>>();
    terminalDevice::rxEnable<false>();
    
    systemTimer::init();

    Meta::visit<ledList>([]<typename L>(Meta::Wrapper<L>){
                             L::template dir<Output>();
                         });
    
    mk4::init();
    
    const auto periodicTimer = alarmTimer::create(100_ms, External::Hal::AlarmFlags::Periodic);

    etl::outl<terminal>("multi8mk4"_pgm);

    using index_t = etl::uint_ranged_circular<uint8_t, 0, mk4::size() - 1>;
    using led_index_t = etl::uint_ranged<uint8_t, 0, 7>;
    
    index_t index;
    
    while(true) {
        terminalDevice::periodic();
        mk4::periodic();        

        auto li_1 = led_index_t(2 * index);
        auto li_2 = led_index_t(2 * index + 1);
        
        if (mk4::swStates[index] == mk4::SwState::Off) {
            Meta::visitAt<ledList>(li_1, []<typename L>(Meta::Wrapper<L>){
                                     L::off();
                                 });
            Meta::visitAt<ledList>(li_2, []<typename L>(Meta::Wrapper<L>){
                                     L::off();
                                 });
        }            
        else if (mk4::swStates[index] == mk4::SwState::Up) {
            Meta::visitAt<ledList>(li_1, []<typename L>(Meta::Wrapper<L>){
                                     L::on();
                                 });
            Meta::visitAt<ledList>(li_2, []<typename L>(Meta::Wrapper<L>){
                                     L::off();
                                 });
        }            
        else if (mk4::swStates[index] == mk4::SwState::Down) {
            Meta::visitAt<ledList>(li_2, []<typename L>(Meta::Wrapper<L>){
                                     L::on();
                                 });
            Meta::visitAt<ledList>(li_1, []<typename L>(Meta::Wrapper<L>){
                                     L::off();
                                 });
        }            
        ++index;
        systemTimer::periodic([&]{
            alarmTimer::periodic([&](const auto& t){
                if (periodicTimer == t) {
                    etl::out<terminal>("sw: [ "_pgm);
                    for(auto l : mk4::swStates) {
                        etl::out<terminal>(uint8_t(l), " "_pgm);
                    }
                    etl::outl<terminal>(" ]"_pgm);
                }
            });
        });
    }
}

