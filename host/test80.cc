#include <cstddef>
#include <cstdint>
#include "etl/meta.h"

#include <iostream>

template<typename In, In Low, In High, typename Event>
struct RangeEvent {
    typedef In input_type;
    typedef Event event_type;
    template<typename FSM>
    inline static void process(const input_type& data) {
        if ((data >= Low) && (data <= High)) {
            FSM::process(event_type{data});
        }
    }
};

template<typename In, In Value, typename Event>
struct ValueEvent {
    typedef In input_type;
    typedef Event event_type;
    template<typename FSM>
    inline static void process(const input_type& data) {
        if (data == Value) {
            FSM::process(event_type{});
        }
    }
};

template<typename FSM, typename... Ms>
struct EventMapper {
    typedef Meta::List<Ms...> mappings;
    template<typename T>
    using get_input_type = typename T::input_type;
    static_assert(Meta::all_same_front<Meta::transform<get_input_type, mappings>>::value);
    typedef Meta::front<Meta::transform<get_input_type, mappings>> input_type;
    
    inline static void process(const input_type& value) {
        (Ms::template process<FSM>(value), ...);        
    }
};


template<typename From, typename To, typename E>
struct Tr {
    typedef From from_state;  
    typedef To   to_state;  
    typedef E    event_type;
    static_assert(!std::is_same<from_state, to_state>::value, "use different states");
};
          

template<typename... TRs>
struct FSM {
    template<typename TR>
    using get_from_state = typename TR::from_state;
    template<typename TR>
    using get_to_state = typename TR::to_state;
    template<typename TR>
    using get_event = typename TR::event_type;
    
    
    using transitions = Meta::List<TRs...>;
    using from_states = Meta::unique<Meta::transform<get_from_state, transitions>>;
    using to_states = Meta::unique<Meta::transform<get_to_state, transitions>>;
    using events = Meta::unique<Meta::transform<get_event, transitions>>;

    using states = Meta::unique<Meta::concat<from_states, to_states>>;
    
    static_assert(Meta::size<states>::value < 256);

    template<typename F, typename E, typename Tx>
    struct to_state_from_impl {
        template<typename Trans, typename From, typename Event>
        struct Predicate {
            inline static constexpr bool value = []{
                if constexpr(std::is_same<typename Trans::from_state, From>::value && std::is_same<typename Trans::event_type, Event>::value) {
                    return true;
                }
                return false;
            }();
        };
        
        template<typename X>
        using Pred = Predicate<X, F, E>;
        
        typedef Meta::filter<Pred, Tx> selected_transitions;
        
        typedef selected_transitions type;
    };

    template<typename F, typename E>
    using to_state_from = typename to_state_from_impl<F, E, transitions>::type;

    template<typename Ev>
    static void process(const Ev& ev) {
        Meta::visitAt<states>(mState, [&]<typename T>(T){
                                  using state = typename T::type;
                                  using txs = to_state_from<state, Ev>;
                                  if constexpr(Meta::size<txs>::value > 0) {
                                      using to = typename Meta::front<txs>::to_state;
//                                      leave(state{});
                                      enter(to{});
                                      mState = Meta::index<states, to>::value;
                                  }
                                  else {
                                      stay(state{}, ev);
                                  }
                                  
        });        
    }
//    inline static void process(uint8_t e) {
//        Meta::visitAt<events>(e, []<typename W>(W){
//                                  typedef typename W::type event;
//                                  process(event{});
//                            });
//    }
    inline static uint8_t mState = 2;
};

struct StateA {};
struct StateB {};
struct StateC {};

struct E1 {
    unsigned char data;
};
struct E2 {};
struct E3 {};

volatile uint8_t x;

void enter(StateA) {
    std::cout << __PRETTY_FUNCTION__ << '\n';
    x = 0;
}
void enter(StateB) {
    std::cout << __PRETTY_FUNCTION__ << '\n';
    x = 1;
}
void enter(StateC) {
    std::cout << __PRETTY_FUNCTION__ << '\n';
    x = 2;
}


void stay(StateA, auto) {
    std::cout << __PRETTY_FUNCTION__ << '\n';
}
void stay(StateB, auto) {
    std::cout << __PRETTY_FUNCTION__ << '\n';
}
void stay(StateC, auto) {
    std::cout << __PRETTY_FUNCTION__ << '\n';
}

using fsm = FSM<Tr<StateA, StateB, E1>, Tr<StateB, StateC, E2>, Tr<StateC, StateA, E3>>;
using mapper = EventMapper<fsm, RangeEvent<unsigned char, 'A', 'z', E1>, ValueEvent<unsigned char, ',', E2>>;

int main(){
    unsigned char c = 'a';
    mapper::process(c);
    c = 'b';
    mapper::process(c);

    //    fsm::process(E3{});
    //    std::cout << (int)fsm::mState << '\n';
    //    fsm::process(E2{});
    //    std::cout << (int)fsm::mState << '\n';
    //    fsm::process(E1{});
    //    std::cout << (int)fsm::mState << '\n';
    //    fsm::process(E2{});
    //    std::cout << (int)fsm::mState << '\n';
    //    fsm::process(E3{});
    //    std::cout << (int)fsm::mState << '\n';
    
        
}
