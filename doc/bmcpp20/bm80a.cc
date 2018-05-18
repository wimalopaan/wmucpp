#define NDEBUG

#include <cstddef>
#include <cstdint>
#include "util/meta.h"

template<typename From, typename To, typename E>
struct Tr {
    typedef From from_state;  
    typedef To   to_state;  
    typedef E    event_type;
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
    requires Meta::contains<events, Ev>::value
    inline static void process(const Ev&) {
        Meta::visitAt<states>(mState, []<typename T>(T){
                                  using state = typename T::type;
                                  using txs = to_state_from<state, Ev>;
                                  if constexpr(Meta::size<txs>::value > 0) {
                                      using to = typename Meta::front<txs>::to_state;
                                      leave(state{});
                                      enter(to{});
                                      mState = Meta::index<states, to>::value;
                                  }
                                  
                              });        
    }
    inline static void process(uint8_t e) {
        Meta::visitAt<events>(e, []<typename W>(W){
                                  typedef typename W::type event;
                                  process(event{});
                              });
    }
    inline static uint8_t mState{};
};

struct StateA {
    inline void leave() const {}
};
struct StateB {
    inline void leave() const {}
};
struct StateC {
    inline void leave() const {}
};

volatile uint8_t x;

template<typename S>
inline void enter(const S& state) {
    state.enter();
}
template<typename S>
inline void leave(const S& state) {
    state.leave();
}

template<>
inline void enter<StateA>(const StateA&) {
    x = 0;
}
template<>
inline void enter<StateB>(const StateB&) {
    x = 1;
}
template<>
inline void enter<StateC>(const StateC&) {
    x = 2;
}

struct E1 {};
struct E2 {};
struct E3 {};

using fsm = FSM<Tr<StateA, StateB, E1>, Tr<StateB, StateC, E2>, Tr<StateC, StateA, E3>>;


int main(){
//    uint8_t e = 0;
//    fsm::process(e);
//    e = 1;
//    fsm::process(e);
//    e = 2;
//    fsm::process(e);

    fsm::process(E1{});
    fsm::process(E2{});
    fsm::process(E3{});
}
