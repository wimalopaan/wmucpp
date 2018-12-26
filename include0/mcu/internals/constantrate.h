#pragma once

template<typename Buffer, typename Device, typename CounterType = typename Buffer::index_type, bool disableRx = true>
class ConstanteRateWriter { 
    using enableFlagType = std::conditional_t<Device::use_isr_type::value, volatile bool, bool>;
public:
    inline static void rateProcess() {
        if (!mEnable) return;
        
        if (counter == 0) {
            if constexpr(disableRx) {
                Device::template rxEnable<false>();
            }
        }
        if (counter < Buffer::size()) {
            if (auto data = Buffer::get(counter++)) {
                Device::put(*data);
            }
        }
        else {
            assert(counter >= Buffer::size());
            if (!Device::isEmpty()) {
                return;
            }
            else {
                if constexpr(disableRx) {
                    Device::template rxEnable<true>();
                }
                mEnable = false;
                counter = 0;
                Buffer::reset();
            }
        }
    }
    template<bool E>
    inline static void enable() { 
        mEnable = E;
    }
    
    // todo: remove (nur für Concepts bzw. Event-Steuerung)
    inline static void start() { 
        Buffer::reset();
        counter = 0;
    }
    inline static void init() {
        Buffer::init();
    }
private:
    inline static enableFlagType mEnable = true; 
    inline static CounterType counter = 0; 
};
