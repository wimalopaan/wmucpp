#pragma once

namespace etl {
    template<typename T>
    struct Event {
        void operator=(const T e) volatile {
            mEvent = e;
        }
        void operator=(const T e) {
            mEvent = e;
        }
        void on(const T which, const auto f) {
            if (is(which)) {
                f();
            }
        }
        bool is(const T which) {
            if (mEvent == which) {
                mEvent = T::None;
                return true;
            }
            return false;
        }
        bool is(const T which) volatile {
            if (mEvent == which) {
                mEvent = T::None;
                return true;
            }
            return false;
        }
        private:
        T mEvent = T::None;
    };
}



