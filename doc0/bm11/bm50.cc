#include <cstdint>
#include <coroutine>
#include <memory>
#include <utility>

namespace {
    
    //template<typename T>
    //struct Generator {
    //    struct promise_type;
    
    //    using handle_type = std::coroutine_handle<promise_type>;
    
    //    struct promise_type { // required
    //        T value_;
    //        //    std::exception_ptr exception_;
    //        char exception_;
    
    //        Generator get_return_object() {
    //            return Generator(handle_type::from_promise(*this));
    //        }
    //        std::suspend_always initial_suspend() noexcept { return {}; }
    //        std::suspend_always final_suspend() noexcept { return {}; }
    
    //        template<typename From> // C++20 concept
    //        std::suspend_always yield_value(From &&from) {
    //            value_ = std::forward<From>(from); // caching the result in promise
    //            return {};
    //        }
    //        void return_void() {}
    
    //        void* operator new(size_t) {
    //            return pool;
    //        }
    //        void operator delete(void*) {
    //        }
    
    //        inline static std::byte pool[100];        
    //    };
    
    //    handle_type h_;
    
    //    Generator(handle_type h) : h_(h) {}
    //    ~Generator() { h_.destroy(); }
    //    explicit operator bool() {
    //        fill(); // The only way to reliably find out whether or not we finished coroutine,
    //        // whether or not there is going to be a next value generated (co_yield) in
    //        // coroutine via C++ getter (operator () below) is to execute/resume coroutine
    //        // until the next co_yield point (or let it fall off end).
    //        // Then we store/cache result in promise to allow getter (operator() below to
    //        // grab it without executing coroutine).
    //        return !h_.done();
    //    }
    //    T operator()() {
    //        fill();
    //        full_ = false; // we are going to move out previously cached
    //        // result to make promise empty again
    //        return std::move(h_.promise().value_);
    //    }
    
    //private:
    //    bool full_ = false;
    
    
    //    void fill() {
    //        if (!full_) {
    //            h_();
    //            if (h_.promise().exception_)
    //                //        std::rethrow_exception(h_.promise().exception_);
    //                // propagate coroutine exception in called context
    
    //                full_ = true;
    //        }
    //    }
    //};
    
    
    template<typename T>
    struct ReturnObject {
    private:
        inline static std::byte pool[100];        
    public:
        struct promise_type {
            T val_{};
            ReturnObject get_return_object() {
                asm(";get return obj");
                return {std::coroutine_handle<promise_type>::from_promise(*this)}; 
            }
            std::suspend_never initial_suspend() const { 
                asm(";initial suspend");
                return {}; 
            }
            std::suspend_never final_suspend() const noexcept { 
                asm(";finalsuspend");
                return {}; 
            }
            std::suspend_always yield_value(T value) {
                asm(";yield_value");
                val_ = std::move(value);
                return {};
            }
            static void* operator new(size_t) {
                asm(";new");
                return pool;
            }
            
            static void operator delete(void*) {
                asm(";delete");
            }
        };
        T operator()() {
            asm(";op()");
            h_();
            return std::move(h_.promise().val_);
        }
    private:
        std::coroutine_handle<promise_type> h_; 
        ReturnObject(std::coroutine_handle<promise_type> h):h_{h}{ } 
        
        //    operator std::coroutine_handle<promise_type>() const { return h_; } 
        
        //    std::coroutine_handle<promise_type> handle() const {
        //        return h_;
        //    }
        
    };
    
    //Generator<uint16_t> f() {
    //    uint16_t i{0};
    //    while (true) {
    //        co_yield ++i;
    //    }
    //}
    
    
    ReturnObject<uint8_t> g() {
        uint8_t i{0};
        while (true) {
            asm(";co while");            
            co_yield ++i;
        }
    }
    
//    volatile uint8_t r; 

//    ReturnObject<bool> g2() {
//        while (true) {
//            asm(";co while");            
//            r = 1;
//            co_yield false;
//            r = 2;
//            co_yield false;
//            r = 10;
//            co_yield false;
//            r = 20;
//            co_yield true;
//        }
//    }
    
}

[[noreturn]] int main() {
    auto h = g();
    while(true) {
        asm(";main while");            
//        r = h();
        h();
    }
}
