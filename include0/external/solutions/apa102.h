#pragma once

#include <cstdint>
#include <etl/ranged.h>
#include "color.h"

namespace External {
    struct APA102 final {
        using array_type = std::array<std::byte, 4>;
        using index_type = etl::index_type_t<array_type>;
        using brightness_type = etl::uint_ranged<uint8_t, 0, 31>;
        
        inline static constexpr std::byte start_byte{0b1110'0000}; 
        
        inline constexpr APA102() = default;
        
        inline explicit constexpr APA102(const External::Crgb& c, brightness_type br = brightness_type::upper()) : 
            mData{start_byte, std::byte{c.b}, std::byte{c.g}, std::byte{c.r}} {
            brightness(br);
        }
        
        inline explicit constexpr APA102(External::Red r, External::Green g, External::Blue b, brightness_type br = brightness_type::upper()) : 
            mData{start_byte, std::byte{b.value}, std::byte{g.value}, std::byte{r.value}} {
            brightness(br);
        }
        
        inline constexpr void operator=(const External::Crgb& c) {
            mData = {start_byte, std::byte{c.b}, std::byte{c.g}, std::byte{c.r}};
            brightness(brightness_type::upper());
        }
        
        inline constexpr void brightness(brightness_type b) {
            mData[0] |= std::byte{b.toInt()};
        }
        
        inline static constexpr auto startFrame() {
            return APA102({0x00_B, 0x00_B, 0x00_B, 0x00_B});
        }
        inline static constexpr auto endFrame() {
            constexpr std::byte ones{0xff};
            return APA102({ones, ones, ones, ones});
        }
        inline static constexpr auto size() {
            return array_type::size();
        }
        inline constexpr auto& data() const {
            return mData;
        }
    private:
        inline constexpr APA102(const array_type& d) : mData{d} {}
        array_type mData{};
    };
    
    template<typename Dev, typename Led, auto Size>
    struct LedStripe final {
        LedStripe() = delete;
        
        enum class State : uint8_t {Complete, Start, Data, End};
        inline static constexpr auto ef = Led::endFrame();
        inline static constexpr auto sf = Led::startFrame();
        
        using device_type = Dev;
        using array_type = std::array<Led, Size>;
        using index_type = etl::index_type_t<array_type>;
        using size_type = array_type::size_type;
        
        inline static void init() {
            device_type::init();
        }
        inline static void clear() {
            all({});
        }
        inline static void all(const External::Crgb& c) {
            std::fill(std::begin(led), std::end(led), c);
        }
        inline static void set(index_type from, index_type to, const External::Crgb& c) {
            std::fill(&led[from], &led[to] + 1, c);
        }
        inline static void set(index_type index, const External::Crgb& c) {
            led[index] = c;
        }
        constexpr inline static size_type size() {
            return led.size();
        }
        inline static void out() {
            if (mState == State::Complete) {
                index.setToBottom();
                byte_index.setToBottom();
                mState = State::Start;
            }
        }
        inline static bool isIdle() {
            return (mState == State::Complete);
        }
        inline static void periodic() {
            Dev::periodic();
            
            switch(mState) {
            case State::Complete:
                break;
            case State::Start:
                if (Dev::put(sf.data()[byte_index])) {
                    if (byte_index.isTop()) {
                        mState = State::Data;
                        byte_index.setToBottom();
                    }
                    else {
                        ++byte_index;
                    }
                }
                break;
            case State::Data:
                if (Dev::put(led[index].data()[byte_index])) {
                    if (byte_index.isTop()) {
                        if (index.isTop()) {
                            mState = State::End;
                        }
                        else {
                            ++index;
                        }
                        byte_index.setToBottom();
                    }
                    else {
                        ++byte_index;
                    }
                }
                break;
            case State::End:
                if (Dev::put(ef.data()[byte_index])) {
                    if (byte_index.isTop()) {
                        mState = State::Complete;
                    }
                    else {
                        ++byte_index;
                    }
                }
                break;
            }
        }
    private:
        inline static State mState{State::Complete};
        inline static index_type index{};
        inline static typename Led::index_type byte_index{};
        inline static array_type led{};
    };    
}
