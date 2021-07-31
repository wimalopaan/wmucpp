#pragma once

namespace SimpleProtocol {
    struct CheckSum final {
        inline void reset() {
            mSum = 0;
        }
        inline const std::byte& operator+=(const std::byte& b) {
            mSum += static_cast<uint8_t>(b);
            return b;
        }
        inline operator std::byte() const {
            return std::byte{mSum};
        }
    private:
        uint8_t mSum{};
    };
    
    template<uint8_t Size>
    struct Sender final {
        using values_type = std::array<std::byte, Size>;
        
        static inline constexpr uint8_t size() {
            return values_type::size();
        }
        static inline constexpr uint8_t length() {
            return values_type::size() + 2;
        }
        template<uint8_t N>
        static inline void set(const std::byte v) {
            static_assert(N < Size);
            mValues[N] = v;
        }
        template<typename Dev>
        static inline void send() {
            CheckSum cs;
            cs += startByte;
            Dev::put(startByte);
            
            for(const auto& v: mValues) {
                cs += v;
                Dev::put(v);
            }
            Dev::put(cs);
        }
    private:
        inline static constexpr std::byte startByte{0xa5};
        inline static values_type mValues{};
    };

    template<auto Size, typename MCU = DefaultMcuType>
    struct Adapter {
        using values_type = std::array<std::byte, Size>;

        inline static constexpr std::byte startByte{0xa5};
        
        enum class State : uint8_t {Init, AwaitData, AwaitCS};

        inline static bool process(const std::byte b) {
            static CheckSum cs;
            switch(mState) {
            case State::Init:
                cs.reset();
                if (b == startByte) {
                    cs += b;
                    mReceivesBytes.setToBottom();
                    mState = State::AwaitData;
                }
                break;
            case State::AwaitData:
                mBuffer[mReceivesBytes] = b;
                cs += b;
                if (mReceivesBytes.isTop()) {
                    mState = State::AwaitCS;
                }
                ++mReceivesBytes;
                break;
            case State::AwaitCS:
                if (b == cs) {
                    etl::copy(mValues, mBuffer);
                }
                mState = State::Init;
                break;
            }
            return true;
        }
        inline static const values_type& data() {
            return mValues;
        }
        inline static void ratePeriodic() {}
    private:
        inline static etl::index_type_t<values_type> mReceivesBytes;
        inline static values_type mBuffer{};
        inline static values_type mValues{};
        inline static State mState{State::Init};
    };
}
