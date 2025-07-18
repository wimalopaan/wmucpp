#include <iostream>

struct Sbus {

    static inline constexpr bool additionalChecks = true;

    static inline constexpr uint16_t min = 172;
    static inline constexpr uint16_t max = 1812;

    static inline std::array<uint16_t, 16> mChannels{};
    static inline uint8_t mFlagsAndSwitches{};

    static inline void clear() {
        for(auto& c : mChannels) {
            c = 0;
        }
    }
    static inline void print() {
        std::cout << "Channels:";
        for(const auto& v : mChannels) {
            std::cout << ' ' << v;
        }
        std::cout << ' ' << "Flags:" << uint16_t{mFlagsAndSwitches} << '\n';
    }
    static inline void decode(const auto& data) {
        const volatile uint8_t* const mData = &data[0];

        std::array<uint16_t, 16> raw;

        raw[0]  = (uint16_t) (((mData[0]    | mData[1] << 8))              & 0x07FF);
        raw[1]  = (uint16_t) ((mData[1]>>3  | mData[2] <<5)                & 0x07FF);
        raw[2]  = (uint16_t) ((mData[2]>>6  | mData[3] <<2 | mData[4]<<10) & 0x07FF);
        raw[3]  = (uint16_t) ((mData[4]>>1  | mData[5] <<7)                & 0x07FF);
        raw[4]  = (uint16_t) ((mData[5]>>4  | mData[6] <<4)                & 0x07FF);
        raw[5]  = (uint16_t) ((mData[6]>>7  | mData[7] <<1 | mData[8]<<9)  & 0x07FF);
        raw[6]  = (uint16_t) ((mData[8]>>2  | mData[9] <<6)                & 0x07FF);
        raw[7]  = (uint16_t) ((mData[9]>>5  | mData[10]<<3)                & 0x07FF);
        raw[8]  = (uint16_t) ((mData[11]    | mData[12]<<8)                & 0x07FF);
        raw[9]  = (uint16_t) ((mData[12]>>3 | mData[13]<<5)                & 0x07FF);
        raw[10] = (uint16_t) ((mData[13]>>6 | mData[14]<<2 | mData[15]<<10)& 0x07FF);
        raw[11] = (uint16_t) ((mData[15]>>1 | mData[16]<<7)                & 0x07FF);
        raw[12] = (uint16_t) ((mData[16]>>4 | mData[17]<<4)                & 0x07FF);
        raw[13] = (uint16_t) ((mData[17]>>7 | mData[18]<<1 | mData[19]<<9) & 0x07FF);
        raw[14] = (uint16_t) ((mData[19]>>2 | mData[20]<<6)                & 0x07FF);
        raw[15] = (uint16_t) ((mData[20]>>5 | mData[21]<<3)                & 0x07FF);
        bool ok = true;
        if constexpr(additionalChecks) {
            for(uint8_t i = 0; i < raw.size(); ++i) {
                if (!((raw[i] >= min) && (raw[i] <= max))) {
                    ok = false;
                    break;
                }
            }
        }
        if (ok) {
            for(uint8_t i = 0; i < raw.size(); ++i) {
                mChannels[i] = raw[i];
            }
            mFlagsAndSwitches = mData[22] & 0x0f;
        }
    }
};

int main() {
    std::array<std::array<uint8_t, 24>, 6> inputs = {{
        {238 ,3 ,31 ,43 ,194 ,7 ,62 ,240 ,129 ,15 ,124 ,224 ,3 ,31 ,248 ,192 ,7 ,62 ,240 ,129 ,207 ,207 ,0 ,0},
        {240 ,11 ,31 ,43 ,194 ,7 ,62 ,240 ,129 ,15 ,124 ,224 ,3 ,31 ,248 ,192 ,7 ,62 ,240 ,129 ,207 ,214 ,0 ,0} ,
        {240 ,11 ,31 ,43 ,194 ,7 ,62 ,240 ,129 ,15 ,124 ,224 ,3 ,31 ,248 ,192 ,7 ,62 ,240 ,129 ,207 ,207 ,0 ,0} ,
        {240 ,11 ,31 ,43 ,194 ,7 ,62 ,240 ,129 ,15 ,124 ,224 ,3 ,31 ,248 ,192 ,7 ,62 ,240 ,129 ,207 ,207 ,0 ,0} ,
        {240 ,11 ,31 ,43 ,194 ,7 ,62 ,240 ,129 ,15 ,124 ,224 ,3 ,31 ,248 ,192 ,7 ,62 ,240 ,129 ,207 ,214 ,0 ,0} ,
        {240 ,3 ,31 ,43 ,194 ,7 ,62 ,240 ,129 ,15 ,124 ,224 ,3 ,31 ,248 ,192 ,7 ,62 ,240 ,129 ,207 ,207 ,0 ,0}
    }};

    for(const auto& input : inputs) {
        Sbus::clear();
        Sbus::decode(input);
        Sbus::print();
    }

}
