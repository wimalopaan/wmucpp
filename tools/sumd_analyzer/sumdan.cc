#include <vector>
#include <string>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <cassert>
#include <experimental/filesystem>

class SumdFrame final {
    friend std::ostream& operator<<(std::ostream& out, const SumdFrame& frame);
public:
    auto size() const {
        return mData.size();
    }
    void push_back(uint16_t v) {
        mData.push_back(v);
    }
    void clear() {
        mData.clear();
    }
    void parity(uint16_t p) {
        mParity = p;
    } 
    explicit operator bool() const {
        return true;
    }
private:
    std::vector<uint16_t> mData;
    uint16_t mParity = 0;
};

std::ostream& operator<<(std::ostream& out, const SumdFrame& frame) {
    out << std::dec;
    out << "Frame(" << frame.mData.size() << "):";
    for(const auto& v: frame.mData) {
        out << " " << std::hex << std::showbase << v;
    }
    out << std::dec;
    return out;
}

int main(int argc, const char** argv) {
    std::vector<std::string> args;
    std::copy(argv, argv + argc, std::back_inserter(args));
    
    if (args.size() <= 1) {
        std::cerr << "usage: " << args[0] << " <file>" << std::endl;
    }
    
    std::ifstream file{args[1], std::ios::binary};
    
    if (file.is_open()) {
        std::vector<uint8_t> data{std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
        std::cout << "Bytes: " << data.size() << std::endl;
        
        enum class State : uint8_t {Invalid, Start1, Start2, DataH, DataL, ParityH, ParityL};
        State state = State::Invalid;
        SumdFrame frame;
        
        std::vector<SumdFrame> frames;
        size_t nChannels = 0;
        uint16_t v = 0;
        
        for(const uint8_t& b : data) {
            switch(state) {
            case State::Invalid:
                if (b == 0xA8) {
                    state = State::Start1;
                }
                break;
            case State::Start1:
                if ((b == 0x01) || (b == 0x81)) {
                    state = State::Start2;
                }
                break;
            case State::Start2:
                nChannels = b;
                state = State::DataH;
                frame.clear();
                break;
            case State::DataH:
                v = ((uint16_t)b << 8);
                state = State::DataL;
                break;
            case State::DataL:
                v += (b & 0xff);
                frame.push_back(v);
                if (frame.size() < nChannels) {
                    state = State::DataH;
                }
                else {
                    state = State::ParityH;
                }
                break;
            case State::ParityH:
                v = ((uint16_t)b << 8);
                state = State::ParityL;
                break;
            case State::ParityL:
                v += (b & 0xff);
                frame.parity(v);
                frames.push_back(frame);
                state = State::Invalid;
                break;
            default:
                assert(false);
                break;
            }            
        }
        std::cout << "N Frames: " << frames.size() << std::endl;
        for(const auto& f : frames) {
            std::cout << f << std::endl;
        }
    }
    else {
        std::cerr << "can't open file: " << argv[1] << std::endl;
    }    
}
