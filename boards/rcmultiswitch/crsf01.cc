#define NDEBUG
#define AUTO_BUS

#include "board.h"
#include "crc.h"
#include "crsf.h"

namespace Crsf {
    namespace Address {
        inline static constexpr std::byte Broadcast{0x00};
        inline static constexpr std::byte CC{0xa0};
        inline static constexpr std::byte Switch{0xa1};
        inline static constexpr std::byte Controller{0xc8};
        inline static constexpr std::byte Handset{0xea};
        inline static constexpr std::byte RX{0xec};
        inline static constexpr std::byte TX{0xee};
    }
    namespace Type {
        // https://github.com/EdgeTX/edgetx/blob/main/radio/src/telemetry/crossfire.h
        // simple
        inline static constexpr std::byte Gps{0x02};
        inline static constexpr std::byte Vario{0x07};
        inline static constexpr std::byte Battery{0x08};
        inline static constexpr std::byte Baro{0x09};
        inline static constexpr std::byte HeartBeat{0x0b};
        inline static constexpr std::byte Link{0x14};
        inline static constexpr std::byte Channels{0x16};
        inline static constexpr std::byte SubsetChannels{0x17};
        inline static constexpr std::byte LinkStatsRx{0x1c};
        inline static constexpr std::byte LinkStatsTx{0x1d};
        inline static constexpr std::byte Altitude{0x1e};
        inline static constexpr std::byte FlightMode{0x1e};
        // extended
        inline static constexpr std::byte Ping{0x28};
        inline static constexpr std::byte Info{0x29};
        inline static constexpr std::byte ParamEntry{0x2b};
        inline static constexpr std::byte ParamRead{0x2c};
        inline static constexpr std::byte ParamWrite{0x2d};
        inline static constexpr std::byte Command{0x32};

        inline static constexpr std::byte Custom1{0x40}; // pack type / instance-nr in payload
        inline static constexpr std::byte Custom2{0x41}; // pack type / instance-nr in payload
        inline static constexpr std::byte Custom3{0x42}; // pack type / instance-nr in payload
        inline static constexpr std::byte Custom4{0x43}; // pack type / instance-nr in payload
        inline static constexpr std::byte Temp1{0x48};
        inline static constexpr std::byte Temp2{0x49};
        inline static constexpr std::byte Temp3{0x4a};
        inline static constexpr std::byte Temp4{0x4b};
        //inline static constexpr std::byte Rpm1{0x78};
        inline static constexpr std::byte Rpm1{0x4c};
        inline static constexpr std::byte Rpm2{0x4d};
        inline static constexpr std::byte Rpm3{0x4e};
        inline static constexpr std::byte Rpm4{0x4f};

        inline static constexpr std::byte KissReq{0x78};
        inline static constexpr std::byte KissResp{0x79};

        inline static constexpr std::byte MspReq{0x7a};
        inline static constexpr std::byte MspResp{0x7b};
        inline static constexpr std::byte MspWrite{0x7c};

        inline static constexpr std::byte DisplayPort{0x7d};
    }
    namespace CommandType {
        // inline static constexpr std::byte bind{0x01}; // bind
        inline static constexpr std::byte rx{0x10}; // receiver command
        inline static constexpr std::byte general{0x0a}; // general command
        inline static constexpr std::byte CC{0xa0}; // CruiseController
        inline static constexpr std::byte Module{0xa1}; // Addressable Module
    }
    namespace CrsfCommand {
        inline static constexpr std::byte SelectID{0x05};
    }
    namespace ModuleCommand {
        inline static constexpr std::byte Set{0x01};
    }
    namespace CcCommand {
        inline static constexpr std::byte SetAltData{0x01}; // Index: [0, 255], value 8bit
        inline static constexpr std::byte SetAltChunk{0x02};
        inline static constexpr std::byte SetChannel{0x03}; // Index: [0, 63], value 16bit
    }
    namespace PacketIndex {
        inline static constexpr uint8_t address = 0;
        inline static constexpr uint8_t sync = 0;
        inline static constexpr uint8_t length = 1;
        inline static constexpr uint8_t type = 2;
    }
    static inline constexpr uint8_t  maxMessageSize = 64;
    static inline constexpr uint8_t  maxPayloadSize = 60;
    static inline constexpr uint8_t  maxExtendedPayloadSize = 58;

    static inline constexpr uint8_t  ValueBits = 11;
    static inline constexpr uint16_t ValueMask = ((1 << ValueBits) - 1);

    static inline constexpr uint16_t CenterValue = 0x3e0; // same center and range as sbus

    struct Adapter {
        enum class State : uint8_t {Undefined,
                                    GotAddress, GotLength,
                                    // Link, Channels, Ping, Info, ParameterEntry, ParameterRead, ParameterWrite,
                                    Data, Command};

        static inline bool process(const std::byte b) {
            ++mBytesCounter;
            switch(mState) {
            case State::Undefined:
                csum.reset();
                if ((b == Crsf::Address::Controller) ||
                    (b == Crsf::Address::Handset) ||
                    (b == Crsf::Address::RX) ||
                    (b == Crsf::Address::TX) ||
                    (b == Crsf::Address::Broadcast))
                {
                    mAddress = b;
                    mState = State::GotAddress;
                }
                break;
            case State::GotAddress:
                if ((static_cast<uint8_t>(b) > 2) && (static_cast<uint8_t>(b) <= mData.size())) {
                    mLength = static_cast<uint8_t>(b) - 2; // only payload (not including type and crc)
                    mPayloadIndex = 0;
                    mState = State::GotLength;
                }
                else {
                    mState = State::Undefined;
                }
                break;
            case State::GotLength:
                csum += b;
                if (b == Crsf::Type::Command) {
                    mState = State::Command;
                }
                else {
                    mState = State::Data;
                }
                break;
            case State::Command:
                if (mPayloadIndex >= mLength) {
                    if (csum == b) {
                        ++mCommandPackagesCounter;
                        ++mPackagesCounter;
                        if (mData[0] == Crsf::Address::Controller) {
                            if (mData[2] == Crsf::CommandType::Module) {
                                if (mData[3] == Crsf::ModuleCommand::Set) {
                                    if (mData[4] == mModuleAddress) {
                                        // decode
                                        if (std::any(mData[5] & 0x01_B)) {
                                            led0::on();
                                        }
                                        else {
                                            led0::off();
                                        }
                                        if (std::any(mData[5] & 0x02_B)) {
                                            led1::on();
                                        }
                                        else {
                                            led1::off();
                                        }
                                        if (std::any(mData[5] & 0x04_B)) {
                                            led2::on();
                                        }
                                        else {
                                            led2::off();
                                        }
                                        if (std::any(mData[5] & 0x08_B)) {
                                            led3::on();
                                        }
                                        else {
                                            led3::off();
                                        }
                                        if (std::any(mData[5] & 0x10_B)) {
                                            led4::on();
                                        }
                                        else {
                                            led4::off();
                                        }
                                        if (std::any(mData[5] & 0x20_B)) {
                                            led5::on();
                                        }
                                        else {
                                            led5::off();
                                        }
                                        if (std::any(mData[5] & 0x40_B)) {
                                            led6::on();
                                        }
                                        else {
                                            led6::off();
                                        }
                                        if (std::any(mData[5] & 0x80_B)) {
                                            led7::on();
                                        }
                                        else {
                                            led7::off();
                                        }
                                    }
                                }
                            }
                        }
                    }
                    mState = State::Undefined;
                }
                else {
                    csum += b;
                    mData[mPayloadIndex] = b;
                    ++mPayloadIndex;
                }
            break;
            case State::Data:
                if (mPayloadIndex >= mLength) {
                    if (csum == b) {
                        ++mDataPackagesCounter;
                        ++mPackagesCounter;
                        // decode
                    }
                    mState = State::Undefined;
                }
                else {
                    csum += b;
                    ++mPayloadIndex;
                }
            break;
            }
            return true;
        }
        static inline void ratePeriodic() {}

        private:
        inline static CRC8 csum;
        inline static State mState = State::Undefined;
        inline static std::array<std::byte, maxMessageSize> mData;
        inline static std::byte mAddress{};
        inline static std::byte mModuleAddress{};
        inline static uint8_t mPayloadIndex{};
        inline static uint8_t mLength{};
        inline static uint16_t mPackagesCounter{};
        inline static uint16_t mCommandPackagesCounter{};
        inline static uint16_t mDataPackagesCounter{};
        inline static uint16_t mBytesCounter{};
    };
}

using crsf_pa = Crsf::Adapter;

using crsf = AVR::Usart<usart0Position, crsf_pa, AVR::UseInterrupts<false>, AVR::ReceiveQueueLength<0>, AVR::SendQueueLength<256>>;

using terminalDevice = crsf;
using terminal = etl::basic_ostream<terminalDevice>;

int main() {
    portmux1::init();

    ccp::unlock([]{
        clock::prescale<1>();
    });

    crsf::init<BaudRate<420000>>();
    systemTimer::init();

    led0::dir<AVR::Output>();
    led1::dir<AVR::Output>();
    led2::dir<AVR::Output>();
    led3::dir<AVR::Output>();
    led4::dir<AVR::Output>();
    led5::dir<AVR::Output>();
    led6::dir<AVR::Output>();
    led7::dir<AVR::Output>();

    while(true) {
        crsf::periodic();
        systemTimer::periodic([&]{
            crsf_pa::ratePeriodic();
        });
    }
}

#ifndef NDEBUG
/*[[noreturn]] */inline void assertOutput(const AVR::Pgm::StringView& expr [[maybe_unused]], const AVR::Pgm::StringView& file[[maybe_unused]], unsigned int line [[maybe_unused]]) noexcept {
#if !(defined(USE_IBUS) || defined(USE_HOTT))
    etl::outl<terminal>("Assertion failed: "_pgm, expr, etl::Char{','}, file, etl::Char{','}, line);
#endif
//    while(true) {
////        dbg1::toggle();
//    }
}

template<typename String1, typename String2>
[[noreturn]] inline void assertFunction(const String1& s1, const String2& s2, unsigned int l) {
    assertOutput(s1, s2, l);
}
#endif
