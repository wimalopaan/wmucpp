#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cassert>
#include <array>
#include <vector>
#include <iostream>
#include <iomanip>
#include <optional>

#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <errno.h>   /* ERROR Number Definitions           */
#include <sys/select.h>

class Serial {
    typedef struct termios port_settings_t;
public:
    explicit Serial(const std::string& name) {
        //        mFd = open(name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY); 
        mFd = open(name.c_str(), O_RDWR | O_NOCTTY); 
        if (mFd == -1) {
            return;
        }
        port_settings_t portSettings = {};
        
//        cfsetispeed(&portSettings,B19200); 
//        cfsetospeed(&portSettings,B19200); 

        cfsetispeed(&portSettings,B115200); 
        cfsetospeed(&portSettings,B115200); 
        
        portSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
        portSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
        portSettings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
        portSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */
        
        portSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
        portSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */ 
        
        portSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
        portSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */
        
        portSettings.c_oflag &= ~OPOST; /*No Output Processing*/
        
        if((tcsetattr(mFd, TCSANOW, &portSettings)) < 0) {
            perror("tcsetattr");
            if (close(mFd) < 0) {
                perror("close");
            }
            mFd = -1;
        }
    }
    template<auto N>
    ssize_t send(const std::array<uint8_t, N>& a) const {
        ssize_t bytes_written = -1;
        if ((bytes_written = write(mFd, &a[0], N)) < 0) {
            perror("write");
        }
        else {
            printf("written: %ld, \n", bytes_written);
        }
        return bytes_written;
    }
    template<auto N>
    ssize_t sendWithDelay(const std::array<uint8_t, N>& a, useconds_t usecs) const {
        for(size_t n = 0; n < N; ++n) {
            if (write(mFd, &a[n], 1) < 0) {
                perror("write");
                return -1;
            }
            if (usleep(usecs) < 0) {
                perror("usleep");
                return -1;
            }
        }
        return N;
    }
    template<auto N>
    ssize_t read(std::array<uint8_t, N>& a) const {
        ssize_t bytes_read = -1;
        bool ready = true;
        while (ready) {
            if ((bytes_read = ::read(mFd, &a[0], N)) < 0) {
                perror("read");
                ready = false;
            }
            else {
                printf("read: %ld, \n", bytes_read);
            }
        }
        return bytes_read;
    }
    std::optional<uint8_t> read() const {
        uint8_t data = 0;
        ssize_t bytes_read = ::read(mFd, &data, 1);
        if (bytes_read > 0) {
            return data;
        }
        return {};
    }
    std::optional<uint8_t> readWithTimeout() const {
        fd_set readSet = {};
        
        while(true) {
            struct timeval timeout = {};
            timeout.tv_sec = 0;
            timeout.tv_usec = 1000;
            FD_ZERO(&readSet);
            FD_SET(mFd, &readSet);
            int ret = select(mFd + 1, &readSet, NULL, NULL, &timeout);
            if (ret > 0) {
                if (FD_ISSET(mFd, &readSet)) {
//                    std::cout << "x1\n";
                    return read();
                }
            }
            else if (ret < 0) {
                if (errno != EINTR) {
                    perror("select");
                    return {};
                }
            }
            else {
                return {};
            }
        }
    }
    explicit operator bool() const {
        return mFd != -1;
    }
    ~Serial() {
        if (close(mFd) < 0) {
            perror("close");
        }
        mFd = -1;
    }
private:
    int mFd = -1;
};


struct CRC16 {
    static constexpr uint16_t calculate(const uint8_t* buf, size_t len) {
        size_t index = 0;
        uint16_t uCrc = 0;
        
        for(size_t ix=0; ix<len; ix++) {
            index = (((uCrc >> 8) & 0xff) ^ buf[ix]) & 0x00FF;	
            uCrc = ((uCrc & 0xff) * 256) ^ crc16_table[index];
        }
        return uCrc;
    }
    inline static constexpr std::array<uint16_t, 256> crc16_table = {
        0x0000,0x1021,0x2042,0x3063,0x4084,0x50A5,0x60C6,0x70E7,
        0x8108,0x9129,0xA14A,0xB16B,0xC18C,0xD1AD,0xE1CE,0xF1EF,
        0x1231,0x0210,0x3273,0x2252,0x52B5,0x4294,0x72F7,0x62D6,
        0x9339,0x8318,0xB37B,0xA35A,0xD3BD,0xC39C,0xF3FF,0xE3DE,
        0x2462,0x3443,0x0420,0x1401,0x64E6,0x74C7,0x44A4,0x5485,
        0xA56A,0xB54B,0x8528,0x9509,0xE5EE,0xF5CF,0xC5AC,0xD58D,
        0x3653,0x2672,0x1611,0x0630,0x76D7,0x66F6,0x5695,0x46B4,
        0xB75B,0xA77A,0x9719,0x8738,0xF7DF,0xE7FE,0xD79D,0xC7BC,
        0x48C4,0x58E5,0x6886,0x78A7,0x0840,0x1861,0x2802,0x3823,
        0xC9CC,0xD9ED,0xE98E,0xF9AF,0x8948,0x9969,0xA90A,0xB92B,
        0x5AF5,0x4AD4,0x7AB7,0x6A96,0x1A71,0x0A50,0x3A33,0x2A12,
        0xDBFD,0xCBDC,0xFBBF,0xEB9E,0x9B79,0x8B58,0xBB3B,0xAB1A,
        0x6CA6,0x7C87,0x4CE4,0x5CC5,0x2C22,0x3C03,0x0C60,0x1C41,
        0xEDAE,0xFD8F,0xCDEC,0xDDCD,0xAD2A,0xBD0B,0x8D68,0x9D49,
        0x7E97,0x6EB6,0x5ED5,0x4EF4,0x3E13,0x2E32,0x1E51,0x0E70,
        0xFF9F,0xEFBE,0xDFDD,0xCFFC,0xBF1B,0xAF3A,0x9F59,0x8F78,
        0x9188,0x81A9,0xB1CA,0xA1EB,0xD10C,0xC12D,0xF14E,0xE16F,
        0x1080,0x00A1,0x30C2,0x20E3,0x5004,0x4025,0x7046,0x6067,
        0x83B9,0x9398,0xA3FB,0xB3DA,0xC33D,0xD31C,0xE37F,0xF35E,
        0x02B1,0x1290,0x22F3,0x32D2,0x4235,0x5214,0x6277,0x7256,
        0xB5EA,0xA5CB,0x95A8,0x8589,0xF56E,0xE54F,0xD52C,0xC50D,
        0x34E2,0x24C3,0x14A0,0x0481,0x7466,0x6447,0x5424,0x4405,
        0xA7DB,0xB7FA,0x8799,0x97B8,0xE75F,0xF77E,0xC71D,0xD73C,
        0x26D3,0x36F2,0x0691,0x16B0,0x6657,0x7676,0x4615,0x5634,
        0xD94C,0xC96D,0xF90E,0xE92F,0x99C8,0x89E9,0xB98A,0xA9AB,
        0x5844,0x4865,0x7806,0x6827,0x18C0,0x08E1,0x3882,0x28A3,
        0xCB7D,0xDB5C,0xEB3F,0xFB1E,0x8BF9,0x9BD8,0xABBB,0xBB9A,
        0x4A75,0x5A54,0x6A37,0x7A16,0x0AF1,0x1AD0,0x2AB3,0x3A92,
        0xFD2E,0xED0F,0xDD6C,0xCD4D,0xBDAA,0xAD8B,0x9DE8,0x8DC9,
        0x7C26,0x6C07,0x5C64,0x4C45,0x3CA2,0x2C83,0x1CE0,0x0CC1,
        0xEF1F,0xFF3E,0xCF5D,0xDF7C,0xAF9B,0xBFBA,0x8FD9,0x9FF8,
        0x6E17,0x7E36,0x4E55,0x5E74,0x2E93,0x3EB2,0x0ED1,0x1EF0
    };
};

namespace Hott {
    namespace Radio {
        struct Parameter {
            static constexpr uint8_t Seq1 = 1;
            static constexpr uint8_t Seq2 = 2;
            static constexpr uint8_t nobLIdx = 3;
            static constexpr uint8_t crcOffset = nobLIdx;
            static constexpr uint8_t nobHIdx = 4;
            static constexpr uint8_t tcmdIdx = 5;
            static constexpr uint8_t pcmdIdx = 6;
        };
        enum class Cmd : uint8_t {DBM = 0x33, Recv = 0x34, GAM = 0x35, El = 0x36, Vario = 0x37, GPS = 0x38, AIR = 0x39, Test1 = 0x3a,
                                  Servo = 0x40, Pupil = 0x41, Ctrl1 = 0x42, Ctrl2 = 0x43, Text = 0x44, Model = 0x45, 
                                  FastMsg = 0x52, FastStart = 0x53, FastStop = 0x54};        
        enum class Ret : uint8_t {Ack = 0x01, Nack = 0x02, Error = 0x03, CRCError = 0x04, Busy = 0x05};
        namespace RX {
            class Message {
                enum class State : uint8_t {Undefined, Seq1, Seq2, N1, N2, TC, PC, Data, CRC1, CRC2};
                State mState = State::Undefined;
            public:
                const auto& data() const {
                    return mData;
                }
                bool receive(Serial& port) {
                    assert(port);
                    
                    size_t numberOfTimeouts = 0;
                    bool needMoreData = true;
                    uint16_t length = 0;
                    uint16_t index = 0;
//                    std::cout << "a\n";
                    while (needMoreData) {
//                        std::cout << "b\n";
                        auto d = port.readWithTimeout();
//                        std::cout << "c\n";
                        if (d) {
                            switch (mState) {
                            case State::Undefined:
                                if (*d == 0x00) {
                                    mState = State::Seq1;
                                }
                                break;
                            case State::Seq1:
                                mData[Parameter::Seq1] = *d;
                                mState = State::Seq2;
                                break;
                            case State::Seq2:
                                mData[Parameter::Seq2] = *d;
                                if (mData[Parameter::Seq1] == (mData[Parameter::Seq2] ^ 0xff)) {
                                    mState = State::N1;
                                }
                                else {
                                    mState = State::Undefined;
                                }
                                break;
                            case State::N1:
                                mData[Parameter::nobLIdx] = *d;
                                mState = State::N2;
                                break;
                            case State::N2:
                                mData[Parameter::nobHIdx] = *d;
                                length = (mData[Parameter::nobHIdx] << 8) + mData[Parameter::nobLIdx];
                                mState = State::TC;
                                break;
                            case State::TC:
                                mData[Parameter::tcmdIdx] = *d;
                                if (*d == 0x04) {
                                    mState = State::PC;
                                }
                                else {
                                    mState = State::Undefined;
                                }
                                break;
                            case State::PC:
                                mData[Parameter::pcmdIdx] = *d;
                                if (length > 0) {
                                    mState = State::Data;
                                    index = 0;
                                }
                                else {
                                    mState = State::CRC1;
                                }
                                break;
                            case State::Data:
                                mData[index + Parameter::pcmdIdx + 1] = *d;
                                ++index;
                                --length;
                                if (length <= 0) {
                                    mState = State::CRC1;
                                }
                                break;
                            case State::CRC1:
                                mData[index + Parameter::pcmdIdx + 1] = *d;
                                mState = State::CRC2;
                                break;
                            case State::CRC2:
                                mData[index + Parameter::pcmdIdx + 1] = *d;
                                needMoreData = false;
                                break;
                            }
                        }
                        else {
                            ++numberOfTimeouts;
                        }
                        if (numberOfTimeouts > 100) {
                            std::cerr << "timeout" << '\n';
                            return false;
                        }
                    }
                    if (!needMoreData) {
                        return true;
                    }
                    return false;
                }
                uint8_t sequenceNumber() const {
                    return mData[Parameter::Seq1];
                }
                uint16_t size() const {
                    return (mData[Parameter::nobHIdx] << 8) + mData[Parameter::nobLIdx];
                }
                bool ack() const {
                    return mData[Parameter::pcmdIdx] == 0x01;
                }
                Hott::Radio::Ret returnCode() const {
                    return Hott::Radio::Ret{mData[Parameter::pcmdIdx]};
                }
            private:
                std::array<uint8_t, 512> mData = {};
            };
            std::ostream& operator<<(std::ostream& o, const Message& m) {
                return o << "RX[Seq:" << std::to_string(m.sequenceNumber()) << ","
                         << std::setbase(16) 
                         << "TC:" << (int)m.data()[Parameter::tcmdIdx] << ","
                         << "PC:" << (int)m.data()[Parameter::pcmdIdx] << ","
                         << std::setbase(10) 
                         << "N:" << m.size() 
                         << "]";
            }
        }
        namespace TX {
            class MessageBase {
            public:
                static uint8_t generateSequenceNumber() {
                    sSequenceNumber = (sSequenceNumber + 1) % (maxSeqNumber + 1);
                    return sSequenceNumber;
                }
            protected:
                inline static uint8_t sSequenceNumber = 0;
                static constexpr uint8_t maxSeqNumber = 254;
            };
            template<size_t PayloadLength>
            class Message {
            public:
                uint8_t sequenceNumber() const {
                    return mData[Parameter::Seq1];
                }
                Message() {
                    mData[0] = 0x00;
                    mData[Hott::Radio::Parameter::Seq1] = MessageBase::generateSequenceNumber();
                    mData[Hott::Radio::Parameter::Seq2] = mData[Hott::Radio::Parameter::Seq1] ^ 0xff;
                    mData[Hott::Radio::Parameter::nobLIdx] = payloadLength & 0xff;
                    mData[Hott::Radio::Parameter::nobHIdx] = (payloadLength >> 8) & 0xff;
                    mData[Hott::Radio::Parameter::tcmdIdx] = 0x04;    
                    mData[Hott::Radio::Parameter::pcmdIdx] = static_cast<uint8_t>(Hott::Radio::Cmd::FastMsg);    
                }
                const auto& data() const {
                    return mData;
                }
                uint16_t size() const {
                    return (mData[Parameter::nobHIdx] << 8) + mData[Parameter::nobLIdx];
                }
                bool transmit(Serial& port) {
                    uint16_t crc = CRC16::calculate(&mData[Hott::Radio::Parameter::crcOffset], length - Hott::Radio::Parameter::crcOffset - crclength);
                    mData[length - 2] = crc & 0xff;
                    mData[length - 1] = (crc >> 8) & 0xff;
                    
                    std::cout << *this << '\n';
                    
                    auto l = port.sendWithDelay(mData, 3000);
                    if (l < 0) {
                        return false;
                    }
                    typename data_type::size_type ul = l;
                    return ul == mData.size();
                }
            protected:
                static constexpr uint8_t prefixlength = 7;
                static constexpr uint8_t crclength = 2;
                static constexpr uint16_t payloadLength = PayloadLength;
                static constexpr uint8_t length = prefixlength + payloadLength + crclength;
                std::array<uint8_t, length> mData = {};
                typedef std::array<uint8_t, length> data_type;
            private:
            };
            template<size_t PayloadLength>
            std::ostream& operator<<(std::ostream& o, const Message<PayloadLength>& m) {
                o << "TX[Seq:" << std::to_string(m.sequenceNumber()) << ","
                  << std::setbase(16) 
                  << "TC:" << (int)m.data()[Parameter::tcmdIdx] << ","
                  << "PC:" << (int)m.data()[Parameter::pcmdIdx] << ","
                  << std::setbase(10) 
                  << "N:" << m.size() 
                  << "]";
                o << std::setbase(16);
                o << "{";
                for(size_t i = 0; i < m.size(); ++i) {
                    o << (int)m.data()[i + Parameter::pcmdIdx + 1] << ",";
                }
                o << std::setbase(10);
                o << "}";
                return o;
            }
            
            
            class Fast : public Message<41> {
            public:  
                Fast() {
                    mData[versionIdx] = 0x01;
                    mData[channelIdx] = 0x10;
                }
                void channel(uint8_t number, uint16_t value) {
                    assert(number <= 15);
                    uint8_t index = 2 * number + channelsOffset;
                    mData[index] = (value >> 8) & 0xff;
                    mData[index + 1] = value & 0xff;
                }
                void sw(uint8_t) {
                }
            private:
                inline static constexpr uint8_t versionIdx = Parameter::pcmdIdx + 1;
                inline static constexpr uint8_t channelIdx = versionIdx + 1;
                inline static constexpr uint8_t channelsOffset = channelIdx + 1;
            };
            
            class SimpleCommand : public Message<0> {
            public:
                explicit SimpleCommand(Cmd cmd) {
                    mData[Hott::Radio::Parameter::pcmdIdx] = static_cast<uint8_t>(cmd);    
                }
            private:
            };
        }
        
        class Radio {
        public:
            explicit Radio(const std::string& portName) : mPort(portName) {
            }
            bool send(Hott::Radio::Cmd command) {
                if (!mPort) {
                    return false;
                }
                if (command == Hott::Radio::Cmd::FastMsg) {
                    Hott::Radio::TX::Fast cmd;
                    cmd.transmit(mPort);
                }
                else {
                    Hott::Radio::TX::SimpleCommand cmd(command);
                    cmd.transmit(mPort);
                }
                return true;
            }
            Hott::Radio::RX::Message receive() {
                Hott::Radio::RX::Message message;
                if (mPort) {
                    if (message.receive(mPort)) {
                        return message;
                    }              
                }
                return {};
            }
            explicit operator bool() const {
                return !!mPort;
            }
        private:
            Serial mPort;        
        };
    }
}

int main() {
    Hott::Radio::Radio radio("/dev/ttyUSB0");
    
    if (!radio) {
        std::cerr << "Can't open radio" << '\n';
        exit(EXIT_FAILURE);
    }
    {
        radio.send(Hott::Radio::Cmd::FastStart);
        auto message = radio.receive();
        std::cout << message << '\n';        
    }
    sleep(1);
//    {
        
//        radio.send(Hott::Radio::Cmd::FastStart);
//        auto message = radio.receive();
//        std::cout << message << '\n';        
//    }
    
    //    for(size_t n = 0; n < 5; ++n) {
    //        sleep(1);
    //        radio.send(Hott::Radio::Cmd::FastMsg);
    //        Hott::Radio::RX::Message message = radio.receive();
    //        std::cout << message << '\n';        
    //    }
}

