#include <cstdint>
#include <array>
#include <tuple>
#include <iostream>

struct DevA;
struct DevB;
struct DevC;

template<typename Device, typename ValueType, auto Size>
struct Sensor {
    explicit Sensor(const std::string& name) : mName{name} {}
    template<typename Stream>
    Stream& out(Stream& stream) const {
        return stream << mName << ' ' << std::size(mData) << '\n'; // zu erweitern ...
    }
    const ValueType& operator[](size_t index) const {
        return mData[index];
    }
private:
    const std::string mName;
    std::array<ValueType, Size> mData;
};

std::tuple sensors{
    Sensor<DevA, uint8_t, 10>{"A"}, 
    Sensor<DevB, uint32_t, 100>{"B"},
    Sensor<DevC, double, 5>{"C"}
};

int main() {
    std::apply([](const auto&... sensor){
        (sensor.out(std::cout), ...);        
    }, sensors);
    
    std::get<0>(sensors)[0];
}
