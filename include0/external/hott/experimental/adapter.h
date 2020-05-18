#pragma once

#include "../hott.h"

namespace Hott {
    namespace Experimental {
        
        template<typename BinaryMsgType>
        struct Adapter;

        template<>
        struct Adapter<Hott::GamMsg> {
            Adapter(Hott::GamMsg& data) : mData(data) {}

            inline void state(uint8_t v) {
                mData.fuel_ml = v;
            }
            
            inline void cellVoltageRaw(uint_ranged<uint8_t, 0, Hott::GamMsg::numberOfCells> cell, uint8_t v) {
                mData.cell[cell] = v;
            } 
            
            inline void voltage(const Hott::Units::battery_voltage_t& bv) {
                mData.Battery1 = bv.value;
                mData.main_voltage = bv.value;
            }
            inline void battery1Raw(uint16_t v) {
                mData.Battery1 = v;
            }
            inline void battery2Raw(uint16_t v) {
                mData.Battery2 = v;
            }
            
//            enum class Battery : uint8_t {One, Two};
            
//            inline void batteryVoltageRaw(Battery battery, uint16_t v) {
//                if (battery == Battery::One) {
//                    mData.Battery1 = v;
//                }
//                else {
//                    mData.Battery2 = v;
//                }
//            }
            // hijacked
            inline void voltageMin(const Hott::Units::battery_voltage_t& bv) {
                mData.Battery2 = bv.value;
            }
            inline void batteryMinimumRaw(uint8_t cell, uint16_t c) {
                mData.min_cell_volt_num = cell;
                mData.min_cell_volt = c;
            }
            inline void current(const Hott::Units::current_t& c) {
                mData.current = c.value;
            }
            inline void currentMax(const Hott::Units::current_t&) {
            }
            inline void currentRaw(uint16_t v) {
                mData.current = v;
            }
            inline void mainVoltageRaw(uint16_t v) {
                mData.main_voltage = v;
            }
            inline void speedRaw(uint16_t v) {
                mData.speed = v;
            }
            inline void rpm1(const RPM& v) {
                if (v) {
                    mData.rpm = v.value() / 10;
                }
            }
            inline void rpm2(const RPM& v) {
                if (v) {
                    mData.rpm2 = v.value() / 10;
                }
            }
            inline void temp(const External::Units::celsius<uint16_t, std::ratio<1,1>>& v) {
                mData.temperature1 = v.value + 20;
            }
            inline void tempMax(const External::Units::celsius<uint16_t, std::ratio<1,1>>& v) {
                mData.temperature2 = v.value + 20;
            }
            inline void temp1(FixedPoint<int, 4> v) {
                mData.temperature1 = v.integer() + 20;
            }
            inline void temp2(FixedPoint<int, 4> v) {
                mData.temperature2 = v.integer() + 20;
            }
            inline void capRaw(uint16_t v) {
                mData.batt_cap = v;
            }
        private:
            Hott::GamMsg& mData;
        };
        
        template<>
        struct Adapter<Hott::EscMsg> {
            Adapter(Hott::EscMsg& data) : mData(data) {}
            inline void rpmRaw(uint16_t v) {
                mData.rpm= v;
            }
            inline void rpmMaxRaw(uint16_t v) {
                mData.rpm_max = v;
            }
            inline void voltageRaw(uint16_t v) {
                mData.voltage = v;
            }
            inline void voltage(const Hott::Units::battery_voltage_t& bv) {
                mData.bec = 51;
                mData.voltage = bv.value;
            }
            inline void voltageMinRaw(uint16_t v) {
                mData.voltage_min = v;
            }
            inline void voltageMin(const Hott::Units::battery_voltage_t& bv) {
                mData.voltage_min = bv.value;
            }
            inline void currentRaw(uint16_t v) {
                mData.current = v;
            }
            inline void current(const Hott::Units::current_t& c) {
                mData.current = c.value;
            }
            inline void currentMaxRaw(uint16_t v) {
                mData.current_max = v;
            }
            inline void currentMax(const Hott::Units::current_t& c) {
                mData.current_max = c.value;
            }
            inline void tempRaw(uint8_t v) {
                mData.temp = v;
            }
            inline void tempMaxRaw(uint8_t v) {
                mData.temp_max = v;
            }
            inline void temp(const External::Units::celsius<uint16_t, std::ratio<1,1>>& v) {
                mData.temp = v.value + 20;
            }
            inline void tempMax(const External::Units::celsius<uint16_t, std::ratio<1,1>>& v) {
                mData.temp_max = v.value + 20;
            }
            inline void capRaw(uint16_t v) {
                mData.capacity = v;
            }
        private:
            Hott::EscMsg& mData;
        };        
    }
}
