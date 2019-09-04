#pragma once

#include "../hott.h"

namespace Hott {
    namespace Experimental {
        
        template<typename BinaryMsgType>
        struct Adapter;

        template<>
        struct Adapter<Hott::GamMsg> {
            Adapter(Hott::GamMsg& data) : mData(data) {}

            inline void cellVoltageRaw(uint_ranged<uint8_t, 0, Hott::GamMsg::numberOfCells> cell, uint8_t v) {
                mData.cell[cell] = v;
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
            inline void batteryMinimumRaw(uint8_t cell, uint16_t c) {
                mData.min_cell_volt_num = cell;
                mData.min_cell_volt = c;
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
            inline void temp1(FixedPoint<int, 4> v) {
                mData.temperature1 = v.integer() + 20;
            }
            inline void temp2(FixedPoint<int, 4> v) {
                mData.temperature2 = v.integer() + 20;
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
                mData.voltage = bv.value;
            }
            inline void voltageMinRaw(uint16_t v) {
                mData.voltage_min = v;
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
            inline void tempRaw(uint16_t v) {
                mData.temp = v;
            }
            inline void tempMaxRaw(uint16_t v) {
                mData.temp_max = v;
            }
        private:
            Hott::EscMsg& mData;
        };        
    }
}
