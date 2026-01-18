/*
 * WMuCpp - Bare Metal C++
 * Copyright (C) 2019 - 2026 Wilhelm Meier <wilhelm.wm.meier@googlemail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// use one(!) of the following options exclusively
// ATTENTION: in case of CRSF / SBUS / IBUS input is via PA1 (RX UART0) (PC1 (RX UART1) on hardware SSMSW01)
#define INPUT_CRSF // input via CRSF (ELRS only) (no rc-channel needed)
// #define INPUT_IBUS // input via IBUS (use subprotocol IBUS16 in 4in1-MPM) on channel 16
// #define INPUT_SBUS // input via SBUS (optional for ELRS, mandatory for other rc-link) on channel 16
// ATTENTION: in case of S.Port input is via PA0 (TX UART0 half-duplex)
// #define INPUT_SPORT // input via SPort (Phy-ID / App-ID see below)

#define DEFAULT_ADDRESS 0 // values: 0 ... 3 (must match value in widget)

// #define SBUS_INVERT // to receive SBus as "normal" uninverted serial data (invert SBUS means: invert an already inverted signal -> normal signal)

// use one(!) of the following options exclusively if input is via SBUS
#define USE_ELRS // SBus input only (see above)
// #define USE_AFHDS2A // SBus input only (see above) (using 4in1 MPM RF-module)
// #define USE_ACCST // SBus input only (see above) (using 4in1 MPM RF-module)

//#define DEBUG_OUTPUT

#define CRSF_BAUDRATE 420'000
#define IBUS_BAUDRATE 115'200
#define SBUS_BAUDRATE 100'000
#define SPORT_BAUDRATE 57'600

#define DEBUG_BAUDRATE 115'200 // only availabe on MCU with more than one UART since S.Port is half-duplex

// Attention: activating S.Port response and if the sensor was discovered by EdgeTx, it
// is impossible to send data from EdgeTx to the sensor: LUA sportTelemetryPush() will fail in this case.
// So, be sure to delete the sensor prior to use the lvglMultiSwitchWidget!
// #define SPORT_NORESPONSE

#define SPORT_PHY External::SPort2::SensorId::ID1 // be aware, that ID1 equals 0 in the widget setup, ID2 equals 1, and so forth ...
#define SPORT_APP 0x51 // dec: 81

// #define USE_ACTIVE_LOW_OUPUTS


