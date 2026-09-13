/*
Copyright (c) Bryan Hughes <bryan@nebri.us>

This file is part of RVL ESP32 WiFi.

RVL ESP32 WiFi is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

RVL ESP32 WiFi is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with RVL ESP32 WiFi.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef RVL_ESP32_WIFI_H_
#define RVL_ESP32_WIFI_H_

#include <rvl.hpp>

namespace RVLESP32Wifi {

// An ESP32-only transport that receives packets via an AsyncUDP callback
// running in the network task. The callback captures each packet's arrival
// time and hands {payload, arrivalTime} to the main loop through a FreeRTOS
// queue, so packetArrivalTime() is accurate regardless of how often the main
// loop runs. Not portable: relies on AsyncUDP (arduino-esp32 core) and
// FreeRTOS primitives.
class System : public rvl::System {
public:
  System(const char* newssid, const char* newpassword, uint16_t newport);
  void loop() override;

  void beginWrite(uint8_t destination) override;
  void write8(uint8_t data) override;
  void write16(uint16_t data) override;
  void write32(uint32_t data) override;
  void write(uint8_t* data, uint16_t length) override;
  void endWrite() override;

  uint16_t parsePacket() override;
  uint8_t read8() override;
  uint16_t read16() override;
  uint32_t read32() override;
  void read(uint8_t* buffer, uint16_t length) override;
  void endRead() override;
  uint32_t packetArrivalTime() override;

  uint16_t getDeviceId() override;

  uint32_t localClock() override;
  void print(const char* str) override;
  void println(const char* str) override;
};

} // namespace RVLESP32Wifi

#endif // RVL_ESP32_WIFI_H_
