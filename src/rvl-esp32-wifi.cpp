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

#include <Arduino.h>
#include <AsyncUDP.h>
#include <WiFi.h>
#include <string.h>

#include "./rvl-esp32-wifi.hpp"

#include <rvl/config.hpp>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace RVLESP32Wifi {

#define STATE_DISCONNECTED 0
#define STATE_CONNECTING 1
#define STATE_CONNECTED 2

#define RX_QUEUE_LENGTH 16

// The largest RVL packet is the wave animation packet at ~92 bytes (10 byte
// header + 82 byte payload)
#define MAX_PACKET_SIZE 128

// rvl's logger formats into a buffer 3x the length of the format string, so the
// dump is capped to keep the expanded message inside it
#define MAX_DUMP_BYTES 16

struct PacketSlot {
  uint32_t arrivalTime;
  uint16_t length;
  uint8_t data[MAX_PACKET_SIZE];
};

// Everything both protocols share, implemented once. It overrides the pure
// virtuals of whichever rvl::System interface it is given, since their read and
// write halves have the same signatures
template <class Base> class UdpEndpoint : public Base {
public:
  UdpEndpoint(uint16_t port, const char* name) : port(port), name(name) {
  }

  void init() {
    rxQueue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(PacketSlot));
  }

  bool open() {
    if (!udp.listen(port)) {
      return false;
    }
    udp.onPacket([this](AsyncUDPPacket& packet) { onPacketReceived(packet); });
    return true;
  }

  void close() {
    udp.close();
    // Discard queued packets: their arrival times predate the disconnect
    xQueueReset(rxQueue);
    currentPacketValid = false;
  }

  void logDroppedPackets() {
    if (droppedPackets != lastLoggedDroppedPackets) {
      lastLoggedDroppedPackets = droppedPackets;
      rvl::error("%s receive queue full, %d packets dropped since boot", name,
          lastLoggedDroppedPackets);
    }
  }

  void write8(uint8_t data) override {
    appendByte(data);
  }

  void write16(uint16_t data) override {
    appendByte(data >> 8);
    appendByte(data & 0xFF);
  }

  void write32(uint32_t data) override {
    appendByte(data >> 24);
    appendByte(data >> 16 & 0xFF);
    appendByte(data >> 8 & 0xFF);
    appendByte(data & 0xFF);
  }

  void write(uint8_t* data, uint16_t length) override {
    if (txLength + length > MAX_PACKET_SIZE) {
      rvl::error("Attempted to write more than %d bytes to a packet",
          MAX_PACKET_SIZE);
      return;
    }
    memcpy(txBuffer + txLength, data, length);
    txLength += length;
  }

  // Every send is a broadcast. RVL never addresses a node, and the receiving
  // side filters on the header
  void endWrite() override {
    size_t written = udp.broadcastTo(txBuffer, txLength, port);
    if (written != txLength) {
      rvl::error("Could not send %s packet", name);
    }
  }

  uint16_t parsePacket() override {
    if (xQueueReceive(rxQueue, &currentPacket, 0) != pdTRUE) {
      return 0;
    }
    currentPacketValid = true;
    readCursor = 0;
    readPastEndLogged = false;
    return currentPacket.length;
  }

  uint8_t read8() override {
    if (!currentPacketValid || readCursor >= currentPacket.length) {
      if (!readPastEndLogged) {
        readPastEndLogged = true;
        logReadPastEnd();
      }
      // Match WiFiUDP's soft-failure semantics: read() returns -1
      return 0xFF;
    }
    return currentPacket.data[readCursor++];
  }

  uint16_t read16() override {
    uint16_t val = 0;
    val |= read8() << 8;
    val |= read8();
    return val;
  }

  uint32_t read32() override {
    uint32_t val = 0;
    val |= read8() << 24;
    val |= read8() << 16;
    val |= read8() << 8;
    val |= read8();
    return val;
  }

  void read(uint8_t* buffer, uint16_t length) override {
    for (uint16_t i = 0; i < length; i++) {
      buffer[i] = read8();
    }
  }

  void endRead() override {
    currentPacketValid = false;
  }

  uint32_t packetArrivalTime() override {
    if (!currentPacketValid) {
      return UINT32_MAX;
    }
    return currentPacket.arrivalTime;
  }

protected:
  void beginPacket() {
    txLength = 0;
  }

private:
  // Runs in the network task, NOT the main loop: it must touch nothing but its
  // own locals, the queue and the drop counter
  void onPacketReceived(AsyncUDPPacket& packet) {
    PacketSlot slot;
    if (packet.length() > MAX_PACKET_SIZE) {
      return;
    }
    slot.arrivalTime = millis();
    slot.length = packet.length();
    memcpy(slot.data, packet.data(), packet.length());
    if (xQueueSend(rxQueue, &slot, 0) != pdTRUE) {
      droppedPackets++;
    }
  }

  void appendByte(uint8_t data) {
    if (txLength >= MAX_PACKET_SIZE) {
      rvl::error("Attempted to write more than %d bytes to a packet",
          MAX_PACKET_SIZE);
      return;
    }
    txBuffer[txLength++] = data;
  }

  void logReadPastEnd() {
    char bytes[MAX_DUMP_BYTES * 3 + 1];
    uint16_t dumpLength = currentPacket.length < MAX_DUMP_BYTES
        ? currentPacket.length
        : MAX_DUMP_BYTES;
    for (uint16_t i = 0; i < dumpLength; i++) {
      snprintf(bytes + i * 3, 4, "%02X ", currentPacket.data[i]);
    }
    bytes[dumpLength * 3] = '\0';
    rvl::error(
        "Read past end of %s packet: valid=%d length=%d cursor=%d bytes=%s",
        name, currentPacketValid, currentPacket.length, readCursor, bytes);
  }

  uint16_t port;
  const char* name;

  AsyncUDP udp;
  QueueHandle_t rxQueue = NULL;

  // Written only by the network task, read only by the main loop
  uint32_t droppedPackets = 0;
  uint32_t lastLoggedDroppedPackets = 0;

  PacketSlot currentPacket;
  bool currentPacketValid = false;
  uint16_t readCursor = 0;
  bool readPastEndLogged = false;

  uint8_t txBuffer[MAX_PACKET_SIZE];
  uint16_t txLength = 0;
};

class AnimationEndpoint : public UdpEndpoint<rvl::System::Animation> {
public:
  using UdpEndpoint::UdpEndpoint;

  void beginChannelWrite() override {
    beginPacket();
  }
};

class InfrastructureEndpoint
    : public UdpEndpoint<rvl::System::Infrastructure> {
public:
  using UdpEndpoint::UdpEndpoint;

  void beginBroadcastWrite() override {
    beginPacket();
  }

  // The coordinator is the access point, so a broadcast reaches it with the
  // same MAC-layer retries a unicast would get
  void beginCoordinatorWrite() override {
    beginPacket();
  }
};

AnimationEndpoint animationEndpoint(RVLA_PORT, "Animation");
InfrastructureEndpoint infrastructureEndpoint(RVLI_PORT, "Infrastructure");

uint8_t state = STATE_DISCONNECTED;
bool socketErrorLogged = false;

const char* ssid;
const char* password;

System::System(const char* newssid, const char* newpassword) {
  ssid = newssid;
  password = newpassword;
  animationEndpoint.init();
  infrastructureEndpoint.init();
}

void System::loop() {
  switch (state) {
  case STATE_DISCONNECTED:
    rvl::info("Connecting to %s", ssid);
    WiFi.begin(ssid, password);
    state = STATE_CONNECTING;
    socketErrorLogged = false;
    rvl::setLinkUpState(false);
    // Fall through here instead of breaking
  case STATE_CONNECTING:
    if (WiFi.status() != WL_CONNECTED) {
      break;
    }
    // Stay here and retry next loop if either socket fails to open. Reporting
    // the link up without both would leave the node unable to talk
    if (!animationEndpoint.open() || !infrastructureEndpoint.open()) {
      animationEndpoint.close();
      infrastructureEndpoint.close();
      if (!socketErrorLogged) {
        socketErrorLogged = true;
        rvl::error("Could not open the RVL sockets, retrying");
      }
      break;
    }
    rvl::info("Connected to WiFi with address %d.%d.%d.%d", WiFi.localIP()[0],
        WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
    state = STATE_CONNECTED;
    rvl::setLinkUpState(true);
    break;
  case STATE_CONNECTED:
    if (WiFi.status() != WL_CONNECTED) {
      rvl::info("Disconnected from WiFi, retrying");
      state = STATE_DISCONNECTED;
      animationEndpoint.close();
      infrastructureEndpoint.close();
      rvl::setLinkUpState(false);
      break;
    }
    animationEndpoint.logDroppedPackets();
    infrastructureEndpoint.logDroppedPackets();
    break;
  }
}

rvl::System::Animation& System::animation() {
  return animationEndpoint;
}

rvl::System::Infrastructure& System::infrastructure() {
  return infrastructureEndpoint;
}

bool System::isLinkUp() {
  return WiFi.status() == WL_CONNECTED;
}

uint32_t System::localClock() {
  return millis();
}

uint32_t System::random() {
  return esp_random();
}

void System::print(const char* str) {
  Serial.print(str);
}

void System::println(const char* str) {
  Serial.println(str);
}

} // namespace RVLESP32Wifi
