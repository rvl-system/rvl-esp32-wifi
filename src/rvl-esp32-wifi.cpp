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

struct PacketSlot {
  uint32_t arrivalTime;
  uint16_t length;
  uint8_t data[MAX_PACKET_SIZE];
};

uint8_t state = STATE_DISCONNECTED;

AsyncUDP udp; // NOLINT

const char* ssid;
const char* password;
uint16_t port;

QueueHandle_t rxQueue = NULL;

// Written only by the network task, read only by the main loop
uint32_t droppedPackets = 0;
uint32_t lastLoggedDroppedPackets = 0;

PacketSlot currentPacket;
bool currentPacketValid = false;
uint16_t readCursor = 0;
bool readPastEndLogged = false;

// rvl's logger formats into a buffer 3x the length of the format string, so the
// dump is capped to keep the expanded message inside it
#define MAX_DUMP_BYTES 16

void logReadPastEnd() {
  char bytes[MAX_DUMP_BYTES * 3 + 1];
  uint16_t dumpLength = currentPacket.length < MAX_DUMP_BYTES
      ? currentPacket.length
      : MAX_DUMP_BYTES;
  for (uint16_t i = 0; i < dumpLength; i++) {
    snprintf(bytes + i * 3, 4, "%02X ", currentPacket.data[i]);
  }
  bytes[dumpLength * 3] = '\0';
  rvl::error("Read past end of packet: valid=%d length=%d cursor=%d bytes=%s",
      currentPacketValid, currentPacket.length, readCursor, bytes);
}

uint8_t txBuffer[MAX_PACKET_SIZE];
uint16_t txLength = 0;
uint8_t txDestination = 255;

// Runs in the network task, NOT the main loop: it must touch nothing but its
// own locals and the queue
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

System::System(const char* newssid, const char* newpassword, uint16_t newport) {
  ssid = newssid;
  password = newpassword;
  port = newport;
  rxQueue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(PacketSlot));
}

void System::loop() {
  switch (state) {
  case STATE_DISCONNECTED:
    rvl::info("Connecting to %s", ssid);
    WiFi.begin(ssid, password);
    state = STATE_CONNECTING;
    this->setConnectedState(false);
    // Fall through here instead of breaking
  case STATE_CONNECTING:
    if (WiFi.status() == WL_CONNECTED) {
      rvl::info("Connected to WiFi with address %d.%d.%d.%d", WiFi.localIP()[0],
          WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
      if (udp.listen(port)) {
        udp.onPacket(onPacketReceived);
      } else {
        rvl::error("Could not listen on port %d", port);
      }
      state = STATE_CONNECTED;
      this->setConnectedState(true);
    }
    break;
  case STATE_CONNECTED:
    if (WiFi.status() != WL_CONNECTED) {
      rvl::info("Disconnected from WiFi, retrying");
      state = STATE_DISCONNECTED;
      udp.close();
      // Discard queued packets: their arrival times predate the disconnect
      xQueueReset(rxQueue);
      currentPacketValid = false;
      this->setConnectedState(false);
      break;
    }
    if (droppedPackets != lastLoggedDroppedPackets) {
      lastLoggedDroppedPackets = droppedPackets;
      rvl::error("Receive queue full, %d packets dropped since boot",
          lastLoggedDroppedPackets);
    }
    break;
  }
}

// Destination: 1 byte
// 0-239: individual device
// 240-254: multicast
// 255: broadcast
void System::beginWrite(uint8_t destination) {
  txDestination = destination;
  txLength = 0;
}

void appendByte(uint8_t data) {
  if (txLength >= MAX_PACKET_SIZE) {
    rvl::error("Attempted to write more than %d bytes to a packet",
        MAX_PACKET_SIZE);
    return;
  }
  txBuffer[txLength++] = data;
}

void System::write8(uint8_t data) {
  appendByte(data);
}

void System::write16(uint16_t data) {
  appendByte(data >> 8);
  appendByte(data & 0xFF);
}

void System::write32(uint32_t data) {
  appendByte(data >> 24);
  appendByte(data >> 16 & 0xFF);
  appendByte(data >> 8 & 0xFF);
  appendByte(data & 0xFF);
}

void System::write(uint8_t* data, uint16_t length) {
  if (txLength + length > MAX_PACKET_SIZE) {
    rvl::error("Attempted to write more than %d bytes to a packet",
        MAX_PACKET_SIZE);
    return;
  }
  memcpy(txBuffer + txLength, data, length);
  txLength += length;
}

void System::endWrite() {
  size_t written;
  // We don't have real multicast, so we fall back to broadcast
  if (txDestination >= 240) {
    written = udp.broadcastTo(txBuffer, txLength, port);
  } else {
    IPAddress ip(WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2],
        txDestination);
    written = udp.writeTo(txBuffer, txLength, ip, port);
  }
  if (written != txLength) {
    rvl::error("Could not send packet to destination %d", txDestination);
  }
}

uint16_t System::parsePacket() {
  if (xQueueReceive(rxQueue, &currentPacket, 0) != pdTRUE) {
    return 0;
  }
  currentPacketValid = true;
  readCursor = 0;
  readPastEndLogged = false;
  return currentPacket.length;
}

uint8_t System::read8() {
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

uint16_t System::read16() {
  uint16_t val = 0;
  val |= read8() << 8;
  val |= read8();
  return val;
}

uint32_t System::read32() {
  uint32_t val = 0;
  val |= read8() << 24;
  val |= read8() << 16;
  val |= read8() << 8;
  val |= read8();
  return val;
}

void System::read(uint8_t* buffer, uint16_t length) {
  for (uint16_t i = 0; i < length; i++) {
    buffer[i] = read8();
  }
}

void System::endRead() {
  currentPacketValid = false;
}

uint32_t System::packetArrivalTime() {
  if (!currentPacketValid) {
    return UINT32_MAX;
  }
  return currentPacket.arrivalTime;
}

uint16_t System::getDeviceId() {
  return WiFi.localIP()[3];
}

uint32_t System::localClock() {
  return millis();
}

void System::print(const char* str) {
  Serial.print(str);
}

void System::println(const char* str) {
  Serial.println(str);
}

} // namespace RVLESP32Wifi
