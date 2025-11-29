#pragma once
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <cstdlib>

#ifndef BLE_CMD_SERVICE_UUID
#define BLE_CMD_SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#endif

#ifndef BLE_CMD_CHAR_UUID
#define BLE_CMD_CHAR_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#endif

class BleCommands {
public:
  typedef void (*Handler)(int code);
  typedef void (*VectorHandler)(float x, float y);

  BleCommands() : _handler(nullptr), _vectorHandler(nullptr), _char(nullptr) {}

  void begin(const char* deviceName = "ESP32_Throughput_Test_Server") {
    BLEDevice::init(deviceName);

    _server  = BLEDevice::createServer();
    _serverCallbacks._parent = this;
    _server->setCallbacks(&_serverCallbacks);
    _service = _server->createService(BLE_CMD_SERVICE_UUID);

    _char = _service->createCharacteristic(
      BLE_CMD_CHAR_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
    );

    _char->addDescriptor(new BLE2902()); // Enable notifications

    _char->setCallbacks(&_cb);
    _cb._parent = this;

    _service->start();
    _advertising = BLEDevice::getAdvertising();
    _advertising->addServiceUUID(BLE_CMD_SERVICE_UUID);
    _advertising->setScanResponse(true);
    _advertising->setMinPreferred(0x06);
    _advertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
  }

  void poll() {
    restartAdvertisingIfNeeded();

    while (_qHead != _qTail) {
      CommandEvent evt = _queue[_qTail];
      _qTail = (uint8_t)((_qTail + 1) % QSIZE);
      if (evt.hasVector) {
        Serial.printf("[BLE] Dispatch joystick vector x=%.3f y=%.3f\n", evt.x, evt.y);
        if (_vectorHandler) {
          _vectorHandler(evt.x, evt.y);
        }
      } else {
        Serial.printf("[BLE] Dispatch command code=%u\n", evt.code);
        if (_handler) _handler((int)evt.code);
      }
    }
  }

  void onCommand(Handler h) { _handler = h; }
  void onVector(VectorHandler h) { _vectorHandler = h; }

  // Send notification to client
  void notify(const char* msg) {
    if (_char) {
      _char->setValue(msg);
      _char->notify();
    }
  }

private:
  struct CommandEvent {
    uint8_t code = 0;
    float x = 0.0f;
    float y = 0.0f;
    bool hasVector = false;
  };

  static constexpr uint8_t QSIZE = 16;
  CommandEvent _queue[QSIZE];
  volatile uint8_t _qHead = 0;
  volatile uint8_t _qTail = 0;

  BLEServer*       _server  = nullptr;
  BLEService*      _service = nullptr;
  BLECharacteristic* _char  = nullptr;
  BLEAdvertising*  _advertising = nullptr;

  Handler _handler;
  VectorHandler _vectorHandler;
  bool _clientConnected = false;
  volatile bool _restartAdvertising = false;

  void enqueueCommand(uint8_t v) {
    uint8_t next = (uint8_t)((_qHead + 1) % QSIZE);
    if (next != _qTail) {
      _queue[_qHead].code = v;
      _queue[_qHead].hasVector = false;
      _qHead = next;
    }
  }

  void enqueueVector(float x, float y) {
    uint8_t next = (uint8_t)((_qHead + 1) % QSIZE);
    if (next != _qTail) {
      _queue[_qHead].code = 0;
      _queue[_qHead].x = x;
      _queue[_qHead].y = y;
      _queue[_qHead].hasVector = true;
      _qHead = next;
    }
  }

  static const char* skipDelimiters(const char* ptr) {
    while (*ptr == ' ' || *ptr == ',' || *ptr == ':' || *ptr == ';' || *ptr == '\t') {
      ++ptr;
    }
    return ptr;
  }

  void handleConnect() {
    _clientConnected = true;
    Serial.println("[BLE] Central connected");
  }

  void handleDisconnect() {
    _clientConnected = false;
    Serial.println("[BLE] Central disconnected");
    _restartAdvertising = true;
  }

  void restartAdvertisingIfNeeded() {
    if (_restartAdvertising) {
      _restartAdvertising = false;
      Serial.println("[BLE] Restart advertising");
      if (_advertising) {
        _advertising->start();
      } else {
        BLEDevice::startAdvertising();
      }
    }
  }

  class ServerCB : public BLEServerCallbacks {
  public:
    BleCommands* _parent = nullptr;

    void onConnect(BLEServer*) override {
      if (_parent) {
        _parent->handleConnect();
      }
    }

    void onDisconnect(BLEServer*) override {
      if (_parent) {
        _parent->handleDisconnect();
      }
    }
  } _serverCallbacks;

  class CharCB : public BLECharacteristicCallbacks {
  public:
    BleCommands* _parent = nullptr;
    void onWrite(BLECharacteristic* c) override {
      if (!_parent) return;
      std::string raw = c->getValue();
      if (raw.empty()) return;

      Serial.printf("[BLE] Raw RX: %s\n", raw.c_str());

      const char* ptr = BleCommands::skipDelimiters(raw.c_str());
      if (*ptr == '\0') {
        Serial.println("[BLE] Ignoring empty payload");
        return;
      }

      if (*ptr == 'J' || *ptr == 'j') {
        ptr = BleCommands::skipDelimiters(ptr + 1);
        char* endPtr = nullptr;
        float x = strtof(ptr, &endPtr);
        if (endPtr == ptr) {
          Serial.println("[BLE] Failed to parse joystick X");
          return;
        }
        ptr = BleCommands::skipDelimiters(endPtr);
        endPtr = nullptr;
        float y = strtof(ptr, &endPtr);
        if (endPtr == ptr) {
          Serial.println("[BLE] Failed to parse joystick Y");
          return;
        }
        if (x < -1.5f) x = -1.5f;
        if (x > 1.5f) x = 1.5f;
        if (y < -1.5f) y = -1.5f;
        if (y > 1.5f) y = 1.5f;
        Serial.printf("[BLE] Enqueue joystick vector x=%.3f y=%.3f\n", x, y);
        _parent->enqueueVector(x, y);
        return;
      }

      char* endPtr = nullptr;
      long value = strtol(ptr, &endPtr, 10);
      if (endPtr != ptr) {
        if (value < 0) value = 0;
        if (value > 255) value = 255;
        Serial.printf("[BLE] Enqueue command code=%ld\n", value);
        _parent->enqueueCommand(static_cast<uint8_t>(value));
        return;
      }

      uint8_t fallback = static_cast<uint8_t>(*ptr);
      Serial.printf("[BLE] Fallback ASCII command code=%u (%c)\n", fallback, *ptr);
      _parent->enqueueCommand(fallback);
    }
  } _cb;
};