#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>

static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
static BLEUUID charUUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");

static BLERemoteCharacteristic* gChar = nullptr;
static BLEClient* gClient = nullptr;
static BLEAdvertisedDevice* gAdvertised = nullptr;

static bool connected = false;
static bool doConnect = false;
static bool scanPending = false;
static bool scanningNow = false;
static unsigned long lastScanRestartMs = 0;

void requestScan();
void pumpScan();

void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData, size_t length, bool isNotify) {
  String s;
  for (size_t i = 0; i < length; ++i) s += (char)pData[i];
  Serial.print("[NOTIF] ");
  Serial.println(s);
}

class ClientCB : public BLEClientCallbacks {
  void onConnect(BLEClient* c) override {
    Serial.println("[BLE] Connected to server");
  }
  void onDisconnect(BLEClient* c) override {
    Serial.println("[BLE] Disconnected, restarting scan...");
    connected = false;
    gChar = nullptr;
    if (gAdvertised) {
      delete gAdvertised;
      gAdvertised = nullptr;
    }
    if (gClient) {
      delete gClient;
      gClient = nullptr;
    }
    doConnect = false;
    requestScan();
  }
};

class ScanCB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice adv) override {
    if (adv.haveServiceUUID() && adv.isAdvertisingService(serviceUUID)) {
      Serial.print("[BLE] Found target: ");
      Serial.println(adv.getAddress().toString().c_str());
      BLEDevice::getScan()->stop();
      if (gAdvertised) delete gAdvertised;
      gAdvertised = new BLEAdvertisedDevice(adv);
      doConnect = true;
    }
  }
};

static ClientCB gClientCallbacks;
static ScanCB gScanCallbacks;

void requestScan() {
  scanPending = true;
}

void pumpScan() {
  if (!scanPending || scanningNow) {
    return;
  }
  BLEScan* scan = BLEDevice::getScan();
  if (!scan) {
    scanPending = false;
    return;
  }
  scanningNow = true;
  scanPending = false;
  Serial.println("[BLE] Scanning...");
  scan->clearResults();
  scan->start(5, false);
  scanningNow = false;
  lastScanRestartMs = millis();
  if (!connected && !doConnect) {
    scanPending = true;
  }
}

bool connectToServer() {
  if (!gAdvertised) {
    Serial.println("[BLE] No advertised device cached.");
    return false;
  }
  Serial.print("[BLE] Connecting to device ");
  Serial.println(gAdvertised->getAddress().toString().c_str());
  gClient = BLEDevice::createClient();
  gClient->setClientCallbacks(&gClientCallbacks);
  if (!gClient->connect(gAdvertised)) {
    Serial.println("[BLE] Connect failed");
    delete gClient;
    gClient = nullptr;
    return false;
  }
  BLERemoteService* svc = gClient->getService(serviceUUID);
  if (!svc) {
    Serial.println("[BLE] Service not found");
    gClient->disconnect();
    delete gClient;
    gClient = nullptr;
    return false;
  }
  gChar = svc->getCharacteristic(charUUID);
  if (!gChar) {
    Serial.println("[BLE] Char not found");
    gClient->disconnect();
    delete gClient;
    gClient = nullptr;
    return false;
  }
  if (gChar->canNotify()) {
    gChar->registerForNotify(notifyCallback);
    Serial.println("[BLE] Registered for notifications.");
  }
  connected = true;
  Serial.println("[BLE] Ready. Type 0-6 or joystick frames like 'J 0.10 -0.35'.");
  return true;
}

void setup() {
  Serial.begin(115200);
  BLEDevice::init("");
  BLEScan* scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(&gScanCallbacks, true);
  scan->setInterval(1349);
  scan->setWindow(449);
  scan->setActiveScan(true);
  requestScan();
}

void loop() {
  pumpScan();
  if (connected && gClient && !gClient->isConnected()) {
    Serial.println("[BLE] Disconnected (poll), restarting scan...");
    connected = false;
    gChar = nullptr;
    requestScan();
  }
  if (doConnect) {
    if (connectToServer()) {
      Serial.println("[BLE] Connection established.");
    } else {
      Serial.println("[BLE] Connection attempt failed, rescanning...");
      requestScan();
    }
    doConnect = false;
  }
  if (!connected && (millis() - lastScanRestartMs) > 7000) {
    requestScan();
  }
  if (connected && gChar) {
    if (Serial.available()) {
      String line = Serial.readStringUntil('\n');
      line.trim();
      if (line.length() == 0) return;
      char c = line.charAt(0);
      if (line.length() == 1 && c >= '0' && c <= '9') {
        uint8_t payload[1];
        payload[0] = static_cast<uint8_t>(c);
        gChar->writeValue(payload, 1, true);
        Serial.print("[WRITE] Sent digit: ");
        Serial.println(static_cast<char>(payload[0]));
      } else {
        String frame = line;
        gChar->writeValue(frame, true);
        Serial.print("[WRITE] Sent frame: ");
        Serial.println(frame);
      }
    }
  }
  delay(10);
}