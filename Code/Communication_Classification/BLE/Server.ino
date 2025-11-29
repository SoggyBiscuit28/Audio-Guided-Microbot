#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// --- Throughput Calculation Vars ---
const int PAYLOAD_SIZE = 500;
volatile long packetsReceived = 0; 
volatile unsigned long startTime = 0; 
volatile unsigned long lastPacketTime = 0; 

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      if (startTime == 0) {
        startTime = millis();
      }
      lastPacketTime = millis();
      packetsReceived++; 
      if (packetsReceived % 10 == 0) {
        unsigned long elapsedTime = lastPacketTime - startTime;
        if (elapsedTime > 0) {
          long totalBytes = packetsReceived * PAYLOAD_SIZE;
          float throughput_bps = (totalBytes * 8.0f) / (elapsedTime / 1000.0f);
          float throughput_kbps = throughput_bps / 1024.0f;
          Serial.print("Received ");
          Serial.print(packetsReceived);
          Serial.print(" packets. ");
          Serial.print("Elapsed Time: ");
          Serial.print(elapsedTime);
          Serial.print(" ms. ");
          Serial.print("Throughput: ");
          Serial.print(throughput_kbps, 2);
          Serial.println(" kbps");
        }
      }
    }
};

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE Server for Throughput Test...");
  BLEDevice::init("ESP32_Throughput_Test_Server");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic( CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_WRITE);
  pCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Server started. Waiting for a client to connect and send data...");
}

void loop() {
  delay(2000); 
}
