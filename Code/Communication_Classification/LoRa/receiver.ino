#include <SPI.h>
#include <LoRa.h>

//Pins:
#define SCK   18
#define MISO  19
#define MOSI  23
#define SS    5
#define RST   14
#define DIO0  26

const long LORA_FREQUENCY = 433E6; 
int LORA_SF  = 12;      
long LORA_BW = 125E3;  
int LORA_CR  = 5;

unsigned long lastPacketTime = 0;
unsigned long packetCount = 0;
unsigned long lastHeartbeat = 0;

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n--- LoRa RX (polling) starting ---");

  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DIO0);

  Serial.print("LoRa.begin()...");
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("FAILED: LoRa.begin() returned false. Check wiring/power/antenna.");
    while (1) { delay(1000); }
  }
  Serial.println("OK");

  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);

  Serial.printf("CONFIG freq=%.0f sf=%d bw=%ld cr=4/%d\n", (double)LORA_FREQUENCY, LORA_SF, (long)LORA_BW, LORA_CR);
  Serial.println("CSV HEADER: timestamp_ms,role,payload_or_seq,freq,sf,bw,cr,payload_len,rssi,snr");
  Serial.println("Entering receive mode (polling).");

  LoRa.receive();
}

void loop() {
  unsigned long now = millis();
  if (now - lastHeartbeat > 5000) {
    Serial.printf("HB: %lu ms  packets_received=%lu  last_pkt_age_ms=%lu\n", now, packetCount, (lastPacketTime==0 ? 0 : (now - lastPacketTime))); lastHeartbeat = now;
  }

  int packetSize = LoRa.parsePacket();
  if (packetSize > 0) {
    String payload = "";
    for (int i = 0; i < packetSize; ++i) {
      int c = LoRa.read();
      if (c < 0) break;
      payload += (char)c;
    }
    payload.trim();
    float rssi = LoRa.packetRssi();
    float snr  = LoRa.packetSnr();
    lastPacketTime = now;
    packetCount++;
    Serial.printf("%lu,RX,%s,%.0f,%d,%ld,4/%d,%d,%.1f,%.2f\n", lastPacketTime, payload.c_str(), (double)LORA_FREQUENCY, LORA_SF, (long)LORA_BW, LORA_CR, payload.length(), rssi, snr);
    Serial.printf("RX: payload='%s' len=%d RSSI=%.1f SNR=%.2f\n", payload.c_str(), payload.length(), rssi, snr);
    LoRa.receive();
  }

  if (now - lastPacketTime > 15000) { 
    Serial.println("No packets for 15s — re-entering receive() to recover.");
    LoRa.receive();
    lastPacketTime = now - 15000; 
  }
  delay(10);
}
