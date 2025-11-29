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
int TX_POWER_DBM = 20;
unsigned long TX_INTERVAL_MS = 1000;

unsigned long seq = 0;
unsigned long lastTx = 0;

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n--- LoRa TX aggressive starting ---");
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DIO0);
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa.begin() FAILED");
    while (1) delay(1000);
  }
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setTxPower(TX_POWER_DBM);
  Serial.printf("TX CONFIG freq=%.0f sf=%d bw=%ld cr=4/%d txp=%d\n", (double)LORA_FREQUENCY, LORA_SF, (long)LORA_BW, LORA_CR, TX_POWER_DBM);
  Serial.println("CSV HEADER: timestamp_ms,role,seq,payload_len");
  lastTx = millis();
}

void loop() {
  unsigned long now = millis();
  if (now - lastTx >= TX_INTERVAL_MS) {
    seq++;
    String payload = String(seq) + "," + String(now);
    LoRa.beginPacket();
    LoRa.print(payload);
    LoRa.endPacket(true); 
    Serial.printf("%lu,TX,%lu,%d\n", now, seq, payload.length());
    lastTx = now;
  }
  delay(2);
}
