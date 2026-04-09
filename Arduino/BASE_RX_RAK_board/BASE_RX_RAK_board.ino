/**
 * Low-power Sheep Tracker receiver for RAK11300 WisBlock.
 * Decodes the compact binary tracker packet and falls back to text output
 * so older transmitters can still be monitored during migration.
 */
#include <Arduino.h>
#include "LoRaWan-Arduino.h"
#include <SPI.h>

// Forward declarations
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnRxTimeout(void);
void OnRxError(void);

// LoRa parameters (must match tracker)
#define RF_FREQUENCY 868300000
#define LORA_BANDWIDTH 0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE 1
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 3000

#define PAYLOAD_VERSION 1
#define PAYLOAD_FLAG_FIX 0x01
#define PAYLOAD_FLAG_LOW_BATT 0x02

struct __attribute__((packed)) tracker_payload_t
{
  uint8_t version;
  uint8_t flags;
  uint8_t sats;
  uint8_t seq;
  uint16_t batt_mv;
  int32_t lat_e7;
  int32_t lon_e7;
  int16_t alt_m;
  uint32_t gps_unix;
};

static RadioEvents_t RadioEvents;

static void print_text_payload(const uint8_t *payload, uint16_t size)
{
  Serial.print("Payload: ");
  for (uint16_t i = 0; i < size; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

static void print_tracker_payload(const tracker_payload_t &packet)
{
  Serial.print("Tracker seq=");
  Serial.print(packet.seq);
  Serial.print(" batt=");
  Serial.print(packet.batt_mv);
  Serial.print("mV");

  if ((packet.flags & PAYLOAD_FLAG_LOW_BATT) != 0)
  {
    Serial.print(" LOW_BATT");
  }

  if ((packet.flags & PAYLOAD_FLAG_FIX) == 0)
  {
    Serial.println(" NO_FIX");
    return;
  }

  const double lat = packet.lat_e7 / 10000000.0;
  const double lon = packet.lon_e7 / 10000000.0;

  Serial.print(" sats=");
  Serial.print(packet.sats);
  Serial.print(" lat=");
  Serial.print(lat, 6);
  Serial.print(" lon=");
  Serial.print(lon, 6);
  Serial.print(" alt=");
  Serial.print(packet.alt_m);
  Serial.print("m gps=");
  Serial.println(packet.gps_unix);
}

void setup()
{
  Serial.begin(115200);
  time_t timeout = millis();
  while (!Serial)
  {
    if ((millis() - timeout) < 5000) delay(100);
    else break;
  }

  lora_rak11300_init();

  Serial.println("=====================================");
  Serial.println("Sheep Tracker Base RX");
  Serial.println("=====================================");

  RadioEvents.TxDone = NULL;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = NULL;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;
  RadioEvents.CadDone = NULL;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  Serial.println("Listening...");
  Radio.Rx(RX_TIMEOUT_VALUE);
}

void loop()
{
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
  Serial.print("RX RSSI=");
  Serial.print(rssi);
  Serial.print("dBm SNR=");
  Serial.println(snr);

  if (size == sizeof(tracker_payload_t))
  {
    tracker_payload_t packet;
    memcpy(&packet, payload, sizeof(packet));
    if (packet.version == PAYLOAD_VERSION)
    {
      print_tracker_payload(packet);
    }
    else
    {
      print_text_payload(payload, size);
    }
  }
  else
  {
    print_text_payload(payload, size);
  }

  Radio.Rx(RX_TIMEOUT_VALUE);
}

void OnRxTimeout(void)
{
  Radio.Rx(RX_TIMEOUT_VALUE);
}

void OnRxError(void)
{
  Radio.Rx(RX_TIMEOUT_VALUE);
}
