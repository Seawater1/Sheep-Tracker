/**
 * LoRa P2P Receiver (RAK11300 WisBlock)
 * Prints received payload as TEXT + RSSI/SNR
 */
#include <Arduino.h>
#include "LoRaWan-Arduino.h" // http://librarymanager/All#SX126x
#include <SPI.h>

// Forward declarations
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnRxTimeout(void);
void OnRxError(void);

// LoRa parameters (EU868)
#define RF_FREQUENCY 868300000  // Hz (must match transmitter)
#define LORA_BANDWIDTH 0        // 0:125kHz
#define LORA_SPREADING_FACTOR 7 // SF7..SF12
#define LORA_CODINGRATE 1       // 1:4/5
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 3000

static RadioEvents_t RadioEvents;

void setup()
{
  Serial.begin(115200);
  time_t timeout = millis();
  while (!Serial)
  {
    if ((millis() - timeout) < 5000) delay(100);
    else break;
  }

  // Initialise LoRa chip on RAK11300
  lora_rak11300_init();

  Serial.println("=====================================");
  Serial.println("LoRa P2P RX (Base Station)");
  Serial.println("=====================================");

  // Callbacks
  RadioEvents.TxDone = NULL;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = NULL;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;
  RadioEvents.CadDone = NULL;

  // Radio init
  Radio.Init(&RadioEvents);

  // Set channel
  Radio.SetChannel(RF_FREQUENCY);

  // Set RX config
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  Serial.println("Listening...");
  Radio.Rx(RX_TIMEOUT_VALUE);
}

void loop()
{
  // nothing needed - callbacks handle RX
}

/** Called when a packet is received */
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
  Serial.print("RX RSSI=");
  Serial.print(rssi);
  Serial.print("dBm SNR=");
  Serial.print(snr);
  Serial.print("  Payload: ");

  // Print as TEXT (readable)
  for (uint16_t i = 0; i < size; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Restart RX
  Radio.Rx(RX_TIMEOUT_VALUE);
}

void OnRxTimeout(void)
{
  // Restart RX
  Radio.Rx(RX_TIMEOUT_VALUE);
}

void OnRxError(void)
{
  // Restart RX
  Radio.Rx(RX_TIMEOUT_VALUE);
}
