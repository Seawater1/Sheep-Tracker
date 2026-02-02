#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include "LoRaWan-Arduino.h"
#include <SPI.h>

// ================= GNSS =================
SFE_UBLOX_GNSS g_myGNSS;

// ================= Battery =================
#define PIN_VBAT WB_A0
#define VBAT_MV_PER_LSB (0.806F)
#define VBAT_DIVIDER_COMP (1.846F)
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)

float readVBAT_mV(void)
{
  unsigned int sum = 0;
  unsigned int adc_max = 0;
  unsigned int adc_min = 4095;

  for (uint8_t i = 0; i < 10; i++)
  {
    unsigned int v = analogRead(PIN_VBAT);
    if (v < adc_min) adc_min = v;
    if (v > adc_max) adc_max = v;
    sum += v;
    delay(2);
  }

  unsigned int avg = (sum - adc_max - adc_min) >> 3;
  return avg * REAL_VBAT_MV_PER_LSB;
}

// ================= USB / CHG =================
#define CHG_PIN WB_IO1
#define USB_PIN WB_IO3

// ================= LoRa (must match RX) =================
#define RF_FREQUENCY 868300000
#define TX_OUTPUT_POWER 22
#define LORA_BANDWIDTH 0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE 1
#define LORA_PREAMBLE_LENGTH 8
#define TX_TIMEOUT_VALUE 3000

static RadioEvents_t RadioEvents;
static bool txBusy = false;

void OnTxDone(void)    { txBusy = false; }
void OnTxTimeout(void) { txBusy = false; }

void sendPacket(const char *msg)
{
  uint8_t buf[64];
  size_t len = strlen(msg);
  if (len > 63) len = 63;

  memcpy(buf, msg, len);
  Radio.Send(buf, (uint16_t)len);
  txBusy = true;
}

// ================= Timing =================
unsigned long lastSend = 0;

void setup()
{
  Serial.begin(115200);
  delay(2000);
  Serial.println("BOOT: Sheep TX");

  analogReadResolution(12);
  pinMode(CHG_PIN, INPUT);
  pinMode(USB_PIN, INPUT);

  // Power cycle GNSS
  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, 0);
  delay(1000);
  digitalWrite(WB_IO2, 1);
  delay(1500);

  Wire.begin();

  if (g_myGNSS.begin())
  {
    g_myGNSS.setI2COutput(COM_TYPE_UBX);
    g_myGNSS.setAutoPVT(true);
    Serial.println("GNSS OK");
  }
  else
  {
    Serial.println("GNSS not found (will still send battery)");
  }

  // LoRa init
  lora_rak11300_init();

  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);

  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0,
                    LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, LORA_PREAMBLE_LENGTH,
                    false, true, 0, 0, false, TX_TIMEOUT_VALUE);

  Serial.println("LoRa ready");
}

void loop()
{
  if (millis() - lastSend > 2000 && !txBusy)
  {
    lastSend = millis();

    // ===== Status pins raw =====
    int usb_raw = digitalRead(USB_PIN);
    int chg_raw = digitalRead(CHG_PIN);

    float vbat_V = readVBAT_mV() / 1000.0;

    // ===== GNSS =====
    bool gotPVT = g_myGNSS.getPVT();
    byte SIV = g_myGNSS.getSIV();

    char msg[64];

    if (!gotPVT || SIV < 4)
    {
      snprintf(msg, sizeof(msg),
               "NOFIX,%.2f,u%d,c%d",
               vbat_V, usb_raw, chg_raw);
    }
    else
    {
      long lat_raw = g_myGNSS.getLatitude();
      long lon_raw = g_myGNSS.getLongitude();
      long alt_mm  = g_myGNSS.getAltitude();

      double lat = lat_raw / 10000000.0;
      double lon = lon_raw / 10000000.0;
      double alt = alt_mm / 1000.0;

      snprintf(msg, sizeof(msg),
               "%.6f,%.6f,%.1f,%u,%.2f,u%d,c%d",
               lat, lon, alt, SIV, vbat_V, usb_raw, chg_raw);
    }

    Serial.print("TX: ");
    Serial.println(msg);

    sendPacket(msg);
  }
}
