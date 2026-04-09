#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include "LoRaWan-Arduino.h"
#include <SPI.h>

// =====================================================================
// LoRaWAN OTAA credentials (from TTN sheep-tracker1 / rak11300-tracker-1)
// =====================================================================
static uint8_t DEVEUI[]  = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x07, 0x6C, 0x9F };
static uint8_t APPEUI[]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static uint8_t APPKEY[]  = { 0x2F, 0x26, 0xF1, 0x34, 0xA7, 0x4C, 0xBD, 0x4C,
                              0xE7, 0x10, 0xB7, 0x8E, 0xE3, 0xAC, 0xF2, 0x30 };

// =====================================================================
// LoRaWAN settings
// =====================================================================
#define LORAWAN_DATARATE    DR_3        // SF9 — good range/speed balance
#define LORAWAN_TX_POWER    TX_POWER_5  // 14 dBm ERP (EU868 fair-use)
#define LORAWAN_PORT        1
#define TX_INTERVAL_MS      30000       // send every 30 s (duty cycle safe)

// =====================================================================
// GNSS
// =====================================================================
SFE_UBLOX_GNSS myGNSS;

// =====================================================================
// Battery
// =====================================================================
#define PIN_VBAT            WB_A0
#define VBAT_MV_PER_LSB     0.806F
#define VBAT_DIVIDER_COMP   1.846F
#define REAL_MV_PER_LSB     (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)

float readVBAT_mV()
{
    unsigned int sum = 0, adc_max = 0, adc_min = 4095;
    for (uint8_t i = 0; i < 10; i++) {
        unsigned int v = analogRead(PIN_VBAT);
        if (v < adc_min) adc_min = v;
        if (v > adc_max) adc_max = v;
        sum += v;
        delay(2);
    }
    return ((sum - adc_max - adc_min) >> 3) * REAL_MV_PER_LSB;
}

uint8_t mvToPercent(float mV)
{
    if (mV < 3300) return 0;
    if (mV < 3600) return (uint8_t)((mV - 3300) / 30.0f);
    return (uint8_t)(10 + (mV - 3600) * 0.15f);
}

// =====================================================================
// LoRaWAN state
// =====================================================================
static TimerEvent_t txTimer;
static bool joined     = false;
static bool txPending  = false;

static void onJoinRequest(LoRaMacEventInfoStatus_t status, uint8_t dataRate)
{
    if (status == LORAMAC_EVENT_INFO_STATUS_OK) {
        Serial.println("[LoRa] Joined TTN via OTAA");
        joined = true;
        TimerSetValue(&txTimer, TX_INTERVAL_MS);
        TimerStart(&txTimer);
    } else {
        Serial.printf("[LoRa] Join failed (%d) — retrying in 10s\n", status);
        delay(10000);
        lmh_join();
    }
}

static void onTxConfirm(LoRaMacEventInfoStatus_t status, uint8_t dataRate)
{
    txPending = false;
    if (status == LORAMAC_EVENT_INFO_STATUS_OK)
        Serial.println("[LoRa] TX confirmed");
    else
        Serial.printf("[LoRa] TX error (%d)\n", status);
}

static void onRxData(lmh_app_data_t *data)
{
    Serial.printf("[LoRa] RX port %d, %d bytes\n", data->port, data->buffsize);
}

static lmh_callback_t loraCallbacks = {
    BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
    onRxData, onJoinRequest, onTxConfirm
};

static lmh_param_t loraParams = {
    LORAWAN_ADR_OFF, LORAWAN_DATARATE, LORAWAN_PUBLIC_NETWORK,
    JOINREQ_NBTRIALS, LORAWAN_TX_POWER, LORAWAN_DUTYCYCLE_OFF
};

// =====================================================================
// Build and send uplink
// =====================================================================
static void sendPayload()
{
    if (!joined || txPending) return;

    bool ok = myGNSS.getPVT();

    double lat  = ok ? myGNSS.getLatitude()    / 10000000.0 : 0.0;
    double lon  = ok ? myGNSS.getLongitude()   / 10000000.0 : 0.0;
    double alt  = ok ? myGNSS.getAltitude()    / 1000.0     : 0.0;
    double spd  = ok ? (myGNSS.getGroundSpeed() / 1000.0) * 3.6 : 0.0;
    uint8_t siv = myGNSS.getSIV();

    float   vbat  = readVBAT_mV();
    uint8_t bpct  = mvToPercent(vbat);

    // CSV payload — matches existing RX parsing expectations
    char buf[64];
    int  len = snprintf(buf, sizeof(buf),
        "%.5f,%.5f,%.1f,%.1f,%u,%.3f,%u",
        lat, lon, alt, spd, siv, vbat / 1000.0f, bpct);

    Serial.printf("[TX] %s\n", buf);

    lmh_app_data_t tx = {
        .buffer    = (uint8_t*)buf,
        .buffsize  = (uint32_t)len,
        .port      = LORAWAN_PORT,
        .confirmed = LMH_UNCONFIRMED_MSG,
        .fpending  = 0
    };

    if (lmh_send(&tx, LMH_UNCONFIRMED_MSG) == LMH_SUCCESS)
        txPending = true;
    else
        Serial.println("[TX] lmh_send failed");
}

static void onTxTimer() { sendPayload(); }

// =====================================================================
// Setup
// =====================================================================
void setup()
{
    time_t t = millis();
    Serial.begin(115200);
    while (!Serial && (millis() - t) < 5000) delay(100);

    Serial.println("=== RAK11300 LoRaWAN GPS Tracker ===");

    analogReadResolution(12);

    // GNSS power cycle
    pinMode(WB_IO2, OUTPUT);
    digitalWrite(WB_IO2, LOW);
    delay(1000);
    digitalWrite(WB_IO2, HIGH);
    delay(1500);

    Wire.begin();
    if (!myGNSS.begin()) {
        Serial.println("[GNSS] Not found — check wiring");
        while (1) delay(100);
    }
    myGNSS.setI2COutput(COM_TYPE_UBX);
    myGNSS.setAutoPVT(true);
    myGNSS.setNavigationFrequency(1);
    Serial.println("[GNSS] Ready");

    // LoRaWAN init
    lora_rak11300_init();
    lmh_setDevEui(DEVEUI);
    lmh_setAppEui(APPEUI);
    lmh_setAppKey(APPKEY);

    if (lmh_init(&loraCallbacks, loraParams, true, CLASS_A, LORAMAC_REGION_EU868) != 0) {
        Serial.println("[LoRa] Init failed");
        while (1) delay(100);
    }

    TimerInit(&txTimer, onTxTimer);

    Serial.println("[LoRa] Joining TTN (OTAA)...");
    lmh_join();
}

// =====================================================================
// Loop
// =====================================================================
void loop()
{
    Radio.IrqProcess();
}
