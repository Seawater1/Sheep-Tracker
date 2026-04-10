#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRaWan-Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

// ===== Hardware =====
// RAK19007 (base), RAK11310 (core), RAK12500 (GNSS)
// Adjust analog pins if your base board wiring differs
#define PIN_GNSS_PWR WB_IO2
#define PIN_VBAT WB_A0

// Battery divider calibration (adjust to your hardware)
#define VBAT_MV_PER_LSB 0.806F
#define VBAT_DIVIDER_COMP 1.846F
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)

// ===== LoRaWAN =====
#define LORAWAN_REGION LORAMAC_REGION_EU868
#define LORAWAN_APP_PORT 1
#define LORAWAN_RX_PORT 2
#define JOINREQ_NBTRIALS 3

// Keep 60 seconds while validating end-to-end behaviour.
// Raise this to 24 hours for field deployment once the tracker is proven.
#define DEFAULT_SEND_INTERVAL_SEC 60UL

// GNSS is one of the biggest power draws on the tracker, so only power it
// when building an uplink and keep the search window bounded.
#define GNSS_POWER_SETTLE_MS 1500UL
#define GNSS_FIX_TIMEOUT_MS 45000UL
#define GNSS_POLL_MS 250UL
#define GPS_FIX_SIV_MIN 4U
#define GNSS_PROGRESS_LOG_MS 5000UL

// Test mode: keep the tracker focused on acquiring and sending real GPS fixes.
// After each successful GPS uplink, reset the acquisition state and start over
// automatically on the next interval so outdoor testing can run unattended.
#define GPS_TEST_MODE 1

static uint32_t g_send_interval_sec = DEFAULT_SEND_INTERVAL_SEC;

// OTAA keys — rak11300-tracker-1 on TTN eu1
static uint8_t nodeDeviceEUI[8] = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x07, 0x6C, 0x9F};
static uint8_t nodeAppEUI[8]    = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t nodeAppKey[16]   = {0x2F, 0x26, 0xF1, 0x34, 0xA7, 0x4C, 0xBD, 0x4C,
                                   0xE7, 0x10, 0xB7, 0x8E, 0xE3, 0xAC, 0xF2, 0x30};

static uint8_t m_lora_app_data_buffer[64];
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0};
static bool g_joined = false;

static lmh_param_t lora_param_init = {
    LORAWAN_ADR_ON,
    DR_3,
    LORAWAN_PUBLIC_NETWORK,
    JOINREQ_NBTRIALS,
    LORAWAN_DEFAULT_TX_POWER,
    LORAWAN_DUTYCYCLE_OFF,
};

static void lorawan_has_joined_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void lorawan_join_failed_handler(void);
static void lorawan_unconf_finished(void);
static void lorawan_conf_finished(bool result);

static lmh_callback_t lora_callbacks = {
    BoardGetBatteryLevel,
    BoardGetUniqueId,
    BoardGetRandomSeed,
    lorawan_rx_handler,
    lorawan_has_joined_handler,
    lorawan_confirm_class_handler,
    lorawan_join_failed_handler,
    lorawan_unconf_finished,
    lorawan_conf_finished};

static TimerEvent_t appTimer;

// ===== GNSS =====
SFE_UBLOX_GNSS g_gnss;
static bool g_gnss_available = false;
static bool g_gnss_powered = false;
static bool g_have_seen_fix = false;
static uint32_t g_boot_ms = 0;
static uint32_t g_fix_attempts = 0;
static uint32_t g_fix_successes = 0;
static uint32_t g_send_attempts = 0;
static uint32_t g_send_with_fix = 0;

struct __attribute__((packed)) payload_t
{
    uint16_t batt_mv;
    int32_t lat_e7;
    int32_t lon_e7;
    int16_t alt_m;
    uint32_t gps_unix;
};

static uint16_t read_mv(uint8_t pin, float mv_per_lsb)
{
    unsigned int sum = 0;
    unsigned int adc_max = 0;
    unsigned int adc_min = 4095;

    for (uint8_t i = 0; i < 10; i++)
    {
        unsigned int v = analogRead(pin);
        if (v < adc_min) adc_min = v;
        if (v > adc_max) adc_max = v;
        sum += v;
        delay(2);
    }

    unsigned int avg = (sum - adc_max - adc_min) >> 3;
    return (uint16_t)(avg * mv_per_lsb);
}

static void gnss_power(bool on)
{
    pinMode(PIN_GNSS_PWR, OUTPUT);
    digitalWrite(PIN_GNSS_PWR, on ? HIGH : LOW);
    g_gnss_powered = on;
}

static uint16_t read_vbat_mv(void)
{
    return read_mv(PIN_VBAT, REAL_VBAT_MV_PER_LSB);
}

static void log_status(const char *label)
{
    Serial.printf("[%8lu ms] %s | joined=%u gnss_on=%u seen_fix=%u attempts=%lu success=%lu sends=%lu gps_sends=%lu\n",
                  (unsigned long)(millis() - g_boot_ms),
                  label,
                  g_joined ? 1 : 0,
                  g_gnss_powered ? 1 : 0,
                  g_have_seen_fix ? 1 : 0,
                  (unsigned long)g_fix_attempts,
                  (unsigned long)g_fix_successes,
                  (unsigned long)g_send_attempts,
                  (unsigned long)g_send_with_fix);
}

static bool ensure_gnss_ready(void)
{
    if (!g_gnss_powered)
    {
        Serial.println("GNSS power ON");
        gnss_power(true);
        delay(GNSS_POWER_SETTLE_MS);
    }

    if (!g_gnss.begin())
    {
        Serial.println("GNSS not found during capture");
        g_gnss_available = false;
        return false;
    }

    g_gnss_available = true;
    g_gnss.setI2COutput(COM_TYPE_UBX);
    g_gnss.setAutoPVT(true);
    g_gnss.setNavigationFrequency(1);
    return true;
}

static bool capture_gnss_fix(payload_t &p)
{
    if (!g_gnss_available && !g_gnss_powered)
    {
        Serial.println("GNSS was unavailable and off, powering back on");
        gnss_power(true);
        delay(GNSS_POWER_SETTLE_MS);
    }

    if (!ensure_gnss_ready())
    {
        return false;
    }

    g_fix_attempts++;
    const uint32_t start = millis();
    uint32_t last_progress_log = start;
    uint8_t best_siv = 0;
    log_status("Starting GNSS fix attempt");

    while ((millis() - start) < GNSS_FIX_TIMEOUT_MS)
    {
        const bool got_pvt = g_gnss.getPVT();
        const uint8_t siv = g_gnss.getSIV();
        if (siv > best_siv)
        {
            best_siv = siv;
        }

        if (got_pvt)
        {
            Serial.printf("GNSS PVT SIV=%u lat=%ld lon=%ld alt=%ld\n",
                          siv,
                          g_gnss.getLatitude(),
                          g_gnss.getLongitude(),
                          g_gnss.getAltitude());
            if (siv >= GPS_FIX_SIV_MIN)
            {
                p.lat_e7 = g_gnss.getLatitude();
                p.lon_e7 = g_gnss.getLongitude();
                p.alt_m = (int16_t)(g_gnss.getAltitude() / 1000);
                p.gps_unix = g_gnss.getUnixEpoch();
                g_have_seen_fix = true;
                g_fix_successes++;
                Serial.printf("GNSS FIX OK after %lu ms (SIV=%u)\n",
                              (unsigned long)(millis() - start),
                              siv);
                return true;
            }
        }

        if ((millis() - last_progress_log) >= GNSS_PROGRESS_LOG_MS)
        {
            Serial.printf("Waiting for GNSS fix... elapsed=%lu ms best_siv=%u current_siv=%u got_pvt=%u\n",
                          (unsigned long)(millis() - start),
                          best_siv,
                          siv,
                          got_pvt ? 1 : 0);
            last_progress_log = millis();
        }

        delay(GNSS_POLL_MS);
    }

    Serial.printf("GNSS fix unavailable for this uplink after %lu ms (best_siv=%u)\n",
                  (unsigned long)(millis() - start),
                  best_siv);
    return false;
}

static void build_payload(payload_t &p)
{
    p.batt_mv = read_vbat_mv();
    p.lat_e7 = 0;
    p.lon_e7 = 0;
    p.alt_m = 0;
    p.gps_unix = 0;

    capture_gnss_fix(p);
}

static void send_lora_frame(void)
{
    g_send_attempts++;

    if (!g_joined)
    {
        Serial.println("Not joined yet, skip send");
        return;
    }

    payload_t payload;
    build_payload(payload);

    memcpy(m_lora_app_data.buffer, &payload, sizeof(payload));
    m_lora_app_data.port = LORAWAN_APP_PORT;
    m_lora_app_data.buffsize = sizeof(payload);

    lmh_error_status err = lmh_send(&m_lora_app_data, LMH_UNCONFIRMED_MSG);
    const bool this_send_has_fix = payload.lat_e7 != 0 || payload.lon_e7 != 0;
    if (this_send_has_fix)
    {
        g_send_with_fix++;
    }
    Serial.printf("lmh_send=%d, batt=%u, lat=%ld, lon=%ld, gps=%lu, gnss_on=%u, fixed=%u\n",
                  err,
                  payload.batt_mv,
                  payload.lat_e7,
                  payload.lon_e7,
                  (unsigned long)payload.gps_unix,
                  g_gnss_powered ? 1 : 0,
                  g_have_seen_fix ? 1 : 0);

#if GPS_TEST_MODE
    if (err == LMH_SUCCESS && this_send_has_fix)
    {
        log_status("GPS TEST COMPLETE: real fix sent");
        Serial.println("Restarting acquisition cycle for the next outdoor trial");
        if (g_gnss_powered)
        {
            Serial.println("GNSS power OFF to force a fresh start next cycle");
            gnss_power(false);
        }
        g_gnss_available = false;
        g_have_seen_fix = false;
        return;
    }
#endif

    if (g_have_seen_fix && g_gnss_powered)
    {
        Serial.println("GNSS power OFF after successful GPS send");
        gnss_power(false);
    }
}

static void tx_lora_periodic_handler(void)
{
    send_lora_frame();
    TimerSetValue(&appTimer, g_send_interval_sec * 1000UL);
    TimerStart(&appTimer);
}

static void lorawan_has_joined_handler(void)
{
    Serial.println("Joined LoRaWAN");
    g_joined = true;

    lmh_class_request(CLASS_A);

    TimerSetValue(&appTimer, g_send_interval_sec * 1000UL);
    TimerStart(&appTimer);
}

static void lorawan_join_failed_handler(void)
{
    Serial.println("Join failed: check DevEUI/AppEUI/AppKey and gateway");
}

static void lorawan_confirm_class_handler(DeviceClass_t Class)
{
    Serial.printf("Switch to Class %c done\n", "ABC"[Class]);
}

static void lorawan_unconf_finished(void)
{
    Serial.println("Uplink sent (unconfirmed)");
}

static void lorawan_conf_finished(bool result)
{
    Serial.printf("Uplink confirmed: %s\n", result ? "true" : "false");
}

static void lorawan_rx_handler(lmh_app_data_t *app_data)
{
    Serial.printf("RX port=%u len=%u\n", app_data->port, app_data->buffsize);

    if (app_data->port != LORAWAN_RX_PORT)
    {
        return;
    }

    // Downlink format (little-endian):
    // - 2 bytes: sleep minutes (uint16)
    // - or 4 bytes: sleep seconds (uint32)
    if (app_data->buffsize == 2)
    {
        uint16_t minutes = (uint16_t)app_data->buffer[0] | ((uint16_t)app_data->buffer[1] << 8);
        if (minutes > 0)
        {
            g_send_interval_sec = (uint32_t)minutes * 60UL;
            Serial.printf("Set interval to %u minutes\n", minutes);
            TimerSetValue(&appTimer, g_send_interval_sec * 1000UL);
            TimerStart(&appTimer);
        }
    }
    else if (app_data->buffsize == 4)
    {
        uint32_t seconds = (uint32_t)app_data->buffer[0] |
                           ((uint32_t)app_data->buffer[1] << 8) |
                           ((uint32_t)app_data->buffer[2] << 16) |
                           ((uint32_t)app_data->buffer[3] << 24);
        if (seconds > 0)
        {
            g_send_interval_sec = seconds;
            Serial.printf("Set interval to %u seconds\n", seconds);
            TimerSetValue(&appTimer, g_send_interval_sec * 1000UL);
            TimerStart(&appTimer);
        }
    }
}

void setup()
{
    Serial.begin(115200);
    delay(2000);
    g_boot_ms = millis();

    Serial.println();
    Serial.println("Sheep Tracker GPS test build");
    Serial.printf("Payload bytes=%u, interval=%lu sec, fix timeout=%lu ms\n",
                  (unsigned)sizeof(payload_t),
                  (unsigned long)g_send_interval_sec,
                  (unsigned long)GNSS_FIX_TIMEOUT_MS);
    Serial.println("Behavior: keep GNSS awake until a real GPS uplink succeeds, then restart the acquisition cycle automatically");

    analogReadResolution(12);

    gnss_power(false);
    Wire.begin();
    if (ensure_gnss_ready())
    {
        Serial.println("GNSS OK");
    }
    else
    {
        g_gnss_available = false;
        Serial.println("GNSS not found (will still send voltages)");
    }
    // Keep GNSS powered between uplinks until we get a first fix. Repeatedly
    // cold-starting every 60 seconds can prevent lock entirely during testing.
    log_status("Post-GNSS init");

    if (lora_rak11300_init() != 0)
    {
        Serial.println("LoRa init failed");
        return;
    }

    // Setup keys
    lmh_setDevEui(nodeDeviceEUI);
    lmh_setAppEui(nodeAppEUI);
    lmh_setAppKey(nodeAppKey);

    // Init LoRaWAN
    uint32_t err = lmh_init(&lora_callbacks, lora_param_init, true, CLASS_A, LORAWAN_REGION);
    if (err != 0)
    {
        Serial.printf("lmh_init failed - %u\n", err);
        return;
    }

    // Timer
    appTimer.timerNum = 3;
    TimerInit(&appTimer, tx_lora_periodic_handler);

    Serial.println("Joining LoRaWAN...");
    lmh_join();
}

void loop()
{
    // LoRaWAN stack is timer-driven; nothing to do here.
    // Keep loop light to reduce power.
}
