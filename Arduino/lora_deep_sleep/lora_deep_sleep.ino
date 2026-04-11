/**
 * Sheep Tracker — Deep Sleep Firmware
 *
 * RAK19007 (base) + RAK11310 (core) + RAK12500 (GNSS u-blox)
 *
 * Behaviour:
 *   1. Wake from deep sleep (or boot).
 *   2. Power on GPS, wait for fix (up to GNSS_FIX_TIMEOUT_MS).
 *   3. Send LoRaWAN uplink with position + battery + time-to-first-fix.
 *   4. Power off GPS, return to deep sleep for SEND_INTERVAL.
 *
 * Two modes controlled by GPS_TIMING_TEST:
 *   GPS_TIMING_TEST = 1  →  short interval, verbose serial logging (USB bench testing)
 *   GPS_TIMING_TEST = 0  →  24-hour interval, minimal serial (battery field test)
 *
 * Payload (packed struct, little-endian, 18 bytes):
 *   Bytes 0-1   batt_mv     uint16   Battery mV
 *   Bytes 2-5   lat_e7      int32    Latitude × 1e7 (0 if no fix)
 *   Bytes 6-9   lon_e7      int32    Longitude × 1e7 (0 if no fix)
 *   Bytes 10-11 alt_m       int16    Altitude in metres
 *   Bytes 12-15 gps_unix    uint32   Unix timestamp from GPS
 *   Bytes 16-17 ttff_ds     uint16   Time-to-first-fix in deciseconds (0 = no fix)
 *
 * Uses WisBlock-API-V2 for deep sleep management on RAK11300 (RP2040).
 */

#include <Arduino.h>
#include <Wire.h>
#include <WisBlock-API-V2.h>
#define Stream arduino::Stream
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#undef Stream

// ============================================================
// MODE SWITCH — change this for bench vs field
// ============================================================
#define GPS_TIMING_TEST 0  // 1 = bench test, 0 = 24 h battery field test

// ============================================================
// Debug logging
// ============================================================
#if GPS_TIMING_TEST
  #define LOG(fmt, ...) Serial.printf(fmt "\n", ##__VA_ARGS__)
#else
  #define LOG(fmt, ...) do {} while(0)
#endif

// ============================================================
// Firmware version
// ============================================================
#define SW_VERSION_1 2
#define SW_VERSION_2 0
#define SW_VERSION_3 0

// ============================================================
// Hardware pins
// ============================================================
#define PIN_VBAT     WB_A0
#define GNSS_PWR_PIN WB_IO2

// Battery divider calibration
#define VBAT_MV_PER_LSB       0.806F
#define VBAT_DIVIDER_COMP     1.846F
#define REAL_VBAT_MV_PER_LSB  (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)

// ============================================================
// GNSS settings
// ============================================================
// Cold start outdoors: typically 30-60 s for u-blox with good sky view.
// Warm start (if backup battery keeps ephemeris): ~5-15 s.
// For the 2-week battery test, cap the GPS window at about 1 minute.
#if GPS_TIMING_TEST
  #define GNSS_FIX_TIMEOUT_MS   90000UL   // 90 s — long enough to measure cold start
#else
  #define GNSS_FIX_TIMEOUT_MS   60000UL   // 60 s — about 1 minute awake per day
#endif

#define GNSS_POWER_SETTLE_MS  1500UL   // Let u-blox power rail stabilise
#define GNSS_POLL_MS           500UL   // PVT polling interval
#define GPS_FIX_SIV_MIN          4U    // Minimum satellites for a good fix

// ============================================================
// Send interval
// ============================================================
#if GPS_TIMING_TEST
  #define SEND_INTERVAL_MS  (2UL * 60UL * 1000UL)          // 2 minutes
#else
  #define SEND_INTERVAL_MS  (24UL * 60UL * 60UL * 1000UL)  // 24 hours
#endif

// ============================================================
// LoRaWAN
// ============================================================
#define LORAWAN_APP_PORT 1

// ============================================================
// Globals
// ============================================================
SFE_UBLOX_GNSS g_gnss;
bool g_gnss_ready = false;

// Stats (survive across wake cycles only during test runs with USB power)
static uint32_t g_cycle_count  = 0;
static uint32_t g_fix_count    = 0;
static uint32_t g_nofix_count  = 0;
static uint32_t g_best_ttff_ms = 0xFFFFFFFF;
static uint32_t g_worst_ttff_ms = 0;
static uint32_t g_total_ttff_ms = 0;

// ============================================================
// Payload — 18 bytes
// ============================================================
struct __attribute__((packed)) payload_t
{
    uint16_t batt_mv;
    int32_t  lat_e7;
    int32_t  lon_e7;
    int16_t  alt_m;
    uint32_t gps_unix;
    uint16_t ttff_ds;     // time-to-first-fix in deciseconds (0.1 s units), 0 = no fix
};

// ============================================================
// Utility: averaged ADC read
// ============================================================
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

// ============================================================
// GNSS power control
// ============================================================
static void gnss_power(bool on)
{
    pinMode(GNSS_PWR_PIN, OUTPUT);
    digitalWrite(GNSS_PWR_PIN, on ? HIGH : LOW);
    LOG("GNSS power %s", on ? "ON" : "OFF");
}

// ============================================================
// GNSS initialise (after power on)
// ============================================================
static bool gnss_start(void)
{
    if (g_gnss_ready) return true;

    g_gnss_ready = g_gnss.begin();
    if (g_gnss_ready)
    {
        g_gnss.setI2COutput(COM_TYPE_UBX);
        g_gnss.setNavigationFrequency(1);
        g_gnss.setAutoPVT(true);
        LOG("GNSS initialised OK");
    }
    else
    {
        LOG("GNSS begin() FAILED");
    }
    return g_gnss_ready;
}

// ============================================================
// Acquire GPS fix with timing
// Returns ttff in milliseconds, or 0 if no fix obtained.
// ============================================================
static uint32_t acquire_fix(payload_t &p)
{
    gnss_power(true);
    delay(GNSS_POWER_SETTLE_MS);

    if (!gnss_start())
    {
        gnss_power(false);
        g_gnss_ready = false;
        return 0;
    }

    LOG("Searching for GPS fix (timeout %lu ms, need %u SIV)...",
        (unsigned long)GNSS_FIX_TIMEOUT_MS, GPS_FIX_SIV_MIN);

    const uint32_t start = millis();
    uint8_t best_siv = 0;
    uint32_t ttff_ms = 0;

    while ((millis() - start) < GNSS_FIX_TIMEOUT_MS)
    {
        bool got_pvt = g_gnss.getPVT();
        uint8_t siv  = g_gnss.getSIV();

        if (siv > best_siv) best_siv = siv;

        if (got_pvt && siv >= GPS_FIX_SIV_MIN)
        {
            ttff_ms = millis() - start;
            p.lat_e7   = g_gnss.getLatitude();
            p.lon_e7   = g_gnss.getLongitude();
            p.alt_m    = (int16_t)(g_gnss.getAltitude() / 1000);
            p.gps_unix = g_gnss.getUnixEpoch();

            LOG("FIX ACQUIRED in %lu ms  SIV=%u  lat=%ld  lon=%ld  alt=%d",
                (unsigned long)ttff_ms, siv,
                (long)p.lat_e7, (long)p.lon_e7, p.alt_m);
            break;
        }

        // Progress logging every 5 seconds
        if (((millis() - start) % 5000) < GNSS_POLL_MS)
        {
            LOG("  ... %lu ms  SIV=%u/%u  pvt=%u",
                (unsigned long)(millis() - start), siv, best_siv, got_pvt);
        }

        delay(GNSS_POLL_MS);
    }

    if (ttff_ms == 0)
    {
        LOG("NO FIX after %lu ms (best SIV=%u)", (unsigned long)(millis() - start), best_siv);
    }

    // Power off GPS immediately to save power
    gnss_power(false);
    g_gnss_ready = false;

    return ttff_ms;
}

// ============================================================
// Build full payload
// ============================================================
static void build_payload(payload_t &p)
{
    // Battery
    p.batt_mv = read_mv(PIN_VBAT, REAL_VBAT_MV_PER_LSB);

    // Zero GPS fields
    p.lat_e7   = 0;
    p.lon_e7   = 0;
    p.alt_m    = 0;
    p.gps_unix = 0;
    p.ttff_ds  = 0;

    // Acquire fix and measure timing
    uint32_t ttff_ms = acquire_fix(p);

    // Convert to deciseconds, cap at 65535 (≈109 minutes, more than enough)
    if (ttff_ms > 0)
    {
        uint32_t ds = (ttff_ms + 50) / 100;  // round to nearest 0.1s
        p.ttff_ds = (ds > 65535) ? 65535 : (uint16_t)ds;
    }

    // Update stats
    g_cycle_count++;
    if (ttff_ms > 0)
    {
        g_fix_count++;
        g_total_ttff_ms += ttff_ms;
        if (ttff_ms < g_best_ttff_ms)  g_best_ttff_ms  = ttff_ms;
        if (ttff_ms > g_worst_ttff_ms) g_worst_ttff_ms = ttff_ms;
    }
    else
    {
        g_nofix_count++;
    }

    LOG("=== Cycle %lu  batt=%u mV  fix=%s  ttff=%lu ms ===",
        (unsigned long)g_cycle_count,
        p.batt_mv,
        ttff_ms > 0 ? "YES" : "NO",
        (unsigned long)ttff_ms);

#if GPS_TIMING_TEST
    if (g_fix_count > 0)
    {
        LOG("=== STATS: cycles=%lu  fixes=%lu  nofixes=%lu  best=%lu ms  worst=%lu ms  avg=%lu ms ===",
            (unsigned long)g_cycle_count,
            (unsigned long)g_fix_count,
            (unsigned long)g_nofix_count,
            (unsigned long)g_best_ttff_ms,
            (unsigned long)g_worst_ttff_ms,
            (unsigned long)(g_total_ttff_ms / g_fix_count));
    }
#endif
}

// ============================================================
// WisBlock-API-V2 callbacks
// ============================================================

void setup_app(void)
{
#if GPS_TIMING_TEST
    Serial.begin(115200);
    time_t serial_timeout = millis();
    while (!Serial)
    {
        if ((millis() - serial_timeout) < 3000)
        {
            delay(100);
            digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
        }
        else
        {
            break;
        }
    }
    digitalWrite(LED_GREEN, LOW);

    Serial.println();
    Serial.println("========================================");
    Serial.println("  Sheep Tracker — Deep Sleep Firmware");
    Serial.println("  MODE: GPS TIMING TEST (2 min cycle)");
    Serial.printf("  Payload: %u bytes\n", (unsigned)sizeof(payload_t));
    Serial.printf("  GPS timeout: %lu ms\n", (unsigned long)GNSS_FIX_TIMEOUT_MS);
    Serial.printf("  Send interval: %lu ms\n", (unsigned long)SEND_INTERVAL_MS);
    Serial.println("========================================");
#endif

    api_set_version(SW_VERSION_1, SW_VERSION_2, SW_VERSION_3);

    // LoRaWAN credentials — rak11300-tracker-1 on TTN eu1
    g_lorawan_settings.node_device_eui[0] = 0x70;
    g_lorawan_settings.node_device_eui[1] = 0xB3;
    g_lorawan_settings.node_device_eui[2] = 0xD5;
    g_lorawan_settings.node_device_eui[3] = 0x7E;
    g_lorawan_settings.node_device_eui[4] = 0xD0;
    g_lorawan_settings.node_device_eui[5] = 0x07;
    g_lorawan_settings.node_device_eui[6] = 0x6C;
    g_lorawan_settings.node_device_eui[7] = 0x9F;

    memset(g_lorawan_settings.node_app_eui, 0x00, 8);

    uint8_t appKey[16] = {0x2F, 0x26, 0xF1, 0x34, 0xA7, 0x4C, 0xBD, 0x4C,
                          0xE7, 0x10, 0xB7, 0x8E, 0xE3, 0xAC, 0xF2, 0x30};
    memcpy(g_lorawan_settings.node_app_key, appKey, 16);
    g_lorawan_settings.send_repeat_time      = SEND_INTERVAL_MS;
    g_lorawan_settings.app_port              = LORAWAN_APP_PORT;
    g_lorawan_settings.otaa_enabled          = true;
    g_lorawan_settings.adr_enabled           = true;
    g_lorawan_settings.auto_join             = true;
    g_lorawan_settings.confirmed_msg_enabled = LMH_UNCONFIRMED_MSG;
    api_set_credentials();
}

bool init_app(void)
{
    LOG("init_app: setting up hardware");

    analogReadResolution(12);
    Wire.begin();
    Wire.setClock(400000);

    // Make sure GPS is OFF at boot
    gnss_power(false);

    // Start the periodic wake timer
    api_timer_restart(SEND_INTERVAL_MS);

    LOG("init_app: done, entering sleep until first wake");
    return true;
}

/**
 * app_event_handler — called by WisBlock-API-V2 on each wake event.
 *
 * This is where the magic happens:
 *   wake → GPS on → fix → build payload → send → GPS off → sleep
 */
void app_event_handler(void)
{
    // STATUS event = periodic timer fired
    if ((g_task_event_type & STATUS) == STATUS)
    {
        g_task_event_type &= N_STATUS;

        if (!g_lpwan_has_joined)
        {
            LOG("Not joined yet, skipping this cycle");
            return;
        }

        LOG("--- Wake cycle start ---");
        uint32_t cycle_start = millis();

        payload_t payload;
        build_payload(payload);

        lmh_error_status result = send_lora_packet(
            (uint8_t *)&payload, sizeof(payload),
            g_lorawan_settings.app_port
        );

        uint32_t cycle_ms = millis() - cycle_start;

        switch (result)
        {
        case LMH_SUCCESS:
            LOG("Uplink enqueued OK  (cycle took %lu ms)", (unsigned long)cycle_ms);
            break;
        case LMH_BUSY:
            LOG("LoRa BUSY  (cycle took %lu ms)", (unsigned long)cycle_ms);
            break;
        case LMH_ERROR:
            LOG("LoRa ERROR  (cycle took %lu ms)", (unsigned long)cycle_ms);
            break;
        }

        LOG("--- Returning to deep sleep for %lu ms ---\n",
            (unsigned long)SEND_INTERVAL_MS);
    }
}

void lora_data_handler(void)
{
    // Join result
    if ((g_task_event_type & LORA_JOIN_FIN) == LORA_JOIN_FIN)
    {
        g_task_event_type &= N_LORA_JOIN_FIN;
        if (g_join_result)
        {
            LOG("Joined LoRaWAN network");
        }
        else
        {
            LOG("LoRaWAN join FAILED — check keys and gateway");
        }
    }

    // Downlink — adjust send interval
    if ((g_task_event_type & LORA_DATA) == LORA_DATA)
    {
        g_task_event_type &= N_LORA_DATA;
        LOG("Downlink received, len=%u", g_rx_data_len);

        if (g_rx_data_len == 2)
        {
            uint16_t minutes = (uint16_t)g_rx_lora_data[0] |
                              ((uint16_t)g_rx_lora_data[1] << 8);
            if (minutes > 0)
            {
                uint32_t new_interval = (uint32_t)minutes * 60UL * 1000UL;
                api_timer_restart(new_interval);
                LOG("Interval changed to %u minutes via downlink", minutes);
            }
        }
        else if (g_rx_data_len == 4)
        {
            uint32_t seconds = (uint32_t)g_rx_lora_data[0] |
                              ((uint32_t)g_rx_lora_data[1] << 8) |
                              ((uint32_t)g_rx_lora_data[2] << 16) |
                              ((uint32_t)g_rx_lora_data[3] << 24);
            if (seconds > 0)
            {
                uint32_t new_interval = seconds * 1000UL;
                api_timer_restart(new_interval);
                LOG("Interval changed to %lu seconds via downlink", (unsigned long)seconds);
            }
        }
    }

    // TX finished
    if ((g_task_event_type & LORA_TX_FIN) == LORA_TX_FIN)
    {
        g_task_event_type &= N_LORA_TX_FIN;
        LOG("TX finished %s", g_rx_fin_result ? "ACK" : "NAK");
    }
}

void ble_data_handler(void) __attribute__((weak));
void ble_data_handler(void) {}
