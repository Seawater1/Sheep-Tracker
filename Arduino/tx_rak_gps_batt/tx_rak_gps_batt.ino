#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#define API_DEBUG 1
#include <WisBlock-API-V2.h>

// ===== Debug =====
#ifndef MY_DEBUG
#define MY_DEBUG 1
#endif

#if MY_DEBUG > 0
#define MYLOG(tag, ...)                     \
  do                                        \
  {                                         \
    if (tag)                                \
    {                                       \
      Serial.printf("[%s] ", tag);          \
    }                                       \
    Serial.printf(__VA_ARGS__);             \
    Serial.printf("\n");                    \
  } while (0)
#else
#define MYLOG(...)
#endif

// ===== Firmware version =====
#define SW_VERSION_1 3
#define SW_VERSION_2 0
#define SW_VERSION_3 0

// ===== Power-first tracker configuration =====
#define GNSS_PWR_PIN WB_IO2
#define PIN_VBAT WB_A0

// Reporting every 30 minutes is a much better starting point for battery life
// than the old seconds-based test sketches. Increase this further if needed.
#define DEFAULT_SEND_INTERVAL_MS (1UL * 60UL * 1000UL)
#define LOW_BATT_SEND_INTERVAL_MS (2UL * 60UL * 60UL * 1000UL)

// Keep the GNSS window bounded so cold starts cannot burn the battery forever.
#define GNSS_POWER_SETTLE_MS 1500UL
#define GNSS_FIX_TIMEOUT_MS 30000UL
#define GNSS_POLL_MS 250UL

#define LOW_BATTERY_MV 3450U
#define APP_PORT 1

#define PAYLOAD_VERSION 1
#define PAYLOAD_FLAG_FIX 0x01
#define PAYLOAD_FLAG_LOW_BATT 0x02

// ===== OTAA credentials =====
// Replace these with the real values from your LoRaWAN network server.
static const uint8_t OTAA_DEV_EUI[8] = {
  0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x07, 0x6A, 0xC4};
static const uint8_t OTAA_APP_EUI[8] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t OTAA_APP_KEY[16] = {
  0x42, 0xAE, 0x32, 0x32, 0xC6, 0x26, 0xE8, 0x87,
  0xC4, 0x93, 0x61, 0x9F, 0xC4, 0xEE, 0xC2, 0x37};

// ===== Battery calibration =====
#define VBAT_MV_PER_LSB 0.806F
#define VBAT_DIVIDER_COMP 1.846F
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)

// ===== Application state =====
SFE_UBLOX_GNSS g_gnss;
bool g_low_batt_mode = false;
uint32_t g_send_interval_ms = DEFAULT_SEND_INTERVAL_MS;
uint8_t g_sequence = 0;
char g_ble_dev_name[10] = "SHEEP-TX";

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

// The bundled WisBlock flash backends do not line up cleanly with the
// installed RAK11300 core on this machine. For this tracker build we keep
// the runtime config in code and stub persistence so the device can boot,
// join and transmit with the hard-coded OTAA values above.
void init_flash(void)
{
  init_flash_done = true;
}

bool save_settings(void)
{
  return true;
}

void flash_reset(void)
{
  g_lorawan_settings = s_lorawan_settings();
  init_flash_done = true;
}

static uint16_t read_batt_mv(void)
{
  unsigned int sum = 0;
  unsigned int adc_max = 0;
  unsigned int adc_min = 4095;

  for (uint8_t i = 0; i < 10; i++)
  {
    unsigned int value = analogRead(PIN_VBAT);
    if (value < adc_min) adc_min = value;
    if (value > adc_max) adc_max = value;
    sum += value;
    delay(2);
  }

  const unsigned int avg = (sum - adc_max - adc_min) >> 3;
  return (uint16_t)(avg * REAL_VBAT_MV_PER_LSB);
}

static void gnss_power(bool on)
{
  pinMode(GNSS_PWR_PIN, OUTPUT);
  digitalWrite(GNSS_PWR_PIN, on ? HIGH : LOW);
}

static bool fill_fix_data(tracker_payload_t &payload)
{
  gnss_power(true);
  delay(GNSS_POWER_SETTLE_MS);

  if (!g_gnss.begin())
  {
    MYLOG("GNSS", "GNSS not found");
    gnss_power(false);
    return false;
  }

  g_gnss.setI2COutput(COM_TYPE_UBX);
  g_gnss.setAutoPVT(true);
  g_gnss.setNavigationFrequency(1);

  const uint32_t start = millis();
  while (millis() - start < GNSS_FIX_TIMEOUT_MS)
  {
    if (g_gnss.getPVT())
    {
      const uint8_t sats = g_gnss.getSIV();
      if (sats >= 4)
      {
        payload.flags |= PAYLOAD_FLAG_FIX;
        payload.sats = sats;
        payload.lat_e7 = g_gnss.getLatitude();
        payload.lon_e7 = g_gnss.getLongitude();
        payload.alt_m = (int16_t)(g_gnss.getAltitude() / 1000);
        payload.gps_unix = g_gnss.getUnixEpoch();
        gnss_power(false);
        return true;
      }
    }

    delay(GNSS_POLL_MS);
  }

  MYLOG("GNSS", "No fix within timeout");
  gnss_power(false);
  return false;
}

static void configure_lorawan_defaults(void)
{
  memcpy(g_lorawan_settings.node_device_eui, OTAA_DEV_EUI, sizeof(OTAA_DEV_EUI));
  memcpy(g_lorawan_settings.node_app_eui, OTAA_APP_EUI, sizeof(OTAA_APP_EUI));
  memcpy(g_lorawan_settings.node_app_key, OTAA_APP_KEY, sizeof(OTAA_APP_KEY));

  g_lorawan_settings.lorawan_enable = true;
  g_lorawan_settings.otaa_enabled = true;
  g_lorawan_settings.adr_enabled = true;
  g_lorawan_settings.public_network = true;
  g_lorawan_settings.auto_join = true;
  g_lorawan_settings.confirmed_msg_enabled = LMH_UNCONFIRMED_MSG;
  g_lorawan_settings.app_port = APP_PORT;
  g_lorawan_settings.send_repeat_time = DEFAULT_SEND_INTERVAL_MS;

  g_send_interval_ms = g_lorawan_settings.send_repeat_time;
}

void setup_app(void)
{
#if MY_DEBUG > 0
  Serial.begin(115200);
  time_t serial_timeout = millis();
  while (!Serial)
  {
    if ((millis() - serial_timeout) < 3000)
    {
      delay(100);
    }
    else
    {
      break;
    }
  }
#endif

  Serial.println("BOOT: Sheep Tracker debug");
  api_set_version(SW_VERSION_1, SW_VERSION_2, SW_VERSION_3);
}

bool init_app(void)
{
  analogReadResolution(12);

  Wire.begin();
  Wire.setClock(400000);

  gnss_power(false);
  configure_lorawan_defaults();

  Serial.println("INIT: Low-power LoRaWAN tracker ready");
  MYLOG("APP", "Low-power LoRaWAN tracker ready");
  return true;
}

void app_event_handler(void)
{
  if ((g_task_event_type & STATUS) == STATUS)
  {
    g_task_event_type &= N_STATUS;

    if (!g_lpwan_has_joined)
    {
      Serial.println("APP: Not joined yet, skip send");
      MYLOG("APP", "Not joined yet, skip send");
      return;
    }

    tracker_payload_t payload = {};
    payload.version = PAYLOAD_VERSION;
    payload.seq = g_sequence++;
    payload.batt_mv = read_batt_mv();

    if (payload.batt_mv <= LOW_BATTERY_MV)
    {
      payload.flags |= PAYLOAD_FLAG_LOW_BATT;
      if (!g_low_batt_mode)
      {
        g_low_batt_mode = true;
        g_send_interval_ms = LOW_BATT_SEND_INTERVAL_MS;
        api_timer_restart(g_send_interval_ms);
      }
    }
    else if (g_low_batt_mode)
    {
      g_low_batt_mode = false;
      g_send_interval_ms = DEFAULT_SEND_INTERVAL_MS;
      api_timer_restart(g_send_interval_ms);
    }

    fill_fix_data(payload);

    const lmh_error_status result =
      send_lora_packet((uint8_t *)&payload, sizeof(payload), g_lorawan_settings.app_port);

    switch (result)
    {
    case LMH_SUCCESS:
      Serial.println("APP: Packet enqueued");
      MYLOG("APP", "Packet enqueued batt=%umV fix=%u sats=%u",
            payload.batt_mv,
            (payload.flags & PAYLOAD_FLAG_FIX) ? 1 : 0,
            payload.sats);
      break;
    case LMH_BUSY:
      MYLOG("APP", "LoRa busy");
      break;
    case LMH_ERROR:
      MYLOG("APP", "Packet error");
      break;
    }
  }
}

void lora_data_handler(void)
{
  if ((g_task_event_type & LORA_JOIN_FIN) == LORA_JOIN_FIN)
  {
    g_task_event_type &= N_LORA_JOIN_FIN;
    if (g_join_result)
    {
      Serial.println("LORA: Joined LoRaWAN");
      MYLOG("APP", "Joined LoRaWAN");
    }
    else
    {
      Serial.println("LORA: Join failed");
      MYLOG("APP", "Join failed");
    }
  }

  if ((g_task_event_type & LORA_DATA) == LORA_DATA)
  {
    g_task_event_type &= N_LORA_DATA;
    MYLOG("APP", "Downlink len=%u", g_rx_data_len);

    // 2-byte downlink sets minutes, 4-byte downlink sets seconds.
    if (g_rx_data_len == 2)
    {
      const uint16_t minutes =
        (uint16_t)g_rx_lora_data[0] | ((uint16_t)g_rx_lora_data[1] << 8);
      if (minutes > 0)
      {
        g_send_interval_ms = (uint32_t)minutes * 60UL * 1000UL;
        g_low_batt_mode = false;
        api_timer_restart(g_send_interval_ms);
        MYLOG("APP", "Set interval to %u minutes", minutes);
      }
    }
    else if (g_rx_data_len == 4)
    {
      const uint32_t seconds =
        (uint32_t)g_rx_lora_data[0] |
        ((uint32_t)g_rx_lora_data[1] << 8) |
        ((uint32_t)g_rx_lora_data[2] << 16) |
        ((uint32_t)g_rx_lora_data[3] << 24);
      if (seconds > 0)
      {
        g_send_interval_ms = seconds * 1000UL;
        g_low_batt_mode = false;
        api_timer_restart(g_send_interval_ms);
        MYLOG("APP", "Set interval to %u seconds", seconds);
      }
    }
  }

  if ((g_task_event_type & LORA_TX_FIN) == LORA_TX_FIN)
  {
    g_task_event_type &= N_LORA_TX_FIN;
    MYLOG("APP", "TX finished %s", g_rx_fin_result ? "ACK" : "NAK");
  }
}

void ble_data_handler(void) __attribute__((weak));
void ble_data_handler(void) {}
