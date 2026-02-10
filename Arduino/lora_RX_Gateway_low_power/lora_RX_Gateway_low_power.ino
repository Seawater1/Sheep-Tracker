#include <Arduino.h>
#include <Wire.h>
#include <WisBlock-API-V2.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

// ===== Debug =====
#ifndef MY_DEBUG
#define MY_DEBUG 0
#endif

#if MY_DEBUG > 0
#define MYLOG(tag, ...)                     \
	do                                      \
	{                                       \
		if (tag)                            \
			PRINTF("[%s] ", tag);           \
		PRINTF(__VA_ARGS__);                \
		PRINTF("\n");                       \
		if (g_ble_uart_is_connected)        \
		{                                   \
			g_ble_uart.printf(__VA_ARGS__); \
			g_ble_uart.printf("\n");        \
		}                                   \
	} while (0)
#else
#define MYLOG(...)
#endif

// ===== Firmware Version =====
#define SW_VERSION_1 1
#define SW_VERSION_2 0
#define SW_VERSION_3 0

// ===== Hardware =====
// RAK19007 (base), RAK11310 (core), RAK12500 (GNSS)
#define PIN_VBAT WB_A0
#define PIN_SOLAR WB_A1
#define GNSS_PWR_PIN WB_IO2

// Battery/solar divider calibration (adjust to your hardware)
#define VBAT_MV_PER_LSB 0.806F
#define VBAT_DIVIDER_COMP 1.846F
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)

#define VSOLAR_MV_PER_LSB 0.806F
#define VSOLAR_DIVIDER_COMP 1.846F
#define REAL_VSOLAR_MV_PER_LSB (VSOLAR_DIVIDER_COMP * VSOLAR_MV_PER_LSB)

// GNSS fix timeout (ms)
#define GNSS_FIX_TIMEOUT_MS 15000UL

// Default send interval: once per day
#define DEFAULT_SEND_INTERVAL_MS (24UL * 60UL * 60UL * 1000UL)

// ===== Globals =====
SFE_UBLOX_GNSS g_gnss;
bool g_gnss_ready = false;
uint32_t g_send_interval_ms = DEFAULT_SEND_INTERVAL_MS;

struct __attribute__((packed)) payload_t
{
	uint16_t batt_mv;
	uint16_t solar_mv;
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
	pinMode(GNSS_PWR_PIN, OUTPUT);
	digitalWrite(GNSS_PWR_PIN, on ? HIGH : LOW);
}

static bool gnss_start(void)
{
	if (g_gnss_ready)
	{
		return true;
	}
	g_gnss_ready = g_gnss.begin();
	if (g_gnss_ready)
	{
		g_gnss.setI2COutput(COM_TYPE_UBX);
		g_gnss.setAutoPVT(true);
	}
	return g_gnss_ready;
}

static void build_payload(payload_t &p)
{
	p.batt_mv = read_mv(PIN_VBAT, REAL_VBAT_MV_PER_LSB);
	p.solar_mv = read_mv(PIN_SOLAR, REAL_VSOLAR_MV_PER_LSB);

	p.lat_e7 = 0;
	p.lon_e7 = 0;
	p.alt_m = 0;
	p.gps_unix = 0;

	gnss_power(true);
	delay(100);
	if (gnss_start())
	{
		uint32_t start = millis();
		while (millis() - start < GNSS_FIX_TIMEOUT_MS)
		{
			if (g_gnss.getPVT() && g_gnss.getSIV() >= 4)
			{
				p.lat_e7 = g_gnss.getLatitude();
				p.lon_e7 = g_gnss.getLongitude();
				p.alt_m = (int16_t)(g_gnss.getAltitude() / 1000);
				p.gps_unix = g_gnss.getUnixEpoch();
				break;
			}
			delay(250);
		}
	}
	gnss_power(false);
}

// ===== Required by WisBlock-API-V2 =====
void setup_app(void)
{
	Serial.begin(115200);
	time_t serial_timeout = millis();
	while (!Serial)
	{
		if ((millis() - serial_timeout) < 5000)
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

	api_set_version(SW_VERSION_1, SW_VERSION_2, SW_VERSION_3);

	// Read credentials and set default send interval + port
	api_read_credentials();
	g_lorawan_settings.send_repeat_time = DEFAULT_SEND_INTERVAL_MS;
	g_lorawan_settings.app_port = 1;
	g_lorawan_settings.otaa_enabled = true;
	g_lorawan_settings.adr_enabled = true;
	g_lorawan_settings.auto_join = true;
	g_lorawan_settings.confirmed_msg_enabled = LMH_UNCONFIRMED_MSG;
	api_set_credentials();

	g_send_interval_ms = g_lorawan_settings.send_repeat_time;
}

bool init_app(void)
{
	MYLOG("APP", "init_app");

	analogReadResolution(12);

	Wire.begin();
	Wire.setClock(400000);

	gnss_power(false);

	// Start application wakeup timer
	api_timer_restart(g_send_interval_ms);
	return true;
}

void app_event_handler(void)
{
	if ((g_task_event_type & STATUS) == STATUS)
	{
		g_task_event_type &= N_STATUS;

		if (!g_lpwan_has_joined)
		{
			MYLOG("APP", "Not joined, skip send");
			return;
		}

		payload_t payload;
		build_payload(payload);

		lmh_error_status result = send_lora_packet((uint8_t *)&payload, sizeof(payload), g_lorawan_settings.app_port);
		switch (result)
		{
		case LMH_SUCCESS:
			MYLOG("APP", "Packet enqueued");
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
	// LoRa Join finished handling
	if ((g_task_event_type & LORA_JOIN_FIN) == LORA_JOIN_FIN)
	{
		g_task_event_type &= N_LORA_JOIN_FIN;
		if (g_join_result)
		{
			MYLOG("APP", "Joined LoRaWAN");
		}
		else
		{
			MYLOG("APP", "Join failed");
		}
	}

	// Downlink handling
	if ((g_task_event_type & LORA_DATA) == LORA_DATA)
	{
		g_task_event_type &= N_LORA_DATA;
		MYLOG("APP", "Downlink len=%u", g_rx_data_len);

		// Downlink format (little-endian):
		// - 2 bytes: sleep minutes (uint16)
		// - 4 bytes: sleep seconds (uint32)
		if (g_rx_data_len == 2)
		{
			uint16_t minutes = (uint16_t)g_rx_lora_data[0] | ((uint16_t)g_rx_lora_data[1] << 8);
			if (minutes > 0)
			{
				g_send_interval_ms = (uint32_t)minutes * 60UL * 1000UL;
				api_timer_restart(g_send_interval_ms);
				MYLOG("APP", "Set interval to %u minutes", minutes);
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
				g_send_interval_ms = seconds * 1000UL;
				api_timer_restart(g_send_interval_ms);
				MYLOG("APP", "Set interval to %u seconds", seconds);
			}
		}
	}

	// LoRa TX finished handling
	if ((g_task_event_type & LORA_TX_FIN) == LORA_TX_FIN)
	{
		g_task_event_type &= N_LORA_TX_FIN;
		MYLOG("APP", "TX finished %s", g_rx_fin_result ? "ACK" : "NAK");
	}
}

void ble_data_handler(void) __attribute__((weak));
void ble_data_handler(void) {}
