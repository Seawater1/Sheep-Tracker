#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

// =========================
// Settings
// =========================
#define BATTERY_DEBUG 0   // 0 = quiet (recommended), 1 = print ADC + voltage debug
#define PRINT_DEGMIN  0   // 0 = off, 1 = also print deg/min line

// =========================
// GNSS
// =========================
SFE_UBLOX_GNSS g_myGNSS;
unsigned long g_lastTime = 0;

// =========================
// Battery (math left unchanged)
// =========================
#define PIN_VBAT WB_A0
uint32_t vbat_pin = PIN_VBAT;

#define VBAT_MV_PER_LSB (0.806F)
#define VBAT_DIVIDER (0.6F)
#define VBAT_DIVIDER_COMP (1.846F)
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)

// OLED
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);

float readVBAT(void)
{
  unsigned int sum = 0, average_value = 0;
  unsigned int read_temp[10] = {0};
  unsigned char i = 0;
  unsigned int adc_max = 0;
  unsigned int adc_min = 4095;

  average_value = analogRead(vbat_pin);
  for (i = 0; i < 10; i++)
  {
    read_temp[i] = analogRead(vbat_pin);
    if (read_temp[i] < adc_min) adc_min = read_temp[i];
    if (read_temp[i] > adc_max) adc_max = read_temp[i];
    sum = sum + read_temp[i];
    delay(2);
  }

  average_value = (sum - adc_max - adc_min) >> 3;

#if BATTERY_DEBUG
  Serial.printf("The ADC value is:%d\r\n", average_value);
#endif

  float volt = average_value * REAL_VBAT_MV_PER_LSB;

#if BATTERY_DEBUG
  // Note: kept same message from your original sketch (even though 'volt' is mV by maths)
  Serial.printf("The battery voltage is: %3.2f V\r\n", volt);
#endif

  return volt; // returns mV (per your original code behaviour)
}

uint8_t mvToPercent(float mvolts)
{
  if (mvolts < 3300) return 0;

  if (mvolts < 3600)
  {
    mvolts -= 3300;
    return (uint8_t)(mvolts / 30);
  }

  mvolts -= 3600;
  return (uint8_t)(10 + (mvolts * 0.15F));
}

uint8_t mvToLoRaWanBattVal(float mvolts)
{
  if (mvolts < 3300) return 0;

  if (mvolts < 3600)
  {
    mvolts -= 3300;
    return (uint8_t)((mvolts / 30) * 2.55F);
  }

  mvolts -= 3600;
  return (uint8_t)((10 + (mvolts * 0.15F)) * 2.55F);
}

#if PRINT_DEGMIN
void printDegMin(double deg, bool isLat)
{
  char hemi;
  if (isLat) hemi = (deg >= 0) ? 'N' : 'S';
  else       hemi = (deg >= 0) ? 'E' : 'W';

  deg = fabs(deg);
  int d = (int)deg;
  double minutes = (deg - d) * 60.0;

  Serial.print(d);
  Serial.print("° ");
  Serial.print(minutes, 4);
  Serial.print("' ");
  Serial.print(hemi);
}
#endif

void setup()
{
  time_t timeout = millis();
  Serial.begin(115200);
  while (!Serial)
  {
    if ((millis() - timeout) < 5000) delay(100);
    else break;
  }

  Serial.println("=====================================");
  Serial.println("RAK11300 GNSS + Battery (clean print)");
  Serial.println("=====================================");

  analogReadResolution(12);
  delay(10);

  // GNSS power reset FIRST
  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, 0);
  delay(1000);
  digitalWrite(WB_IO2, 1);
  delay(1500);

  // I2C AFTER GNSS power-up
  Wire.begin();

  // GNSS init
  if (g_myGNSS.begin() == false)
  {
    Serial.println("u-blox GNSS not detected at default I2C address. Freezing.");
    while (1) { delay(100); }
  }

  g_myGNSS.setI2COutput(COM_TYPE_UBX);
  g_myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
  g_myGNSS.setAutoPVT(true);
  g_myGNSS.setNavigationFrequency(1);

  // OLED after GNSS
  u8g2.begin();

  // Throw away one battery read
  (void)readVBAT();
}

void loop()
{
  if (millis() - g_lastTime >= 1000)
  {
    g_lastTime = millis();

    // Get fresh GNSS solution
    bool gotPVT = g_myGNSS.getPVT();

    long latitude  = gotPVT ? g_myGNSS.getLatitude() : 0;
    long longitude = gotPVT ? g_myGNSS.getLongitude() : 0;
    long altitude  = gotPVT ? g_myGNSS.getAltitude() : 0;
    long speed     = gotPVT ? g_myGNSS.getGroundSpeed() : 0;
    long heading   = gotPVT ? g_myGNSS.getHeading() : 0;
    byte SIV       = g_myGNSS.getSIV();

    // Convert to human units
    double lat_deg = latitude  / 10000000.0;
    double lon_deg = longitude / 10000000.0;
    double alt_m   = altitude  / 1000.0;
    double spd_kmh = (speed / 1000.0) * 3.6;
    double hdg_deg = heading / 100000.0;

    // Battery (your code)
    float vbat_mv = readVBAT();
    uint8_t vbat_per = mvToPercent(vbat_mv);
    uint8_t loraBatt = mvToLoRaWanBattVal(vbat_mv);

    // ONE clean print line per second (like before)
    Serial.print("Lat: "); Serial.print(lat_deg, 7);
    Serial.print("  Lon: "); Serial.print(lon_deg, 7);
    Serial.print("  Alt(m): "); Serial.print(alt_m, 1);
    Serial.print("  Speed(km/h): "); Serial.print(spd_kmh, 1);
    Serial.print("  Heading(deg): "); Serial.print(hdg_deg, 1);
    Serial.print("  SIV: "); Serial.print(SIV);
    Serial.print("  | Battery: "); Serial.print(vbat_mv / 1000.0, 3);
    Serial.print(" V ("); Serial.print(vbat_per); Serial.print("%)");
    Serial.print("  LoRaBatt: "); Serial.println(loraBatt);

#if PRINT_DEGMIN
    Serial.print("Lat (deg/min): ");
    printDegMin(lat_deg, true);
    Serial.print("   Lon (deg/min): ");
    printDegMin(lon_deg, false);
    Serial.println();
#endif

    // OLED: simple, readable
    char line1[32], line2[32], line3[32], line4[32];
    snprintf(line1, sizeof(line1), "Bat: %.3fV %u%%", vbat_mv / 1000.0, vbat_per);
    snprintf(line2, sizeof(line2), "Lat: %.5f", lat_deg);
    snprintf(line3, sizeof(line3), "Lon: %.5f", lon_deg);
    snprintf(line4, sizeof(line4), "SIV:%u Spd:%.1f", SIV, spd_kmh);

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, 12, line1);
    u8g2.drawStr(0, 28, line2);
    u8g2.drawStr(0, 44, line3);
    u8g2.drawStr(0, 60, line4);
    u8g2.sendBuffer();
  }
}
