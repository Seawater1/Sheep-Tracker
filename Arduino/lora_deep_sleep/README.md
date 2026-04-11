## Deep Sleep Battery Test

This sketch is the low-duty-cycle battery-test firmware for the Sheep Tracker.

Current intended use:
- wake once every 24 hours
- power GNSS for up to 60 seconds
- send one LoRaWAN uplink with battery voltage
- include GPS fix data if available
- return to deep sleep

Current payload:
- `batt_mv`
- `lat_e7`
- `lon_e7`
- `alt_m`
- `gps_unix`
- `ttff_ds`

This is the firmware currently used for the two-week battery consumption test.
