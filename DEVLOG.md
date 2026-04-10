# Sheep Tracker — Dev Log

## 2026-04-09 — First Successful LoRaWAN Uplink

### What was done

- Removed all P2P (point-to-point) LoRa sketches. Project is LoRaWAN only.
- Finalised `lora_RX_Gateway` as the main firmware sketch for the RAK11300 tracker.
- Compiled and flashed firmware to `rak11300-tracker-1`.
- Device joined TTN (OTAA) and began sending uplinks on first boot.
- Confirmed uplinks received by TTN on `sheep-tracker1` application, eu1.

### TTN Setup

| Item | Value |
|---|---|
| Server | eu1.cloud.thethings.network |
| Application | `sheep-tracker1` |
| Gateway | `allihies` (Connected) |
| Device | `rak11300-tracker-1` |
| DevEUI | `70B3D57ED0076C9F` |
| LoRaWAN version | 1.0.2 (RP001 1.0.2 rev B) |
| Frequency plan | Europe 863–870 MHz (SF9 for RX2) |

### Firmware — `lora_RX_Gateway`

**Hardware:** RAK19007 base + RAK11310 core + RAK12500 GNSS (u-blox)

**Payload (packed struct, little-endian, 16 bytes):**

| Bytes | Field | Type | Notes |
|---|---|---|---|
| 0–1 | `batt_mv` | uint16 | Battery mV |
| 2–5 | `lat_e7` | int32 | Latitude × 1e7 (0 if no fix) |
| 6–9 | `lon_e7` | int32 | Longitude × 1e7 (0 if no fix) |
| 10–11 | `alt_m` | int16 | Altitude in metres |
| 12–15 | `gps_unix` | uint32 | Unix timestamp from GPS |

**Downlink (port 2) — adjust send interval:**
- 2 bytes: interval in minutes (uint16, little-endian)
- 4 bytes: interval in seconds (uint32, little-endian)

Default send interval: 60 seconds (for testing). Increase to 5–15 min in production.

**GPS fix threshold:** requires ≥ 4 satellites (SIV). Sends zeros if no fix.

### First readings (serial monitor, indoors)

```
lmh_send=0, batt=4581
Uplink sent (unconfirmed)
lmh_send=0, batt=4429
```

- `lmh_send=0` = success
- Battery ~4.58 V
- GPS zeros expected indoors — needs outdoor test for fix

### Toolchain notes

**Board:** `rakwireless:mbed_rp2040:WisCoreRAK11300Board`  
**arduino-cli:** v1.4.1 at `/home/brian/bin/arduino-cli`  
**Libraries:** `/home/brian/Documents/sheep_tracker/Arduino/libraries/`

**Compiling:**
```bash
/home/brian/bin/arduino-cli compile \
  --fqbn rakwireless:mbed_rp2040:WisCoreRAK11300Board \
  --libraries /home/brian/Documents/sheep_tracker/Arduino/libraries \
  Arduino/lora_RX_Gateway
```

**Flashing (UF2 method — arduino-cli upload does not work for this board):**
1. Double-tap RESET on the RAK11300 to enter bootloader mode.
2. `RPI-RP2` appears as a USB drive (e.g. `/media/brian/RPI-RP2`).
3. Copy the UF2:
```bash
cp ~/.cache/arduino/sketches/<hash>/lora_RX_Gateway.ino.elf.uf2 /media/brian/RPI-RP2/
```
The device reboots and runs the new firmware automatically.

**Serial monitoring (arduino-cli monitor has exec format error on this machine — use pyserial):**
```bash
python3 -c "
import serial, time
s = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
start = time.time()
while time.time() - start < 60:
    line = s.readline()
    if line: print(line.decode('utf-8', errors='replace').rstrip(), flush=True)
s.close()
"
```

---

## What's next

- [ ] TTN payload decoder (JavaScript) — parse binary payload in TTN console
- [ ] Outdoor GPS fix test
- [ ] Increase send interval via downlink once confirmed working
- [ ] Website / farmer dashboard
- [ ] Mobile app
- [ ] Second device (`taylor`, DevEUI `70B3D57ED0076AC4`) — flash and test
- [ ] Low-power firmware (`lora_RX_Gateway_low_power`) — deploy once basic loop is stable
