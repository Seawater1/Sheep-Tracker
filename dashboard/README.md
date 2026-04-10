# Sheep Tracker Test Dashboard

This is a simple dashboard for testing the tracker flow with TTN.

## What it does

- shows the latest animal position on a map
- shows battery, altitude, and update time
- accepts pasted TTN webhook JSON
- polls the local Sheep Tracker API for live updates
- uses `uplink_message.decoded_payload` from TTN

## Recommended TTN flow

1. Tracker sends binary payload to TTN.
2. TTN uplink formatter decodes the payload.
3. TTN webhook sends JSON to the Sheep Tracker backend.
4. The frontend map polls the backend and shows the latest dog or sheep location.

## Local testing

You can open `dashboard/index.html` in a browser for a quick test.

For a cleaner local run, serve the repo folder:

```bash
python3 -m http.server 8000
```

Then open:

```text
http://localhost:8000/dashboard/
```

## Live local server

For the full TTN bridge locally, run:

```bash
python3 server/tracker_api.py
```

Then open:

```text
http://localhost:8781/dashboard/
```

## Expected TTN payload shape

The dashboard expects fields like:

```json
{
  "uplink_message": {
    "decoded_payload": {
      "batt_mv": 4429,
      "lat_e7": 515542345,
      "lon_e7": -97762345,
      "alt_m": 48,
      "gps_unix": 1775765430
    }
  }
}
```
