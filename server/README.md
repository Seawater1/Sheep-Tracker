# Sheep Tracker Server

Small Python backend for live TTN ingest and map APIs.

## Endpoints

- `POST /ttn/uplink` — TTN webhook target
- `GET /api/latest` — latest tracker point
- `GET /api/latest?device=dog-collar-1` — latest point for one device
- `GET /api/history?device=dog-collar-1&limit=100` — recent track points
- `GET /api/health` — health check
- `GET /dashboard/` — serves the test dashboard

## Run locally

```bash
python3 server/tracker_api.py
```

Then open:

```text
http://localhost:8781/dashboard/
```

## TTN webhook target

Set the TTN webhook URL to:

```text
http://YOUR-SERVER:8781/ttn/uplink
```

Use the uplink formatter in:

`ttn/uplink_formatter.js`

The backend expects `uplink_message.decoded_payload`.
