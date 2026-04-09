#!/usr/bin/env python3
"""Sheep Tracker API.

Receives TTN webhook uplinks and serves the latest tracker positions to the
dashboard. Uses only the Python standard library so deployment stays simple.
"""

from __future__ import annotations

import json
import os
import sqlite3
import sys
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse


HTTP_PORT = int(os.environ.get("SHEEP_TRACKER_PORT", "8781"))
ROOT_DIR = Path(__file__).resolve().parent.parent
DB_PATH = Path(os.environ.get("SHEEP_TRACKER_DB", ROOT_DIR / "server" / "tracker.db"))
STATIC_DIR = Path(os.environ.get("SHEEP_TRACKER_STATIC_DIR", ROOT_DIR / "dashboard"))


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def init_db() -> None:
    DB_PATH.parent.mkdir(parents=True, exist_ok=True)
    conn = sqlite3.connect(DB_PATH)
    conn.execute(
        """
        CREATE TABLE IF NOT EXISTS tracker_positions (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            device_id TEXT NOT NULL,
            dev_eui TEXT,
            received_at TEXT NOT NULL,
            gps_unix INTEGER,
            batt_mv INTEGER,
            solar_mv INTEGER,
            lat_e7 INTEGER,
            lon_e7 INTEGER,
            alt_m INTEGER,
            raw_event_json TEXT NOT NULL
        )
        """
    )
    conn.execute(
        """
        CREATE INDEX IF NOT EXISTS idx_tracker_positions_device_time
        ON tracker_positions(device_id, received_at DESC)
        """
    )
    conn.commit()
    conn.close()


def decoded_payload_from_event(event: dict[str, Any]) -> dict[str, Any]:
    payload = event.get("uplink_message", {}).get("decoded_payload")
    if not isinstance(payload, dict):
        raise ValueError("TTN event is missing uplink_message.decoded_payload")
    return payload


def position_from_event(event: dict[str, Any]) -> dict[str, Any]:
    payload = decoded_payload_from_event(event)
    device = event.get("end_device_ids", {})
    lat_e7 = int(payload.get("lat_e7", 0) or 0)
    lon_e7 = int(payload.get("lon_e7", 0) or 0)

    return {
        "device_id": device.get("device_id", "unknown-device"),
        "dev_eui": device.get("dev_eui"),
        "received_at": event.get("received_at", utc_now()),
        "gps_unix": payload.get("gps_unix"),
        "batt_mv": payload.get("batt_mv"),
        "solar_mv": payload.get("solar_mv"),
        "lat_e7": lat_e7,
        "lon_e7": lon_e7,
        "alt_m": payload.get("alt_m"),
        "has_fix": lat_e7 != 0 or lon_e7 != 0,
        "raw_event_json": json.dumps(event),
    }


def store_event(event: dict[str, Any]) -> dict[str, Any]:
    position = position_from_event(event)
    conn = sqlite3.connect(DB_PATH)
    conn.execute(
        """
        INSERT INTO tracker_positions (
            device_id, dev_eui, received_at, gps_unix, batt_mv, solar_mv,
            lat_e7, lon_e7, alt_m, raw_event_json
        ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """,
        (
            position["device_id"],
            position["dev_eui"],
            position["received_at"],
            position["gps_unix"],
            position["batt_mv"],
            position["solar_mv"],
            position["lat_e7"],
            position["lon_e7"],
            position["alt_m"],
            position["raw_event_json"],
        ),
    )
    conn.commit()
    conn.close()
    return position


def row_to_point(row: sqlite3.Row) -> dict[str, Any]:
    lat_e7 = row["lat_e7"]
    lon_e7 = row["lon_e7"]
    return {
        "deviceId": row["device_id"],
        "devEui": row["dev_eui"],
        "animalName": row["device_id"],
        "receivedAt": row["received_at"],
        "gpsUnix": row["gps_unix"],
        "battMv": row["batt_mv"],
        "solarMv": row["solar_mv"],
        "altitude": row["alt_m"],
        "lat_e7": lat_e7,
        "lon_e7": lon_e7,
        "latitude": (lat_e7 / 1e7) if lat_e7 is not None else None,
        "longitude": (lon_e7 / 1e7) if lon_e7 is not None else None,
        "hasFix": bool((lat_e7 or 0) != 0 or (lon_e7 or 0) != 0),
    }


def fetch_latest(device_id: str | None) -> dict[str, Any] | None:
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    if device_id:
        row = conn.execute(
            """
            SELECT * FROM tracker_positions
            WHERE device_id = ?
            ORDER BY received_at DESC, id DESC
            LIMIT 1
            """,
            (device_id,),
        ).fetchone()
    else:
        row = conn.execute(
            """
            SELECT * FROM tracker_positions
            ORDER BY received_at DESC, id DESC
            LIMIT 1
            """
        ).fetchone()
    conn.close()
    return row_to_point(row) if row else None


def fetch_history(device_id: str | None, limit: int) -> list[dict[str, Any]]:
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    if device_id:
        rows = conn.execute(
            """
            SELECT * FROM tracker_positions
            WHERE device_id = ?
            ORDER BY received_at DESC, id DESC
            LIMIT ?
            """,
            (device_id, limit),
        ).fetchall()
    else:
        rows = conn.execute(
            """
            SELECT * FROM tracker_positions
            ORDER BY received_at DESC, id DESC
            LIMIT ?
            """,
            (limit,),
        ).fetchall()
    conn.close()
    points = [row_to_point(row) for row in reversed(rows)]
    return [point for point in points if point["hasFix"]]


def json_bytes(payload: dict[str, Any]) -> bytes:
    return json.dumps(payload, indent=2).encode("utf-8")


class TrackerHandler(BaseHTTPRequestHandler):
    server_version = "SheepTrackerAPI/0.1"

    def _send(self, status: int, body: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, status: int, payload: dict[str, Any]) -> None:
        self._send(status, json_bytes(payload), "application/json; charset=utf-8")

    def _send_file(self, path: Path) -> None:
        if not path.exists() or not path.is_file():
            self.send_error(404, "Not found")
            return

        suffix = path.suffix.lower()
        content_type = {
            ".html": "text/html; charset=utf-8",
            ".js": "application/javascript; charset=utf-8",
            ".css": "text/css; charset=utf-8",
            ".json": "application/json; charset=utf-8",
        }.get(suffix, "application/octet-stream")
        self._send(200, path.read_bytes(), content_type)

    def do_OPTIONS(self) -> None:
        self._send(204, b"", "text/plain; charset=utf-8")

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        query = parse_qs(parsed.query)

        if parsed.path == "/api/health":
            self._send_json(200, {"ok": True, "time": utc_now()})
            return

        if parsed.path == "/api/latest":
            device_id = query.get("device", [None])[0]
            latest = fetch_latest(device_id)
            if latest is None:
                self._send_json(404, {"error": "No tracker positions stored yet"})
            else:
                self._send_json(200, {"point": latest})
            return

        if parsed.path == "/api/history":
            device_id = query.get("device", [None])[0]
            limit = min(max(int(query.get("limit", ["50"])[0]), 1), 500)
            self._send_json(200, {"points": fetch_history(device_id, limit)})
            return

        if parsed.path == "/" or parsed.path == "/dashboard" or parsed.path == "/dashboard/":
            self._send_file(STATIC_DIR / "index.html")
            return

        if parsed.path.startswith("/dashboard/"):
            rel = parsed.path.removeprefix("/dashboard/")
            if rel == "":
                self._send_file(STATIC_DIR / "index.html")
                return
            self._send_file(STATIC_DIR / rel)
            return

        self.send_error(404, "Not found")

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path != "/ttn/uplink":
            self.send_error(404, "Not found")
            return

        try:
            length = int(self.headers.get("Content-Length", "0"))
            raw = self.rfile.read(length) if length > 0 else b"{}"
            event = json.loads(raw.decode("utf-8"))
            stored = store_event(event)
        except json.JSONDecodeError:
            self._send_json(400, {"error": "Request body must be valid JSON"})
            return
        except ValueError as exc:
            self._send_json(400, {"error": str(exc)})
            return
        except Exception as exc:  # pragma: no cover - operational guard
            self._send_json(500, {"error": f"Failed to store event: {exc}"})
            return

        self._send_json(
            200,
            {
                "ok": True,
                "device_id": stored["device_id"],
                "has_fix": stored["has_fix"],
                "received_at": stored["received_at"],
            },
        )

    def log_message(self, fmt: str, *args: Any) -> None:
        sys.stdout.write("%s - - [%s] %s\n" % (self.client_address[0], self.log_date_time_string(), fmt % args))


def main() -> None:
    init_db()
    server = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), TrackerHandler)
    print(f"Sheep Tracker API listening on http://0.0.0.0:{HTTP_PORT}")
    print(f"Dashboard served from {STATIC_DIR}")
    server.serve_forever()


if __name__ == "__main__":
    main()
