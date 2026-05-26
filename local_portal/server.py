"""
Local buoy dashboard.

Setup: documentation/local-portal.md

  cd local_portal
  pip install -r requirements.txt
  python server.py

Open http://127.0.0.1:8080/

Buoy (LTE): ngrok http 8080, then set telemetryUrl in secrets.h to:
  https://YOUR-ID.ngrok-free.app/api/ingest
"""

from __future__ import annotations

import json
import os
import sqlite3
import time
from pathlib import Path

from flask import Flask, jsonify, request, send_from_directory

APP_DIR = Path(__file__).resolve().parent
DB_PATH = APP_DIR / "buoy.db"
STATIC = APP_DIR / "static"

app = Flask(__name__, static_folder=str(STATIC))


def db_conn():
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    return conn


def init_db():
    with db_conn() as conn:
        conn.execute(
            """
            CREATE TABLE IF NOT EXISTS readings (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                device_id TEXT NOT NULL,
                received_at REAL NOT NULL,
                fix INTEGER, rtk TEXT, sats INTEGER,
                lat REAL, lon REAL, alt_m REAL,
                bus_v REAL, power_mw REAL, rssi INTEGER, ntrip INTEGER,
                raw_json TEXT NOT NULL
            )
            """
        )


def save_payload(payload: dict) -> str:
    device_id = str(payload.get("id") or payload.get("device_id") or "unknown")

    def num(k):
        v = payload.get(k)
        return float(v) if v is not None else None

    def integer(k):
        v = payload.get(k)
        return int(v) if v is not None else None

    row = {
        "device_id": device_id,
        "received_at": time.time(),
        "fix": integer("fix"),
        "rtk": str(payload["rtk"]) if payload.get("rtk") is not None else None,
        "sats": integer("sats"),
        "lat": num("lat"),
        "lon": num("lon"),
        "alt_m": num("alt_m"),
        "bus_v": num("bus_v"),
        "power_mw": num("power_mw"),
        "rssi": integer("rssi"),
        "ntrip": 1 if payload.get("ntrip") in (1, True, "1") else 0 if payload.get("ntrip") is not None else None,
        "raw_json": json.dumps(payload),
    }
    with db_conn() as conn:
        conn.execute(
            """
            INSERT INTO readings (
                device_id, received_at, fix, rtk, sats, lat, lon, alt_m,
                bus_v, power_mw, rssi, ntrip, raw_json
            ) VALUES (
                :device_id, :received_at, :fix, :rtk, :sats, :lat, :lon, :alt_m,
                :bus_v, :power_mw, :rssi, :ntrip, :raw_json
            )
            """,
            row,
        )
    return device_id


@app.route("/")
def index():
    return send_from_directory(STATIC, "index.html")


@app.route("/api/ingest", methods=["POST"])
def ingest():
    payload = request.get_json(silent=True)
    if not isinstance(payload, dict):
        return jsonify({"ok": False, "error": "JSON object required"}), 400
    device_id = save_payload(payload)
    return jsonify({"ok": True, "device_id": device_id})


@app.route("/api/latest")
def latest():
    with db_conn() as conn:
        rows = conn.execute(
            """
            SELECT r.* FROM readings r
            INNER JOIN (
                SELECT device_id, MAX(received_at) AS t FROM readings GROUP BY device_id
            ) x ON r.device_id = x.device_id AND r.received_at = x.t
            ORDER BY r.received_at DESC
            """
        ).fetchall()
    out = []
    for row in rows:
        d = dict(row)
        d["received"] = time.strftime(
            "%Y-%m-%d %H:%M:%S", time.localtime(d["received_at"])
        )
        out.append(d)
    return jsonify(out)


if __name__ == "__main__":
    init_db()
    port = int(os.environ.get("PORT", "8080"))
    print(f"Dashboard:  http://127.0.0.1:{port}/")
    print(f"Ingest:      http://127.0.0.1:{port}/api/ingest")
    print("Buoy (LTE): ngrok http", port)
    app.run(host="0.0.0.0", port=port, debug=True)
