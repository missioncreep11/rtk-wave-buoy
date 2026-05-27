# Local telemetry portal

Buoy → PC live map without GitHub:

```
Buoy (LTE) --plain TCP--> ngrok --> local_portal on your PC --> browser
```

Plain TCP (not HTTPS): the SIM7000A on this hardware ships with firmware
**B03**, whose HTTPS AT-command stacks (`AT+SH*`, `AT+HTTP*+HTTPSSL`, `AT+CAOPEN`)
all fail once an NTRIP socket has been opened. Plain TCP is reliable on B03
(NTRIP itself proves this). See [troubleshooting.md](troubleshooting.md) for
the diagnostic trail. The firmware sends a hand-built `HTTP/1.1 POST` over a
raw `CIPSTART` socket; ngrok TCP simply forwards bytes to local Flask.

Code: [`local_portal/`](../local_portal/).

## 1. Run the dashboard

```powershell
cd local_portal
pip install -r requirements.txt
python server.py
```

Open **http://127.0.0.1:8080/**

To require an auth header from the buoy, set `BUOY_SECRET` before launching:

```powershell
$env:BUOY_SECRET = "YE6nwNY$c6jKpL!0nx0c5TtH"
python server.py
```

The server then requires `X-Buoy-Secret: <BUOY_SECRET>` on every POST.

Endpoints:

| Route | Method | Purpose |
|-------|--------|---------|
| `/` | GET | Dashboard (`static/index.html`) |
| `/api/ingest` | POST | Buoy JSON payload |
| `/api/latest` | GET | Latest reading per device (SQLite) |

## 2. Test without hardware

```powershell
cd local_portal
.\test_ingest.ps1
```

Refresh the browser — map and metrics should update.

## 3. Expose to the buoy (ngrok TCP)

1. Install [ngrok](https://ngrok.com/) (agent **3.20+**; run `ngrok update` if winget shipped an old build).
2. `ngrok config add-authtoken YOUR_TOKEN` from the ngrok dashboard.
3. With `server.py` running, in another terminal:

   ```powershell
   cd local_portal
   .\start-ngrok.ps1 tcp 8080
   ```

4. ngrok prints a line like:

   ```
   Forwarding   tcp://0.tcp.ngrok.io:14723 -> localhost:8080
   ```

   On the free tier this address changes every session.

## 4. Configure firmware

In `esp32/buoy_combo/secrets.h`, paste the ngrok host:port into `telemetryUrl`,
keeping the `tcp://` scheme and `/api/ingest` path:

```cpp
const char *telemetryUrl    = "tcp://0.tcp.ngrok.io:14723/api/ingest";
const char *telemetrySecret = "YE6nwNY$c6jKpL!0nx0c5TtH";  // must match BUOY_SECRET
```

Leave `hologramDeviceKey` empty.

Flash `buoy_combo`. Serial every ~60 s (with GPRS):

- `[TELEM] HTTP POST...`
- `[TELEM] POST OK`

The firmware closes the NTRIP socket briefly for each POST, then the main
loop reconnects NTRIP. Expect a 2–5 s gap in `[RTCM]` bytes around telemetry.

## 5. What about HTTPS / Hologram cloud / GitHub Pages?

| Path | Status | Why |
|------|--------|-----|
| `https://...` direct from buoy | **Broken** | SIM7000A B03 firmware. Re-enable after a B05+ swap. |
| Hologram Cloud Socket (CSR device key) | **Blocked** | Post-Routes account; no CSR device key issued. |
| Hologram Outbound Webhook → Cloudflare → GitHub Pages | **Possible** | See [github-pages.md](github-pages.md) Flow B — uses Hologram's server-side outbound webhook to forward TCP telemetry to a Cloudflare Worker, bypassing the modem HTTPS issue entirely. |
| Cloudflare Worker → GitHub Pages (direct HTTPS from buoy) | **On hold** | Needs B05+ firmware for working HTTPS from the modem. Worker config kept in `secrets.h` comments. |
| **ngrok TCP → local Flask** | **Working** | This document. |

## Testing checklist

| Step | Check |
|------|-------|
| A | `test_ingest.ps1` updates local dashboard |
| B | ngrok host:port in `secrets.h`, `[TELEM] POST OK` on serial |
| C | Browser map updates after buoy POST |

Public dashboard without a PC: [github-pages.md](github-pages.md) — see Flow B (Hologram Outbound Webhook) for a path that works on B03 firmware today, or Flow A (direct HTTPS) after a B05+ upgrade.
