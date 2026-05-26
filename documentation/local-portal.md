# Local telemetry portal

Default path for a live buoy map without GitHub:

```
Buoy (LTE) --HTTPS POST--> ngrok --> local_portal on your PC --> browser
```

Code: [`local_portal/`](../local_portal/).

## 1. Run the dashboard

```powershell
cd local_portal
pip install -r requirements.txt
python server.py
```

Open **http://127.0.0.1:8080/**

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

## 3. Expose to the buoy (ngrok)

1. Install [ngrok](https://ngrok.com/) (agent **3.20+**; run `ngrok update` if winget shipped an old build)
2. `ngrok config add-authtoken YOUR_TOKEN` from the ngrok dashboard
3. With `server.py` running, in another terminal:

   ```powershell
   cd local_portal
   .\start-ngrok.ps1
   ```

   Or: `ngrok http 8080`

4. Copy the **https** forwarding URL (changes each session on free tier)

## 4. Configure firmware

In `esp32/buoy_combo/secrets.h`:

```cpp
#define HAS_TELEMETRY_URL 1
const char *telemetryUrl = "https://YOUR-ID.ngrok-free.app/api/ingest";
```

Leave `hologramDeviceKey` empty unless you only want messages in the Hologram dashboard.

Flash `buoy_combo`. Serial every ~60 s (with GPRS):

- `[TELEM] HTTP POST...`
- `[TELEM] POST OK`

Firmware uses the modem HTTPS stack (`AT+SH*`), not the NTRIP TCP socket. NTRIP may reconnect briefly after each POST.

## 5. Optional — Hologram cloud only

Set `hologramDeviceKey` (CSR key from Hologram **Configuration → Router Credentials**) and clear `telemetryUrl`. View messages under SIM **Activity**. No local map unless you add a Hologram Route to your ngrok URL later.

## Testing checklist

| Step | Check |
|------|-------|
| A | `test_ingest.ps1` updates local dashboard |
| B | ngrok URL in `secrets.h`, `[TELEM] POST OK` on serial |
| C | Browser map updates after buoy POST |

Public dashboard without a PC: [github-pages.md](github-pages.md).
