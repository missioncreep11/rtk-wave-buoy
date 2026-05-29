# Point One Polaris — exploration log and developer guide

This document records **everything the RTK wave buoy project tried** with [Point One Navigation Polaris](https://pointonenav.com/) (network RTK / VRS over NTRIP), what worked, what failed, and **where to pick up** if you integrate Polaris on the LTE buoy again.

**Production firmware today:** `esp32/buoy_combo/` uses a **plain NTRIP 1.x-style client** (`HTTP/1.0`, no GGA, raw RTCM after the HTTP header). The repo template `secrets.h.example` points at a **local/UCSD caster** (`BASE_CCS`), not Polaris.

**Polaris on LTE was proven in the field** (logs show `rtk=FIXED` over Hologram) but the full Polaris stack lived in **uncommitted / reverted** work (`ntrip_profile.h`, May 2026). Only **modem battery-like recovery** from that period was kept in `buoy_combo.h`.

---

## What Polaris is (and why the buoy cares)

Polaris is a **cloud network RTK** service. It uses **Virtual Reference Station (VRS)** corrections: the server synthesizes RTCM as if a base station sat ~1–2 km from the rover.

| Requirement | Why |
|-------------|-----|
| **Rover position (NMEA GGA)** | VRS must know where to place the virtual base. Without GGA, Polaris cannot tailor corrections. |
| **NTRIP 2.0 / HTTP/1.1** | Responses use **chunked transfer encoding**, not a raw byte stream after `ICY 200`. |
| **Periodic GGA refresh** | Buoy drifts slowly; legacy WiFi firmware refreshes GGA every **10 s** (was 5 min early on — too sparse for stable VRS near the rover). |

Deeper theory (ionosphere, VRS vs FKP/MAC): see `tutorials/POLARIS_NETWORK_RTK.md` on branch **`main`** (not always present on `Base+PowerLog`).

---

## Timeline of attempts

| Phase | Where | What we did | Outcome |
|-------|--------|-------------|---------|
| **1. WiFi bench — Polaris + BLE** | `esp32/legacy/esp32_polaris.ino` | SparkFun-style client; GGA on connect + refresh; BLE config in NVS; NTRIP over WiFi | Good for **classroom / bench**; not for sealed LTE buoy |
| **2. WiFi bench — merged “best of”** | `esp32/legacy/esp32_polaris_wifi.ino` | HTTP/1.1 + `Ntrip-Version: Ntrip/2.0`; **chunked RTCM decoder**; GGA every **10 s**; direct `gpsSerial.write()`; 5 Hz nav rate | **Reference implementation** for Polaris protocol on ESP32 |
| **3. u-center / OLA** | `tutorials/UCENTER_NTRIP_OLA_LOGGING.md` (main) | Logged with Polaris mount in u-center for comparison | Validates ZED + Polaris outside custom firmware |
| **4. LTE production caster (non-Polaris)** | `esp32/buoy_combo/` | Plain TCP NTRIP to campus/regional caster in `secrets.h` | Stable path for **telemetry + NTRIP** on SIM7000; no chunked HTTP |
| **5. LTE + Polaris profile** | Uncommitted `ntrip_profile.h` + `secrets.polaris.h.example` (May 2026) | Port WiFi Polaris client to modem `AT+CIP*`; `profile=POLARIS`; static buffers; GGA; chunked read; backlog drain | **RTK FIXED achieved on LTE**; long-run **modem** and **RTK drop** issues remained |
| **6. Stability pass (same branch)** | `buoy_combo.h` + `ntrip_profile.h` (reverted except recovery) | CNACT-first CIP (skip `CIPSHUT` when IP up); telemetry gating; task WDT; catch-up drain; battery-like PWRKEY recovery | Partially validated in logs; **reverted to last push** except **improved full reset** in `buoy_combo.h` |

---

## Caster configuration (Polaris)

Use **class credentials** from the course (Canvas), not personal API keys in the repo.

| Setting | Typical value |
|---------|----------------|
| Host | `polaris.pointonenav.com` |
| Port | `2101` |
| Mount | `POLARIS` |
| User | Class email / account id (per assignment) |
| Password | Class API key (per assignment) |

Template (copy to `secrets.h` for experiments — **do not commit** real keys):

```c
// See esp32/buoy_combo/secrets.polaris.h.example
const char *casterHost   = "polaris.pointonenav.com";
const uint16_t casterPort = 2101;
const char *mountPoint   = "POLARIS";
const char *casterUser   = "<from Canvas>";
const char *casterUserPW = "<from Canvas>";
```

Flash guide (WiFi sketch): `tutorials/ESP32_FIRMWARE_FLASH_POLARIS.md` on **`main`**.

---

## Protocol comparison: legacy WiFi vs production LTE vs Polaris-on-LTE (reverted)

| Feature | `esp32_polaris_wifi.ino` | `buoy_combo` (shipped) | `ntrip_profile.h` (reverted) |
|---------|--------------------------|-------------------------|------------------------------|
| Transport | WiFi `WiFiClient` | SIM7000 `AT+CIP*` plain TCP | Same as shipped |
| HTTP version | **1.1** + `Ntrip-Version: Ntrip/2.0` | **1.0** | **1.1** + Ntrip/2.0 |
| RTCM body | **Chunked decoder** | Assumes bytes after header (works for ICY/simple casters) | Chunked decoder |
| GGA to caster | Yes, ~10 s | **No** | Yes |
| Hologram telemetry | No | Yes (~60 s) | Yes |
| Modem recovery | N/A | RST → PWRKEY (docs); **code: PWRKEY first** + `esp_restart` on fail | Same + stability tweaks |

### HTTP request shape (Polaris)

From `esp32_polaris_wifi.ino` (reference):

```http
GET /POLARIS HTTP/1.1
Host: polaris.pointonenav.com
Ntrip-Version: Ntrip/2.0
User-Agent: NTRIP SparkFun u-blox Client v1.0
Authorization: Basic <base64(user:password)>

```

Then send **GGA** (`$GPGGA,...*hh\r\n`) on connect and every **10 s**.

### Chunked RTCM read (concept)

Polaris does not stream raw RTCM immediately after `200 OK`. The body is **HTTP chunked**:

1. Read hex chunk size line (e.g. `1A4\r\n`).
2. Read exactly that many bytes → write to ZED UART.
3. Read trailing `\r\n`.
4. Repeat until chunk size `0` (end of stream → reconnect).

The WiFi sketch uses **blocking reads with timeout** so a drained socket does not desync the decoder. Any LTE port must do the same on `modem.TCPread()`.

---

## Field results (LTE + Polaris profile)

### Success pattern

Serial sequence when the stack is healthy:

```text
[GPRS] enabled
[NTRIP] profile=POLARIS connecting to polaris.pointonenav.com:2101 mount=POLARIS
[NTRIP] connected
[NET] CSQ=26 CGREG=5 (roaming)
[RTCM] 250 B/10s backlog=1216
[GPS] fix=3 rtk=FIXED sats=20
[TELEM] Hologram OK
```

Interpretation: LTE registered → Polaris handshake OK → RTCM flowing → integer RTK → telemetry still works.

### Failure patterns observed

| Symptom | Likely cause | Notes |
|---------|----------------|-------|
| `CGREG=0`, `CSQ=99` for hours | Modem stuck / not camped | GPS still `fix=3`; no RTCM. Old recover used RST + `configureNetwork(true)` and often failed (`CPIN` echo only). **Mitigation kept:** PWRKEY 20 s off, boot UART path, `esp_restart`. |
| `[NTRIP] connected` but `rtk=none` | GGA missing, wrong HTTP profile, or stale RTCM | Plain `buoy_combo` without GGA/chunked will not work reliably on Polaris. |
| `rtk=FIXED` → `rtk=none`, NTRIP still up, `[RTCM]` ~4–5 KB/s | **RTK lock lost**, not network | `fix=3` unchanged; modem healthy. Suspects: **modem RX backlog** (FIFO stale corrections), multipath, VRS/GGA drift, ZED timeout. |
| `backlog=1500–2800` steady | ESP reads slower than caster sends | `[RTCM]` reports `modem.TCPavailable()` — queue never drains. Catch-up drain was added in reverted `ntrip_profile.h` but backlog often stayed high. |
| Second `[NTRIP] connecting...` after `[TELEM] Hologram OK` | Telemetry closes NTRIP socket for Hologram HTTP | Expected in current `post_telemetry_f()` design; can disturb RTK if reconnect is slow. |

### Log tags to search

| Tag | Meaning |
|-----|---------|
| `[NTRIP] profile=POLARIS` | Polaris profile active (reverted build only) |
| `[MODEM] CIP stack init (CIPSHUT)` | Full legacy bearer teardown (CNACT may drop) |
| `[RTCM] catch-up drain` | Backlog drain pass (reverted build) |
| `[MODEM] modem off — settling` | Battery-like PWRKEY off (20 s) |
| `[MODEM] restarting ESP32` | `esp_restart()` after failed power-cycle config |

---

## Reverted LTE work (`ntrip_profile.h`) — what to restore

The following lived in **`esp32/buoy_combo/ntrip_profile.h`** (deleted on revert, **never on `Base+PowerLog` remote**). Rebuild from `esp32_polaris_wifi.ino` + `buoy_combo.h` hooks:

1. **`beginNTRIPClient` / `handleNTRIPData` replacements**  
   - Select profile via `NTRIP_PROFILE` or secrets.  
   - Polaris: build HTTP/1.1 request in **static buffer** (no `String` per retry).  
   - After `200`, run chunked loop; optional **sustained drain** while `TCPavailable() > NTRIP_BACKLOG_TARGET`.

2. **GGA builder**  
   - Read lat/lon/alt from `myGNSS.getPVT()`; `quality=0` if no fix (Polaris waits).  
   - Send on connect + every `NTRIP_GGA_INTERVAL_MS` (10 000 ms recommended).

3. **Chunked parser**  
   - Port `readByteBlocking` + hex size line + payload + CRLF from `esp32_polaris_wifi.ino`.  
   - On `chunkSize=0` or parse error → `AT+CIPCLOSE` and reconnect.

4. **Stability tweaks (optional but tested)**  
   - Skip `CIPSHUT` when `AT+CIFSR` already has IP (`bringUpCipStack` in `buoy_combo.h`).  
   - `TELEMETRY_SKIP_WHEN_NTRIP` or send telemetry only when `rtk=FIXED` to avoid tearing down NTRIP every 60 s.  
   - ESP task WDT during long modem delays.  
   - Reduce `String` allocations in NTRIP path.

5. **`secrets.polaris.h.example`**  
   - Document Polaris host/mount; copy fields into `secrets.h`.

**Include pattern (reverted `buoy_combo.ino`):**

```cpp
#include "secrets.h"
#include "buoy_combo.h"
#include "ntrip_profile.h"   // provides beginNTRIPClient / handleNTRIPData
```

---

## Architectural tensions (LTE + Polaris + Hologram)

These are the main reasons “Polaris works on WiFi” but “LTE + Polaris + telemetry” is hard:

```text
                    ┌─────────────────┐
   Polaris NTRIP ──►│ SIM7000 CIP TCP │──► RTCM ──► ZED UART
                    └────────┬────────┘
                             │ CIPSHUT / CIPCLOSE
                    ┌────────▼────────┐
   Hologram telem ─►│ CNACT PDP +    │
                    │ HTTP/CIP again │
                    └─────────────────┘
```

| Issue | Detail |
|-------|--------|
| **Dual stack** | NTRIP uses legacy `AT+CIP*`; Hologram uses `CNACT` + Botletics HTTP. `CIPSHUT` during NTRIP retry can kill the PDP Hologram needs. |
| **Telemetry vs NTRIP** | `post_telemetry_f()` may **close the NTRIP socket** before opening Hologram — fine for plain casters, painful for Polaris (re-handshake + GGA). |
| **FIFO backlog** | Modem buffers RTCM if the main loop is slow (telemetry, drain, health checks). ZED may see **stale** corrections → `rtk=FIXED` drops while `[NTRIP] connected`. |
| **No GGA in shipped build** | Production `beginNTRIPClient()` cannot satisfy VRS long-term. |

**Future directions (pick one per milestone):**

- **A.** Re-merge `ntrip_profile.h` + telemetry gating + kept modem recovery.  
- **B.** Single socket architecture: `AT+CAOPEN` / CNACT-only NTRIP (large refactor).  
- **C.** Offload Polaris to WiFi for bench only; keep LTE on fixed/mount-based caster for deployment.  
- **D.** External correction path (L-band / second radio) — out of scope for current HW.

---

## Recommended path for the next developer

### Bench (no cellular)

1. Flash `esp32/legacy/esp32_polaris_wifi.ino` with `secrets.h` (WiFi + Polaris creds).  
2. Confirm `rtk=FIXED` outdoors; watch `[diag]` and GGA refresh lines.  
3. Compare u-center NTRIP to the same mount (`tutorials/UCENTER_NTRIP_OLA_LOGGING.md` on `main`).

### LTE integration

1. Start from current `Base+PowerLog` `buoy_combo` (has **battery-like modem recovery**).  
2. Re-implement Polaris client from `esp32_polaris_wifi.ino` into a new `ntrip_profile.h` (or inline behind `#ifdef NTRIP_PROFILE_POLARIS`).  
3. Add `secrets.polaris.h.example`; never commit real keys.  
4. Test in order:  
   - TCP + `200 OK` only  
   - Chunked RTCM + ZED `rtk=none` → `float`/`FIXED`  
   - GGA every 10 s for 30+ min  
   - Enable Hologram telemetry — verify RTK survives `[TELEM]`  
   - Overnight soak — watch `backlog=`, `CGREG`, `[MODEM]`  
5. Update [failure-paths.md](failure-paths.md) if escalation order changed (PWRKEY before RST in current code).

### Validation checklist

- [ ] `[NTRIP] connected` within 2 min of `[GPRS] enabled`  
- [ ] `[RTCM]` > 1 KB/s average in open sky  
- [ ] `rtk=FIXED` held **> 30 min** without manual reset  
- [ ] After `[TELEM] Hologram OK`, RTK recovers within 2 min  
- [ ] `backlog=` usually **< 500** (not stuck at 1500+)  
- [ ] Overnight: no endless `CGREG=0` / `CSQ=99` without `[MODEM] power cycle` recovery  

---

## Repository map

| Path | Role |
|------|------|
| `esp32/legacy/esp32_polaris_wifi.ino` | **Best Polaris protocol reference** (WiFi) |
| `esp32/legacy/esp32_polaris.ino` | Older Polaris + BLE + NVS config |
| `esp32/legacy/esp32_rtk_mae.ino` | MAE course WiFi RTK; chunked parser history |
| `esp32/buoy_combo/buoy_combo.h` | Production LTE; plain NTRIP; modem recovery |
| `esp32/buoy_combo/secrets.h.example` | Default **non-Polaris** caster template |
| `esp32/buoy_combo/secrets.polaris.h.example` | Polaris field template (credentials from Canvas) |
| `documentation/ntrip-and-caster-setup.md` | Production NTRIP setup |
| `documentation/legacy-vs-buoy-combo.md` | WiFi vs LTE sketches |
| `tutorials/POLARIS_NETWORK_RTK.md` | VRS theory (**on `main`**) |
| `tutorials/ESP32_FIRMWARE_FLASH_POLARIS.md` | WiFi flash steps (**on `main`**) |

### Git commits (Polaris-related, `main` history)

- `d8121b0` — Add `esp32_polaris.ino`  
- `77a167d` — Add `esp32_polaris_wifi.ino`  
- `f370c69` / `865a643` — `POLARIS_NETWORK_RTK.md`, GGA interval notes  
- `31abada` — Canvas credentials note in flash tutorial  

LTE Polaris integration was **session work (May 2026)** and is documented here; it was **not** merged to `origin/Base+PowerLog` except modem recovery.

---

## Related docs

- [ntrip-and-caster-setup.md](ntrip-and-caster-setup.md) — production caster / `secrets.h`  
- [failure-paths.md](failure-paths.md) — modem recovery (update PWRKEY-first if docs lag code)  
- [firmware-walkthrough.md](firmware-walkthrough.md) — `beginNTRIPClient`, `handleNTRIPData`  
- [legacy-vs-buoy-combo.md](legacy-vs-buoy-combo.md) — do not flash WiFi sketches on LTE buoy  

---

*Last updated: May 2026 — reflects revert to `cb756f9` + retained battery-like modem recovery.*
