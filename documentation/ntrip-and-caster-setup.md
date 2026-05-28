# NTRIP and caster setup

How the buoy connects to an RTK correction caster over LTE. Firmware: `esp32/buoy_combo/`. RTK background: [student-guide.md](student-guide.md) §2.

---

## Requirements

| Requirement | Detail |
|-------------|--------|
| Protocol | NTRIP over **plain TCP** (typically port **2101**) |
| SSL | **Off** — `BOTLETICS_SSL` must be `0` in `buoy_combo.ino` |
| Network | Hologram SIM with data; `CGREG` 1 or 5 before NTRIP |
| Rover position | ZED must receive RTCM and have sky view for RTK fixed |

---

## Configure `secrets.h`

```bash
cp esp32/buoy_combo/secrets.h.example esp32/buoy_combo/secrets.h
```

| Setting | Example | Notes |
|---------|---------|--------|
| `casterHost` | IP or hostname | Must be reachable from cellular (not localhost) |
| `casterPort` | `2101` | Standard NTRIP port |
| `mountPoint` | e.g. `BASE_CCS` | Caster-specific mount |
| `casterUser` / `casterUserPW` | account credentials | From your caster operator |

Template in repo uses a UCSD-area caster example — replace with your account.

Optional:

```c
#define TELEMETRY_INTERVAL_MS 60000UL  // increase if link is marginal
```

---

## Verify on serial (115200)

Healthy sequence:

```text
[NET] connected
[GPRS] enabled
[NTRIP] connected
[RTCM] ...
[GPS] fix=3 rtk=FIXED sats=...
```

| Log | Meaning |
|-----|---------|
| `[NTRIP] TCP connect failed` | Host/port/firewall or no GPRS |
| HTTP error from caster | Wrong mountpoint or password |
| `fix=3` but `rtk=none` | No RTCM yet — wait several minutes |
| `rtk=float` | Ambiguities not fixed — normal before FIXED |
| Brief disconnect after `[TELEM]` | Normal — NTRIP closes for Hologram send |

---

## Caster distance (baseline)

RTK accuracy depends on distance to the correction source:

- **Near caster / VRS:** often fixes in minutes with open sky.
- **Long baseline:** float longer or no fix; multipath on water makes it harder.

The buoy does not send NMEA GGA to the caster in all builds — confirm your caster allows fixed-position or IP-based rovers. If your caster requires periodic GGA (e.g. some VRS services), you may need firmware support not described here; legacy WiFi Polaris sketches do send GGA.

---

## LTE vs WiFi NTRIP

| Path | Sketch |
|------|--------|
| **Production buoy** | `buoy_combo` — LTE SIM7000 |
| **Bench WiFi experiments** | `esp32/legacy/*` — see [legacy-vs-buoy-combo.md](legacy-vs-buoy-combo.md) |

Do not use WiFi-only sketches for the cellular deployment.

---

## Band and carrier

NTRIP needs IP connectivity, not just registration:

- Set `LTE_CATM_BAND` in `buoy_combo.h` (12 vs 13) for your carrier.
- Hologram dashboard: SIM active, data enabled.
- Outages: [Hologram status](https://status.hologram.io) — firmware cannot fix carrier-side failures.

---

## Troubleshooting

| Symptom | Action |
|---------|--------|
| Never `[NTRIP] connected` | Ping caster from a phone on same carrier if possible; verify credentials |
| Works then 10 failures | Escalated modem recover — [failure-paths.md](failure-paths.md) |
| `[TELEM] Hologram failed` with good NTRIP | Separate issue — Hologram key / CIP busy; see [../README.md](../README.md) |

---

## Related

- [firmware-walkthrough.md](firmware-walkthrough.md) — `beginNTRIPClient`, `handleNTRIPData`
- [at-command-primer.md](at-command-primer.md) — `CGREG`, GPRS
- [deployment-checklist.md](deployment-checklist.md)
