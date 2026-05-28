# AT command primer (SIM7000)

What the modem AT responses mean in **buoy_combo** serial logs. The ESP32 uses the Botletics library; diagnostics also appear from `BuoyModem::printDiagnostics()`.

Full recovery behavior: [failure-paths.md](failure-paths.md). Pin/control: [wiring-and-pins.md](wiring-and-pins.md).

---

## Commands the firmware cares about

| Command | Purpose in this project |
|---------|-------------------------|
| `AT+CPIN?` | SIM ready — must see `READY` before band config |
| `AT+CFUN?` / `AT+CFUN=0/1` | Radio off/on; recover path may use CFUN=0 for band change |
| `AT+CGREG?` | **LTE registration** — primary `[NET]` status |
| `AT+CSQ` | Signal strength 0–31 (`99` = not known) |
| `AT+CGATT?` | Packet service attached |
| `AT+CNACT?` | PDP context / assigned IP |
| `AT+COPS?` | Operator name |
| `AT+CBANDCFG` | LTE CAT-M band table |
| `AT+CIP*` | TCP stack for NTRIP and Hologram |

Firmware logs **`CGREG`** for registration, not `CREG` (circuit switched legacy).

---

## `AT+CGREG?` — registration (most important)

Format: `+CGREG: <n>,<stat>`

| `stat` | Meaning | Firmware |
|--------|---------|----------|
| 0 | Not registered, not searching | Bad — data path invalidated after grace |
| 1 | Registered, home network | OK — `networkConnected` |
| 2 | Searching | May extend timeout if `CSQ` good |
| 3 | Denied | Bad |
| 5 | Registered, roaming | OK — treated same as 1 |

Serial: `[NET] CSQ=… CGREG=…` and `[NET] registration lost` on confirmed loss.

**Grace:** If `CGREG` drops but RTCM still flows, loss is ignored up to `CELLULAR_LINK_ALIVE_MS` (2 min).

---

## `AT+CSQ` — signal quality

Response: `+CSQ: <rssi>,<ber>`

| RSSI | Rough meaning |
|------|----------------|
| 0–9 | Weak |
| 10–14 | Fair |
| 15–19 | Good |
| 20–31 | Strong |
| 99 | Not known / not detectable |

`CSQ=99` can appear while `CGREG=5` still works — do not treat 99 alone as total failure.

`CSQ=0` with `CGREG=0` for many minutes triggers escalated recover (see failure-paths).

---

## `AT+CPIN?` — SIM

| Response | Action |
|----------|--------|
| `READY` | Normal |
| `SIM PIN` / `SIM PUK` | PIN locked — not expected on Hologram data SIMs |
| `SIM failure` | Reseat SIM, power cycle — band config skipped |

Log: `[DIAG] CPIN: …` or `[MODEM] SKIP band config — CPIN: …`

---

## `AT+CFUN?` — radio mode

| Value | Meaning |
|-------|---------|
| 0 | Minimum functionality (radio off) — used briefly on **recover** band config |
| 1 | Full functionality — required for service |

**Boot** path turns radio on first (`CFUN=1`), then configures bands at full power — log `(boot)`.

**Recover** may use `CFUN=0` before `CBANDCFG` — log `(recover)`.

---

## `AT+CNACT?` — IP address

Shows PDP context activation and IP (e.g. `10.x.x.x`).

- `0.0.0.0` or no dot: no usable IP yet.
- Firmware may refresh GPRS if data path is stale while `CGREG` still OK.

---

## TCP / `AT+CIP*` (NTRIP + Hologram)

Buoy uses **plain** TCP (`tcpConnectPlain`), not SSL:

1. PDP active (`ensurePdpActive`)
2. `CIPSTART` to caster or `cloudsocket.hologram.io`
3. `CIPSEND` / read for NTRIP or telemetry
4. `CIPCLOSE` — telemetry closes NTRIP before Hologram send

Logs: `[NTRIP]`, `[TELEM]`, `[HOLO]`, `[GPRS] refresh`.

---

## Manual AT from USB Serial

If you type in the Arduino serial monitor while the buoy runs, characters go to the modem (`modemSS`) and replies print back — useful for one-off `AT+CGREG?` checks.

Prefix shown: `modem>`.

---

## Escalated recovery (not single AT commands)

| Log | Hardware action |
|-----|-----------------|
| `[MODEM] hard recover` | RST pin + `configureNetwork(true)` |
| `[MODEM] power cycle` | `AT+CPOWD=1` + PWRKEY sequence |
| `[MODEM] * skipped (cooldown)` | Timer blocking repeat |

---

## Related

- [failure-paths.md](failure-paths.md) — timers and triggers
- [firmware-walkthrough.md](firmware-walkthrough.md) — where polls run
- Botletics / SIM7000 AT manual — full command reference
