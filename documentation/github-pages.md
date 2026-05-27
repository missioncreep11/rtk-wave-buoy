# GitHub Pages dashboard

Use this when you want a **public** map without running a PC or ngrok. The buoy cannot POST directly to `github.io`, so data flows through Hologram (as a TLS relay), a **Cloudflare Worker** (which holds the GitHub PAT), and a **GitHub repository_dispatch** workflow that commits `docs/data.json`.

## Data flow

```
Buoy --plain TCP--> cloudsocket.hologram.io:9999
     --Alert webhook--> Cloudflare Worker
     --repository_dispatch--> GitHub Actions workflow
     --> docs/data.json --> GitHub Pages
```

Why route through Hologram? SIM7000A B03 firmware cannot establish HTTPS once an NTRIP socket is open ([troubleshooting](troubleshooting.md#why-not-https-b03-firmware-findings)). Hologram's Cloud Socket is plain TCP from the modem; Hologram does the TLS to Cloudflare on the buoy's behalf.

Static site files live in [`docs/`](../docs/) — do not move them; workflows deploy that folder.

## Prerequisites

- Repository on GitHub with workflows on the **default branch**:
  - [`.github/workflows/hologram-telemetry.yml`](../.github/workflows/hologram-telemetry.yml)
  - [`.github/workflows/github-pages.yml`](../.github/workflows/github-pages.yml)
  - `docs/index.html`, `docs/config.js`, `docs/data.json`
- A Hologram device with a Cloud Socket device key (8 chars, from the device's **Router Credentials**)

## 1. Enable GitHub Pages

1. Repo **Settings → Pages**
2. Source: **Deploy from a branch**, branch: default (e.g. `Base+PowerLog`), folder **`/docs`**
3. Save. Note the Pages URL (e.g. `https://<user>.github.io/rtk-wave-buoy/`)

The deploy workflow runs when `docs/index.html` or `docs/config.js` change on that branch.

## 2. Configure the dashboard

Edit [`docs/config.js`](../docs/config.js):

```javascript
window.BUOY_REPO = "YOUR_GITHUB_USER/rtk-wave-buoy";
window.BUOY_BRANCH = "Base+PowerLog";  // must match default branch
```

The page loads telemetry from `https://raw.githubusercontent.com/<owner>/<repo>/<branch>/docs/data.json`. If the branch name contains `+`, URL-encode it (`Base+PowerLog` → `Base%2BPowerLog`).

## 3. GitHub personal access token

Create a classic PAT (or fine-grained token) with **`repo`** scope. Stored only in the Cloudflare Worker — never committed.

## 4. Deploy the Cloudflare Worker

Follow [`../cloudflare_worker/README.md`](../cloudflare_worker/README.md). Required environment variables on the Worker:

| Variable | Purpose |
|----------|---------|
| `GITHUB_OWNER` | GitHub username or org |
| `GITHUB_REPO` | Repository name |
| `GITHUB_PAT` | PAT (encrypted) |
| `BUOY_SECRET` | Shared secret for header auth (encrypted) |

## 5. Configure the Hologram Alert

In the Hologram dashboard, create an **Alert** with these exact settings:

| Field | Value |
|-------|-------|
| Trigger / Subscribed topic | `_SOCKETAPI_` (matches every Cloud Socket inbound message) |
| Scope | The SIM group or device that contains your buoy |
| Notification method | **Webhook** |
| Destination URL | `https://YOUR-WORKER.workers.dev/buoy` (the `/buoy` path is required by the Worker — see below) |
| Message payload for POST | `<<decdata>>` (sends just the decoded device JSON, no Hologram envelope) |
| Header — Key | `BUOY_SECRET` |
| Header — Value | Same value as the Cloudflare `BUOY_SECRET` env var |
| Click | **Add JSON content-type header** (adds `Content-Type: application/json`) |

Notes:

- The Worker only accepts POSTs at the `/buoy` path. POSTs to the bare worker URL get `404 Not Found`. This is intentional — it lets stale/duplicate Hologram alerts coexist on the account without triggering GitHub workflow runs (point unwanted alerts at the bare URL or just leave them).
- `_SOCKETAPI_` is technically marked legacy in Hologram's docs but is still emitted on every Cloud Socket inbound today. Alternative tags on the same event are `_DEVICE_<id>_` (per-device) and `_JSONSTRING_` (any JSON Cloud Socket payload).
- `<<decdata>>` is Hologram's template variable for the decoded `data` field. Without it, the Worker receives the full Hologram envelope as a string and the workflow can't parse it cleanly.
- The header **name** must be exactly `BUOY_SECRET` — the Worker checks `request.headers.get("BUOY_SECRET")`. Don't confuse this with the Cloudflare env var of the same name (the env var is the *value* the header must match).

## 6. Configure the buoy

In `esp32/buoy_combo/secrets.h`:

```cpp
#define HAS_HOLOGRAM_DEVICE_KEY 1
const char hologramDeviceKey[] = "<8-char key from Hologram>";

// Leave telemetryUrl unset to force the Hologram relay branch.
// telemetrySecret is unused on this path (the Alert attaches the header).
```

Flash `buoy_combo`. Serial should show `[TELEM] Hologram OK` on each interval.

## 7. Test end-to-end without the buoy

Fake a Cloud Socket message from your PC and walk the chain:

```powershell
.\local_portal\test-hologram.ps1
```

(See the script for what it sends. You can also do the test pieces independently — see Appendix below.)

| Check | Where | Expected |
|-------|-------|----------|
| Cloud Socket accepted | PowerShell output | `[0,0]` |
| Hologram event created | Hologram dashboard → SIM → Events | Recent event with tags `_SOCKETAPI_`, `_DEVICE_<id>_`, `_JSONSTRING_` and `matched_rules` listing your alert |
| Webhook fired | Hologram alert log / event detail | HTTP 200 from Cloudflare (not 401) |
| Worker forwarded | GitHub repo → Actions tab | `Hologram telemetry → data.json` run in green |
| `docs/data.json` updated | GitHub web UI | New `device_id` and `received` timestamp |
| Pages refreshed | Pages URL | Map marker updated (hard-refresh if cached) |

## 8. Verify with the buoy

Flash `buoy_combo` and watch serial:

```
[NTRIP] connected
[RTCM] active
[TELEM] Hologram cloud...
[TELEM] Hologram OK
```

`[TELEM] Hologram OK` triggers the same chain as the PowerShell test. `docs/data.json` should now show your real device IMEI as `device_id`.

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| Hologram Activity shows the message but no event in Events tab | Wait a few seconds and refresh; events are emitted but indexed asynchronously |
| Event exists but `matched_rules: []` | Alert subscription doesn't match `_SOCKETAPI_` (or your device's `_DEVICE_<id>_`). Open the alert config and check the topic |
| Alert log shows HTTP 404 from Worker | Destination URL is missing the `/buoy` path. The Worker rejects POSTs to the bare URL. Update the alert URL to end in `/buoy` |
| Alert log shows HTTP 401 from Worker | Header `BUOY_SECRET` value doesn't match the Cloudflare env var. Verify both. Header *name* must be exactly `BUOY_SECRET` |
| Alert log shows HTTP 400 from Worker | Body template isn't `<<decdata>>`, so the Worker can't parse JSON. Update body template, save alert |
| Multiple workflow runs per Hologram event | More than one Hologram Alert is pointed at `/buoy`. Either delete the duplicates, or point them at the bare worker URL so the Worker drops them with 404 |
| Webhook returns 200 but no workflow run | `GITHUB_PAT` missing `repo` scope, or `GITHUB_OWNER`/`GITHUB_REPO` wrong on Worker |
| Workflow ran but `data.json` didn't change | Open the run logs; payload may have lacked recognized fields. Confirm the buoy JSON includes `id` (or `device_id`), `fix`, `rtk`, `lat`, `lon` |
| Raw JSON looks stale in browser | Hard-refresh; URL-encode `+` in branch name (`Base%2BPowerLog`); confirm `docs/config.js` branch matches |
| Pages 404 | Enable Pages from `/docs` on default branch |

## Appendix — individual smoke tests

**Cloudflare Worker only (skip Hologram):**

```powershell
$body = '{"id":"test-direct","fix":3,"rtk":"FIXED","sats":14,"lat":32.8651,"lon":-117.2573}'
Invoke-RestMethod -Uri "https://YOUR-WORKER.workers.dev/buoy" `
    -Method Post `
    -Headers @{ "BUOY_SECRET" = 'YOUR_SECRET' } `
    -Body $body `
    -ContentType "application/json"
```

Expect `Successfully forwarded to GitHub Actions`. (Hitting the bare URL without `/buoy` returns `404 Not Found` — that's by design.)

**GitHub workflow only (skip Worker too):** Repo Actions → **Hologram telemetry → data.json** → **Run workflow** → use the default test payload.

Default development path remains the [local portal](local-portal.md).
