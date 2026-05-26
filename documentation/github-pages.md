# GitHub Pages dashboard (optional)

Use this when you want a **public** map without running a PC or ngrok. The buoy cannot POST directly to `github.io`; data flows through **Hologram Cloud** and a **GitHub repository_dispatch** workflow.

```
Buoy (Hologram socket) --> Hologram Route --> GitHub API --> workflow --> docs/data.json --> Pages
```

Static site files live in [`docs/`](../docs/) (do not move — workflows deploy that folder).

## Prerequisites

- Repository on GitHub with default branch `main` (or adjust branch names below)
- These files committed on the default branch:
  - [`.github/workflows/hologram-telemetry.yml`](../.github/workflows/hologram-telemetry.yml)
  - [`.github/workflows/github-pages.yml`](../.github/workflows/github-pages.yml)
  - `docs/index.html`, `docs/config.js`, `docs/data.json`

## 1. Enable GitHub Pages

1. Repo **Settings → Pages**
2. Source: **Deploy from a branch**
3. Branch: `main`, folder **`/docs`**
4. Save. Note the Pages URL (e.g. `https://<user>.github.io/rtk-wave-buoy/`)

The deploy workflow also runs when `docs/index.html` or `docs/config.js` change.

## 2. Configure the dashboard

Edit [`docs/config.js`](../docs/config.js):

```javascript
window.BUOY_REPO = "YOUR_GITHUB_USER/rtk-wave-buoy";
window.BUOY_BRANCH = "main";
```

Replace `YOUR_GITHUB_USER` with your GitHub username or org (e.g. `KentaT1/rtk-wave-buoy`).

The page loads telemetry from:

```
https://raw.githubusercontent.com/<owner>/<repo>/<branch>/docs/data.json
```

Updating `data.json` via the Hologram workflow does **not** require redeploying Pages.

## 3. GitHub personal access token

Create a classic PAT or fine-grained token with **`repo`** scope (needed for `repository_dispatch` and the workflow push).

Store it only in Hologram Route headers or your shell — never commit it.

## 4. Hologram Route

In the [Hologram dashboard](https://dashboard.hologram.io/), create a **Route** on your device topic (e.g. `_DEVICE_<device_id>_`):

| Setting | Value |
|---------|--------|
| Method | POST |
| URL | `https://api.github.com/repos/<OWNER>/<REPO>/dispatches` |
| Header `Accept` | `application/vnd.github+json` |
| Header `Authorization` | `Bearer <YOUR_PAT>` |
| Header `X-GitHub-Api-Version` | `2022-11-28` |
| Body | JSON (see below) |

The buoy sends cloud messages as JSON. The Route should forward that payload into `client_payload`. Minimal test body for manual dispatch:

```json
{
  "event_type": "buoy-telemetry",
  "client_payload": {
    "id": "test",
    "fix": 3,
    "rtk": "FIXED",
    "sats": 14,
    "lat": 32.8651,
    "lon": -117.2573,
    "alt_m": 12.4,
    "bus_v": 3.92,
    "power_mw": 850,
    "rssi": 22,
    "ntrip": 1
  }
}
```

If Hologram wraps the buoy JSON in a string field `payload`, the workflow unwraps it automatically.

## 5. Test without the buoy

From the `docs` folder:

```powershell
$env:GITHUB_TOKEN = "ghp_..."
.\test-github-dispatch.ps1 -Owner "YOUR_USER" -Repo "rtk-wave-buoy"
```

Check:

1. **Actions** — workflow **Hologram telemetry → data.json** succeeds
2. **Raw JSON** — `https://raw.githubusercontent.com/<owner>/<repo>/main/docs/data.json`
3. **Pages site** — map and metrics refresh (may take a minute for CDN)

## 6. Configure the buoy

In `esp32/buoy_combo/secrets.h`:

```cpp
// Clear local portal URL
const char *telemetryUrl = "";

// Hologram CSR / Router key (Configuration -> Router Credentials)
const char hologramDeviceKey[] = "ABCD1234";
```

Flash `buoy_combo`. Serial should show `[TELEM] Hologram OK` on interval. NTRIP pauses briefly during each cloud send.

## 7. Verify end-to-end

| Step | Expected |
|------|----------|
| Buoy posts to Hologram | Message visible in SIM Activity |
| Route fires | GitHub Actions run |
| Workflow commits | `docs/data.json` updated on `main` |
| Pages | Dashboard shows new position and metrics |

## Troubleshooting

| Issue | Fix |
|-------|-----|
| 401/403 on dispatch | PAT scope or wrong owner/repo in URL |
| Workflow never runs | `event_type` must be exactly `buoy-telemetry` |
| Pages 404 | Enable Pages from `/docs`; wait for deploy workflow |
| Stale map | Hard-refresh; confirm `config.js` repo/branch match |
| No Hologram messages | `hologramDeviceKey` wrong or empty `telemetryUrl` still set |

Default development path remains the [local portal](local-portal.md).
