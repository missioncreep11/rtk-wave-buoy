# GitHub Pages dashboard (optional)

Use this when you want a **public** map without running a PC or ngrok. The buoy cannot POST directly to `github.io`; data flows through a **Cloudflare Worker proxy** and a **GitHub repository_dispatch** workflow.

```
Buoy (HTTP POST) --> Cloudflare Worker --> GitHub API --> workflow --> docs/data.json --> Pages
```

Static site files live in [`docs/`](../docs/) (do not move — workflows deploy that folder).

## Prerequisites

- Repository on GitHub with workflows on the **default branch**
- These files committed on the default branch:
  - [`.github/workflows/hologram-telemetry.yml`](../.github/workflows/hologram-telemetry.yml)
  - [`.github/workflows/github-pages.yml`](../.github/workflows/github-pages.yml)
  - `docs/index.html`, `docs/config.js`, `docs/data.json`

## 1. Enable GitHub Pages

1. Repo **Settings → Pages**
2. Source: **Deploy from a branch**
3. Branch: your default branch (e.g. `Base+PowerLog`), folder **`/docs`**
4. Save. Note the Pages URL (e.g. `https://<user>.github.io/rtk-wave-buoy/`)

The deploy workflow runs when `docs/index.html` or `docs/config.js` change on that branch.

## 2. Configure the dashboard

Edit [`docs/config.js`](../docs/config.js):

```javascript
window.BUOY_REPO = "YOUR_GITHUB_USER/rtk-wave-buoy";
window.BUOY_BRANCH = "Base+PowerLog";  // must match default branch
```

The page loads telemetry from:

```
https://raw.githubusercontent.com/<owner>/<repo>/<branch>/docs/data.json
```

If the branch name contains `+`, URL-encode it in browser links (`Base+PowerLog` → `Base%2BPowerLog`).

Updating `data.json` via the telemetry workflow does **not** require redeploying Pages.

## 3. GitHub personal access token

Create a classic PAT or fine-grained token with **`repo`** scope (needed for `repository_dispatch` and the workflow push).

Store it only in your Cloudflare Worker environment variables or your shell — never commit it.

## 4. Cloudflare Worker proxy

Follow [`../cloudflare_worker/README.md`](../cloudflare_worker/README.md) to deploy the worker and set:

| Variable | Purpose |
|----------|---------|
| `GITHUB_OWNER` | GitHub username or org |
| `GITHUB_REPO` | Repository name |
| `GITHUB_PAT` | PAT (encrypted) |
| `BUOY_SECRET` | Shared secret for buoy auth (encrypted) |

## 5. Test without the buoy

**Option A — Cloudflare Worker** (tests full chain):

```powershell
$secret = 'YOUR_BUOY_SECRET'   # single quotes if secret contains $

$body = @{
    id = "test-pc"
    fix = 3
    rtk = "FIXED"
    sats = 14
    lat = 32.8651
    lon = -117.2573
} | ConvertTo-Json

Invoke-RestMethod -Uri "https://YOUR-WORKER.workers.dev" `
    -Method Post `
    -Headers @{ "X-Buoy-Secret" = $secret } `
    -Body $body `
    -ContentType "application/json"
```

**Option B — GitHub only** (from the `docs` folder):

```powershell
$env:GITHUB_TOKEN = "ghp_..."
.\test-github-dispatch.ps1 -Owner "YOUR_USER" -Repo "rtk-wave-buoy" -Branch "Base+PowerLog"
```

**Option C — Manual workflow** — Actions → **Hologram telemetry → data.json** → **Run workflow**

Check:

1. **Actions** — workflow succeeds
2. **Raw JSON** — `https://raw.githubusercontent.com/<owner>/<repo>/Base%2BPowerLog/docs/data.json`
3. **Pages site** — map and metrics refresh (hard-refresh if stale)

## 6. Configure the buoy

In `esp32/buoy_combo/secrets.h`:

```cpp
#define HAS_TELEMETRY_URL 1
const char *telemetryUrl = "https://YOUR-WORKER.workers.dev";
#define HAS_TELEMETRY_SECRET 1
const char *telemetrySecret = "same-as-cloudflare-BUOY_SECRET";

const char hologramDeviceKey[] = "";
```

Flash `buoy_combo`. Serial should show `[TELEM] POST OK` on interval.

## 7. Verify end-to-end

| Step | Expected |
|------|----------|
| Buoy posts to Cloudflare | `[TELEM] POST OK` on serial monitor |
| Worker proxies | GitHub Actions run |
| Workflow commits | `docs/data.json` updated on default branch |
| Pages | Dashboard shows new position and metrics |

## Troubleshooting

| Issue | Fix |
|-------|-----|
| Worker returns `Unauthorized` | `telemetrySecret` must match Cloudflare `BUOY_SECRET`; use single quotes in PowerShell if secret contains `$` |
| 401/403 on dispatch | PAT scope or wrong owner/repo in Cloudflare variables |
| Workflow never runs | Workflow file must be on default branch; `event_type` must be `buoy-telemetry` |
| Raw JSON looks stale | Hard-refresh; use `%2B` for `+` in branch URL; confirm `config.js` branch matches |
| Pages 404 | Enable Pages from `/docs` on default branch |
| No telemetry from buoy | Worker URL must start with `https://`; check secret header |

Default development path remains the [local portal](local-portal.md).
