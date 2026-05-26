# GitHub Pages dashboard (optional)

Use this when you want a **public** map without running a PC or ngrok. The buoy cannot POST directly to `github.io`; data flows through a **Cloudflare Worker proxy** and a **GitHub repository_dispatch** workflow (replacing the deprecated Hologram Route feature).

```
Buoy (HTTP POST) --> Cloudflare Worker Proxy --> GitHub API --> workflow --> docs/data.json --> Pages
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

Updating `data.json` via the GitHub workflow does **not** require redeploying Pages.

## 3. GitHub personal access token

Create a classic PAT or fine-grained token with **`repo`** scope (needed for `repository_dispatch` and the workflow push).

Store it only in your Cloudflare Worker environment variables or your shell — never commit it.

## 4. Cloudflare Worker Proxy

Since Hologram Routes are deprecated, we use a Cloudflare Worker to securely hold your GitHub PAT and proxy the HTTP request to the GitHub API.

Follow the instructions in [`../cloudflare_worker/README.md`](../cloudflare_worker/README.md) to set up and deploy your worker.

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
// Set this to your Cloudflare Worker URL
#define HAS_TELEMETRY_URL 1
const char *telemetryUrl = "https://rtk-buoy-proxy.YOUR_USERNAME.workers.dev";

// Leave empty
const char hologramDeviceKey[] = "";
```

Flash `buoy_combo`. Serial should show `[TELEM] POST OK` on interval. Note that you may need to add your `X-Buoy-Secret` header to the HTTP POST code in `buoy_combo.cpp`.

## 7. Verify end-to-end

| Step | Expected |
|------|----------|
| Buoy posts to Cloudflare | `[TELEM] POST OK` on serial monitor |
| Worker proxies | GitHub Actions run |
| Workflow commits | `docs/data.json` updated on `main` |
| Pages | Dashboard shows new position and metrics |

## Troubleshooting

| Issue | Fix |
|-------|-----|
| 401/403 on dispatch | PAT scope or wrong owner/repo in URL |
| Workflow never runs | `event_type` must be exactly `buoy-telemetry` |
| Pages 404 | Enable Pages from `/docs`; wait for deploy workflow |
| Stale map | Hard-refresh; confirm `config.js` repo/branch match |
| No telemetry | Cloudflare Worker URL incorrect, or secret header missing |

Default development path remains the [local portal](local-portal.md).
