# Cloudflare Worker — Telemetry Proxy

Proxies buoy telemetry from a Hologram Alert webhook to a GitHub `repository_dispatch`, which triggers the workflow that commits `docs/data.json`.

The Worker only accepts POSTs at the `/buoy` path. Requests to the bare URL return `404 Not Found`.

## Setup

1. Log into the [Cloudflare Dashboard](https://dash.cloudflare.com/).
2. Navigate to **Workers & Pages** → **Create Application** → **"Hello World"** template (do NOT click Connect GitHub).
3. Name your worker (e.g. `rtk-buoy-proxy`) and click **Deploy**.
4. Click **Edit code**, replace with contents of [`worker.js`](worker.js), click **Deploy**.
5. Go to **Settings → Variables** and add:

   | Variable | Type | Value |
   |----------|------|-------|
   | `GITHUB_OWNER` | Plain text | Your GitHub username or org |
   | `GITHUB_REPO` | Plain text | Repository name (e.g. `rtk-wave-buoy`) |
   | `GITHUB_PAT` | Encrypted | GitHub PAT with `repo` scope |
   | `BUOY_SECRET` | Encrypted | Shared secret — must match the Hologram Alert header |

6. Configure the Hologram Alert to POST to `https://YOUR-WORKER.workers.dev/buoy` with header `BUOY_SECRET` and body template `<<decdata>>`. See [root README](../README.md) for the full recipe.

## Smoke test

```powershell
$body = '{"id":"test","fix":3,"rtk":"FIXED","sats":14,"lat":32.8651,"lon":-117.2573}'
Invoke-RestMethod -Uri "https://YOUR-WORKER.workers.dev/buoy" `
    -Method Post `
    -Headers @{ "BUOY_SECRET" = 'YOUR_SECRET' } `
    -Body $body `
    -ContentType "application/json"
```

Use **single quotes** around the secret if it contains `$` (PowerShell would otherwise interpolate). Success returns `Successfully forwarded to GitHub Actions`. POSTing to the bare URL (without `/buoy`) returns `404 Not Found`.
