# Cloudflare Worker Telemetry Proxy

This directory contains the code to securely proxy telemetry data from the RTK Wave Buoy to GitHub Actions without exposing your GitHub Personal Access Token (PAT).

The Worker accepts POSTs from either of two entry points:

- **Hologram Alert webhook** (deployed path) — buoy sends a Cloud Socket message; Hologram's Alert system forwards it here. See [`../documentation/github-pages.md`](../documentation/github-pages.md).
- **Direct HTTPS from the buoy** — only possible on a SIM7000A B05+ modem; B03 cannot do HTTPS reliably with NTRIP active.

Both paths POST the same JSON body and the same `BUOY_SECRET` header; the Worker doesn't care which side originates the request.

## 1. Create the Worker

1. Log into the [Cloudflare Dashboard](https://dash.cloudflare.com/).
2. Navigate to **Workers & Pages** in the left sidebar.
3. Click **Create Application**, and select the **"Hello World"** template (Do NOT click Connect GitHub).
4. Name your worker (e.g., `rtk-buoy-proxy`) and click **Deploy**.
5. Once deployed, click **Edit code** (or **Quick edit**).
6. Replace the default code with the contents of [`worker.js`](worker.js).
7. Click **Deploy** in the top right corner.

## 2. Configure Secrets and Variables

Your worker needs to know where to send the data, and it needs authorization to do so.

1. Back on the Worker's overview page, go to **Settings** -> **Variables**.
2. Under **Environment Variables**, add the following (choose "Add variable", *not* encrypted for the first two):
   - `GITHUB_OWNER`: Your GitHub username (e.g., `KentaT1`)
   - `GITHUB_REPO`: The repository name (e.g., `rtk-wave-buoy`)
3. Under **Environment Variables**, click **Add variable**, select **Encrypt** (the lock icon), and add:
   - `GITHUB_PAT`: Your GitHub Personal Access Token (must have the `repo` scope).
   - `BUOY_SECRET`: A secure, random string (e.g., a long password) that you will also configure on the buoy. This prevents unauthorized people from triggering your workflow.
4. Click **Deploy** or **Save** to apply the variables.

## 3. Hook the Worker up to a data source

Pick one — both result in the Worker getting POSTs that update `docs/data.json`:

**Path A — Hologram Alert (works on B03 modem firmware)**

Configure a Hologram Alert with destination URL = the Worker URL, header `BUOY_SECRET` = the value you stored above, and body template `<<decdata>>`. Full recipe in [`../documentation/github-pages.md`](../documentation/github-pages.md). The buoy then needs `hologramDeviceKey` set in `secrets.h` and `telemetryUrl` empty.

**Path B — Direct HTTPS from the buoy (needs B05+ modem firmware)**

In `esp32/buoy_combo/secrets.h`:

```cpp
#define HAS_TELEMETRY_URL 1
const char *telemetryUrl = "https://rtk-buoy-proxy.YOUR_USERNAME.workers.dev";
#define HAS_TELEMETRY_SECRET 1
const char *telemetrySecret = "YOUR_BUOY_SECRET";
```

The firmware sends `telemetrySecret` as the `BUOY_SECRET` header automatically.

## 4. Test the Worker

Test the worker using PowerShell before involving Hologram or the buoy:

```powershell
$body = @{
    id = "test"
    fix = 3
    rtk = "FIXED"
    sats = 14
    lat = 32.8651
    lon = -117.2573
} | ConvertTo-Json

Invoke-RestMethod -Uri "https://rtk-buoy-proxy.YOUR_USERNAME.workers.dev" `
    -Method Post `
    -Headers @{ "BUOY_SECRET" = 'YOUR_BUOY_SECRET' } `
    -Body $body `
    -ContentType "application/json"
```

Use **single quotes** around the secret if it contains `$` (PowerShell would otherwise try to interpolate). Success returns `Successfully forwarded to GitHub Actions`; check the GitHub repo's **Actions** tab to see the workflow run.

For an end-to-end test that goes through Hologram too, see [`../local_portal/test-hologram.ps1`](../local_portal/test-hologram.ps1).
