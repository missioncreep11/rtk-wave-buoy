# Cloudflare Worker Telemetry Proxy

This directory contains the code to securely proxy telemetry data from the RTK Wave Buoy to GitHub Actions. This approach replaces the deprecated Hologram Route feature, allowing you to trigger the GitHub Pages dashboard update without exposing your GitHub Personal Access Token (PAT).

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

## 3. Configure the Buoy Firmware

Update your buoy's firmware to send data to the new Cloudflare Worker URL, and configure the secret header.

1. Note the public URL of your Cloudflare Worker (e.g., `https://rtk-buoy-proxy.YOUR_USERNAME.workers.dev`).
2. In `esp32/buoy_combo/secrets.h`, set the URL:
   ```cpp
   #define HAS_TELEMETRY_URL 1
   const char *telemetryUrl = "https://rtk-buoy-proxy.YOUR_USERNAME.workers.dev";
   ```
3. Set `telemetrySecret` in `secrets.h` to the same value as `BUOY_SECRET`. Firmware sends it as the `X-Buoy-Secret` header automatically.

## 4. Test the Worker

You can test the worker using PowerShell or cURL before deploying the buoy.

**PowerShell:**
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
    -Headers @{ "X-Buoy-Secret" = "YOUR_BUOY_SECRET" } `
    -Body $body `
    -ContentType "application/json"
```

If successful, it will return `Successfully forwarded to GitHub Actions`, and you can check the **Actions** tab on your GitHub repository to see the workflow running.
