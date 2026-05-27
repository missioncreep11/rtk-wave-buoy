#requires -Version 5.1
<#
.SYNOPSIS
  Smoke-test the buoy -> Hologram -> Cloudflare -> GitHub Pages chain
  without involving the actual buoy.

.DESCRIPTION
  Opens a plain TCP socket to cloudsocket.hologram.io:9999 and sends the same
  JSON Cloud Socket message that the buoy firmware sends in
  BuoyModem::sendHologramCloudMessage. Hologram should respond with [0,0],
  emit a _SOCKETAPI_ event, and (if the Alert is configured per
  documentation/github-pages.md) forward to the Cloudflare Worker which
  triggers the GitHub Actions workflow that commits docs/data.json.

.PARAMETER DeviceKey
  Hologram Cloud Socket device key (8 chars). Defaults to the
  HOLOGRAM_DEVICE_KEY environment variable. Required.

.PARAMETER Id
  Device id reported in the test payload. Default "test-pc".

.EXAMPLE
  PS> $env:HOLOGRAM_DEVICE_KEY = 'paste-key-here'   # single quotes preserve special chars
  PS> .\test-hologram.ps1

.EXAMPLE
  PS> .\test-hologram.ps1 -DeviceKey 'paste-key-here' -Id "test-pc" -Lat 32.8651 -Lon -117.2573

.NOTES
  After running, walk the chain:
    1. Hologram dashboard -> SIM -> Events -> new event with _SOCKETAPI_ tag
    2. Event detail should show matched_rules listing your Alert
    3. Cloudflare Worker logs should show a POST returning 200
    4. GitHub Actions tab should show a "Hologram telemetry -> data.json" run
    5. docs/data.json should reflect the new id / coordinates
#>

[CmdletBinding()]
param(
    [string]$DeviceKey = $env:HOLOGRAM_DEVICE_KEY,
    [string]$Id        = "test-pc",
    [int]   $Fix       = 3,
    [string]$Rtk       = "FIXED",
    [int]   $Sats      = 14,
    [double]$Lat       = 32.8651,
    [double]$Lon       = -117.2573,
    [string]$Hostname  = "cloudsocket.hologram.io",
    [int]   $Port      = 9999
)

if (-not $DeviceKey) {
    Write-Host "DeviceKey not provided." -ForegroundColor Red
    Write-Host "Set HOLOGRAM_DEVICE_KEY env var, or pass -DeviceKey 'KEY' (single quotes)." -ForegroundColor Red
    Write-Host "Single quotes are required because the key contains shell-special characters." -ForegroundColor Red
    exit 1
}

$innerObj = [ordered]@{
    id   = $Id
    fix  = $Fix
    rtk  = $Rtk
    sats = $Sats
    lat  = $Lat
    lon  = $Lon
}
$innerJson = $innerObj | ConvertTo-Json -Compress

# Hologram Cloud Socket frame format (buoy firmware mirrors this exactly):
#   {"k":"<deviceKey>","d":"<inner json with escaped quotes>"}\n\n
$innerEscaped = $innerJson.Replace('\', '\\').Replace('"', '\"')
$keyEscaped   = $DeviceKey.Replace('\', '\\').Replace('"', '\"')
$msg          = "{`"k`":`"$keyEscaped`",`"d`":`"$innerEscaped`"}`n`n"

Write-Host ("Sending to {0}:{1}" -f $Hostname, $Port) -ForegroundColor Cyan
Write-Host ("Payload: {0}" -f $innerJson) -ForegroundColor DarkGray

$tcp = New-Object System.Net.Sockets.TcpClient
try {
    $tcp.ReceiveTimeout = 5000
    $tcp.SendTimeout    = 5000
    $tcp.Connect($Hostname, $Port)
    $stream = $tcp.GetStream()
    $bytes  = [System.Text.Encoding]::ASCII.GetBytes($msg)
    $stream.Write($bytes, 0, $bytes.Length)
    $stream.Flush()

    Start-Sleep -Milliseconds 1500

    $buf  = New-Object byte[] 128
    $read = 0
    try { $read = $stream.Read($buf, 0, $buf.Length) } catch { $read = 0 }
    $reply = if ($read -gt 0) {
        [System.Text.Encoding]::ASCII.GetString($buf, 0, $read).Trim()
    } else {
        "<no reply>"
    }

    Write-Host ("Hologram replied: {0}" -f $reply) -ForegroundColor Green
    if ($reply -eq "[0,0]") {
        Write-Host "OK. Next: check Hologram Events tab, then GitHub Actions, then docs/data.json." -ForegroundColor Green
        exit 0
    } else {
        Write-Warning "Unexpected reply. Expected [0,0]. Verify the device key matches the SIM Router Credentials."
        exit 2
    }
}
catch {
    Write-Error ("TCP error: {0}" -f $_.Exception.Message)
    exit 3
}
finally {
    if ($tcp.Connected) { $tcp.Close() }
}
