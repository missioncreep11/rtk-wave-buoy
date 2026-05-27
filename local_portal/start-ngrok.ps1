# Run ngrok in http or tcp mode.
#
# Usage:
#   ./start-ngrok.ps1               # defaults: tcp 8080  (what the buoy needs)
#   ./start-ngrok.ps1 tcp 8080
#   ./start-ngrok.ps1 http 8080     # for browser/desktop testing only — buoy can't do HTTPS
#
# SIM7000A B03 cannot do HTTPS, so the buoy uses `tcp://` URLs. Browsers can
# still hit the http (HTTPS) tunnel for the dashboard if you want a public URL.

$mode = if ($args.Count -gt 0) { $args[0] } else { "tcp" }
$port = if ($args.Count -gt 1) { $args[1] } else { "8080" }

if ($mode -ne "tcp" -and $mode -ne "http") {
    Write-Error "First arg must be 'tcp' or 'http' (got '$mode')"
    exit 1
}

$ngrok = "$env:LOCALAPPDATA\Microsoft\WinGet\Links\ngrok.exe"
if (-not (Test-Path $ngrok)) {
    $ngrok = (Get-Command ngrok -ErrorAction SilentlyContinue).Source
}
if (-not $ngrok -or -not (Test-Path $ngrok)) {
    Write-Error "ngrok not found. Install: winget install Ngrok.Ngrok"
    exit 1
}

Write-Host "Starting ngrok $mode $port (Ctrl+C to stop)"
if ($mode -eq "tcp") {
    Write-Host "Look for line:  Forwarding   tcp://X.tcp.ngrok.io:NNNNN -> localhost:$port"
    Write-Host "Paste host:port into secrets.h as:  tcp://X.tcp.ngrok.io:NNNNN/api/ingest"
}
& $ngrok $mode $port
