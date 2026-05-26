# Test the GitHub telemetry workflow (repository_dispatch).
# Guide: documentation/github-pages.md
# Requires: repo on GitHub, PAT with repo scope, workflow on default branch.
#
# Usage:
#   $env:GITHUB_TOKEN = "ghp_xxxx"
#   .\test-github-dispatch.ps1 -Owner "your-user" -Repo "rtk-wave-buoy" -Branch "main"

param(
    [Parameter(Mandatory = $true)]
    [string]$Owner,
    [Parameter(Mandatory = $true)]
    [string]$Repo,
    [string]$Branch = "main"
)

$token = $env:GITHUB_TOKEN
if (-not $token) {
    Write-Error "Set GITHUB_TOKEN first: `$env:GITHUB_TOKEN = 'ghp_...'"
    exit 1
}

$headers = @{
    Accept                 = "application/vnd.github+json"
    Authorization          = "Bearer $token"
    "X-GitHub-Api-Version" = "2022-11-28"
}

$body = @{
    event_type     = "buoy-telemetry"
    client_payload = @{
        id       = "test-local"
        fix      = 3
        rtk      = "FIXED"
        sats     = 14
        lat      = 32.8651
        lon      = -117.2573
        alt_m    = 12.4
        bus_v    = 3.92
        power_mw = 850
        rssi     = 22
        ntrip    = 1
    }
} | ConvertTo-Json -Depth 5

$uri = "https://api.github.com/repos/$Owner/$Repo/dispatches"
$rawUrl = "https://raw.githubusercontent.com/$Owner/$Repo/$Branch/docs/data.json"

Write-Host "POST $uri"
try {
    Invoke-RestMethod -Uri $uri -Method POST -Headers $headers -Body $body
    Write-Host "OK - check Actions tab for Hologram telemetry workflow"
    Write-Host "Then open:"
    Write-Host "  $rawUrl"
}
catch {
    Write-Host $_.Exception.Message
    if ($_.ErrorDetails.Message) {
        Write-Host $_.ErrorDetails.Message
    }
}
