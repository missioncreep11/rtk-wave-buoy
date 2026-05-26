$body = @{
    id = "test-local"
    fix = 3
    rtk = "FIXED"
    sats = 12
    lat = 32.8651
    lon = -117.2573
    bus_v = 3.92
    rssi = 20
    ntrip = 1
} | ConvertTo-Json

Invoke-RestMethod -Uri "http://127.0.0.1:8080/api/ingest" -Method POST -Body $body -ContentType "application/json"
Write-Host "Open http://127.0.0.1:8080/"
