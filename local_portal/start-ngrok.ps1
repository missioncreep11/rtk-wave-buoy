# Run ngrok when "ngrok" is not on PATH (common after winget install in Cursor).
$ngrok = "$env:LOCALAPPDATA\Microsoft\WinGet\Links\ngrok.exe"
if (-not (Test-Path $ngrok)) {
    $ngrok = (Get-Command ngrok -ErrorAction SilentlyContinue).Source
}
if (-not $ngrok -or -not (Test-Path $ngrok)) {
    Write-Error "ngrok not found. Install: winget install Ngrok.Ngrok"
    exit 1
}
$port = if ($args.Count -gt 0) { $args[0] } else { "8080" }
Write-Host "Starting ngrok http $port (Ctrl+C to stop)"
& $ngrok http $port
