# One-command demo (Windows PowerShell): starts the API and the dashboard,
# then opens the browser. Press Ctrl+C in this window to stop both.
#
#   powershell -ExecutionPolicy Bypass -File scripts\demo.ps1 [-NoBrowser]
#
param([switch]$NoBrowser)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot

if (-not (Get-Command uv -ErrorAction SilentlyContinue)) {
    Write-Host 'uv was not found on PATH. Install it with "pip install uv" (or add its folder to PATH) and open a new terminal.' -ForegroundColor Red
    exit 1
}
if (-not (Get-Command npm -ErrorAction SilentlyContinue)) {
    Write-Host 'npm was not found. Install Node.js 20.19+ from https://nodejs.org' -ForegroundColor Red
    exit 1
}

Write-Host '1/3 Installing backend dependencies (uv sync)...' -ForegroundColor Cyan
Push-Location (Join-Path $root 'backend'); uv sync --quiet; Pop-Location

Write-Host '2/3 Installing dashboard dependencies (npm install)...' -ForegroundColor Cyan
Push-Location (Join-Path $root 'frontend')
if (-not (Test-Path 'node_modules')) { npm install --silent }
Pop-Location

Write-Host '3/3 Starting API (http://127.0.0.1:8000) and dashboard (http://localhost:5173)...' -ForegroundColor Cyan
$api = Start-Process -PassThru -NoNewWindow -WorkingDirectory (Join-Path $root 'backend') `
    -FilePath 'uv' -ArgumentList 'run', 'uvicorn', 'awi.api.main:app', '--port', '8000'
$web = Start-Process -PassThru -NoNewWindow -WorkingDirectory (Join-Path $root 'frontend') `
    -FilePath 'npm.cmd' -ArgumentList 'run', 'dev', '--', '--port', '5173', '--strictPort'

try {
    foreach ($i in 1..60) {
        try {
            Invoke-WebRequest -UseBasicParsing 'http://localhost:5173/api/health' -TimeoutSec 2 | Out-Null
            break
        } catch { Start-Sleep -Seconds 1 }
    }
    if (-not $NoBrowser) { Start-Process 'http://localhost:5173' }
    Write-Host 'Demo running. Open the Compare tab and pick a "Demo scenario". Ctrl+C to stop.' -ForegroundColor Green
    Wait-Process -Id $api.Id
} finally {
    foreach ($p in @($api, $web)) {
        if ($p -and -not $p.HasExited) { & taskkill /F /T /PID $p.Id | Out-Null }
    }
}
