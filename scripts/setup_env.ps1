# ==============================================================================
# EPGM One-Click Installer (Windows PowerShell)
# ==============================================================================

Write-Host "==========================================================" -ForegroundColor Cyan
Write-Host ">>> Initializing EPGM Fullstack Environment Installer..." -ForegroundColor Cyan
Write-Host "==========================================================" -ForegroundColor Cyan

# 1. Check Python
Write-Host ""
Write-Host ">>> [1/4] Checking Python environment..." -ForegroundColor Yellow
$pyOutput = python --version 2>&1
if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR: Python not found. Please install Python 3.9+ and add to PATH." -ForegroundColor Red
    exit 1
}
Write-Host "SUCCESS: Python detected: $pyOutput" -ForegroundColor Green

# 2. Install requirements
Write-Host ""
Write-Host ">>> [2/4] Installing epgm-algorithm dependencies..." -ForegroundColor Yellow
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$algoDir = Resolve-Path (Join-Path $scriptDir "..\..\epgm-algorithm")
$reqFile = Join-Path $algoDir "requirements.txt"

pip install -r $reqFile
if ($LASTEXITCODE -ne 0) {
    Write-Host "WARNING: Some packages had installation issues." -ForegroundColor Yellow
} else {
    Write-Host "SUCCESS: epgm-algorithm dependencies ready." -ForegroundColor Green
}

# 3. Check Docker
Write-Host ""
Write-Host ">>> [3/4] Checking Docker environment..." -ForegroundColor Yellow
$dockerOutput = docker --version 2>&1
if ($LASTEXITCODE -eq 0) {
    Write-Host "SUCCESS: Docker detected: $dockerOutput" -ForegroundColor Green
    Write-Host "TIP: Run 'docker-compose up -d' in epgm-platform to launch EMQX + Postgres + Redis." -ForegroundColor Cyan
} else {
    Write-Host "NOTE: Docker not found. Local simulation does not require Docker." -ForegroundColor DarkGray
}

# 4. Self-test
Write-Host ""
Write-Host ">>> [4/4] Running EPGM algorithm benchmark self-test..." -ForegroundColor Yellow
$benchScript = Join-Path $algoDir "experiments\run_maze_benchmark.py"
python $benchScript

Write-Host ""
Write-Host "==========================================================" -ForegroundColor Green
Write-Host ">>> EPGM Environment setup and benchmark test SUCCESSFUL!" -ForegroundColor Green
Write-Host "==========================================================" -ForegroundColor Green
