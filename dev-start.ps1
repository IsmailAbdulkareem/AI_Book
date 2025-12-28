# Development Server Startup Script
# This script starts both frontend and backend servers for local development

Write-Host "=== Physical AI & Humanoid Robotics Book - Dev Environment ===" -ForegroundColor Cyan
Write-Host ""

# Check if we're in the right directory
if (-not (Test-Path "frontend") -or -not (Test-Path "backend")) {
    Write-Host "Error: Please run this script from the project root directory" -ForegroundColor Red
    exit 1
}

# Start backend in a new window
Write-Host "Starting backend server..." -ForegroundColor Green
Start-Process powershell -ArgumentList "-NoExit", "-Command", "cd '$PWD\backend'; Write-Host 'Backend Server' -ForegroundColor Yellow; uv run python main.py"

# Wait a bit for backend to start
Start-Sleep -Seconds 2

# Start frontend in a new window
Write-Host "Starting frontend server..." -ForegroundColor Green
Start-Process powershell -ArgumentList "-NoExit", "-Command", "cd '$PWD\frontend'; Write-Host 'Frontend Server' -ForegroundColor Yellow; npm start"

Write-Host ""
Write-Host "Development servers are starting..." -ForegroundColor Cyan
Write-Host "- Backend will be available at: http://localhost:8000" -ForegroundColor White
Write-Host "- Frontend will be available at: http://localhost:3000/AI_Book/" -ForegroundColor White
Write-Host ""
Write-Host "Press any key to exit (servers will keep running)..." -ForegroundColor Gray
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
