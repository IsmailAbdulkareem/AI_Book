# Test All Services Script
# Starts backend and frontend for testing

Write-Host "=== Testing All Services ===" -ForegroundColor Cyan
Write-Host ""

# Check if we're in the right directory
if (-not (Test-Path "frontend") -or -not (Test-Path "backend")) {
    Write-Host "Error: Please run this script from the project root directory" -ForegroundColor Red
    exit 1
}

# Start Backend
Write-Host "[1/2] Starting Backend Server..." -ForegroundColor Yellow
Start-Process powershell -ArgumentList "-NoExit", "-Command", "cd '$PWD\backend'; Write-Host 'Backend Server (Port 8000)' -ForegroundColor Yellow; python main.py"

Write-Host "Waiting for backend to start..." -ForegroundColor Gray
Start-Sleep -Seconds 5

# Test backend health
try {
    $response = Invoke-WebRequest -Uri "http://localhost:8000/health" -UseBasicParsing -TimeoutSec 5 -ErrorAction Stop
    Write-Host "✓ Backend is running on http://localhost:8000" -ForegroundColor Green
} catch {
    Write-Host "⚠ Backend may still be starting..." -ForegroundColor Yellow
}

Write-Host ""

# Start Frontend
Write-Host "[2/2] Starting Frontend Server..." -ForegroundColor Yellow
Start-Process powershell -ArgumentList "-NoExit", "-Command", "cd '$PWD\frontend'; Write-Host 'Frontend Server (Port 3000)' -ForegroundColor Yellow; npm start"

Write-Host ""
Write-Host "=== Services Starting ===" -ForegroundColor Cyan
Write-Host "Backend:  http://localhost:8000" -ForegroundColor White
Write-Host "Frontend: http://localhost:3000/AI_Book/" -ForegroundColor White
Write-Host ""
Write-Host "=== Testing Instructions ===" -ForegroundColor Cyan
Write-Host "1. Wait for frontend to open in browser (auto-opens)" -ForegroundColor White
Write-Host "2. Test the chatbot widget (bottom-right corner)" -ForegroundColor White
Write-Host "3. Ask a question like 'What is Physical AI?'" -ForegroundColor White
Write-Host "4. Verify you get a response with sources" -ForegroundColor White
Write-Host ""
Write-Host "Note: Auth-server is NOT needed (chatbot works without login)" -ForegroundColor Gray
Write-Host ""
Write-Host "Press any key to exit (servers will keep running)..." -ForegroundColor Gray
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
