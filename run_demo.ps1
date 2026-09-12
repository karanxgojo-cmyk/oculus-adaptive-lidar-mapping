# OCULUS PowerShell Launch Script
Write-Host "======================================================================" -ForegroundColor Cyan
Write-Host "  OCULUS: AUTONOMOUS VEHICLE RESEARCH PLATFORM" -ForegroundColor White
Write-Host "  Perception-Guided Adaptive 2.5D LiDAR Mapping" -ForegroundColor Cyan
Write-Host "======================================================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "Launching live sensor visualizer on http://127.0.0.1:8050 ..." -ForegroundColor Green
Write-Host ""

python main.py --mode web --port 8050
