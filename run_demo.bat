@echo off
title OCULUS - Perception-Guided Adaptive 2.5D LiDAR Mapping
echo ======================================================================
echo   OCULUS: AUTONOMOUS VEHICLE RESEARCH PLATFORM
echo   Perception-Guided Adaptive 2.5D LiDAR Mapping
echo ======================================================================
echo.
echo Launching live sensor visualizer on http://127.0.0.1:8050 ...
echo.
python main.py --mode web --port 8050
pause
