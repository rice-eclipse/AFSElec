@echo off
rem Double-click to start the viewer, or drag CSV files onto this file to open them.
python "%~dp0flight_viewer.py" %*
if errorlevel 1 pause
