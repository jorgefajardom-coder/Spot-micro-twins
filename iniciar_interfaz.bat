@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
    echo Primero ejecuta setup_env.bat para instalar las dependencias.
    pause
    exit /b 1
)
".venv\Scripts\python.exe" "desktop\spotmicro_gui.py"
if errorlevel 1 (
    echo No se pudo abrir la interfaz. Revisa el error anterior.
    pause
    exit /b 1
)
