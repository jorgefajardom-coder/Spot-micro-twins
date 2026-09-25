@echo off
REM Crea el entorno virtual del proyecto en .venv (no toca las librerías globales)
cd /d "%~dp0"
if not exist .venv python -m venv .venv
.venv\Scripts\python -m pip install --upgrade pip
.venv\Scripts\python -m pip install -r desktop\requirements.txt
echo.
echo Listo. Para usarlo:  .venv\Scripts\activate
